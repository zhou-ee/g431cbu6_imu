/**
 * @file    imu_service.cpp
 * @brief   IMU 采集与解算服务实现（任务层）。
 *
 * 两个任务一条单向链路，无共享锁：
 *   capture（高优先级，1 ms 周期）跑 read()，DMA 完成中断在此让出 CPU 后又唤醒；
 *   algo   （普通优先级）由 capture 用二值信号量唤醒，取数后归还环形缓冲。
 *
 * 样本交接是单生产者 / 单消费者：
 *   capture 只写新槽、不动 head；algo 只前移 head。前提是未消费数不达容量，
 *   1 ms 唤醒一次、每次全量 drain 时 count ≈ 8，离 24 的容量有余量。
 *   若 algo 被拖慢到 3 ms 以上，驱动会开始覆盖最旧样本并计入 err_data，
 *   该计数即为链路拥塞的观测口径。
 */

#include "imu_service.h"

#include "lsm6dsv16x_drv.h"
#include "pyro_task.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* ======================= 调试观测变量 ======================= */
extern "C" {
volatile int32_t  imu_service_init_result  = -1;
volatile uint32_t imu_service_read_ok      = 0U;
volatile uint32_t imu_service_algo_rounds  = 0U;
volatile uint32_t imu_service_algo_samples = 0U;
volatile uint32_t imu_service_err_bus      = 0U;
volatile uint32_t imu_service_err_data     = 0U;
volatile uint32_t imu_service_err_timeout  = 0U;
volatile float    imu_service_accel_mps2[3] = {0.0f, 0.0f, 0.0f};
volatile float    imu_service_gyro_rps[3]   = {0.0f, 0.0f, 0.0f};
volatile uint32_t imu_service_algo_ts      = 0U;
}

namespace
{

/* ======================= 产品配置 ======================= */
const imu_cfg_t k_imu_cfg = {
    false,                  /* calibrate_flag ：关闭上电标定，使用 imu_cali_t 中的偏置 */
    500U,                   /* calibrate_times：7680 Hz 下约 65 ms */
    4U,                     /* acc_range      ：±4 g   */
    7680U,                  /* acc_freq       ：7680 Hz */
    2000U,                  /* gyro_range     ：±2000 dps */
    7680U,                  /* gyro_freq      ：7680 Hz */
    true,                   /* fifo_en        ：true=FIFO 批量；false=寄存器直读回退 */
    imu_direct_t::DIRECT_1, /* direct         ：按实际安装方向改这里 */
};

/* ======================= 板级绑定 ======================= */
/* 片选由产品装配层绑定（CubeMX 中 PA4 初始为高）。换芯片/换板只改这两行。 */
GPIO_TypeDef *const k_cs_port = GPIOA;
const uint16_t k_cs_pin = GPIO_PIN_4;

/* ======================= 总线与驱动装配 ======================= */
/* 换 SPI/I²C 只需换这一行；驱动不感知总线类型。
 * 第 4 参 dma_enable：true = 收发走 DMA，完成经中断通知并唤醒任务。 */
lsm6dsv16x_spi_t s_bus(&hspi1, k_cs_port, k_cs_pin, true);
lsm6dsv16x_drv_t s_imu(k_imu_cfg, s_bus);

/* capture → algo 的唤醒信号量（二值：只表示"有新数据待取"，不累积计数）。 */
SemaphoreHandle_t s_algo_sem = nullptr;

/* ======================= 采集任务 ======================= */
class imu_capture_task_t final : public pyro::task_base_t
{
  public:
    imu_capture_task_t() : task_base_t("imu_cap", 0U, 256U, pyro::task_base_t::priority_t::HIGH) {}

  protected:
    /* 器件初始化放在 init()：task_base_t::start() 会在调用者上下文同步执行它，
     * 因此 imu_service_start() 返回时就能拿到真实的初始化结果。
     * 调用时机要求调度器已启动（read() 内部要等 DMA 完成信号量）。 */
    pyro::status_t init() override
    {
        const bool ok = s_imu.init();
        imu_service_init_result = ok ? 0 : 1;
        return ok ? pyro::PYRO_OK : pyro::PYRO_ERROR;
    }

    void run_loop() override
    {
        TickType_t wake = xTaskGetTickCount();
        for (;;)
        {
            /* 一拍的全部动作：内部自行完成状态查询与两段 DMA 等待。
             * 本任务不感知数据来源，也不感知状态字节。 */
            if (s_imu.read())
            {
                ++imu_service_read_ok;

                /* 唤醒算法任务取数。二值信号量：上一次若还没取走，
                 * 这里 give 失败也无妨——数据在环形缓冲里，不会丢。 */
                (void)xSemaphoreGive(s_algo_sem);
            }

            const lsm6dsv16x_drv_t::diag_t d = s_imu.diag();
            imu_service_err_bus     = d.err_bus;
            imu_service_err_data    = d.err_data;
            imu_service_err_timeout = d.err_timeout;

            /* 固定周期：把本拍耗时从间隔里扣掉，避免执行时间累积成降频。 */
            vTaskDelayUntil(&wake, 1U);
        }
    }
};

/* ======================= 算法任务（本轮留空） ======================= */
class imu_algo_task_t final : public pyro::task_base_t
{
  public:
    imu_algo_task_t() : task_base_t("imu_algo", 0U, 192U, pyro::task_base_t::priority_t::NORMAL) {}

  protected:
    pyro::status_t init() override { return pyro::PYRO_OK; }

    void run_loop() override
    {
        for (;;)
        {
            if (xSemaphoreTake(s_algo_sem, portMAX_DELAY) != pdTRUE)
            {
                continue;
            }

            imu_data_t &data = s_imu.read_imu_data();
            const uint8_t ng = data.gyro.count;
            const uint8_t na = data.acc.count;
            if ((ng == 0U) && (na == 0U))
            {
                continue;
            }

            ++imu_service_algo_rounds;

            /* 姿态融合留待下一阶段：当前只取最新一帧供观测，
             * 随后整批归还——不归还的话 24 帧（约 3 ms）就被填满并触发覆盖计数。 */
            if (ng != 0U)
            {
                const gyro_sample_t &g = data.gyro.cur();
                imu_service_gyro_rps[0] = g.Gyro[0];
                imu_service_gyro_rps[1] = g.Gyro[1];
                imu_service_gyro_rps[2] = g.Gyro[2];
                imu_service_algo_ts     = g.timestamp;
            }
            if (na != 0U)
            {
                const acc_sample_t &a = data.acc.cur();
                imu_service_accel_mps2[0] = a.Accel[0];
                imu_service_accel_mps2[1] = a.Accel[1];
                imu_service_accel_mps2[2] = a.Accel[2];
            }

            s_imu.assume_gyro_point(ng);
            s_imu.assume_acc_point(na);
            imu_service_algo_samples += static_cast<uint32_t>(ng) + na;
        }
    }
};

imu_capture_task_t s_capture_task;
imu_algo_task_t s_algo_task;

bool s_started = false;

} // namespace

/* ============================== 对外接口 ============================== */

imu_data_t &imu_service_data()
{
    return s_imu.read_imu_data();
}

pyro::status_t imu_service_start()
{
    if (s_started)
    {
        return pyro::PYRO_ALREADY_INIT;
    }

    /* 先建同步量：任务跑起来之前必须就绪，失败则整链路不可用。
     * （xSemaphoreCreateBinary 依赖 FreeRTOS 堆，故放在这里而不是静态初始化期） */
    if (s_algo_sem == nullptr)
    {
        s_algo_sem = xSemaphoreCreateBinary();
        if (s_algo_sem == nullptr)
        {
            return pyro::PYRO_NO_MEMORY;
        }
    }

    /* 先拉起算法任务（消费者），再拉起采集任务（生产者）：
     * capture 的优先级高于调用者，任务一建成就会立刻抢占并开始 give，
     * 先有消费者可避免开局几拍"给了没人收"的噪声。 */
    const pyro::status_t algo_ret = s_algo_task.start();
    if (algo_ret != pyro::PYRO_OK)
    {
        return pyro::PYRO_PARAM_ERROR;
    }

    /* 器件初始化在 capture 的 init() 里同步执行，失败则回滚已建的算法任务。 */
    const pyro::status_t ret = s_capture_task.start();
    if (ret != pyro::PYRO_OK)
    {
        s_algo_task.stop();
        return ret;
    }

    s_started = true;
    return pyro::PYRO_OK;
}
