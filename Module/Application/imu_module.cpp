/**
 * @file    imu_module.cpp
 * @brief   产品装配入口：创建并启动全部常驻任务，定义彼此的唤醒关系。
 *
 * ============================ 任务拓扑 ============================
 *
 *   ins_cap (HIGH)            ins_algo (ABOVE_NORMAL)        imu_pub (NORMAL)
 *   ─────────────             ──────────────────────         ────────────────
 *   每 1 ms 硬周期            等 s_cap_sem（有新样本）        等 s_algo_sem（有新快照）
 *   read() → SPSC publish      fuse_once() → 发快照           取快照 → 编码 → 发送
 *                                   │                              ▲
 *                                   └──── s_algo_sem ──────────────┘
 *                                   └──── s_cap_sem（回给自己，等下一批）▲
 *
 * ============================ 为什么这样分工 ============================
 * ① 采集与融合必须解耦：1920 Hz 下 update() 峰值曾达 835 µs，串在 1 ms 周期里
 *    会挤压采集；拆开后采集由 SPSC 缓冲与信号量驱动，融合再慢也不影响它。
 * ② 发布由【算法完成时】唤醒，而不是自己 1 ms 轮询：
 *    · 无竞态——发布只在快照刚写好之后被唤醒，读到的必是最新一份；
 *    · 无空转——融合若因某种原因降频，发布不会抢在它前面反复编码同一份旧数据；
 *    · 发出去的永远是最新数据，而不是"上一拍算完、这一拍才发"。
 * ③ 任务创建权在本层而非服务内部：装配顺序是产品决策，
 *    采集与融合各自只对外提供 capture_once() / fuse_once() 与快照。
 *
 * ============================ 顺序不可交换 ============================
 *   ① capture::init() 完成器件初始化；fusion::init() 完成算法配置与装配参数导出；
 *   ② 再创建采集与融合任务；
 *   ③ 最后创建发布任务并建立信号量关系。
 * 发布任务创建失败不应阻止采集/融合（板级调试时仍能通过调试器读到姿态）。
 */

#include "imu_module.h"

#include "FreeRTOS.h"
#include "imu_config.h"
#include "imu_publisher.h"
#include "imu_capture.h"
#include "imu_fusion.h"
#include "pyro_core_def.h"
#include "pyro_task.h"
#include "semphr.h"
#include "task.h"

volatile int32_t imu_module_init_result = 1;

namespace imu_module
{
namespace
{

/* ======================= 任务间信号量 =======================
 * s_cap_sem  —— 采集完成后唤醒融合（二值，不累积：融合总处理"当时可用的全部样本"）
 * s_algo_sem —— 融合产出新快照后唤醒发布（二值，同上：发布总取最新快照）
 * 二值信号量的语义在这里正好：积压没有意义，只要"有新数据"这一个事实。 */
SemaphoreHandle_t s_cap_sem  = nullptr;
SemaphoreHandle_t s_algo_sem = nullptr;

/* ======================= 采集任务 =======================
 * 1 ms 固定周期驱动 read()。挂起在 DMA 完成信号量上，不占 CPU。
 * 解出样本后唤醒融合任务；无样本时也唤醒一次，让融合有机会归还/推进计时。 */
class cap_task_t final : public pyro::task_base_t
{
  public:
    cap_task_t()
        : task_base_t("ins_cap", 0U, config::kCapTaskStackWords,
                      pyro::task_base_t::priority_t::HIGH)
    {
    }

  protected:
    pyro::status_t init() override { return pyro::PYRO_OK; }

    void run_loop() override
    {
        TickType_t wake = xTaskGetTickCount();
        for (;;)
        {
            capture::capture_once();
            (void)xSemaphoreGive(s_cap_sem);
            vTaskDelayUntil(&wake, config::kPublishPeriodMs);
        }
    }
};

/* ======================= 融合任务 =======================
 * 由 s_cap_sem 唤醒，不再自己按 1 ms 轮询：
 *   · 采集一旦产出就立刻融合，延迟最小；
 *   · 融合本身就慢于采集时，信号量自然合并成"尽量快"，不会堆积。
 * 产出新快照后唤醒发布任务。 */
class algo_task_t final : public pyro::task_base_t
{
  public:
    algo_task_t()
        : task_base_t("ins_algo", 0U, config::kInsTaskStackWords,
                      pyro::task_base_t::priority_t::ABOVE_NORMAL)
    {
    }

  protected:
    pyro::status_t init() override { return pyro::PYRO_OK; }

    void run_loop() override
    {
        for (;;)
        {
            if (xSemaphoreTake(s_cap_sem, portMAX_DELAY) != pdTRUE)
            {
                continue;
            }
            if (fusion::fuse_once())
            {
                /* 只在真正产出新快照时唤醒发布，避免发布任务被空拍唤醒 */
                (void)xSemaphoreGive(s_algo_sem);
            }
        }
    }
};

/* 任务实例（静态存储，不占堆的 TCB 之外的额外 RAM） */
cap_task_t  s_cap_task;
algo_task_t s_algo_task;

} // namespace
} // namespace imu_module

extern "C" int imu_module_init(void)
{
    using namespace imu_module;

    if (imu_module_init_result == 0)
    {
        return 0;
    }

    /* ① 器件初始化 + 算法配置（在调用者上下文同步完成） */
    const pyro::status_t ins = capture::init();
    switch (ins)
    {
        case pyro::PYRO_OK:
        case pyro::PYRO_ALREADY_INIT:
            break;
        case pyro::PYRO_NO_MEMORY:
            imu_module_init_result = -2;
            return imu_module_init_result;
        case pyro::PYRO_PARAM_ERROR:
            imu_module_init_result = -3;
            return imu_module_init_result;
        default:
            imu_module_init_result = -1;
            return imu_module_init_result;
    }

    /* ② 信号量：先于任务创建，任务一跑起来就能拿到它们 */
    s_cap_sem  = xSemaphoreCreateBinary();
    s_algo_sem = xSemaphoreCreateBinary();
    if (s_cap_sem == nullptr || s_algo_sem == nullptr)
    {
        imu_module_init_result = -2;
        return imu_module_init_result;
    }

    /* ③ 采集 + 融合。采集先起：它是一切数据的源头 */
    const pyro::status_t cap = s_cap_task.start();
    if (cap != pyro::PYRO_OK)
    {
        imu_module_init_result = (cap == pyro::PYRO_NO_MEMORY) ? -2 : -1;
        return imu_module_init_result;
    }
    const pyro::status_t algo = s_algo_task.start();
    if (algo != pyro::PYRO_OK)
    {
        imu_module_init_result = (algo == pyro::PYRO_NO_MEMORY) ? -2 : -1;
        return imu_module_init_result;
    }

    imu_service_heap_free_min = static_cast<uint32_t>(xPortGetFreeHeapSize());

    /* ③.5 融合侧配置：装配参数要读采集侧的解包参数与零偏，
     *      因此必须放在 capture::init() 之后。 */
    const pyro::status_t fus = fusion::init();
    if (fus != pyro::PYRO_OK)
    {
        imu_module_init_result = (fus == pyro::PYRO_PARAM_ERROR) ? -3 : -1;
        return imu_module_init_result;
    }

    /* ④ 发布任务：拿到 s_algo_sem 才算建立完整链路。
     *    失败不回滚采集/融合——板级调试时仍能从调试器读到姿态。 */
    publisher_t::instance().set_wake_semaphore(s_algo_sem);
    const pyro::status_t pub = publisher_t::instance().init();
    if (pub != pyro::PYRO_OK && pub != pyro::PYRO_ALREADY_INIT)
    {
        imu_module_init_result = -4;
        return imu_module_init_result;
    }

    imu_module_init_result = 0;
    return imu_module_init_result;
}
