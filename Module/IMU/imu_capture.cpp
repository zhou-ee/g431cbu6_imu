/**
 * @file    imu_capture.cpp
 * @brief   采集侧实现：器件总线绑定 + read() 一拍 + 采样统计。
 *
 * 本文件是【唯一】持有硬件对象的地方：换芯片/换板只改这里的三个常量。
 * 采集输出器件坐标系的物理量，不感知装配方向与融合算法。
 */

#include "imu_capture.h"

#include <new> /* std::nothrow：init 中按需 new 总线/器件对象 */

#include "FreeRTOS.h"
#include "imu_config.h"
#include "lsm6dsv16x_drv.h"
#include "pyro_bsp_spi.h"
#include "pyro_dwt_drv.h"
#include "spi.h"
#include "task.h"

/* ======================= 调试观测变量 =======================
 * 必须放全局作用域（extern "C"），否则 ELF 里的 C 链接裸名字与调试器
 * 按命名空间查找的名字对不上，gdb 会报 Missing ELF symbol。 */
extern "C" {
volatile int32_t  imu_service_init_result      = 1;
volatile uint32_t imu_service_read_ok          = 0U;
volatile uint32_t imu_service_cap_rounds       = 0U;
volatile uint32_t imu_service_cap_us_max       = 0U;
volatile uint32_t imu_service_cap_us_avg       = 0U;
volatile uint32_t imu_service_cap_stack_free   = 0U;
volatile uint32_t imu_service_err_bus          = 0U;
volatile uint32_t imu_service_err_data         = 0U;
volatile uint32_t imu_service_err_ovr          = 0U;
volatile uint32_t imu_service_err_backpressure = 0U;
volatile uint32_t imu_service_err_overrun      = 0U;
volatile uint32_t imu_service_err_timeout      = 0U;
volatile uint32_t imu_service_heap_free_min    = 0U;
}

namespace imu_module
{
namespace capture
{
namespace
{

/* ======================= 板级绑定与总线装配 =======================
 * 换芯片/换板只改这几行。片选由产品装配层绑定（CubeMX 中 PA4 初始为高）。
 *
 * 【为什么用指针 + init 中创建，而不是文件作用域对象】
 * 程序启动顺序是：
 *   .bss 清零 → __libc_init_array（C++ 静态构造）→ main()
 *     → HAL_Init → MX_GPIO_Init → MX_DMA_Init → MX_SPI1_Init → osKernelStart
 * 若把这些对象写成文件作用域变量，它们会在 __libc_init_array 阶段构造——
 * 那时 hspi1 尚未初始化、FreeRTOS 堆也未经过任何初始化，而 spi_drv_t 的
 * 构造函数会调用 xSemaphoreCreateBinary()。当前能工作只是因为 heap_4 的堆
 * 恰好是 .bss 里的静态数组、不需要额外初始化，这是运气而非保证。
 *
 * 因此改为：全局指针 + 在 capture::init() 中按需 new。
 * 此时 main() 已跑完外设初始化、调度器已启动，构造是安全的。
 *
 * 【一主多从】s_spi_bus 是总线级对象（bsp_spi 唯一持有），可挂多颗器件；
 * s_bus / s_imu 是设备级对象，将来加第二颗只需再 new 一个 lsm6dsv16x_spi_t，
 * 换一个 cs_pin、共用同一个 s_spi_bus。 */
GPIO_TypeDef *const k_cs_port = GPIOA;
constexpr uint16_t  k_cs_pin  = GPIO_PIN_4;

/* 总线级：由 BSP 持有唯一实例，这里只是引用，不参与构造时机决策。
 * 引用在静态初始化期绑定到 get_spi1() 的返回值——注意 get_spi1() 内部的
 * 函数局部静态此刻就会被构造，这正是上面要改成 init 中构造的原因：
 * 所以这里也改为惰性获取，只保留指针。 */
pyro::spi_drv_t *s_spi_bus = nullptr;

/* 设备级：可以有多个，各自一根 CS，初始化时按需创建。 */
lsm6dsv16x_spi_t *s_bus = nullptr;
lsm6dsv16x_drv_t *s_imu = nullptr;

/* 最新一帧解包物理量（器件坐标系）。允许陈旧一拍：
 * 协议取"最新驱动物理量"语义，与融合结果同组封包。 */
float s_last_acc_dev[3]  = {0.0f, 0.0f, 0.0f};
float s_last_gyro_dev[3] = {0.0f, 0.0f, 0.0f};

/* 采集侧 1 s 窗口统计（与融合侧各用各的边界，互不影响）。 */
uint32_t s_cap_us_max  = 0U;
uint32_t s_cap_us_sum  = 0U;
uint32_t s_cap_win     = 0U;
uint32_t s_cap_next_1s = 0U;

} // namespace

pyro::status_t init()
{
    /* DWT 用于统计采集/融合单批耗时 */
    pyro::dwt_drv_t::init(config::kCpuMhz);

    /* ① 总线：此刻 main() 已完成 MX_SPI1_Init()、调度器已启动，
     *    get_spi1() 内部的函数局部静态在这里才首次构造，安全。 */
    if (nullptr == s_spi_bus)
    {
        s_spi_bus = &pyro::bsp_spi::get_spi1();
    }

    /* ② 设备：按需创建。加第二颗器件就在这里再 new 一个，
     *    换 cs_pin、共用 s_spi_bus 即可（多从机还需总线互斥，见 spi_drv_t）。 */
    if (nullptr == s_bus)
    {
        s_bus = new (std::nothrow) lsm6dsv16x_spi_t(*s_spi_bus, k_cs_port, k_cs_pin, true);
        if (nullptr == s_bus)
        {
            imu_service_init_result = -2;
            return pyro::PYRO_NO_MEMORY;
        }
    }
    if (nullptr == s_imu)
    {
        s_imu = new (std::nothrow) lsm6dsv16x_drv_t(config::driver_config(), *s_bus);
        if (nullptr == s_imu)
        {
            imu_service_init_result = -2;
            return pyro::PYRO_NO_MEMORY;
        }
    }

    const bool ok = s_imu->init();
    imu_service_init_result = ok ? 0 : 1;
    if (!ok)
    {
        return pyro::PYRO_ERROR;
    }

    s_cap_next_1s = static_cast<uint32_t>(xTaskGetTickCount()) + 1000U;
    return pyro::PYRO_OK;
}

void capture_once()
{
    const uint32_t t0 = pyro::dwt_drv_t::get_current_ticks();
    if (s_imu != nullptr && s_imu->read())
    {
        ++imu_service_read_ok;

        imu_data_t &d = s_imu->read_imu_data();
        if (d.gyro.available() != 0U)
        {
            const gyro_sample_t &g = d.gyro.cur();
            s_last_gyro_dev[0] = g.Gyro[0];
            s_last_gyro_dev[1] = g.Gyro[1];
            s_last_gyro_dev[2] = g.Gyro[2];
        }
        if (d.acc.available() != 0U)
        {
            const acc_sample_t &a = d.acc.cur();
            s_last_acc_dev[0] = a.Accel[0];
            s_last_acc_dev[1] = a.Accel[1];
            s_last_acc_dev[2] = a.Accel[2];
        }
    }

    /* 驱动侧诊断直通（累计计数）：read() 失败时也要刷新，
     * 否则看不出坏在哪一步。数据源是驱动自己的计数器，本层不重复统计。 */
    if (s_imu != nullptr)
    {
        const lsm6dsv16x_drv_t::diag_t dg = s_imu->diag();
        imu_service_err_bus          = dg.err_bus;
        imu_service_err_data         = dg.err_data;
        imu_service_err_timeout      = dg.err_timeout;
        imu_service_err_ovr          = dg.err_ovr;
        imu_service_err_backpressure = dg.err_backpressure;
        imu_service_err_overrun      = dg.err_overrun;
    }

    /* 采集侧耗时（含两段 DMA 等待，是"采集是否守住 1 ms"的直接口径）。 */
    const uint32_t us = static_cast<uint32_t>(
        (pyro::dwt_drv_t::get_current_ticks() - t0) / config::kCpuMhz);
    if (us > s_cap_us_max) { s_cap_us_max = us; }
    s_cap_us_sum += us;
    ++s_cap_win;                 /* 先计数，重置时才不会把本次漏掉 */
    ++imu_service_cap_rounds;

    const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount());
    if (static_cast<int32_t>(now - s_cap_next_1s) >= 0)
    {
        s_cap_next_1s += 1000U;
        imu_service_cap_us_max     = s_cap_us_max;
        imu_service_cap_us_avg     = (s_cap_win > 0U) ? (s_cap_us_sum / s_cap_win) : 0U;
        imu_service_cap_stack_free = uxTaskGetStackHighWaterMark(nullptr);
        s_cap_us_max = 0U;
        s_cap_us_sum = 0U;
        s_cap_win    = 0U;
    }
}

imu_data_t &data()
{
    /* 返回引用，不能返回 nullptr：init 尚未成功时给一个静态空对象。
     * 消费侧（fusion）此时还没启动，不会真的读到有意义的样本。 */
    static imu_data_t s_empty{};
    return (s_imu != nullptr) ? s_imu->read_imu_data() : s_empty;
}

void release_consumed(uint16_t gyro_count, uint16_t acc_count)
{
    if (s_imu == nullptr) { return; }
    s_imu->assume_gyro_point(gyro_count);
    s_imu->assume_acc_point(acc_count);
}

float acc_mps2_per_lsb()
{
    return (s_imu != nullptr) ? s_imu->acc_mps2_per_lsb() : 0.0f;
}

float gyro_rps_per_lsb()
{
    return (s_imu != nullptr) ? s_imu->gyro_rps_per_lsb() : 0.0f;
}

const imu_cali_t &cali()
{
    static const imu_cali_t s_empty{};
    return (s_imu != nullptr) ? s_imu->read_imu_data().cali : s_empty;
}

const float *last_acc_dev()
{
    return s_last_acc_dev;
}

const float *last_gyro_dev()
{
    return s_last_gyro_dev;
}

diag_t diag()
{
    if (s_imu == nullptr)
    {
        return {0U, 0U, 0U, 0U, 0U, 0U};
    }
    const lsm6dsv16x_drv_t::diag_t d = s_imu->diag();
    return {d.err_bus, d.err_data, d.err_timeout, d.err_ovr, d.err_backpressure,
            d.err_overrun};
}

uint32_t cap_rounds()      { return imu_service_cap_rounds; }
uint32_t cap_us_max()      { return imu_service_cap_us_max; }
uint32_t cap_us_avg()      { return imu_service_cap_us_avg; }
uint32_t cap_stack_free()  { return imu_service_cap_stack_free; }

void clear_error_counters()
{
    /* 清驱动内部计数器 + 同步镜像，避免下一拍 capture_once() 把旧值写回来。 */
    if (s_imu != nullptr) { s_imu->clear_error_counters(); }
    imu_service_err_bus          = 0U;
    imu_service_err_data         = 0U;
    imu_service_err_timeout      = 0U;
    imu_service_err_ovr          = 0U;
    imu_service_err_backpressure = 0U;
    imu_service_err_overrun      = 0U;
}

} // namespace capture
} // namespace imu_module
