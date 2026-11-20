/**
 * @file pyro_spi_drv.cpp
 * @brief Implementation file for the PYRO SPI driver.
 * PYRo SPI 驱动实现文件。
 *
 * 结构仿 pyro_uart_drv.cpp：类实现 + 文件尾部的 HAL 弱回调覆盖。
 * 阻塞路径直接映射 HAL 返回值；DMA 路径"发起 + 等二值信号量"，
 * 完成/错误经弱回调（TxCplt / TxRxCplt / Error）在 ISR 中给信号量。
 *
 * @version 1.0.0
 * @date 2026-11-13
 */

/* Includes ------------------------------------------------------------------*/
#include "pyro_spi_drv.h"
#include "pyro_platform.h"

namespace pyro
{

/* Constructor and Destructor ------------------------------------------------*/
spi_drv_t::spi_drv_t(SPI_HandleTypeDef *hspi) : _hspi(hspi), _err(HAL_SPI_ERROR_NONE)
{
    _done = xSemaphoreCreateBinary();
    /* 移植补充：堆不足时不再 configASSERT 死循环（那会把故障点埋进中断里，
     * 只能靠 halt + 反汇编才能定位）。改为置 state.error 并把 busy 置位，
     * 后续 tx_dma/txrx_dma 会因 _done==nullptr 直接返回 PYRO_ERROR，
     * 故障可从驱动层观察到，不会卡住整个系统。 */
    if (_done == nullptr)
    {
        state.error = 0x01U;
        return; /* 不注册到 spi_map()：该实例不可用，调用方拿到 PYRO_ERROR */
    }

    /* 总线互斥量：多从机时串行化各设备的访问，并保证 _done 上
     * 同一时刻只有一个等待者（二值信号量只能唤醒一个任务）。
     * configUSE_MUTEXES = 1 已确认开启，带优先级继承。 */
    _bus_mutex = xSemaphoreCreateMutex();
    if (_bus_mutex == nullptr)
    {
        /* 退化为「无总线仲裁」：单设备仍可工作，多设备并发会出错。
         * 不阻止注册——lock()/unlock() 会因句柄为空返回 PYRO_ERROR，
         * 调用方据此可观察到故障。 */
        state.error = 0x01U;
    }

    /* 无条件注册：构造函数已私有，外部只能经 bsp_spi 拿到唯一实例，
     * 因此这里不存在「同句柄重复注册」的可能。
     *
     * 原实现是「if (!spi_map().exist(hspi)) 才注册」，那会把重复构造的
     * 编程错误静默吞掉（第二个实例照常构造成功、却永远收不到完成通知）。
     * 更糟的是它在多从机场景下必然触发——多个设备共用同一句柄时，
     * 第二个实例会被无声丢弃。此后重复构造由「构造函数私有」在编译期拦住，
     * 这里不需要再做运行时防重。 */
    spi_map()[hspi] = this;
}

spi_drv_t::~spi_drv_t()
{
    if (spi_map().exist(_hspi) && spi_map()[_hspi] == this)
    {
        spi_map().erase(_hspi);
    }
    if (_done != nullptr)
    {
        vSemaphoreDelete(_done);
        _done = nullptr;
    }
    if (_bus_mutex != nullptr)
    {
        vSemaphoreDelete(_bus_mutex);
        _bus_mutex = nullptr;
    }
}

/* Bus Ownership -------------------------------------------------------------*/
status_t spi_drv_t::acquire_bus(const uint32_t waittime)
{
    if (nullptr == _bus_mutex)
    {
        return PYRO_ERROR; /* 构造时分配失败：本实例无总线仲裁能力 */
    }
    /* 阻塞在互斥量上：总线被别的设备占用时，调用方任务在此挂起，
     * 而不是"查一下忙不忙然后返回失败"。超时才报 PYRO_BUSY。
     *
     * 实测开销（Release -Os / 170MHz）：单次约 1.2 us。
     * 主要是 FreeRTOS 的 configASSERT（含 xTaskGetSchedulerState 调用）、
     * 非内联的 vPortEnterCritical/vPortExitCritical，以及 mutex 的
     * 优先级继承记录（pvTaskIncrementMutexHeldCount）。 */
    return (xSemaphoreTake(_bus_mutex, pdMS_TO_TICKS(waittime)) == pdTRUE) ? PYRO_OK
                                                                          : PYRO_BUSY;
}

status_t spi_drv_t::release_bus()
{
    if (nullptr == _bus_mutex)
    {
        return PYRO_ERROR;
    }
    /* 重复释放由设备层的幂等封装负责屏蔽（见 lsm6dsv16x_spi_t::bus_release），
     * 这里只是一个直接的 give。实测约 1.9 us。 */
    (void)xSemaphoreGive(_bus_mutex);
    return PYRO_OK;
}

/* Static Map Management -----------------------------------------------------*/
map_t<SPI_HandleTypeDef *, spi_drv_t *> &spi_drv_t::spi_map()
{
    static map_t<SPI_HandleTypeDef *, spi_drv_t *> instance;
    return instance;
}

spi_drv_t *spi_drv_t::get(SPI_HandleTypeDef *hspi)
{
    if (!spi_map().exist(hspi))
    {
        return nullptr;
    }
    return spi_map()[hspi];
}

uint32_t spi_drv_t::last_error() const
{
    return _err;
}

bool spi_drv_t::dma_busy() const
{
    return state.busy != 0U;
}

/* Blocking Transmission Methods ---------------------------------------------*/
status_t spi_drv_t::tx(const uint8_t *tx, const uint16_t size, const uint32_t waittime)
{
    if (nullptr == _hspi || nullptr == tx || 0U == size)
    {
        return PYRO_PARAM_ERROR;
    }

    switch (HAL_SPI_Transmit(_hspi, tx, size, waittime))
    {
        case HAL_OK:
            return PYRO_OK;
        case HAL_BUSY:
            return PYRO_BUSY;
        case HAL_TIMEOUT:
            state.timeout = 0x01U;
            return PYRO_TIMEOUT;
        default:
            state.error = 0x01U;
            return PYRO_ERROR;
    }
}

status_t spi_drv_t::txrx(const uint8_t *tx, uint8_t *rx, const uint16_t size, const uint32_t waittime)
{
    if (nullptr == _hspi || nullptr == tx || nullptr == rx || 0U == size)
    {
        return PYRO_PARAM_ERROR;
    }

    switch (HAL_SPI_TransmitReceive(_hspi, tx, rx, size, waittime))
    {
        case HAL_OK:
            return PYRO_OK;
        case HAL_BUSY:
            return PYRO_BUSY;
        case HAL_TIMEOUT:
            state.timeout = 0x01U;
            return PYRO_TIMEOUT;
        default:
            state.error = 0x01U;
            return PYRO_ERROR;
    }
}

/* DMA Transmission Methods (fire-and-forget) --------------------------------*/
status_t spi_drv_t::tx_dma(const uint8_t *tx, const uint16_t size)
{
    if (nullptr == _hspi || nullptr == tx || 0U == size)
    {
        return PYRO_PARAM_ERROR;
    }
    /* 构造时信号量分配失败：该实例不可用，直接报错（不碰 _done） */
    if (nullptr == _done)
    {
        state.error = 0x01U;
        return PYRO_ERROR;
    }
    /* 移植补充（同 pyro_uart_drv）：CubeMX 未给该 SPI 分配 TX DMA 通道时
     * hdmatx 为 NULL，HAL 会跳过 DMA 配置并永久留在 BUSY_TX，必须提前拦截。 */
    if (nullptr == _hspi->hdmatx)
    {
        state.error = 0x01U;
        return PYRO_ERROR;
    }
    if (state.busy)
    {
        return PYRO_BUSY;
    }

    /* 清掉可能残留的迟到信号：保证本次等待只被本次事务的完成唤醒 */
    (void)xSemaphoreTake(_done, 0);

    _err       = HAL_SPI_ERROR_NONE;
    state.busy = 0x01U; /* 必须先于 HAL 调用置位：完成中断可能在 HAL 返回前触发 */

    const HAL_StatusTypeDef hal = HAL_SPI_Transmit_DMA(_hspi, tx, size);
    if (hal != HAL_OK)
    {
        state.busy = 0x00U;
        /* HAL_BUSY：外设被另一笔事务占用。多从机时这由总线锁先行拦截，
         * 这里是第二道防线。必须报 PYRO_BUSY 而非 ERROR：调用方据此
         * 走"重试/等待"，而不是当成硬故障累计 err_bus。 */
        if (hal == HAL_BUSY)
        {
            return PYRO_BUSY;
        }
        state.error = 0x01U;
        return PYRO_ERROR;
    }

    return PYRO_OK; /* 已发起，立即返回；完成用 wait_dma() 查询 */
}

status_t spi_drv_t::txrx_dma(const uint8_t *tx, uint8_t *rx, const uint16_t size)
{
    if (nullptr == _hspi || nullptr == tx || nullptr == rx || 0U == size)
    {
        return PYRO_PARAM_ERROR;
    }
    if (nullptr == _done)
    {
        state.error = 0x01U;
        return PYRO_ERROR;
    }
    if (nullptr == _hspi->hdmatx || nullptr == _hspi->hdmarx)
    {
        state.error = 0x01U;
        return PYRO_ERROR;
    }
    if (state.busy)
    {
        return PYRO_BUSY;
    }

    (void)xSemaphoreTake(_done, 0);

    _err       = HAL_SPI_ERROR_NONE;
    state.busy = 0x01U;

    const HAL_StatusTypeDef hal = HAL_SPI_TransmitReceive_DMA(_hspi, tx, rx, size);
    if (hal != HAL_OK)
    {
        state.busy = 0x00U;
        /* 同 tx_dma：外设忙 → PYRO_BUSY（可重试），其余为硬错误 */
        if (hal == HAL_BUSY)
        {
            return PYRO_BUSY;
        }
        state.error = 0x01U;
        return PYRO_ERROR;
    }

    return PYRO_OK; /* 已发起，立即返回；完成用 wait_dma() 查询 */
}

status_t spi_drv_t::wait_dma(const uint32_t waittime)
{
    if (nullptr == _done)
    {
        return PYRO_ERROR; /* 实例不可用 */
    }
    /* 无在途事务：绝不干等。若尚有未收的完成信号（上一笔完成但未被等待），
     * 收掉并按快照错误码报告其结果；否则直接返回 OK。 */
    if (!state.busy)
    {
        if (xSemaphoreTake(_done, 0) == pdTRUE)
        {
            return (_err == HAL_SPI_ERROR_NONE) ? PYRO_OK : PYRO_ERROR;
        }
        return PYRO_OK;
    }

    if (xSemaphoreTake(_done, pdMS_TO_TICKS(waittime)) != pdTRUE)
    {
        /* 超时：作废迟到的完成/错误信号 → 中止在途 DMA。
         * DMAStop 内部的 DMA Abort 可能触发完成回调，已被 state.busy=0 挡掉。 */
        state.busy    = 0x00U;
        state.timeout = 0x01U;
        (void)HAL_SPI_DMAStop(_hspi);
        return PYRO_TIMEOUT;
    }

    state.busy = 0x00U;
    if (_err != HAL_SPI_ERROR_NONE)
    {
        state.error = 0x01U;
        return PYRO_ERROR;
    }
    return PYRO_OK;
}

status_t spi_drv_t::abort()
{
    if (!state.busy)
    {
        return PYRO_OK;
    }
    state.busy = 0x00U;
    (void)HAL_SPI_DMAStop(_hspi);
    /* 吞掉中止过程中可能到来的 give，避免残留信号唤醒下一次等待 */
    if (_done != nullptr)
    {
        (void)xSemaphoreTake(_done, 0);
    }
    return PYRO_OK;
}

void spi_drv_t::set_cplt_hook(void (*hook)(void *), void *ctx)
{
    /* 绑定/解绑都必须发生在总线所有权窗口内（acquire_bus 之后、
     * release_bus 之前），详见头文件注释。
     * 解绑时传 (nullptr, nullptr)，两条字写入，无分支。 */
    _cplt_hook = hook;
    _cplt_ctx  = ctx;
}

void spi_drv_t::notify_from_isr()
{
    if (!state.busy)
    {
        return; /* 已被超时/abort 路径作废，忽略迟到信号 */
    }

    /* ISR 只碰 state.busy 与 _err（字写入，原子）：错误码到 state.error 的
     * 映射留给任务侧，避免位域 RMW 与任务并发丢标志。 */
    state.busy = 0x00U;
    _err       = HAL_SPI_GetError(_hspi);

    /* 完成点先释放硬件时序（如 CS）：数据已完整（BSY=0），不等任务唤醒，
     * CS 低电平尾部最短；错误路径同样调用，语义为"事务结束、可以松手" */
    if (_cplt_hook != nullptr)
    {
        _cplt_hook(_cplt_ctx);
    }

    if (_done == nullptr)
    {
        return; /* 构造时分配失败：不该有在途事务，防御性返回 */
    }
    BaseType_t higher_prio_task_woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(_done, &higher_prio_task_woken);
    portYIELD_FROM_ISR(higher_prio_task_woken);
}

} // namespace pyro

/* External HAL/ISR Callbacks ------------------------------------------------*/
/* HAL 弱回调覆盖（USE_HAL_SPI_REGISTER_CALLBACKS = 0，RegisterCallback API 不可用）。
 * 三条路径都收敛到 notify_from_isr()：
 *   - 全双工收发（HAL_SPI_TransmitReceive_DMA）→ TxRxCplt（RX 通道 TC 驱动）
 *   - 仅发送（HAL_SPI_Transmit_DMA）           → TxCplt
 *   - DMA / SPI 错误                           → ErrorCallback（HAL 已中止 DMA）
 * 错误路径也必须通知：否则等待任务要干等满超时。
 * DMA1_Channel1/2 与 SPI1_IRQn 优先级均为 5，等于 configMAX_SYSCALL_INTERRUPT_PRIORITY
 * （5<<4=0x50），FromISR API 合法（FreeRTOS 要求 NVIC 优先级数值 >= 该值）。 */

namespace
{

void spi_notify(SPI_HandleTypeDef *hspi)
{
    pyro::spi_drv_t *drv = pyro::spi_drv_t::get(hspi);
    if (drv != nullptr)
    {
        drv->notify_from_isr();
    }
}

} // namespace

extern "C" PYRO_ISR_TEXT void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_notify(hspi);
}

extern "C" PYRO_ISR_TEXT void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_notify(hspi);
}

extern "C" PYRO_ISR_TEXT void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    spi_notify(hspi);
}
