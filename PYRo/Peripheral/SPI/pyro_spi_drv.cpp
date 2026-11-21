/**
 * @file pyro_spi_drv.cpp
 * @brief PYRo SPI 总线驱动实现。
 */

#include "pyro_spi_drv.h"
#include "pyro_platform.h"

namespace pyro
{

spi_drv_t::spi_drv_t(SPI_HandleTypeDef *hspi) : _hspi(hspi), _err(HAL_SPI_ERROR_NONE)
{
    _done = xSemaphoreCreateBinary();
    /* 堆不足时不 configASSERT 死循环——那会把故障点埋进中断里。
     * 改为置 state.error 并跳过注册，后续调用拿到 PYRO_ERROR。 */
    if (_done == nullptr)
    {
        state.error = 0x01U;
        return;
    }

    /* 保证 _done 上同一时刻只有一个等待者（二值信号量只唤醒一个任务）。 */
    _bus_mutex = xSemaphoreCreateMutex();
    if (_bus_mutex == nullptr)
    {
        state.error = 0x01U; /* 退化为无总线仲裁，acquire_bus 会返回错误 */
    }

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
    /* 阻塞在互斥量上：总线被占用时调用方任务挂起，而非"查一下忙就返回失败"。 */
    return (xSemaphoreTake(_bus_mutex, pdMS_TO_TICKS(waittime)) == pdTRUE) ? PYRO_OK
                                                                          : PYRO_BUSY;
}

status_t spi_drv_t::release_bus()
{
    if (nullptr == _bus_mutex)
    {
        return PYRO_ERROR;
    }
    /* 重复释放由设备层的幂等封装屏蔽（见 lsm6dsv16x_spi_t::bus_release）。 */
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
    /* CubeMX 未分配 TX DMA 通道时 hdmatx 为 NULL，HAL 会跳过 DMA 配置并永久
     * 留在 BUSY_TX，必须提前拦截。 */
    if (nullptr == _hspi->hdmatx)
    {
        state.error = 0x01U;
        return PYRO_ERROR;
    }
    if (state.busy)
    {
        return PYRO_BUSY;
    }

    /* 清掉残留的迟到信号，保证本次等待只被本次事务唤醒。 */
    (void)xSemaphoreTake(_done, 0);

    _err       = HAL_SPI_ERROR_NONE;
    state.busy = 0x01U; /* 必须先于 HAL 调用：完成中断可能在 HAL 返回前触发 */

    const HAL_StatusTypeDef hal = HAL_SPI_Transmit_DMA(_hspi, tx, size);
    if (hal != HAL_OK)
    {
        state.busy = 0x00U;
        /* HAL_BUSY 是可重试状态，不能当成硬故障让上层累计 err_bus。 */
        if (hal == HAL_BUSY)
        {
            return PYRO_BUSY;
        }
        state.error = 0x01U;
        return PYRO_ERROR;
    }

    return PYRO_OK;
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
        if (hal == HAL_BUSY)
        {
            return PYRO_BUSY;
        }
        state.error = 0x01U;
        return PYRO_ERROR;
    }

    return PYRO_OK;
}

status_t spi_drv_t::wait_dma(const uint32_t waittime)
{
    if (nullptr == _done)
    {
        return PYRO_ERROR;
    }
    /* 无在途事务时不干等：有残留完成信号就按快照错误码报告，否则直接返回。 */
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
        /* 先清 busy 再 DMAStop：Stop 内部的 Abort 可能触发完成回调，
         * 已被 busy==0 挡掉。 */
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
    /* 吞掉中止过程中可能到来的 give，避免唤醒下一次等待。 */
    if (_done != nullptr)
    {
        (void)xSemaphoreTake(_done, 0);
    }
    return PYRO_OK;
}

void spi_drv_t::set_cplt_hook(void (*hook)(void *), void *ctx)
{
    _cplt_hook = hook;
    _cplt_ctx  = ctx;
}

void spi_drv_t::notify_from_isr()
{
    if (!state.busy)
    {
        return; /* 已被超时/abort 作废，忽略迟到信号 */
    }

    /* ISR 只写字，不碰位域 RMW：错误码到 state.error 的映射留给任务侧。 */
    state.busy = 0x00U;
    _err       = HAL_SPI_GetError(_hspi);

    /* 数据已完整（BSY=0），在给信号量之前释放 CS——CS 上升沿是器件原子事务边界。 */
    if (_cplt_hook != nullptr)
    {
        _cplt_hook(_cplt_ctx);
    }

    if (_done == nullptr)
    {
        return;
    }
    BaseType_t higher_prio_task_woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(_done, &higher_prio_task_woken);
    portYIELD_FROM_ISR(higher_prio_task_woken);
}

} // namespace pyro

/* HAL 弱回调覆盖（USE_HAL_SPI_REGISTER_CALLBACKS = 0）。三条路径都收敛到
 * notify_from_isr()；错误路径也必须通知，否则等待任务要干等满超时。 */

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
