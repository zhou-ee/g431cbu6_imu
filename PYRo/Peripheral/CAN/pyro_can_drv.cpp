#include "pyro_can_drv.h"
#include "main.h"
#include "pyro_platform.h"
#include <cstring>

namespace pyro
{
namespace
{
constexpr uint32_t CAN_NOTIFY_FLAGS =
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
    FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_ERROR_WARNING |
    FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF;
    /* 不开 ARB/DATA_PROTOCOL_ERROR：它们逐帧触发中断，而当前无消费者。 */

/* 撤销全部在发请求。**仅用于 BusOff 恢复**：此时发送队列已无意义，必须清空才能
 * 重新 Start。正常发送路径不得调用它——见 send_msg() 的说明。 */
void abort_pending_tx(FDCAN_HandleTypeDef *hfdcan)
{
    if (nullptr == hfdcan)
        return;

    const uint32_t pending = hfdcan->Instance->TXBRP;
    if (0U != pending)
        (void)HAL_FDCAN_AbortTxRequest(hfdcan, pending);
}

status_t reactivate_can_notifications(FDCAN_HandleTypeDef *hfdcan)
{
    if (HAL_OK != HAL_FDCAN_ActivateNotification(hfdcan, CAN_NOTIFY_FLAGS, 0))
        return PYRO_ERROR;

    return PYRO_OK;
}

status_t recover_bus_off(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_ProtocolStatusTypeDef protocol_status{};

    if (HAL_OK != HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status))
        return PYRO_ERROR;

    if (0U == protocol_status.BusOff)
        return PYRO_OK;

    abort_pending_tx(hfdcan);

    (void)HAL_FDCAN_Stop(hfdcan);
    hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;

    if (HAL_OK != HAL_FDCAN_Start(hfdcan))
        return PYRO_ERROR;

    const status_t notify_status = reactivate_can_notifications(hfdcan);
    if (PYRO_OK != notify_status)
        return notify_status;

    return PYRO_BUSY;
}
} // namespace

// ==========================================
// can_msg_buffer_t Implementation
// ==========================================
can_msg_buffer_t::can_msg_buffer_t(const uint32_t id)
    : _id(id), _buffer{}, _is_fresh(false), _last_update_time(0)
{
}

can_msg_buffer_t::~can_msg_buffer_t() = default;

uint32_t can_msg_buffer_t::get_id() const
{
    return _id;
}

bool can_msg_buffer_t::is_fresh() const
{
    return _is_fresh;
}

void can_msg_buffer_t::mark_read()
{
    _is_fresh = false;
}

TickType_t can_msg_buffer_t::get_last_update_time() const
{
    return _last_update_time;
}

PYRO_ISR_TEXT void
can_msg_buffer_t::update_data(const uint8_t *data)
{
    const UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    memcpy(_buffer.data(), data, 8);
    _last_update_time = xTaskGetTickCountFromISR();
    _is_fresh         = true;

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

bool can_msg_buffer_t::get_data(std::array<uint8_t, 8> &data) const
{
    taskENTER_CRITICAL();
    memcpy(data.data(), _buffer.data(), 8);
    const bool fresh_status = _is_fresh;
    taskEXIT_CRITICAL();

    return fresh_status;
}

// ==========================================
// can_drv_t Implementation
// ==========================================
can_drv_t::can_drv_t(FDCAN_HandleTypeDef *hfdcan) : _hfdcan(hfdcan)
{
    _registerlist.clear();
    can_map()[hfdcan] = this;
}

can_drv_t::~can_drv_t()
{
    can_map().erase(_hfdcan);
}

map_t<FDCAN_HandleTypeDef *, can_drv_t *> &can_drv_t::can_map()
{
    static map_t<FDCAN_HandleTypeDef *, can_drv_t *> instance;
    return instance;
}

pyro::status_t can_drv_t::init() const
{
    /* 移植补充：CubeMX 未给 FDCAN 分配任何滤波器（StdFiltersNbr = ExtFiltersNbr = 0）时，
     * HAL_FDCAN_ConfigFilter() 里的 assert_param 因未开 USE_FULL_ASSERT 而静默通过，
     * 但硬件侧 RXGFC.LSS = 0 —— 没有任何滤波器生效；再叠加下面的全局滤波 REJECT，
     * 结果是“CAN 一帧都收不到却查不出错”。这里把静默失效变成显式错误。 */
    if (nullptr == _hfdcan || (0U == _hfdcan->Init.StdFiltersNbr &&
                               0U == _hfdcan->Init.ExtFiltersNbr))
        return pyro::PYRO_PARAM_ERROR;

    FDCAN_FilterTypeDef fdcan_filter;
    fdcan_filter.IdType       = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex  = 0;
    fdcan_filter.FilterType   = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1    = 0x00;
    fdcan_filter.FilterID2    = 0x00;

    if (HAL_OK != HAL_FDCAN_ConfigFilter(_hfdcan, &fdcan_filter))
        return pyro::PYRO_ERROR;
    if (HAL_OK !=
        HAL_FDCAN_ConfigGlobalFilter(_hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE))
        return pyro::PYRO_ERROR;
#if PYRO_CAN_HAS_FIFO_WATERMARK
    /* 仅 H7 HAL 提供该 API；本工程(G4) FIFO 元素数由 CubeMX 的 HAL_FDCAN_Init 配置决定 */
    if (HAL_OK != HAL_FDCAN_ConfigFifoWatermark(_hfdcan, FDCAN_CFG_RX_FIFO0, 1))
        return pyro::PYRO_ERROR;
#endif

    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::start() const
{
    if (HAL_OK != HAL_FDCAN_Start(_hfdcan))
        return pyro::PYRO_ERROR;

    return reactivate_can_notifications(_hfdcan);
}

/* 运行模式切换：internal loopback（TEST.LBCK）把 Tx 送回 Rx，硬件自答 ACK，
 * 用于无对端的台架验证"编码 -> 驱动 -> 硬件 -> 接收"整条链路。
 * 只能在没有在途帧的初始化阶段调用：HAL_FDCAN_Init 会重配消息 RAM/过滤器/通知，
 * 因此这里直接复用 init()/start()。正常运行必须保持关闭。 */
pyro::status_t can_drv_t::set_loopback(const bool enable) const
{
    if (nullptr == _hfdcan)
        return pyro::PYRO_PARAM_ERROR;

    _hfdcan->Init.Mode = enable ? FDCAN_MODE_INTERNAL_LOOPBACK : FDCAN_MODE_NORMAL;
    (void)HAL_FDCAN_Stop(_hfdcan);
    const pyro::status_t init_status = init();
    if (pyro::PYRO_OK != init_status)
        return init_status;
    return start();
}

/* 轮询取一帧，不经过回调注册表。FIFO 空返回 PYRO_NOT_FOUND。
 * 只有 init() 配置过过滤器后才会收到帧（过滤器为全通：ID 0 / 掩码 0）。 */
pyro::status_t can_drv_t::poll_rx(uint32_t &id, uint8_t data[8]) const
{
    if (nullptr == _hfdcan || nullptr == data)
        return pyro::PYRO_PARAM_ERROR;

    if (0U == HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, FDCAN_RX_FIFO0))
        return pyro::PYRO_NOT_FOUND;

    FDCAN_RxHeaderTypeDef header{};
    if (HAL_OK != HAL_FDCAN_GetRxMessage(_hfdcan, FDCAN_RX_FIFO0, &header, data))
        return pyro::PYRO_ERROR;

    id = header.Identifier;
    return pyro::PYRO_OK;
}

pyro::status_t can_drv_t::send_msg(const uint32_t id, const uint8_t *data) const
{
    const tx_frame_t frame{id, data};
    return send_group(&frame, 1U);
}

status_t can_drv_t::send_group(const tx_frame_t *frames, uint32_t count) const
{
    if (_hfdcan == nullptr || frames == nullptr || count == 0U)
        return PYRO_PARAM_ERROR;
    for (uint32_t i = 0U; i < count; ++i)
    {
        if (frames[i].data == nullptr || frames[i].id > 0x7FFU)
            return PYRO_PARAM_ERROR;
    }

    if (_hfdcan->State != HAL_FDCAN_STATE_BUSY)
        return PYRO_BUSY;
    const status_t recover_status = recover_bus_off(_hfdcan);
    if (recover_status != PYRO_OK)
        return recover_status;

    FDCAN_TxHeaderTypeDef header{};
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    // 持锁期间只有本任务能占用发送槽，控制器发送完成只会增加空槽。
    taskENTER_CRITICAL();
    status_t result = PYRO_BUSY;
    if (HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) >= count)
    {
        result = PYRO_OK;
        for (uint32_t i = 0U; i < count; ++i)
        {
            header.Identifier = frames[i].id;
            if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &header, frames[i].data) != HAL_OK)
            {
                // 预检后硬件仍可故障。这里报告错误，并保留已交给控制器的帧。
                result = PYRO_ERROR;
                break;
            }
        }
    }
    taskEXIT_CRITICAL();
    return result;
}

pyro::status_t can_drv_t::register_rx_msg(can_msg_buffer_t *msg_buffer)
{
    const uint32_t id = msg_buffer->get_id();

    taskENTER_CRITICAL();
    if (this->_registerlist.exist(id))
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_ERROR;
    }
    /* 移植补充：map_t 容量有限（_max_size，默认 10），上游没有容量判断，
     * 超限后会越界写。这里显式报告内存不足而不是静默损坏内存。 */
    if (this->_registerlist.full())
    {
        taskEXIT_CRITICAL();
        return pyro::PYRO_NO_MEMORY;
    }
    this->_registerlist[id] = msg_buffer;
    taskEXIT_CRITICAL();

    return pyro::PYRO_OK;
}

PYRO_ISR_TEXT pyro::status_t
can_drv_t::handle_rx_msg(const uint32_t id, const uint8_t *data)
{
    if (!this->_registerlist.exist(id))
    {
        return pyro::PYRO_NOT_FOUND;
    }

    can_msg_buffer_t *msg = this->_registerlist[id];
    msg->update_data(data);

    return pyro::PYRO_OK;
}

}; // namespace pyro

// ==========================================
// Global Hardware Interruption Callbacks
// ==========================================
PYRO_ISR_TEXT void
can_global_handle(FDCAN_HandleTypeDef *hfdcan, const uint32_t identifier,
                  const uint8_t *data)
{
    auto &map = pyro::can_drv_t::can_map();
    if (map.exist(hfdcan))
    {
        (void)map[hfdcan]->handle_rx_msg(identifier, data);
    }
}

extern "C" PYRO_ISR_TEXT void
HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    if (HAL_OK !=
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data))
    {
        return;
    }

    if (FDCAN_FRAME_CLASSIC == rx_header.RxFrameType &&
        FDCAN_STANDARD_ID == rx_header.IdType)
    {
        can_global_handle(hfdcan, rx_header.Identifier, data);
    }
}
