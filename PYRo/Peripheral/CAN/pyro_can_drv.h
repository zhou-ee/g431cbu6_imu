#ifndef CAN_DRV_H
#define CAN_DRV_H

#include <array>
#include <cstdint>
#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "map.h"

namespace pyro
{
class can_msg_buffer_t
{
public:
    explicit can_msg_buffer_t(uint32_t id);
    ~can_msg_buffer_t();

    [[nodiscard]] uint32_t get_id() const;
    [[nodiscard]] bool is_fresh() const;
    void mark_read();
    void update_data(const uint8_t *data);
    [[nodiscard]] bool get_data(std::array<uint8_t, 8> &data) const;
    [[nodiscard]] TickType_t get_last_update_time() const;

private:
    uint32_t _id;
    std::array<uint8_t, 8> _buffer;
    volatile bool _is_fresh;
    volatile TickType_t _last_update_time;
};

class bsp_can;

class can_drv_t
{
    friend class bsp_can;
    const uint8_t MAX_ID_REGIST_NUM = 32;
    using can_id_regist_t           = uint16_t;

public:
    ~can_drv_t();

    status_t init() const;
    status_t start() const;

    /* 运行模式：internal loopback 用于无对端台架自测（详见 .cpp）。
     * 必须在没有在途帧的初始化阶段调用。 */
    status_t set_loopback(bool enable) const;

    /* 轮询取一帧（不依赖回调注册）。FIFO 空返回 PYRO_NOT_FOUND。 */
    status_t poll_rx(uint32_t &id, uint8_t data[8]) const;

    /* 提交一个经典 CAN 标准数据帧（固定 8 字节）。
     *
     * 契约：本函数**从不撤销已经入队的帧**。
     *   PYRO_OK    已入队
     *   PYRO_BUSY  发送队列已满（G4 固定 3 个元素）或上一帧 BusOff 刚刚恢复，
     *              本次未入队；调用方保留内容并重试。已入队的帧仍会正常发出。
     *   PYRO_ERROR 参数/状态错误
     * 因此"一组多帧"的原子性由调用方用有界队列 + 整组重试保证，
     * 不能靠本函数回滚——回滚会把同组已入队的前几帧一起丢掉。 */
    status_t send_msg(uint32_t id, const uint8_t *data) const;

    struct tx_frame_t
    {
        uint32_t id;
        const uint8_t *data; // 8 字节有效载荷
    };

    /** 在同一个临界区里预检并提交全组。容量不足时不入队任何帧。 */
    status_t send_group(const tx_frame_t *frames, uint32_t count) const;
    status_t register_rx_msg(can_msg_buffer_t *msg_buffer);
    status_t handle_rx_msg(uint32_t id, const uint8_t *data);

    static map_t<FDCAN_HandleTypeDef *, can_drv_t *> &can_map();

private:
    explicit can_drv_t(FDCAN_HandleTypeDef *hfdcan);

    FDCAN_HandleTypeDef *_hfdcan;
    map_t<uint32_t, can_msg_buffer_t *> _registerlist;
};

}

#endif
