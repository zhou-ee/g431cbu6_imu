/**
 * @file    imu_publisher.h
 * @brief   1 kHz 测量发布：快照 → 协议编码 → CAN / UART 传输。
 *
 * 传输策略（对应 PROTOCOL.md「发送与重组」）：
 *   · CAN 一组三帧按 base_id+0/1/2 顺序提交，同组保持顺序；
 *   · G4 的发送 FIFO 固定 3 个元素，一组恰好占满。驱动在队列满时只返回 PYRO_BUSY、
 *     不撤销已入队的帧；本层计一次 busy 并放弃该组剩余帧——周期遥测优先保留最新
 *     快照，不做积压重排（否则接收端的到达顺序配对会错位）。
 *   · UART 用非阻塞 DMA 写，上一帧未发完就跳过本拍并计数。
 * 所有丢弃路径都有计数器，可从调试器观察；`imu_pub_uart_last`/`imu_pub_can_last`
 * 保存最近一次真正提交的字节，便于逐字节核对协议。
 */

#ifndef IMU_MODULE_COMMUNICATION_IMU_PUBLISHER_H
#define IMU_MODULE_COMMUNICATION_IMU_PUBLISHER_H

#include <cstdint>

#include "FreeRTOS.h"
#include "imu_protocol.h"
#include "pyro_core_def.h"
#include "semphr.h"

namespace imu_module
{

class publisher_t
{
  public:
    static publisher_t &instance();

    /** 绑定"有新快照"的唤醒信号量（由 imu_module.cpp 创建并传入）。
     *  必须在 init() 之前调用；未绑定时发布任务退化为 1 ms 轮询。 */
    void set_wake_semaphore(SemaphoreHandle_t sem) { _wake = sem; }

    /** 当前绑定的唤醒信号量（发布任务循环内读取；nullptr 表示退化为轮询）。 */
    [[nodiscard]] SemaphoreHandle_t wake_semaphore() const { return _wake; }

    /** 初始化传输层并创建常驻发布任务。仅可在调度器启动后的任务上下文调用。
     *  @retval PYRO_OK 成功；PYRO_ERROR 总线初始化/任务创建失败 */
    pyro::status_t init();

    [[nodiscard]] bool started() const { return _started; }

  private:
    publisher_t() = default;
    friend class pub_task_t;
    bool              _started = false;
    SemaphoreHandle_t _wake    = nullptr;  /**< 有新快照时由融合任务 give */
};

} // namespace imu_module
/* ============================ 调试观测 ============================ */
extern "C" {
extern volatile uint32_t imu_pub_frames;       /* 已提交的测量组数 */
extern volatile uint32_t imu_pub_can_busy;     /* CAN 队列满、放弃本组剩余帧的次数 */
extern volatile uint32_t imu_pub_can_err;      /* CAN 发送错误次数 */
extern volatile uint32_t imu_pub_uart_busy;    /* UART 忙、跳过本拍的次数 */
extern volatile uint32_t imu_pub_uart_err;     /* UART 发送错误次数 */
extern volatile uint32_t imu_pub_snapshot_seq; /* 最近一次发布的快照序号 */
extern volatile uint32_t imu_pub_stack_free;   /* 发布任务栈余量（词，1 s 刷新） */
extern volatile uint32_t imu_pub_can_rx;       /* 回环自测取回的帧数 */
extern volatile uint32_t imu_pub_can_rx_bad;   /* 回环自测里内容/ID 不匹配的帧数 */
extern volatile uint32_t imu_pub_loopback;     /* 0=正常 1=回环已启用 2=回环设置失败 */
extern volatile uint8_t  imu_pub_uart_last[imu_module::protocol::kUartFrameSize]; /* 最近发往 UART 的帧 */
extern volatile uint32_t imu_pub_can_base;     /* 最近一组 CAN 的基址 */
extern volatile uint8_t  imu_pub_can_last[imu_module::protocol::kCanFrameCount][8]; /* 最近三帧内容 */
}


#endif /* IMU_MODULE_COMMUNICATION_IMU_PUBLISHER_H */
