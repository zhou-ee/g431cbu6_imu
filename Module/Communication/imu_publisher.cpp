/**
 * @file    imu_publisher.cpp
 * @brief   1 kHz 测量发布实现：取快照 → 编码 → CAN/UART 提交。
 *
 * 与融合侧的关系：本任务只读 fusion::snapshot()（值拷贝，seqlock 保证
 * 同拍完整），不触碰驱动与算法状态，因此发布抖动不会影响采集周期。
 */

#include "imu_publisher.h"

#include <cstring>

#include "FreeRTOS.h"
#include "imu_config.h"
#include "imu_fusion.h"
#include "pyro_bsp_can.h"
#include "pyro_bsp_uart.h"
#include "pyro_task.h"
#include "task.h"

/* ======================= 调试观测变量 =======================
 * 放全局作用域（理由同 imu_capture.cpp）：extern "C" 定义若留在命名空间里，
 * 调试器记的名字与 ELF 裸名字对不上。 */
extern "C" {
volatile uint32_t imu_pub_frames       = 0U;
volatile uint32_t imu_pub_can_busy     = 0U;
volatile uint32_t imu_pub_can_err      = 0U;
volatile uint32_t imu_pub_uart_busy    = 0U;
volatile uint32_t imu_pub_uart_err     = 0U;
volatile uint32_t imu_pub_snapshot_seq = 0U;
volatile uint32_t imu_pub_stack_free   = 0U;
volatile uint32_t imu_pub_can_rx       = 0U;
volatile uint32_t imu_pub_can_rx_bad   = 0U;
volatile uint32_t imu_pub_loopback     = 0U;
volatile uint8_t  imu_pub_uart_last[imu_module::protocol::kUartFrameSize] = {};
volatile uint32_t imu_pub_can_base     = 0U;
volatile uint8_t  imu_pub_can_last[imu_module::protocol::kCanFrameCount][8] = {};
}

namespace imu_module
{

namespace
{

/* ======================= 常驻发布任务 ======================= */
class pub_task_t final : public pyro::task_base_t
{
  public:
    pub_task_t()
        : task_base_t("imu_pub", 0U, config::kPublisherTaskStackWords,
                      pyro::task_base_t::priority_t::NORMAL)
    {
    }

  protected:
    pyro::status_t init() override
    {
        if (config::kPublishCan)
        {
            /* init() 内部完成位时序/滤波器配置，start() 才真正进入 Normal 模式
             * 并使能收发；两步都由 bsp_can::init_all() 代劳。 */
            const pyro::status_t ret = pyro::bsp_can::init_all();
            if (ret != pyro::PYRO_OK)
            {
                return ret;
            }

            /* 台架自测：切到 internal loopback 后硬件自答 ACK（无对端也能发出并收回），
             * 发布循环会把发出去的帧再取回来逐字节比对。正常运行必须关闭。 */
            if (config::kCanLoopbackSelfTest)
            {
                const pyro::status_t loop = pyro::bsp_can::get_can1().set_loopback(true);
                imu_pub_loopback = (loop == pyro::PYRO_OK) ? 1U : 2U;
                if (loop != pyro::PYRO_OK)
                {
                    return loop;
                }
            }
        }
        if (config::kPublishUart)
        {
            /* 触发单例构造并开启 RX DMA（本版只用 TX，但驱动要求先 enable）。 */
            pyro::bsp_uart::init_all();
        }
        return pyro::PYRO_OK;
    }

    void run_loop() override
    {
        uint32_t divisor = 0U;
        for (;;)
        {
            /* 等待"融合产出了新快照"。由 imu_module.cpp 把算法任务与本任务
             * 用同一对信号量连起来：算法任务在 fuse_once() 返回 true 时 give。
             *
             * 这样做的三点好处：
             *   ① 无竞态——被唤醒时快照刚写好，读到的必是最新一份；
             *   ② 不空转——融合若降频，发布不会抢在它前面反复编码旧数据；
             *   ③ 发出的永远是最新数据，而不是"上一拍算完、这一拍才发"。
             *
             * 未绑定信号量时（独立测试/单任务退化）退回 1 ms 轮询。 */
            if (publisher_t::instance().wake_semaphore() != nullptr)
            {
                if (xSemaphoreTake(publisher_t::instance().wake_semaphore(),
                                   portMAX_DELAY) != pdTRUE)
                {
                    continue;
                }
            }
            else
            {
                /* 退化路径：未绑定信号量时按 1 ms 轮询（独立测试用） */
                TickType_t tick = xTaskGetTickCount();
                vTaskDelayUntil(&tick, config::kPublishPeriodMs);
            }

            /* 快照是值拷贝：与采集任务零共享，读侧不会阻塞写侧。
             * 若采集尚未产出（valid=false），快照里 quat 会被协议层置零。 */
            const imu_snapshot_t snapshot = fusion::snapshot();
            const protocol::payload_t payload = protocol::encode_payload(snapshot);
            imu_pub_snapshot_seq = snapshot.sequence;

            if (config::kPublishUart)
            {
                uint8_t frame[protocol::kUartFrameSize];
                (void)protocol::encode_uart_frame(payload, frame);
                for (std::size_t i = 0U; i < protocol::kUartFrameSize; ++i)
                {
                    imu_pub_uart_last[i] = frame[i]; /* 调试镜像：实际提交的字节 */
                }
                const pyro::status_t st = pyro::bsp_uart::get_uart2().write(
                    frame, static_cast<uint16_t>(protocol::kUartFrameSize));
                if (st == pyro::PYRO_BUSY)
                {
                    ++imu_pub_uart_busy; /* 上一帧还没发完：跳过本拍，不排队 */
                }
                else if (st != pyro::PYRO_OK)
                {
                    ++imu_pub_uart_err;
                }
            }

            if (config::kPublishCan)
            {
                protocol::can_frame_t frames[protocol::kCanFrameCount];
                protocol::encode_can_frames(config::kNodeId, payload, frames);
                imu_pub_can_base = frames[0].id;
                for (std::size_t f = 0U; f < protocol::kCanFrameCount; ++f)
                {
                    for (std::size_t b = 0U; b < 8U; ++b)
                    {
                        imu_pub_can_last[f][b] = frames[f].data[b];
                    }
                }

                /* 整组按序提交：任一阵列返回忙就放弃本组剩余帧。
                 * 不重试——下一拍会带来更新的快照，重试会把旧组插到新组前面，
                 * 反而破坏接收端"按到达顺序配对"的前提。 */
                bool group_ok = true;
                for (std::size_t f = 0U; f < protocol::kCanFrameCount; ++f)
                {
                    const pyro::status_t st = pyro::bsp_can::get_can1().send_msg(
                        frames[f].id, frames[f].data);
                    if (st == pyro::PYRO_BUSY)
                    {
                        ++imu_pub_can_busy;
                        group_ok = false;
                        break;
                    }
                    if (st != pyro::PYRO_OK)
                    {
                        ++imu_pub_can_err;
                        group_ok = false;
                        break;
                    }
                }
                if (group_ok)
                {
                    ++imu_pub_frames;
                }
            }
            else
            {
                ++imu_pub_frames;
            }

            if (config::kCanLoopbackSelfTest && config::kPublishCan)
            {
                /* 回环自测：硬件已把刚发的帧送回 RX FIFO0，逐字节比对。 */
                uint32_t rx_id     = 0U;
                uint8_t  rx_data[8] = {};
                const uint32_t base = protocol::can_base_id(config::kNodeId);
                while (pyro::bsp_can::get_can1().poll_rx(rx_id, rx_data) == pyro::PYRO_OK)
                {
                    ++imu_pub_can_rx;
                    const uint32_t index = rx_id - base;
                    bool same = (index < protocol::kCanFrameCount);
                    for (std::size_t b = 0U; same && b < 8U; ++b)
                    {
                        same = (rx_data[b] == imu_pub_can_last[index][b]);
                    }
                    if (!same)
                        ++imu_pub_can_rx_bad;
                }
            }

            /* 每 1000 拍刷新一次栈余量（词）。uxTaskGetStackHighWaterMark 要遍历
             * 栈填充字，放在 1 ms 循环里做会引入抖动，因此降频到 1 Hz。 */
            if (++divisor >= 1000U)
            {
                divisor = 0U;
                imu_pub_stack_free = uxTaskGetStackHighWaterMark(nullptr);
            }

            /* 循环回到顶部等下一个信号量，不再自己延时：
             * 节拍由融合完成事件驱动，这是"发最新数据"与"无竞态"的来源。 */
        }
    }
};

pub_task_t s_pub_task;

} // namespace

publisher_t &publisher_t::instance()
{
    static publisher_t publisher;
    return publisher;
}

pyro::status_t publisher_t::init()
{
    if (_started)
    {
        return pyro::PYRO_ALREADY_INIT;
    }
    const pyro::status_t ret = s_pub_task.start();
    if (ret != pyro::PYRO_OK)
    {
        return ret;
    }
    _started = true;
    return pyro::PYRO_OK;
}

} // namespace imu_module
