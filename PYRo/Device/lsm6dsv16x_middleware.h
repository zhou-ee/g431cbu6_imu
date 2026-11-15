#ifndef __LSM6DSV16X_MIDDLEWARE_H__
#define __LSM6DSV16X_MIDDLEWARE_H__
#include "spi.h"
#include "pyro_spi_drv.h"

/* 单次 SPI 事务的最大数据字节数（不含地址字节）。
 * 普通寄存器读 1~2 字节；FIFO 经 read_fifo 批读：一次最多 MAX_BURST/7 = 36 个字
 * （1 字 = 1 TAG + 6 数据 = 7 字节，252 字节）。
 * 7.68 kHz 下 DIFF_FIFO 每毫秒约 +15 字，一次事务通常即够；
 * 更多时分多次事务读（每次 CS 独立拉低/拉高，FIFO 读指针连续推进，不会错位）。 */
constexpr uint16_t LSM6DSV16X_MAX_BURST = 256U;
constexpr uint8_t LSM6DSV16X_TIMEOUT_MS = 0xFF;

/* 单笔 DMA 事务的完成等待上限（ms）。驱动 read() 依赖它把"等通知"限时，
 * 避免总线异常时把 1 ms 采集周期拖死。正常路径远小于此值：
 * 状态帧 3 B ≈ 5 µs，满帧 252 B ≈ 380 µs（SPI 5.3125 Mbit/s）。 */
constexpr uint32_t LSM6DSV16X_CPLT_TIMEOUT_MS = 2U;

/* 普通寄存器小读使用独立缓冲；FIFO DMA 使用 read_fifo() 管理。 */

constexpr uint16_t LSM6DSV16X_SYNC_READ_MAX = 2U;

class lsm6dsv16x_t {
public:
    lsm6dsv16x_t() = default;

    virtual ~lsm6dsv16x_t();

    /* 普通寄存器读写同步完成，成功时读缓冲已填好。 */
    virtual bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len) = 0;
    virtual bool write_regs(uint8_t reg, uint8_t *buf, uint16_t len) = 0;
    /* FIFO 流水读取。request_len 为本次请求字节数，可为 0 以仅取回完成数据；
     * received_len 独立报告本次交付字节数，容量须覆盖请求和待交付的完整帧。
     * 返回 false 时 received_len 仍可能非零（上一帧已交付，本次启动失败）。
     * 默认实现同步读取；DMA 实现交付已完成帧后发起下一笔，忙时返回 true/0。
     * 调用方始终先处理 received_len，再处理返回的总线状态。 */
    virtual bool read_fifo(uint8_t *buf, uint16_t capacity, uint16_t request_len,
                           uint16_t &received_len);

    /* ---------------- 异步寄存器读（与 read_fifo 同套流水语义） ----------------
     * request → wait_dma → collect 三段，供驱动按"发起 / 让出 CPU 等通知 / 收取"
     * 组织一轮采集。不支持异步的实现返回 false / true，由驱动退化同步读取。 */

    /* 发起一次寄存器读：成功时事务已在途，须用 wait_dma() 等完成再 collect。
     * false = 不支持异步 / 已有事务在途 / 启动失败。 */
    virtual bool read_regs_request(uint8_t reg, uint16_t len) {
        (void)reg;
        (void)len;
        return false;
    }
    /* 收取 read_regs_request 已完成的字节：仅在 wait_dma() 成功后调用。
     * false = 无在途事务或长度不符（缓冲不变）。 */
    virtual bool read_regs_collect(uint8_t *buf, uint16_t len) {
        (void)buf;
        (void)len;
        return false;
    }
    /* 让出 CPU 等待在途事务的完成通知（DMA 完成中断在 ISR 中给出信号）。
     * true = 已完成；false = 超时或总线错误，在途状态已被清理。
     * 同步实现无事可等，直接返回 true。 */
    virtual bool wait_dma(uint32_t timeout_ms) {
        (void)timeout_ms;
        return true;
    }

    bool read_reg(const uint8_t reg, uint8_t *buf) {
        return read_regs(reg, buf, 1);
    }
    bool write_reg(const uint8_t reg, uint8_t val) {
        return write_regs(reg, &val, 1);
    }
};

class lsm6dsv16x_spi_t : public lsm6dsv16x_t {
public:
    /* dma_enable 控制 FIFO 是否采用 DMA 流水，普通寄存器读写同步完成。 */
    lsm6dsv16x_spi_t(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin, bool dma_enable);

    bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len) override;
    bool write_regs(uint8_t reg, uint8_t *buf, uint16_t len) override;
    bool read_fifo(uint8_t *buf, uint16_t capacity, uint16_t request_len,
                   uint16_t &received_len) override;
    bool read_regs_request(uint8_t reg, uint16_t len) override;
    bool read_regs_collect(uint8_t *buf, uint16_t len) override;
    bool wait_dma(uint32_t timeout_ms) override;
private:
    SPI_HandleTypeDef *_spi_handle;
    GPIO_TypeDef *_cs_port;
    uint16_t _cs_pin;

    /* FIFO DMA 开关（构造时指定）。 */
    bool _dma_enable;

    /* 大读双缓冲槽下标（0/1 交替）：本次 DMA 写入 _slot，另一槽 (_slot^1) 里
     * 保存着上一次大读的结果 = 上一帧，交付完才轮到它做下一笔的目标。
     * 单个下标即可表达两槽含义，无需再记"上一个槽"。
     * 注意：本类不可重入（单总线串行访问），无需保护。 */
    uint8_t _slot{};

    /* 尚待交付的 FIFO DMA 字节数（不含地址字节），完整交付后清零。 */
    uint16_t _prev_len{};

    /* 小读（≤ LSM6DSV16X_SYNC_READ_MAX 字节）专用暂存：容量刚好装下地址字节 + 数据。
     * 独立于 _tx/_rx，保证阻塞小读不会踩到在途的大读 DMA（小读每周期都发生，
     * 若共用大读的双缓冲会立刻打乱其交替次序）。写事务也用这块。
     * 异步小读（read_regs_request/collect）同样借用这块：同一时刻只有一笔小读在途。 */
    uint8_t _sm_tx[LSM6DSV16X_SYNC_READ_MAX + 1U]{};
    uint8_t _sm_rx[LSM6DSV16X_SYNC_READ_MAX + 1U]{};

    /* 在途异步小读的数据长度（不含地址字节），0 表示没有在途小读。 */
    uint16_t _sm_len{};

    /* SPI 收发引擎（PYRo/Peripheral/SPI，仿 UART 驱动：阻塞 + DMA 双 API）：
     * 小读/写走 txrx/tx（阻塞），大读走 txrx_dma（发起即返回）。
     * HAL 完成/错误回调经 spi_map() 句柄表路由到本对象。 */
    pyro::spi_drv_t _spi_drv;

    /* 大读的收发暂存（随对象静态生命周期，不进调用者栈，支撑 FIFO 大批量读）。
     * 维度是 [槽][字节]：外层 2 个槽，内层每槽 LSM6DSV16X_MAX_BURST+1 字节，
     * 两槽在内存里彼此独立、互不重叠——这是"一笔 DMA 在途时另一槽可安全交付
     * 上一帧"的前提（原来的 [MAX+1][2] 实际是 257 个 2 字节槽，
     * 写满 len+1 字节会越界踩到相邻成员）。
     * 槽内容：
     *   _tx[slot]：地址字节 + 全 0（读阶段 MOSI 内容无关，MISO 有效）；
     *   _rx[slot]：第 0 字节是地址阶段的 MISO 残留，数据从 [1] 起。
     * 注意：本类不可重入——本器件单总线 + 软 CS，只允许任务上下文串行访问。 */
    uint8_t _tx[2][LSM6DSV16X_MAX_BURST + 1U]{};
    uint8_t _rx[2][LSM6DSV16X_MAX_BURST + 1U]{};

    void cs_low() const;

    void cs_high() const;

    /* DMA 完成钩子（ISR 上下文）：在数据搬运完成点释放 CS，
     * 而非等任务唤醒——CS 低电平尾部最短，且时序不依赖阻塞等待前提。
     * 经 spi_drv_t::set_cplt_hook 注册，完成/错误时都会调用。 */
    static void release_cs_from_isr(void *ctx);
};


#endif
