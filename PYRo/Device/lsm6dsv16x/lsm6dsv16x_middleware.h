#ifndef __LSM6DSV16X_MIDDLEWARE_H__
#define __LSM6DSV16X_MIDDLEWARE_H__
#include "spi.h"
#include "pyro_spi_drv.h"

/* 单次 SPI 事务的最大数据字节数（不含地址字节）。
 * FIFO 批读一次最多 MAX_BURST/7 = 36 字（1 字 = 1 TAG + 6 数据）。 */
constexpr uint16_t LSM6DSV16X_MAX_BURST = 256U;
constexpr uint8_t LSM6DSV16X_TIMEOUT_MS = 0xFF;

/* 单笔 DMA 事务的完成等待上限（ms），避免总线异常时拖死采集周期。 */
constexpr uint32_t LSM6DSV16X_CPLT_TIMEOUT_MS = 2U;

/* 小读上限；超过此长度改用大读双缓冲槽。 */
constexpr uint16_t LSM6DSV16X_SYNC_READ_MAX = 2U;

class lsm6dsv16x_t {
public:
    lsm6dsv16x_t() = default;

    virtual ~lsm6dsv16x_t();

    virtual bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len) = 0;
    virtual bool write_regs(uint8_t reg, uint8_t *buf, uint16_t len) = 0;

    /* FIFO 流水读取。request_len 可为 0 以仅取回完成数据；received_len 独立
     * 报告本次交付字节数。返回 false 时 received_len 仍可能非零（上一帧已交付
     * 而本次启动失败）——调用方须先处理 received_len，再处理总线状态。 */
    virtual bool read_fifo(uint8_t *buf, uint16_t capacity, uint16_t request_len,
                           uint16_t &received_len);

    /* 异步寄存器读：request → wait_dma → collect 三段，供驱动按
     * 「发起 / 让出 CPU 等通知 / 收取」组织采集。不支持时退化为同步读取。 */
    virtual bool read_regs_request(uint8_t reg, uint16_t len) {
        (void)reg;
        (void)len;
        return false;
    }
    virtual bool read_regs_collect(uint8_t *buf, uint16_t len) {
        (void)buf;
        (void)len;
        return false;
    }
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
    /* bus 是该 SPI 总线的唯一驱动实例；同总线的多颗器件共用同一引用，
     * 仅 CS 引脚不同。dma_enable 控制 FIFO 是否采用 DMA 流水。 */
    lsm6dsv16x_spi_t(pyro::spi_drv_t &bus, GPIO_TypeDef *cs_port, uint16_t cs_pin, bool dma_enable);

    bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len) override;
    bool write_regs(uint8_t reg, uint8_t *buf, uint16_t len) override;
    bool read_fifo(uint8_t *buf, uint16_t capacity, uint16_t request_len,
                   uint16_t &received_len) override;
    bool read_regs_request(uint8_t reg, uint16_t len) override;
    bool read_regs_collect(uint8_t *buf, uint16_t len) override;
    bool wait_dma(uint32_t timeout_ms) override;
private:
    pyro::spi_drv_t &_spi_drv;

    GPIO_TypeDef *_cs_port;
    uint16_t _cs_pin;
    bool _dma_enable;

    /* 双缓冲槽下标（0/1 交替）：本次 DMA 写入 _slot，另一槽保存上一帧。
     * 本类不可重入，无需保护。 */
    uint8_t _slot{};
    uint16_t _prev_len{};   /* 尚待交付的 FIFO 字节数（不含地址字节） */

    /* 小读专用暂存（含地址字节），独立于大读双槽；写事务与异步小读也借用。 */
    uint8_t _sm_tx[LSM6DSV16X_SYNC_READ_MAX + 1U]{};
    uint8_t _sm_rx[LSM6DSV16X_SYNC_READ_MAX + 1U]{};
    uint16_t _sm_len{};     /* 在途异步小读长度，0 表示无 */

    /* 发起至 wait_dma 完成期间持有总线；完成结果缓存到收取为止。 */
    bool _holding_bus{false};
    bool _result_ready{false};

    pyro::status_t bus_acquire(uint32_t timeout_ms);
    void           bus_release();

    /* 大读收发暂存 [槽][字节]，两槽互不重叠——这是一笔 DMA 在途时另一槽
     * 可安全交付上一帧的前提。_tx 为地址字节 + 全 0（MOSI 内容无关）；
     * _rx 第 0 字节是地址阶段残留，数据从 [1] 起。 */
    uint8_t _tx[2][LSM6DSV16X_MAX_BURST + 1U]{};
    uint8_t _rx[2][LSM6DSV16X_MAX_BURST + 1U]{};

    void cs_low() const;
    void cs_high() const;

    /* 拉低 CS 并发起 DMA。调用方须已持有总线（完成钩子已在那一步绑定）。 */
    pyro::status_t dma_start(const uint8_t *tx, uint8_t *rx, uint16_t size);

    static void release_cs_from_isr(void *ctx);
};


#endif
