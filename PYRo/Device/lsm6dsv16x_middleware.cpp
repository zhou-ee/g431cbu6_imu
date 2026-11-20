#include "lsm6dsv16x_middleware.h"
#include "lsm6dsv16x_reg.h"

/* 虚析构必须有定义,否则 vtable 无法生成(链接期 undefined reference) */
lsm6dsv16x_t::~lsm6dsv16x_t() = default;

/* ---------------------------------------------------------------------------
 * 异步寄存器读（request → wait_dma → collect）：
 * 借用 _sm_tx/_sm_rx，与小读共用同一块暂存——同一时刻只允许一笔小读在途，
 * 由 _sm_len 标记。大读（双槽 _tx[_slot]/_rx[_slot]）与之完全隔离。
 * ------------------------------------------------------------------------- */

bool lsm6dsv16x_spi_t::read_regs_request(const uint8_t reg, const uint16_t len) {
    if (!_dma_enable || len == 0U || len > LSM6DSV16X_SYNC_READ_MAX ||
        _sm_len != 0U || _spi_drv.dma_busy()) {
        return false; /* 不支持异步 / 参数非法 / 已有事务在途 */
    }

    _sm_tx[0] = static_cast<uint8_t>(reg | LSM6DSV16X::SPI_READ_BIT);
    for (uint16_t i = 1U; i <= len; ++i) {
        _sm_tx[i] = 0x00U;
    }

    /* dma_start 内部负责：绑定完成钩子 → 拉低 CS → 发起 */
    if (dma_start(_sm_tx, _sm_rx, static_cast<uint16_t>(len + 1U)) != pyro::PYRO_OK) {
        cs_high(); /* 未成立的事务：任务侧释放 CS（DMA 路径无完成回调） */
        return false;
    }
    _sm_len = len;
    return true;
}

bool lsm6dsv16x_spi_t::read_regs_collect(uint8_t *buf, const uint16_t len) {
    if (len == 0U || len > LSM6DSV16X_SYNC_READ_MAX || _sm_len != len || !buf) {
        return false; /* 无在途事务或长度与发起时不一致 */
    }
    if (_spi_drv.dma_busy()) {
        return false; /* 事务未完，调用方应先 wait_dma() 成功后再来收取 */
    }
    if (_spi_drv.wait_dma(0U) != pyro::PYRO_OK) {
        _sm_len = 0U; /* 错误帧作废，避免下一次被误当成功交付 */
        return false;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        buf[i] = _sm_rx[i + 1U]; /* 第 0 字节是地址阶段的 MISO 残留 */
    }
    _sm_len = 0U;
    return true;
}

/* 让出 CPU 等待在途事务的完成通知（ISR 中给信号量并由 portYIELD_FROM_ISR 唤醒）。
 * 超时/错误时：wait_dma 已中止 DMA 且把 state.busy 清零，notify_from_isr 会因
 * busy==0 提前返回、不再调用完成钩子，因此 CS 必须由任务侧在这里释放；
 * 否则 CS 一直为低，后续每一帧都会失效。 */
bool lsm6dsv16x_spi_t::wait_dma(const uint32_t timeout_ms) {
    if (_spi_drv.wait_dma(timeout_ms) == pyro::PYRO_OK) {
        return true;
    }
    (void)_spi_drv.abort();
    cs_high();
    _sm_len  = 0U;
    _prev_len = 0U; /* 在途大读一并作废，下次调用可重新发起 */
    return false;
}

/* 普通寄存器同步读写。FIFO 单独维护请求长度与完整交付长度，
 * 已完成帧只交付一次；即使下一笔启动失败，已交付帧仍由调用方处理。
 * 小读使用独立暂存，普通大读使用当前空闲槽，FIFO DMA 使用双槽轮转。 */

bool lsm6dsv16x_t::read_fifo(uint8_t *buf, const uint16_t capacity,
                            const uint16_t request_len, uint16_t &received_len) {
    received_len = 0U;
    if (request_len == 0U) {
        return true;
    }
    if (!buf || request_len > capacity || request_len > LSM6DSV16X_MAX_BURST) {
        return false;
    }
    if (!read_regs(LSM6DSV16X::FIFO_DATA_OUT_TAG, buf, request_len)) {
        return false;
    }
    received_len = request_len;
    return true;
}

bool lsm6dsv16x_spi_t::read_regs(const uint8_t reg, uint8_t *buf, const uint16_t len) {
    if (!buf || len == 0U || len > LSM6DSV16X_MAX_BURST || _spi_drv.dma_busy()) {
        return false;
    }

    uint8_t *tx = (len <= LSM6DSV16X_SYNC_READ_MAX) ? _sm_tx : _tx[_slot];
    uint8_t *rx = (len <= LSM6DSV16X_SYNC_READ_MAX) ? _sm_rx : _rx[_slot];
    tx[0] = static_cast<uint8_t>(reg | LSM6DSV16X::SPI_READ_BIT);
    for (uint16_t i = 1U; i <= len; ++i) {
        tx[i] = 0x00U;
    }

    cs_low();
    const auto result = _spi_drv.txrx(tx, rx, static_cast<uint16_t>(len + 1U),
                                     LSM6DSV16X_TIMEOUT_MS);
    cs_high();
    if (result != pyro::PYRO_OK) {
        return false;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        buf[i] = rx[i + 1U];
    }
    return true;
}

bool lsm6dsv16x_spi_t::read_fifo(uint8_t *buf, const uint16_t capacity,
                                const uint16_t request_len, uint16_t &received_len) {
    received_len = 0U;
    if (!_dma_enable) {
        return lsm6dsv16x_t::read_fifo(buf, capacity, request_len, received_len);
    }
    if (request_len > LSM6DSV16X_MAX_BURST || request_len > capacity ||
        _prev_len > capacity || ((request_len > 0U || _prev_len > 0U) && !buf)) {
        return false;
    }
    if (_spi_drv.dma_busy()) {
        return true; // 在途数据由后续调用取回，保持缓冲与片选状态。
    }

    if (_prev_len > 0U) {
        if (_spi_drv.wait_dma(0U) != pyro::PYRO_OK) {
            _prev_len = 0U; // DMA 错误帧作废并报告，下一次可重新发起。
            return false;
        }
        const uint8_t *prev = _rx[_slot ^ 1U];
        for (uint16_t i = 0U; i < _prev_len; ++i) {
            buf[i] = prev[i + 1U];
        }
        received_len = _prev_len;
        _prev_len = 0U; // 先确认已交付，防止下一笔启动失败后重复交付。
    }
    if (request_len == 0U) {
        return true;
    }

    uint8_t *tx = _tx[_slot];
    uint8_t *rx = _rx[_slot];
    tx[0] = static_cast<uint8_t>(LSM6DSV16X::FIFO_DATA_OUT_TAG |
                                  LSM6DSV16X::SPI_READ_BIT);
    for (uint16_t i = 1U; i <= request_len; ++i) {
        tx[i] = 0x00U;
    }
    if (dma_start(tx, rx, static_cast<uint16_t>(request_len + 1U)) != pyro::PYRO_OK) {
        cs_high();
        return false; // received_len 仍报告本次已经交付的完整帧。
    }
    _slot ^= 1U;
    _prev_len = request_len;
    return true;
}

/* 写事务：**始终阻塞**（无论 _dma_enable）。
 * 理由：写只发生在配置期（init 连续写十几个寄存器），单笔 1~2 字节；
 * 若走 tx_dma，连续两次调用只隔几微秒，第二笔必然拿到 PYRO_BUSY 而中断初始化，
 * 且 tx 缓冲在传输完成前不能复用。阻塞写最简单可靠。
 * 缓冲复用 _sm_tx（写长度 ≤ LSM6DSV16X_SYNC_READ_MAX，与大读双缓冲完全隔离）。 */
bool lsm6dsv16x_spi_t::write_regs(const uint8_t reg, uint8_t *buf, const uint16_t len) {
    if (!buf || len == 0U || len > LSM6DSV16X_SYNC_READ_MAX || _spi_drv.dma_busy()) {
        return false;
    }

    uint8_t *tx = _sm_tx;
    tx[0]       = static_cast<uint8_t>(reg & LSM6DSV16X::SPI_WRITE_MASK); /* bit7 = 0 -> 写 */
    for (uint16_t i = 0U; i < len; i++) {
        tx[i + 1U] = buf[i];
    }

    cs_low();
    const bool ok =
        (_spi_drv.tx(tx, static_cast<uint16_t>(len + 1U), LSM6DSV16X_TIMEOUT_MS) == pyro::PYRO_OK);
    cs_high(); /* 阻塞路径无完成回调，任务侧释放 */

    return ok;
}

lsm6dsv16x_spi_t::lsm6dsv16x_spi_t(pyro::spi_drv_t &bus, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     const bool dma_enable)
    : _spi_drv(bus), _cs_port(cs_port), _cs_pin(cs_pin), _dma_enable(dma_enable) {
    /* 这里【不】注册完成钩子：钩子在总线驱动上是单槽的，同一总线可能挂多颗
     * 器件，构造期注册会被后构造者覆盖。改由 dma_start() 在每笔事务发起前
     * 绑定到自身（见下），这才是钩子应有的生命周期粒度。 */
}

/* 发起一笔 DMA 事务：绑定完成钩子 → 拉低 CS → 发起。
 * 顺序不可换：
 *   - 钩子必须在 txrx_dma 之前设好，因为完成中断可能在 HAL 返回前就触发；
 *   - CS 必须在 txrx_dma 之前拉低，否则地址字节会被别的从机"听"走。 */
pyro::status_t lsm6dsv16x_spi_t::dma_start(const uint8_t *tx, uint8_t *rx, const uint16_t size) {
    _spi_drv.set_cplt_hook(&lsm6dsv16x_spi_t::release_cs_from_isr, this);
    cs_low();
    return _spi_drv.txrx_dma(tx, rx, size);
}

/* DMA 完成/错误时由 spi_drv_t 在 ISR 上下文调用：数据已完整（BSY=0），
 * 立即释放 CS。写 BSRR 为原子操作，与任务侧的可能重复释放（错误路径）
 * 语义幂等，无副作用。 */
void lsm6dsv16x_spi_t::release_cs_from_isr(void *ctx) {
    static_cast<lsm6dsv16x_spi_t *>(ctx)->cs_high();
}

void lsm6dsv16x_spi_t::cs_low() const {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
}

void lsm6dsv16x_spi_t::cs_high() const {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
}
