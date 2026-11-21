#include "lsm6dsv16x_middleware.h"
#include "lsm6dsv16x_reg.h"

/* 虚析构必须有定义，否则 vtable 无法生成（链接期 undefined reference）。 */
lsm6dsv16x_t::~lsm6dsv16x_t() = default;

/* 总线锁幂等封装。完成钩子的绑定与解绑集中在这里：
 * 绑定在 acquire_bus 之后，解绑在 release_bus 之前——钩子单槽且被多从机
 * 共享，只有与总线所有权对齐才能保证不串台、也不残留。 */

pyro::status_t lsm6dsv16x_spi_t::bus_acquire(const uint32_t timeout_ms) {
    if (_holding_bus) {
        return pyro::PYRO_OK; /* 幂等 */
    }
    if (_spi_drv.acquire_bus(timeout_ms) != pyro::PYRO_OK) {
        return pyro::PYRO_BUSY;
    }
    _holding_bus = true;
    _spi_drv.set_cplt_hook(&lsm6dsv16x_spi_t::release_cs_from_isr, this);
    return pyro::PYRO_OK;
}

void lsm6dsv16x_spi_t::bus_release() {
    if (!_holding_bus) {
        return; /* 未持有：no-op，避免重复 give */
    }
    /* 先解绑再放锁：总线交出去时钩子必须已清空，否则迟到中断会误调本设备的
     * CS 释放函数。这一步同时覆盖发起失败与正常结束两条路径。 */
    _spi_drv.set_cplt_hook(nullptr, nullptr);
    _holding_bus = false;
    (void)_spi_drv.release_bus();
}

bool lsm6dsv16x_spi_t::read_regs_request(const uint8_t reg, const uint16_t len) {
    if (!_dma_enable || len == 0U || len > LSM6DSV16X_SYNC_READ_MAX || _sm_len != 0U || _prev_len != 0U) {
        return false; /* 不支持异步 / 参数非法 / 本对象已有小读在途 */
    }
    if (bus_acquire(LSM6DSV16X_CPLT_TIMEOUT_MS) != pyro::PYRO_OK) {
        return false;
    }

    _sm_tx[0] = static_cast<uint8_t>(reg | LSM6DSV16X::SPI_READ_BIT);
    for (uint16_t i = 1U; i <= len; ++i) {
        _sm_tx[i] = 0x00U;
    }

    if (dma_start(_sm_tx, _sm_rx, static_cast<uint16_t>(len + 1U)) != pyro::PYRO_OK) {
        cs_high();
        bus_release();
        return false;
    }
    _sm_len = len;
    return true;
}

bool lsm6dsv16x_spi_t::read_regs_collect(uint8_t *buf, const uint16_t len) {
    if (len == 0U || len > LSM6DSV16X_SYNC_READ_MAX || _sm_len != len || !buf) {
        return false; /* 无在途事务或长度与发起时不一致 */
    }
    if (!_result_ready) {
        return false;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        buf[i] = _sm_rx[i + 1U]; /* 第 0 字节是地址阶段的 MISO 残留 */
    }
    _sm_len = 0U;
    _result_ready = false;
    return true;
}

/* 结果缓存在本设备对象内，收取阶段不再访问共享 DMA 状态。 */
bool lsm6dsv16x_spi_t::wait_dma(const uint32_t timeout_ms) {
    if (!_dma_enable) {
        return true;
    }
    if (!_holding_bus) {
        return _result_ready;
    }
    if (_spi_drv.wait_dma(timeout_ms) == pyro::PYRO_OK) {
        _result_ready = true;
        bus_release();
        return true;
    }
    /* 超时时 ISR 不会再调完成钩子，CS 必须由任务侧释放。 */
    (void)_spi_drv.abort();
    cs_high();
    _result_ready = false;
    _sm_len   = 0U;
    _prev_len = 0U;
    bus_release();
    return false;
}

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
    if (!buf || len == 0U || len > LSM6DSV16X_MAX_BURST || _sm_len != 0U || _prev_len != 0U) {
        return false;
    }
    /* 阻塞路径同样持锁：HAL 轮询期间 SCK 一直被本器件占用。 */
    if (bus_acquire(LSM6DSV16X_TIMEOUT_MS) != pyro::PYRO_OK) {
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
    bus_release();
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
    if (_sm_len != 0U || request_len > LSM6DSV16X_MAX_BURST || request_len > capacity ||
        _prev_len > capacity || ((request_len > 0U || _prev_len > 0U) && !buf)) {
        return false;
    }

    if (_prev_len > 0U) {
        if (!_result_ready) {
            return false;
        }
        const uint8_t *prev = _rx[_slot ^ 1U];
        for (uint16_t i = 0U; i < _prev_len; ++i) {
            buf[i] = prev[i + 1U];
        }
        received_len = _prev_len;
        _prev_len = 0U; // 先确认已交付，防止下一笔启动失败后重复交付。
        _result_ready = false;
    }
    if (request_len == 0U) {
        return true;
    }
    if (bus_acquire(LSM6DSV16X_CPLT_TIMEOUT_MS) != pyro::PYRO_OK) {
        return false;
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
        bus_release(); // 发起失败即收尾，锁不能留给调用方
        return false;  // received_len 仍报告本次已经交付的完整帧。
    }
    _slot ^= 1U;
    _prev_len = request_len;
    return true;
}

/* 写事务始终阻塞：只发生在配置期，单笔 1~2 字节，走 DMA 会因连续调用而
 * 拿到 PYRO_BUSY。缓冲复用 _sm_tx。 */
bool lsm6dsv16x_spi_t::write_regs(const uint8_t reg, uint8_t *buf, const uint16_t len) {
    if (!buf || len == 0U || len > LSM6DSV16X_SYNC_READ_MAX || _sm_len != 0U || _prev_len != 0U) {
        return false;
    }
    if (bus_acquire(LSM6DSV16X_TIMEOUT_MS) != pyro::PYRO_OK) {
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
    cs_high();
    bus_release();

    return ok;
}

lsm6dsv16x_spi_t::lsm6dsv16x_spi_t(pyro::spi_drv_t &bus, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     const bool dma_enable)
    : _spi_drv(bus), _cs_port(cs_port), _cs_pin(cs_pin), _dma_enable(dma_enable) {
}

/* CS 必须在 txrx_dma 之前拉低，否则地址字节会被别的从机"听"走。
 * 钩子已由 bus_acquire() 绑定，此处不再重绑。 */
pyro::status_t lsm6dsv16x_spi_t::dma_start(const uint8_t *tx, uint8_t *rx, const uint16_t size) {
    cs_low();
    return _spi_drv.txrx_dma(tx, rx, size);
}

/* 数据搬运完成点释放 CS。写 BSRR 原子且幂等，与任务侧的超时兜底释放不冲突。 */
void lsm6dsv16x_spi_t::release_cs_from_isr(void *ctx) {
    static_cast<lsm6dsv16x_spi_t *>(ctx)->cs_high();
}

void lsm6dsv16x_spi_t::cs_low() const {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
}

void lsm6dsv16x_spi_t::cs_high() const {
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
}
