#ifndef __LSM6DSV16X_DRV_H__
#define __LSM6DSV16X_DRV_H__
#include "imu_base_drv.h"
#include "lsm6dsv16x_middleware.h"

/* 每流样本容量。未及时消费导致缓冲满时覆盖最旧样本，并计入 err_data。 */
static constexpr uint8_t LSM6DSV16X_SAMPLE_BUF_FRAMES = 24U;

/* 单次传输上限，超出部分留待后续读取。每个 FIFO 字占 7 字节。 */
static constexpr uint16_t LSM6DSV16X_MAX_WORDS_PER_XFER = LSM6DSV16X_MAX_BURST / 7U;

class lsm6dsv16x_drv_t : public imu_base_drv {
    public:
    explicit lsm6dsv16x_drv_t(const imu_cfg_t &imu_cfg, lsm6dsv16x_t &lsm6dsv16x) : imu_base_drv(imu_cfg),
        _device(lsm6dsv16x) {
        /* 样本缓冲由本类申请，绑定到两路流（环形状态由流自己维护） */
        _data.acc.sample  = _acc_buf;
        _data.acc.capacity = LSM6DSV16X_SAMPLE_BUF_FRAMES;
        _data.gyro.sample = _gyro_buf;
        _data.gyro.capacity = LSM6DSV16X_SAMPLE_BUF_FRAMES;
    }
    ~lsm6dsv16x_drv_t() override;
    bool init() override;
    /* FIFO 路径（异步流水）：本拍解析的是**上一拍发起的那一帧**，解析完立刻为下一拍
     * 发起新帧，随后不等待、正常返回。端到端延迟固定 +1 拍（1 ms），
     * 但样本自带器件时间戳，不影响时间精度。
     * 两路各自成流（每样本带时间戳），数量见 acc.count / gyro.count。
     * fifo_en=false 走直读回退（同构：状态同步、数据区按 DMA 与否分派）。
     * 无返回值：有没有新数据看 count；样本由 assume_*_point() 消费归还。 */
    void update() override;

    /* 运行期诊断（只读）。err_bus/err_data 稳态都应恒为 0。 */
    struct diag_t {
        uint32_t err_bus;   /* 总线失败（状态读或帧读失败） */
        uint32_t err_data;  /* FIFO 溢出 / 未知 TAG / 样本缓冲覆盖 */
    };
    [[nodiscard]] diag_t diag() const {
        return {_fifo.err_bus, _fifo.err_data};
    }

    private:
    lsm6dsv16x_t& _device;
    float _acc_fs_g{4.0f};        /* g，init() 时按 imu_cfg_t 设定 */
    float _gyro_dps_per_lsb{0.070f}; /* UI 陀螺灵敏度，°/s/LSB，默认 ±2000 dps */

    /* 两路样本缓冲 + FIFO 原始字缓冲（容量 = 单帧上限的字数） */
    acc_sample_t  _acc_buf[LSM6DSV16X_SAMPLE_BUF_FRAMES]{};
    gyro_sample_t _gyro_buf[LSM6DSV16X_SAMPLE_BUF_FRAMES]{};
    uint8_t _fifo_raw[LSM6DSV16X_MAX_WORDS_PER_XFER * 7U]{};

    /* ============ 环形缓冲（本类内部维护） ============
     * 纯回绕、不搬移：写入位置 = (head + count) 绕回，消费只前移 head。
     * 未消费样本在物理内存里可能分成两段，一律通过 stream 的 at()/cur() 访问。 */

    /* 追加一个样本槽：满时覆盖最旧的未消费样本，因此总返回有效指针。
     * 槽内容先清零，避免上一次的残值被误当成本次数据。 */
    acc_sample_t *acc_alloc() {
        if (_data.acc.count == _data.acc.capacity) {
            ++_fifo.err_data;
        }
        uint16_t slot = 0U;
        (void)_data.acc.push(slot); /* capacity 由构造保证非 0，必定成功 */
        _acc_buf[slot] = acc_sample_t{};
        return &_acc_buf[slot];
    }
    gyro_sample_t *gyro_alloc() {
        if (_data.gyro.count == _data.gyro.capacity) {
            ++_fifo.err_data;
        }
        uint16_t slot = 0U;
        (void)_data.gyro.push(slot);
        _gyro_buf[slot] = gyro_sample_t{};
        return &_gyro_buf[slot];
    }

    /* FIFO 运行状态（私有）：最小错误检测 + 跨调用的解析中间态，上层不依赖。
     * 样本计数不在这里——stream 的 count 本身就是"待消费样本数"。 */
    struct fifo_state_t {
        volatile uint32_t err_bus;    /* 总线失败次数 */
        volatile uint32_t err_data;   /* 数据损坏：FIFO 溢出丢样本 / 未知 TAG / 缓冲覆盖最旧 */
        uint32_t pend_ts;             /* 当前槽时间戳，跨调用保持（读边界可能切在槽中间；
                                         个别槽无时间戳字时沿用上次值，下一槽即自愈）*/
        bool     ovr_prev;            /* 溢出标志（上升沿计数，兼容标志清除语义）*/
    };
    fifo_state_t _fifo{};

    /* fifo_en 分派：true = FIFO 批量路径（异步流水）；false = 直读回退 */
    void update_batch();
    void fifo_parse_word(const uint8_t *word);
    /* fifo_en=false 回退路径：STATUS_REG 门控 + 直读 20h~43h（含时间戳）+ 组装 1 帧 */
    void update_direct();
};

#endif
