#ifndef __LSM6DSV16X_DRV_H__
#define __LSM6DSV16X_DRV_H__
#include "imu_base_drv.h"
#include "../../../Device/lsm6dsv16x/lsm6dsv16x_middleware.h"

/* 每流样本容量。SPSC 环形缓冲，必须是 2 的幂（掩码代替取模）；留一个哨兵槽
 * 判别满，故实际可用深度 = 63。
 *
 * 容量按"采集任务最多被推迟多久"倒推，而不是按典型一拍：
 *   1920 Hz → 64 帧 ≈ 33 ms（当前产品配置，单拍 1 ms，余量 33 拍）
 *   7680 Hz → 64 帧 ≈ 8.3 ms（产品目标速率，仍需 ≥3 拍余量）
 * **满时丢弃新样本并计 err_data**（不再覆盖最旧）：覆盖需要移动 head，
 * 而 head 属于消费者；让生产者写 head 就等于退回需要加锁的设计。
 * 因此 err_data 现在纯粹是"背压丢样"计数，可与器件 FIFO 溢出分别归因。 */
static constexpr uint8_t LSM6DSV16X_SAMPLE_BUF_FRAMES = 64U;

/* 容量必须是 2 的幂，否则掩码取址会错。编译期拦住，避免改常量时踩坑。 */
static_assert((LSM6DSV16X_SAMPLE_BUF_FRAMES & (LSM6DSV16X_SAMPLE_BUF_FRAMES - 1U)) == 0U,
              "LSM6DSV16X_SAMPLE_BUF_FRAMES must be a power of two (SPSC uses mask indexing)");

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
    /* 单一读取入口（仅限任务上下文）：数据来源对上层不可见。
     *
     *   fifo_en=true ：两段式 FIFO 批量读取。先发起 FIFO 状态读、让出 CPU 等
     *                  完成中断唤醒，判读待读字数并处理溢出；有数据则发起 FIFO
     *                  读取、再次唤醒后逐 7 字节 TAG 解包成 acc/gyro 两路样本。
     *                  状态判读完全在内部消化，调用方看不到也无需关心。
     *   fifo_en=false：状态门控 + 直读输出寄存器，一拍产出一帧（回退路径）。
     *
     * 两次等待都通过 DMA 完成信号量挂起任务（非轮询），单次上限
     * LSM6DSV16X_CPLT_TIMEOUT_MS。
     *
     * @retval true  本拍解出了样本，数量见 acc.available() / gyro.available()
     * @retval false 本拍无数据（器件未产生新样本）或超时/总线失败
     *
     * 样本由 assume_*_point() 消费归还；无返回值时代可看两路 count。 */
    bool read() override;

    private:
    /* 运行期诊断（内部维护，不对外暴露）。
     * err_bus/err_data/err_timeout 稳态应恒为 0；err_data 是合计，
     * err_ovr / err_backpressure / err_overrun 用来区分成因。
     * 正式版只在调试器里直接看 _fifo 的对应字段，不提供公有 getter。 */
    struct diag_t {
        uint32_t err_bus;          /* 总线失败（状态读或帧读失败） */
        uint32_t err_data;         /* 数据异常合计（见下方各分项） */
        uint32_t err_timeout;      /* 等完成通知超时（DMA 未在期限内完成） */
        uint32_t err_ovr;          /* 其中：器件 FIFO 溢出（整拍未被读走） */
        uint32_t err_backpressure; /* 其中：软件缓冲满、丢弃新样本（消费者跟不上） */
        uint32_t err_overrun;      /* 其中：单次回读字数超限，整批丢弃并排空 */
    };
    [[nodiscard]] diag_t diag() const {
        return {_fifo.err_bus, _fifo.err_data, _fifo.err_timeout, _fifo.err_ovr,
                _fifo.err_backpressure, _fifo.err_overrun};
    }

    /* 解包参数（只读）：仅供本类内部在标定/自检路径上换算，通信层已改为
     * 按模块量程自行编码，不再依赖器件灵敏度。 */
    [[nodiscard]] float acc_mps2_per_lsb() const;
    [[nodiscard]] float gyro_rps_per_lsb() const;
    lsm6dsv16x_t& _device;
    float _acc_fs_g{4.0f};        /* g，init() 时按 imu_cfg_t 设定 */
    float _gyro_dps_per_lsb{0.070f}; /* UI 陀螺灵敏度，°/s/LSB，默认 ±2000 dps */

    /* 两路样本缓冲 + FIFO 原始字缓冲（容量 = 单帧上限的字数） */
    acc_sample_t  _acc_buf[LSM6DSV16X_SAMPLE_BUF_FRAMES]{};
    gyro_sample_t _gyro_buf[LSM6DSV16X_SAMPLE_BUF_FRAMES]{};
    uint8_t _fifo_raw[LSM6DSV16X_MAX_WORDS_PER_XFER * 7U]{};

    /* ============ 环形缓冲（本类内部维护） ============
     * SPSC：生产侧只移动 tail（publish），消费侧只移动 head（release）。
     * 本类是同一条总线上的唯一写者，故这里是唯一生产者。
     * 未消费样本在物理内存里可能分成两段，一律通过 stream 的 at()/cur() 访问。 */

    /* 取一个待写槽并清零，返回后调用方【必须】在填满内容后 commit 发布。
     * 满时返回 nullptr（A1：丢弃新样本），由调用方计 err_data ——
     * 不覆盖最旧，因为那需要写 head，会把生产者变成第二个 head 写者。 */
    acc_sample_t *acc_reserve() {
        acc_sample_t *slot = _data.acc.reserve();
        if (slot == nullptr) {
            ++_fifo.err_data;
            ++_fifo.err_backpressure;
                return nullptr;
        }
        *slot = acc_sample_t{}; /* 清残值，避免上一次的数据被误读 */
        return slot;
    }
    gyro_sample_t *gyro_reserve() {
        gyro_sample_t *slot = _data.gyro.reserve();
        if (slot == nullptr) {
            ++_fifo.err_data;
            ++_fifo.err_backpressure;
            return nullptr;
        }
        *slot = gyro_sample_t{};
        return slot;
    }
    /* 槽内容填完后调用，才让消费者可见。 */
    void acc_commit() { _data.acc.publish(); }
    void gyro_commit() { _data.gyro.publish(); }

    /* FIFO 运行状态（私有）：最小错误检测 + 跨调用的解析中间态，上层不依赖。
     * 样本计数不在这里——stream 的 (tail-head) 本身就是"待消费样本数"。
     * 本拍的状态字节、待读字数、交付字节数都是 read() 内的局部量，不落成员。 */
    struct fifo_state_t {
        volatile uint32_t err_bus;     /* 总线失败次数 */
        volatile uint32_t err_data;    /* 数据异常合计（下面各分项之和 + 未知 TAG） */
        volatile uint32_t err_timeout; /* 等完成通知超时次数 */
        /* ---- 分类计数（err_data 的主要成因，现场必须能分开归因）----
         * err_ovr          —— 器件 FIFO 溢出：continuous 模式满后覆盖最旧，
         *                      通常意味着"一整拍没被读走"（调度抖动/总线慢）。
         * err_backpressure —— 软件缓冲满：消费者（融合任务）跟不上生产，
         *                      即 SPSC 侧丢新样本。
         * err_overrun      —— 单次回读字数超过 cfg.fifo_max_words_per_read，
         *                      整批丢弃并排空器件 FIFO（防积压淹没算法）。
         * 三者处置完全不同：ovr 查采集时序，backpressure 查算力，overrun 查卡顿。 */
        volatile uint32_t err_ovr;
        volatile uint32_t err_backpressure;
        volatile uint32_t err_overrun;
        uint32_t pend_ts;
        bool     timestamp_valid;      /* 等到一个全新的时间戳字才重新交付样本 */
        bool     ovr_prev;
    };
    fifo_state_t _fifo{};

    /* read() 分派：true = FIFO 批量路径（两段式）；false = 直读回退 */
    bool read_fifo_batch();
    void fifo_parse_word(const uint8_t *word);
    /* fifo_en=false 回退路径：STATUS_REG 门控 + 直读 20h~43h（含时间戳）+ 组装 1 帧 */
    bool read_direct();
};

#endif
