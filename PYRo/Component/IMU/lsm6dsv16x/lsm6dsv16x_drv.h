#ifndef __LSM6DSV16X_DRV_H__
#define __LSM6DSV16X_DRV_H__
#include "imu_base_drv.h"
#include "lsm6dsv16x_middleware.h"

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

    /* 解包参数（只读）：通信层反向恢复器件原码时按这些值换算，保证与正向解包
     * 同源——量程切换后不必在别处再维护一份灵敏度表。
     * acc 单位 m/s²/LSB，gyro 单位 rad/s/LSB（= dps_per_lsb × π/180）。 */
    [[nodiscard]] float acc_mps2_per_lsb() const;
    [[nodiscard]] float gyro_rps_per_lsb() const;

    /* 运行期诊断（只读）。err_bus/err_data/err_timeout 稳态都应恒为 0。
     * err_data 是合计数；err_ovr / err_backpressure 用来区分成因。 */
    struct diag_t {
        uint32_t err_bus;          /* 总线失败（状态读或帧读失败） */
        uint32_t err_data;         /* 数据异常合计（见下方两个分项） */
        uint32_t err_timeout;      /* 等完成通知超时（DMA 未在期限内完成） */
        uint32_t err_ovr;          /* 其中：器件 FIFO 溢出（整拍未被读走） */
        uint32_t err_backpressure; /* 其中：软件缓冲满、丢弃新样本（消费者跟不上） */
    };
    [[nodiscard]] diag_t diag() const {
        return {_fifo.err_bus, _fifo.err_data, _fifo.err_timeout, _fifo.err_ovr,
                _fifo.err_backpressure};
    }

    /** 清零累计错误计数（不动 ovr_prev 边沿状态与 pend_ts）。
     *  供产品层在"启动瞬态结束、消费侧就绪"时调用一次，
     *  使 err_* 只反映稳态，便于用其判断持续堆积。 */
    void clear_error_counters() {
        _fifo.err_bus = 0U;
        _fifo.err_data = 0U;
        _fifo.err_timeout = 0U;
        _fifo.err_ovr = 0U;
        _fifo.err_backpressure = 0U;
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
            ++_fifo.err_backpressure; /* 消费者跟不上：与器件溢出分开计数 */
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
        volatile uint32_t err_data;    /* 数据异常合计 = err_ovr + err_backpressure + 未知 TAG */
        volatile uint32_t err_timeout; /* 等完成通知超时次数 */
        /* ---- 分类计数（err_data 的两个主要成因，现场必须能分开归因）----
         * err_ovr          —— 器件 FIFO 溢出：continuous 模式满后覆盖最旧，
         *                      通常意味着"一整拍没被读走"（调度抖动/总线慢）。
         * err_backpressure —— 软件缓冲满：消费者（融合任务）跟不上生产，
         *                      即 SPSC 侧丢新样本。两者的处置完全不同。 */
        volatile uint32_t err_ovr;
        volatile uint32_t err_backpressure;
        uint32_t pend_ts;              /* 当前槽时间戳，跨调用保持（读边界可能切在槽中间；
                                          个别槽无时间戳字时沿用上次值，下一槽即自愈）*/
        bool     ovr_prev;             /* 溢出标志（上升沿计数，兼容标志清除语义）*/
    };
    fifo_state_t _fifo{};

    /* read() 分派：true = FIFO 批量路径（两段式）；false = 直读回退 */
    bool read_fifo_batch();
    void fifo_parse_word(const uint8_t *word);
    /* fifo_en=false 回退路径：STATUS_REG 门控 + 直读 20h~43h（含时间戳）+ 组装 1 帧 */
    bool read_direct();
};

#endif
