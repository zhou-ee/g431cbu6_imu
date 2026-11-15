#ifndef __IMU_BASE_DRV_H__
#define __IMU_BASE_DRV_H__
#include <cstdint>

struct acc_sample_t {
    uint32_t timestamp;   /* 器件时间戳计数器原始值（tick），见上方说明 */
    float Accel[3];       /* 单位 m/s²       */
};

struct gyro_sample_t {
    uint32_t timestamp;
    float Gyro[3];
};

enum class imu_direct_t : uint8_t {
    NONE     = 0,
    DIRECT_1 = 1,
    DIRECT_2 = 2,
    DIRECT_3 = 3,
    DIRECT_4 = 4,
    DIRECT_5 = 5,
    DIRECT_6 = 6,
    DIRECT_7 = 7,
    DIRECT_8 = 8
};

struct imu_cfg_t {
    bool calibrate_flag{};
    uint16_t calibrate_times = 60000;
    uint16_t acc_range{};   // 单位 g（配置用 g,输出换算成 m/s²）
    uint16_t acc_freq{};    // 单位 Hz
    uint16_t gyro_range{};  // 单位 dps
    uint16_t gyro_freq{};   // 单位 Hz
    bool fifo_en{};
    imu_direct_t direct = imu_direct_t::NONE;
};

struct imu_cali_t {
    float gx_offset = -0.010767111;
    float gy_offset = 0.013010208;
    float gz_offset =-0.0014672983;
    float g_norm;
};

/**
     * 单路样本流（回绕环形缓冲的对外视图）。
 * 逻辑顺序旧 → 新：count 个未消费样本里，下标 0 最旧、count-1 最新。
 *
 * 物理上数据可能分两段（尾部 + 头部），一律用 at()/cur() 取值，它们处理回绕。
 * 生产侧（驱动）用 push() 追加（满时覆盖最旧），上层用 assume_*() 归还消费数量。
 */
template <typename T>
struct sample_stream_t {
    T*       sample{};     /* 物理基址 */
    uint16_t head{};       /* 最旧未消费样本的物理下标 */
    uint16_t capacity{};   /* 物理缓冲元素数 */
    uint8_t  count{};      /* 未消费样本数 */

    /* 逻辑下标 i（0 = 最旧）→ 实际元素。i 必须 < count。 */
    const T& at(const uint8_t i) const {
        const uint32_t idx = static_cast<uint32_t>(head) + i;
        return sample[idx >= capacity ? idx - capacity : idx];
    }
    /* 最新样本（count 必须 > 0） */
    const T& cur() const {
        return at(static_cast<uint8_t>(count - 1U));
    }

    /* 内部状态推进：追加一个槽 / 归还 count 个。
     * 满时覆盖最旧（head 前移一格），保证新样本总能写入——
     * 与器件 FIFO continuous 模式"满后覆盖最旧"一致。 */
    bool push(uint16_t& slot) {
        if (capacity == 0U) {
            return false;
        }
        if (count >= capacity) {
            /* 满：丢弃最旧的一个，腾出位置（不改变 count） */
            ++head;
            if (head >= capacity) {
                head = 0U;
            }
        } else {
            ++count;
        }
        slot = static_cast<uint16_t>(head + count - 1U);
        if (slot >= capacity) {
            slot = static_cast<uint16_t>(slot - capacity);
        }
        return true;
    }
    void advance(uint8_t n) {
        if (n >= count) {
            count = 0U;
            head  = 0U;
            return;
        }
        head = static_cast<uint16_t>(head + n);
        if (head >= capacity) {
            head = static_cast<uint16_t>(head - capacity);
        }
        count = static_cast<uint8_t>(count - n);
    }
};

struct imu_data_t {
    /**
     * 两路样本流（陀螺 / 加速度）。
     * 取样本：acc.at(i)/acc.cur()、gyro.at(i)/gyro.cur()；
     * 数量：acc.count / gyro.count（0 表示没有新数据）；
     * 消费后归还：assume_acc_point()/assume_gyro_point()。
     */
    sample_stream_t<acc_sample_t>  acc{};
    sample_stream_t<gyro_sample_t> gyro{};

    float temperature{};
    imu_cali_t cali{};
    imu_cfg_t cfg = imu_cfg_t{};
    float timestamp_tick_s = 1.0e-6f; /* timestamp 每 tick 的秒数，由具体器件驱动填写 */
};

class imu_base_drv {
    public:
    explicit imu_base_drv(const imu_cfg_t& imu_cfg) {
        _data.cfg = imu_cfg;
    };
    imu_data_t& read_imu_data() {
        return _data;
    }

    /* 归还已消费的样本数（非虚：环形状态统一在 sample_stream_t 里，
     * 与具体 IMU 型号无关，派生类不需要也无法改变这个语义）。
     * 只前移 head 指针，O(1)，不搬数据；超出未消费数量时按实际数量处理。 */
    void assume_acc_point(uint8_t count) {
        _data.acc.advance(count);
    }
    void assume_gyro_point(uint8_t count) {
        _data.gyro.advance(count);
    }

    bool calibrate();
    virtual ~imu_base_drv();
    virtual bool init() = 0;

    /**
     * @brief 取一批样本（非阻塞，仅限任务上下文）。
     * @note  无返回值：本批有没有新数据看 acc.count / gyro.count。
     *        实现须保证：无新数据时不改变 count；缓冲满时覆盖最旧的未消费样本。
     */
    virtual void update() = 0;

    protected:
    imu_data_t _data;
};

#endif
