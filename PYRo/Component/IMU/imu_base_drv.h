#ifndef __IMU_BASE_DRV_H__
#define __IMU_BASE_DRV_H__
#include <cstdint>

/* 物理量一律为【器件坐标系】（LSM6DSV16X 的 X/Y/Z，与输出原码一一对应）。
 * 板级装配方向映射不属于器件契约，由产品装配层在输出端完成，见
 * Module/Config/imu_mount.h。 */
struct acc_sample_t {
    uint32_t timestamp;   /* 器件时间戳计数器原始值（tick），见上方说明 */
    float Accel[3];       /* 器件坐标系，单位 m/s² */
};

struct gyro_sample_t {
    uint32_t timestamp;
    float Gyro[3];        /* 器件坐标系，单位 rad/s */
};

struct imu_cfg_t {
    bool calibrate_flag{};
    uint16_t calibrate_times = 60000;
    uint16_t acc_range{};   // 单位 g（配置用 g,输出换算成 m/s²）
    uint16_t acc_freq{};    // 单位 Hz
    uint16_t gyro_range{};  // 单位 dps
    uint16_t gyro_freq{};   // 单位 Hz
    bool fifo_en{};
};

struct imu_cali_t {
    /* 陀螺静态零偏，器件坐标系，rad/s。标定在器件系进行（见 calibrate()）。 */
    float gx_offset = -0.010767111;
    float gy_offset = 0.013010208;
    float gz_offset =-0.0014672983;
    /* 静止加速度模长，m/s²。目前只有 calibrate() 写入、尚无读者
     * （本产品不输出"去重力线性加速度"）。
     * 默认取标准重力而不是 0：将来按下式取线性加速度时
     *     linear = acc - (g_norm != 0 ? g_norm : 9.80665) * 单位重力方向
     * 默认 0 会让分母为 0、方向无定义，且这种错误是静默的。 */
    float g_norm = 9.80665f;
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
    /* 全部物理量为【器件坐标系】。驱动只做单位换算与器件零偏扣除，
     * 板级装配方向映射由产品装配层在输出端完成。 */
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
     * @brief 读取一拍数据并解包（仅限任务上下文）。
     *
     * 单一读取入口：内部按配置自行决定数据来源（器件 FIFO 批量 / 输出寄存器直读），
     * 调用方不需要知道走了哪条路径，也不需要感知"状态查询"这一步。
     * 典型实现为两段式：先发起状态事务、经完成中断唤醒后判读，再发起数据事务、
     * 再次唤醒后收取并解包。等待期间让出 CPU，不做轮询。
     *
     * @retval true  本拍解出了新样本（见 acc.count / gyro.count）
     * @retval false 本拍没有新数据，或超时 / 总线失败（区分方式见 diag()）
     *
     * @note  无数据时不得改变 count；缓冲满时覆盖最旧的未消费样本。
     */
    virtual bool read() = 0;

    protected:
    imu_data_t _data;
};

#endif
