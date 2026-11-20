/**
  ******************************************************************************
  * @file    lsm6dsv16x_drv.cpp
  * @brief   LSM6DSV16X 驱动实现（挂在 imu_base_drv 契约下，总线由外部注入）
  *
  * 文件结构：
  *   ① 常量与"映射/工具函数"的前向声明
  *   ② 关键实现：init() / read()（FIFO 两段式 + 器件时间戳 / 直读回退）
  *   ③ 文件末尾：映射表的具体实现（配置项 → 寄存器编码，与主流程无关）
  ******************************************************************************
  */

#include "lsm6dsv16x_drv.h"
#include "lsm6dsv16x_reg.h"

using namespace LSM6DSV16X;

/* ============================ ① 常量 ============================ */
namespace {

constexpr float DEG_TO_RAD = 0.01745329251994330f; /* π/180 */
constexpr float GRAVITY    = 9.80665f;             /* 标准重力加速度 m/s²（acc_range 配 g，输出换算成 m/s²）*/

/* 时间戳计数器复位值：向 TIMESTAMP2(42h) 写 0xAA 可将 32 位计数器清零（AN5640 6.4 节） */
constexpr uint8_t TIMESTAMP_RESET_VALUE = 0xAA;

/* 小端逐字节读取。
 * FIFO 字与直读缓冲都只有 1 字节对齐，原先的 reinterpret_cast<int16_t*>/
 * <uint32_t*> 属于未对齐访问 + 严格别名违规：Cortex-M4 的 LDR/LDRH 容忍未对齐
 * 所以"能跑"，但换架构/编译器或开更激进的优化就会出问题。
 * 逐字节组装顺带把线上字节序写明，不再依赖目标机字节序。 */
inline int16_t load_i16le(const uint8_t *p)
{
    const uint16_t lo = static_cast<uint16_t>(p[0]);
    const uint16_t hi = static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8U);
    return static_cast<int16_t>(static_cast<uint16_t>(lo | hi));
}

inline uint32_t load_u32le(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8U) |
           (static_cast<uint32_t>(p[2]) << 16U) | (static_cast<uint32_t>(p[3]) << 24U);
}

/* ---------------- 映射/工具函数：前向声明（实现见文件末尾 ③） ---------------- */
bool xl_odr_code(uint16_t hz, uint8_t &code);
bool gy_odr_code(uint16_t hz, uint8_t &code);
bool xl_fs_code(uint16_t g, uint8_t &code, float &full_scale);
bool gy_fs_code(uint16_t dps, uint8_t &code, float &dps_per_lsb);

} /* namespace */

/* =====================================================================
 *  ② 关键实现
 * ===================================================================== */

lsm6dsv16x_drv_t::~lsm6dsv16x_drv_t() = default;

float lsm6dsv16x_drv_t::acc_mps2_per_lsb() const
{
    return (_acc_fs_g * GRAVITY) / 32768.0f;
}

float lsm6dsv16x_drv_t::gyro_rps_per_lsb() const
{
    return _gyro_dps_per_lsb * DEG_TO_RAD;
}

/**
 * @brief 初始化：WHO_AM_I 自检 → 软复位 → BDU/IF_INC → 满量程 → 模式+ODR
 *               → FIFO（fifo_en）→（可选）标定
 * @note  FIFO 配置必须放在软复位之后（软复位会清空 FIFO 并冻结时间戳计数器）。
 */
bool lsm6dsv16x_drv_t::init() {
    uint8_t v = 0U;

    /* 1) 器件自检：WHO_AM_I 必须为 0x70 */
    if (!_device.read_reg(WHO_AM_I, &v) || (v != WHO_AM_I_ID)) {
        return false;
    }

    /* 2) 软复位，等 SW_RESET 自动清零 */
    if (!_device.write_reg(CTRL3, CTRL3_SW_RESET)) {
        return false;
    }
    HAL_Delay(10U);
    for (uint8_t i = 0U; i < 20U; ++i) {
        if (!_device.read_reg(CTRL3, &v)) {
            return false;
        }
        if ((v & CTRL3_SW_RESET) == 0U) {
            break;
        }
        HAL_Delay(2U);
    }
    if ((v & CTRL3_SW_RESET) != 0U) {
        return false;
    }

    /* 3) BDU=1（读 MSB 前输出不更新）+ IF_INC=1（多字节地址自增），即 0x44 */
    if (!_device.write_reg(CTRL3, CTRL3_DEFAULT_BDU_IF_INC)) {
        return false;
    }

    /* DS13510 9.52：FREQ_FINE 是有符号 8 位数，校正器件时间戳时钟。 */
    if (!_device.read_reg(INTERNAL_FREQ_FINE, &v)) {
        return false;
    }
    const int freq_fine = v < 128U ? static_cast<int>(v) : static_cast<int>(v) - 256;
    _data.timestamp_tick_s = 1.0f / (46080.0f * (1.0f + 0.0013f * freq_fine));

    /* 4) 先定满量程，再落 ODR */
    uint8_t xl_odr = 0U;
    uint8_t gy_odr = 0U;
    uint8_t xl_fs  = 0U;
    uint8_t gy_fs  = 0U;

    if (!xl_odr_code(_data.cfg.acc_freq, xl_odr) ||
        !gy_odr_code(_data.cfg.gyro_freq, gy_odr) ||
        !xl_fs_code(_data.cfg.acc_range, xl_fs, _acc_fs_g) ||
        !gy_fs_code(_data.cfg.gyro_range, gy_fs, _gyro_dps_per_lsb)) {
        return false; /* 配置里的量程/频率不受支持 */
    }

    if (!_device.write_reg(CTRL8, xl_fs) ||
        !_device.write_reg(CTRL6, gy_fs) ||
        !_device.write_reg(CTRL1, static_cast<uint8_t>(CTRL1_OP_MODE_XL_HIGH_PERFORMANCE | xl_odr)) ||
        !_device.write_reg(CTRL2, static_cast<uint8_t>(CTRL2_OP_MODE_G_HIGH_PERFORMANCE | gy_odr))) {
        return false;
    }

    _fifo.timestamp_valid = false;
    /* 5) 时间戳计数器：独立于 FIFO 的器件特性，无条件使能后清零
     *    （FUNCTIONS_ENABLE(50h) 主页面无需切 bank；tick 秒数由 FREQ_FINE 校正）。
     *    FIFO 路径逐槽批入时间戳字；直读回退路径直接读 TIMESTAMP0~3 寄存器。 */
    if (!_device.write_reg(FUNCTIONS_ENABLE, FUNCTIONS_ENABLE_TIMESTAMP_EN) ||
        !_device.write_reg(TIMESTAMP2, TIMESTAMP_RESET_VALUE)) {
        return false;
    }

    /* 6) FIFO 路径（fifo_en）：BDR=ODR 不降采样；先定 BDR 后切模式，bypass→continuous
     *    会复位 FIFO，此时为空无害；时间戳逐槽批入、温度 60Hz、Continuous 满后覆盖最旧。
     *    关闭则显式留 bypass，read() 走直读回退。 */
    if (_data.cfg.fifo_en) {
        if (!_device.write_reg(FIFO_CTRL3,
                               static_cast<uint8_t>((gy_odr << FIFO_CTRL3_BDR_GY_POS) | xl_odr)) ||
            !_device.write_reg(FIFO_CTRL1, 16U) ||
            !_device.write_reg(FIFO_CTRL4, FIFO_CTRL4_DEC_TS_BATCH_DEC_1 |
                                           FIFO_CTRL4_ODR_T_BATCH_60Hz |
                                           FIFO_CTRL4_FIFO_MODE_CONTINUOUS)) {
            return false;
        }
    } else if (!_device.write_reg(FIFO_CTRL4, FIFO_CTRL4_FIFO_MODE_BYPASS)) {
        return false; /* 软复位后默认即 bypass，显式写一次便于阅读 */
    }

    /* 7) 可选：陀螺零偏标定（标定期间器件需保持静止，走 read() 批量平均） */
    if (_data.cfg.calibrate_flag) {
        if (!calibrate()) {
            return false;
        }
    }

    return true;
}

/**
 * @brief fifo_en=false 回退路径：STATUS_REG 门控（XLDA&GDA 同时置位）→ 直读 20h~2Dh，
 *        两路各产出 1 个样本。高 ODR 下可能漏帧，仅作对比/调试用。
 * @note  时间戳直读 TIMESTAMP0~3 计数器（与 FIFO 路径同一计数器、同一时间基准）；
 *        只需相对时间（帧间差），不做绝对时刻补偿。
 * @note  普通 read_regs 同步返回；此处保留逐寄存器读取路径供直读对比。
 * @return true = 本拍产出了样本；false = 无新数据 / 总线失败。
 */
bool lsm6dsv16x_drv_t::read_direct() {
    uint8_t st = 0U;
    if (!_device.read_reg(STATUS_REG, &st)) {
        ++_fifo.err_bus;
        return false;
    }
    if ((st & STATUS_REG_XLDA) == 0U || (st & STATUS_REG_GDA) == 0U) {
        return false; /* 无新数据：不动已有样本 */
    }

    /* 20h~2Dh：温度 1 + 陀螺 3 + 加速度 3，共 7 个 int16（小端）。
     * 逐字节同步读取，FIFO 批量传输使用独立的 read_fifo 接口。 */
    for (uint16_t i = 0U; i < OUT_BURST_LEN; ++i) {
        if (!_device.read_regs(static_cast<uint8_t>(OUT_BURST_START + i), &_fifo_raw[i], 1U)) {
            ++_fifo.err_bus;
            return false;
        }
    }
    /* 20h~2Dh 解包：温度 1 + 陀螺 3 + 加速度 3，逐个小端 int16 */
    _data.temperature = (static_cast<float>(load_i16le(&_fifo_raw[0])) / TEMP_LSB_PER_C) + TEMP_ZERO_C;

    /* 一个连续读事务取回 32 位时间戳。失败时不把旧时间戳赋给新样本。 */
    uint8_t ts[4]{};
    if (!_device.read_regs(TIMESTAMP0, ts, sizeof(ts))) {
        ++_fifo.err_bus;
        ++_fifo.discontinuities;
        return false;
    }
    _fifo.pend_ts = load_u32le(ts);

    /* 两路各自组装 1 个样本（直读路径独立实现，不借道 FIFO 字格式）*/
    /* SPSC 生产侧：先 reserve 拿槽（满则返回 nullptr，计 err_data），
     * 原地写满内容，最后 commit 发布 tail —— 消费者不会看到半写样本。 */
    gyro_sample_t *gp = gyro_reserve();
    acc_sample_t  *ap = acc_reserve();

    /* 陀螺：dps → rad/s，逐轴减零偏。raw[1..3] 对应字节偏移 2/4/6 */
    if (gp != nullptr) {
        gyro_sample_t &g = *gp;
        g.timestamp = _fifo.pend_ts;
        const float gyro_lsb = _gyro_dps_per_lsb; /* 数据手册指定的 dps/LSB */
        g.Gyro[0] = (static_cast<float>(load_i16le(&_fifo_raw[2])) * gyro_lsb * DEG_TO_RAD) - _data.cali.gx_offset;
        g.Gyro[1] = (static_cast<float>(load_i16le(&_fifo_raw[4])) * gyro_lsb * DEG_TO_RAD) - _data.cali.gy_offset;
        g.Gyro[2] = (static_cast<float>(load_i16le(&_fifo_raw[6])) * gyro_lsb * DEG_TO_RAD) - _data.cali.gz_offset;
        gyro_commit();
    }

    /* 加速度：g → m/s²。raw[4..6] 对应字节偏移 8/10/12 */
    if (ap != nullptr) {
        acc_sample_t &a = *ap;
        a.timestamp = _fifo.pend_ts;
        const float acc_lsb = (_acc_fs_g * GRAVITY) / 32768.0f; /* m/s²/LSB */
        a.Accel[0] = static_cast<float>(load_i16le(&_fifo_raw[8])) * acc_lsb;
        a.Accel[1] = static_cast<float>(load_i16le(&_fifo_raw[10])) * acc_lsb;
        a.Accel[2] = static_cast<float>(load_i16le(&_fifo_raw[12])) * acc_lsb;
        acc_commit();
    }
    return true;
}

/**
 * @brief 单一读取入口（仅限任务上下文）：按 fifo_en 分派，数据来源对上层不可见。
 */
bool lsm6dsv16x_drv_t::read() {
    if (_data.cfg.fifo_en) {
        return read_fifo_batch();
    }
    return read_direct();
}

/**
 * @brief FIFO 两段式批量读取：状态事务 → 判读 → 数据事务 → 解包。
 *
 * 每笔事务都是"发起 → 让出 CPU 等完成中断唤醒 → 收取"，等待期间任务挂在
 * DMA 完成信号量上，不做轮询。状态字节、待读字数、交付字节数全是本函数局部量，
 * 判读结果（有无数据、溢出）在内部消化，不上浮给调用方。
 *
 * @retval true  解出了样本（数量见 acc.count / gyro.count）
 * @retval false 本拍无数据（diff==0，正常情况）或超时 / 总线失败
 * @note  两段各自独立限时；任一失败即结束本拍，不残留跨拍的中间状态。
 */
bool lsm6dsv16x_drv_t::read_fifo_batch() {
    /* ---- ① 状态事务：发起 → 让出 CPU 等唤醒 → 取回 2 字节 ----
     * 失败即结束本拍：read_regs_request 返回 false 的原因可能是"已有事务在途"，
     * 此时若改走同步 read_regs 会与在途 DMA 抢总线，故不做同步回退。
     * 不支持异步的总线由基类 read_regs_request 的默认实现返回 false，
     * 这类实现会走 read_direct() 路径（fifo_en 场合通常都是异步总线）。 */
    uint8_t st[2] = {0U, 0U};
    if (!_device.read_regs_request(FIFO_STATUS1, 2U)) {
        ++_fifo.err_bus;
        return false;
    }
    if (!_device.wait_dma(LSM6DSV16X_CPLT_TIMEOUT_MS)) {
        ++_fifo.err_timeout;
        return false;
    }
    if (!_device.read_regs_collect(st, 2U)) {
        ++_fifo.err_bus;
        return false;
    }

    /* ---- ② 判读：待读字数与溢出（上升沿计数，兼容标志清除语义） ---- */
    const uint16_t diff =
        static_cast<uint16_t>(st[0] | ((st[1] & FIFO_STATUS2_DIFF_FIFO_8) << 8));
    const bool ovr_now =
        (st[1] & (FIFO_STATUS2_FIFO_OVR_IA | FIFO_STATUS2_FIFO_OVR_LATCHED)) != 0U;
    if (ovr_now && !_fifo.ovr_prev) {
        ++_fifo.err_data; /* continuous 模式满后覆盖最旧：已丢样本 */
        ++_fifo.err_ovr;  /* 器件侧溢出，与软件背压分开计数 */
        ++_fifo.discontinuities;
        _fifo.timestamp_valid = false;
    }
    _fifo.ovr_prev = ovr_now;

    if (diff == 0U) {
        return false; /* 器件尚未产生新样本：不是错误 */
    }

    /* ---- ②b 积压保护：单次回读字数超限则整批丢弃 ----
     *
     * 正常 1 ms 一拍只有 2~3 个字。一旦 diff 远超此值，说明本任务曾被长时间
     * 阻塞（调度延迟、总线异常、断点停机等），器件 FIFO 里积的还是旧数据。
     * 这些数据同时压垮软件环形缓冲与算法（每样本一次协方差传播），
     * 而且"陈旧的一大批"对姿态解算没有价值——最新数据才重要。
     *
     * 因此整批抛弃，并且【必须排空器件 FIFO】：只丢不解的话，下一拍 diff
     * 依然超限，会永久卡在这个分支里采集不到任何数据。
     * 排空方式 = 按当前 diff 发起一次丢弃读（读到 _fifo_raw 里随即忽略），
     * 单次最多 MAX_WORDS_PER_XFER 字，若仍未排空则留到下一拍继续。
     * 丢弃后恢复交付前，先等到一个新的时间戳字。 */
    const uint16_t cap = _data.cfg.fifo_max_words_per_read;
    if (cap != 0U && diff > cap) {
        ++_fifo.err_data;
        ++_fifo.err_overrun;
        ++_fifo.discontinuities;
        _fifo.timestamp_valid = false;

        uint16_t drain_words = diff;
        if (drain_words > LSM6DSV16X_MAX_WORDS_PER_XFER) {
            drain_words = LSM6DSV16X_MAX_WORDS_PER_XFER;
        }
        uint16_t drained = 0U;
        if (!_device.read_fifo(_fifo_raw, sizeof(_fifo_raw),
                               static_cast<uint16_t>(drain_words * 7U), drained)) {
            ++_fifo.err_bus;
            return false;
        }
        if (!_device.wait_dma(LSM6DSV16X_CPLT_TIMEOUT_MS)) {
            ++_fifo.err_timeout;
            return false;
        }
        if (!_device.read_fifo(_fifo_raw, sizeof(_fifo_raw), 0U, drained)) {
            ++_fifo.err_bus;
            return false;
        }
        return false; /* 本拍不交付任何样本 */
    }

    /* 本拍请求长度：≤ MAX_WORDS_PER_XFER 字（一笔 DMA 装得下），剩余下拍续读 */
    uint16_t words = diff;
    if (words > LSM6DSV16X_MAX_WORDS_PER_XFER) {
        words = LSM6DSV16X_MAX_WORDS_PER_XFER;
    }

    /* ---- ③ 数据事务：发起 → 让出 CPU 等唤醒 → 取回 ---- */
    uint16_t received_len = 0U;
    if (!_device.read_fifo(_fifo_raw, sizeof(_fifo_raw),
                           static_cast<uint16_t>(words * 7U), received_len)) {
        ++_fifo.err_bus;
        return false;
    }
    if (!_device.wait_dma(LSM6DSV16X_CPLT_TIMEOUT_MS)) {
        ++_fifo.err_timeout;
        ++_fifo.discontinuities;
        _fifo.timestamp_valid = false;
        return false;
    }
    /* request_len=0：只取回已完成的那一帧，不再发起新请求 */
    if (!_device.read_fifo(_fifo_raw, sizeof(_fifo_raw), 0U, received_len)) {
        ++_fifo.err_bus;
        ++_fifo.discontinuities;
        _fifo.timestamp_valid = false;
        return false;
    }

    /* ---- ④ 解包：逐 7 字节字按 TAG 分派到两路样本流 ---- */
    const uint16_t count = static_cast<uint16_t>(received_len / 7U);
    const uint16_t gyro_tail = _data.gyro.tail;
    const uint16_t acc_tail = _data.acc.tail;
    for (uint16_t i = 0U; i < count; ++i) {
        fifo_parse_word(&_fifo_raw[i * 7U]);
    }
    return _data.gyro.tail != gyro_tail || _data.acc.tail != acc_tail;
}

/**
 * @brief 解析一个 FIFO 字（7 字节）：按首字节 TAG 分派，数据区（word+1）按小端
 *        逐字节解包（陀螺/加速度 = 3 个 int16，温度 = 1 个 int16，时间戳 = uint32）。
 *        TAG 位[7:3]：0x01 陀螺 / 0x02 加速度 / 0x03 温度 / 0x04 时间戳。
 * @note  pend_ts 跨调用保持：读边界切在槽中间时（如只读到陀螺字），
 *        下一轮读到同槽的加速度字仍用同一个时间戳，对齐不会错。
 * @note  追加一律走 gyro_reserve()/acc_reserve() + commit：SPSC 生产侧，
 *        先写满 payload 再 publish()，消费者不会看到半写样本；
 *        满时丢弃新样本并计入 err_data（不覆盖最旧，见 sample_stream_t 头注释）。
 */
void lsm6dsv16x_drv_t::fifo_parse_word(const uint8_t *word) {
    switch (word[0] & FIFO_DATA_OUT_TAG_TAG_SENSOR_MSK) {
        case FIFO_DATA_OUT_TAG_GYRO: {
            if (!_fifo.timestamp_valid) { break; }
            gyro_sample_t *ps = gyro_reserve(); /* 满则 nullptr，已计 err_data */
            if (ps == nullptr) {
                break;
            }
            gyro_sample_t &s = *ps;
            s.timestamp = _fifo.pend_ts;

            const float gyro_lsb = _gyro_dps_per_lsb; /* 数据手册指定的 dps/LSB */
            /* dps → rad/s，逐轴减零偏。数据区 6 字节 = X/Y/Z 各一个小端 int16 */
            s.Gyro[0] = (static_cast<float>(load_i16le(word + 1)) * gyro_lsb * DEG_TO_RAD) - _data.cali.gx_offset;
            s.Gyro[1] = (static_cast<float>(load_i16le(word + 3)) * gyro_lsb * DEG_TO_RAD) - _data.cali.gy_offset;
            s.Gyro[2] = (static_cast<float>(load_i16le(word + 5)) * gyro_lsb * DEG_TO_RAD) - _data.cali.gz_offset;
            gyro_commit();
            break;
        }

        case FIFO_DATA_OUT_TAG_XL: {
            if (!_fifo.timestamp_valid) { break; }
            acc_sample_t *ps = acc_reserve();
            if (ps == nullptr) {
                break;
            }
            acc_sample_t &s = *ps;
            s.timestamp = _fifo.pend_ts;

            const float acc_lsb = (_acc_fs_g * GRAVITY) / 32768.0f; /* m/s²/LSB */
            s.Accel[0] = static_cast<float>(load_i16le(word + 1)) * acc_lsb;
            s.Accel[1] = static_cast<float>(load_i16le(word + 3)) * acc_lsb;
            s.Accel[2] = static_cast<float>(load_i16le(word + 5)) * acc_lsb;
            acc_commit();
            break;
        }

        case FIFO_DATA_OUT_TAG_TIMESTAMP:
            _fifo.pend_ts = load_u32le(word + 1);
            _fifo.timestamp_valid = true;
            break;

        case FIFO_DATA_OUT_TAG_TEMP:
            /* 温度 = raw / 256 + 25（°C）；60Hz 才刷新，直接更新最新值 */
            _data.temperature =
                (static_cast<float>(load_i16le(word + 1)) / TEMP_LSB_PER_C) + TEMP_ZERO_C;
            break;

        default:
            ++_fifo.err_data; /* 未知 TAG：本配置不产生 */
            break;
    }
}


/* =====================================================================
 *  ③ 映射表实现：配置项(Hz / g / dps / 安装方向) → 寄存器编码
 *     与主流程无关，放文件末尾便于阅读上面的关键代码
 * ===================================================================== */
namespace {

/* ---------------- ODR：Hz → CTRL1/CTRL2 的 ODR 编码 ---------------- */
/* 注意：加速度计与陀螺的 ODR 编码不同（陀螺 7.5Hz 起，码 0x2；加速度计多一档 1.875Hz，码 0x1）*/

bool xl_odr_code(uint16_t hz, uint8_t &code) {
    switch (hz) {
        case 7U:    code = CTRL1_ODR_XL_7Hz5;   return true; /* 7 表示 7.5 Hz */
        case 15U:   code = CTRL1_ODR_XL_15Hz;   return true;
        case 30U:   code = CTRL1_ODR_XL_30Hz;   return true;
        case 60U:   code = CTRL1_ODR_XL_60Hz;   return true;
        case 120U:  code = CTRL1_ODR_XL_120Hz;  return true;
        case 240U:  code = CTRL1_ODR_XL_240Hz;  return true;
        case 480U:  code = CTRL1_ODR_XL_480Hz;  return true;
        case 960U:  code = CTRL1_ODR_XL_960Hz;  return true;
        case 1920U: code = CTRL1_ODR_XL_1920Hz; return true;
        case 3840U: code = CTRL1_ODR_XL_3840Hz; return true;
        case 7680U: code = CTRL1_ODR_XL_7680Hz; return true;
        default:    return false;
    }
}

bool gy_odr_code(uint16_t hz, uint8_t &code) {
    switch (hz) {
        case 7U:    code = CTRL2_ODR_G_7Hz5;   return true; /* 7 表示 7.5 Hz */
        case 15U:   code = CTRL2_ODR_G_15Hz;   return true;
        case 30U:   code = CTRL2_ODR_G_30Hz;   return true;
        case 60U:   code = CTRL2_ODR_G_60Hz;   return true;
        case 120U:  code = CTRL2_ODR_G_120Hz;  return true;
        case 240U:  code = CTRL2_ODR_G_240Hz;  return true;
        case 480U:  code = CTRL2_ODR_G_480Hz;  return true;
        case 960U:  code = CTRL2_ODR_G_960Hz;  return true;
        case 1920U: code = CTRL2_ODR_G_1920Hz; return true;
        case 3840U: code = CTRL2_ODR_G_3840Hz; return true;
        case 7680U: code = CTRL2_ODR_G_7680Hz; return true;
        default:    return false;
    }
}

/* ---------------- 满量程：g / dps → CTRL8 / CTRL6 的编码 ---------------- */
/* 加速度返回物理满量程；陀螺仪返回数据手册指定的 dps/LSB 灵敏度。 */

bool xl_fs_code(uint16_t g, uint8_t &code, float &full_scale) {
    switch (g) {
        case 2U:  code = CTRL8_FS_XL_2g;  full_scale = Range::XL_2g;  return true;
        case 4U:  code = CTRL8_FS_XL_4g;  full_scale = Range::XL_4g;  return true;
        case 8U:  code = CTRL8_FS_XL_8g;  full_scale = Range::XL_8g;  return true;
        case 16U: code = CTRL8_FS_XL_16g; full_scale = Range::XL_16g; return true;
        default:  return false;
    }
}

bool gy_fs_code(uint16_t dps, uint8_t &code, float &dps_per_lsb) {
    switch (dps) {
        case 125U:  code = CTRL6_FS_G_125dps;  dps_per_lsb = Sensitivity::G_125dps;  return true;
        case 250U:  code = CTRL6_FS_G_250dps;  dps_per_lsb = Sensitivity::G_250dps;  return true;
        case 500U:  code = CTRL6_FS_G_500dps;  dps_per_lsb = Sensitivity::G_500dps;  return true;
        case 1000U: code = CTRL6_FS_G_1000dps; dps_per_lsb = Sensitivity::G_1000dps; return true;
        case 2000U: code = CTRL6_FS_G_2000dps; dps_per_lsb = Sensitivity::G_2000dps; return true;
        case 4000U: code = CTRL6_FS_G_4000dps; dps_per_lsb = Sensitivity::G_4000dps; return true;
        default:    return false;
    }
}

} /* namespace */
