# IMU/INS 层重构规划：算法可插拔 + 顶层接口对齐

| 项 | 内容 |
|---|---|
| 参考实现 | `E:\Software\Develop\RM\Wheel_leg\PYRo-wl\PYRo\Component\INS`（`pyro_ins.{h,cpp}`、`pyro_ins_attitude.{h,cpp}`） |
| 目标工程 | `IMU_Module`（STM32G431 / LSM6DSV16X / VQF-ESKF） |
| 本轮范围 | **只出规划，不改代码** |
| 前置状态 | P3 装配方向归一化 ✅、航向归零 ✅（见 `P3_HEADING_ZERO_PLAN.md`） |
| 承前 | 本规划**接管**上一轮挂起的"统一出口 `imu_snapshot.h`"（原 P3 规划书的第 3 步） |

---

## 1. 参考实现评估

### 1.1 值得照搬的

| 参考做法 | 价值 |
|---|---|
| **语义化 getter**：`get_angles` / `get_rads` / `get_gyro` / `get_accel` / `get_accel_without_g` / `get_quaternion` | 调用方按"物理量语义"取数，不按内部变量取数；接口即文档 |
| **算法层接口极简**：一个 `update()` 吃满一拍物理量，输出姿态；算法不感知器件、总线、任务 | 换算法只需换一个类，插槽处零改动 |
| **装配方向映射在 INS 层应用**（`_imu_x_idx` / `_x_neg` 那套），不在器件驱动里 | 与我们 P3 的结论**独立吻合**，是对该决策的外部印证 |
| 组件自带采集任务（`init()` 内 `xTaskCreate`） | 与现有 `imu_service` 结构一致，无需改动 |
| `accel_without_g` 作为产品级输出 | 控制环常用，值得纳入 |

### 1.2 明确不照搬的

| 参考做法 | 问题（含证据） | 我们的做法 |
|---|---|---|
| 13 个 getter，`_b`/`_n` 两套 | `get_gyro_n()` 实际返回 `_gyro_b`（`pyro_ins.cpp:299-302`）；`get_angles_b()` 与 `get_angles_n()` 实现**完全相同**，都返回 `_angle_n`。说明 `_b` 版根本没实现，双套接口在实践中是冗余 | 按物理意义只保留有意义的那一半（§4.3） |
| 各 getter 各自读私有缓存变量 | **无跨字段一致性**：先 `get_quaternion()` 再 `get_accel_b()`，两次之间任务可能已翻拍 → 四元数与加速度差 1 拍。协议要求"同组"时就是错数据 | 主接口是"一次取全量快照（seqlock）"；便捷 getter 明确标注"跨字段不保证同拍" |
| 所有 getter 恒 `return PYRO_OK` | 调用方无法知道姿态是否已收敛；`accel_without_g` 在姿态未收敛时**结果是错的**却不报错 | 快照带 `valid` / `heading_zeroed`；去重力量在 `g_norm` 无效或 `valid=false` 时返回错误 |
| 算法层直接产出 `Roll/Pitch/Yaw`（度） | 每个算法都要重实现一遍欧拉转换，多算法并存时口径不一致风险高 | 契约只出四元数，欧拉角在产品层算**一次** |
| `init(10, 0.001, 10000000, 0.9996, 0.01f)` 收 5 个裸 float | 参数含义靠位置，无类型保护；基类接口被具体算法参数污染 | 参数由派生类构造函数注入（强类型 config），基类 `configure()` **无参** |
| 算法状态 30+ 字段全 `public` | 换算法时漏填字段编译器不报错，"约定"无法强制 | 纯虚基类 + `attitude_state_t`，漏实现即编译失败 |
| `pyro_ins.h` 直接 include `BMI088_driver.h` / `FreeRTOS.h` / `task.h` | 上层只要 include 接口头，就把具体 IMU 驱动和 RTOS 拖进来 | 顶层接口头**零 RTOS / 零驱动依赖** |
| 硬编码 `BMI088_Read` / `hspi2` | 换器件要改 INS 层 | 驱动经 `imu_base_drv` 注入（已有） |
| 无航向零点 | 6 轴 yaw 不可观测，零点随开机姿态漂，产品不可重复 | 产品层航向零点（已实现） |
| `init()` 中 `for (count<10) { if (count>=255) return ERROR; }`，无 `break` | 死分支 + 无效重试（10 次都不 break） | 不照搬；`imu_base_drv::init()` 已返回单次结果 |
| 单例 `get_instance()` + 全局可变状态 | 难以单测、隐式依赖 | 保留 `imu_service` 的"静态实例 + 自由函数接口"；契约层做单测时直接栈上构造 |

---

## 2. 目标架构

### 2.1 分层与数据流

```text
┌──────────────── 顶层接口（Module/IMU/imu_snapshot.h + imu_service.h）────────────────┐
│  ins_get() 一次取全量快照（推荐主接口）  /  语义化便捷 getter  /  ins_zero_heading()  │
└───────────────────────────────────────────▲──────────────────────────────────────────┘
                                            │ imu_snapshot_t（安装系 + 已归零）
┌──────────────── 产品层（Module/IMU/imu_service.cpp）─────────────────────────────────┐
│  任务装配 · 驱动/算法注入 · 装配归一化(P3) · 航向归零 · 欧拉角 · 去重力 · 快照发布    │
└──────────────▲────────────────────────────────────────────▲─────────────────────────┘
               │ attitude_state_t（器件系 → 算法世界系）      │ imu_data_t（器件系）
┌──────────────┴──── 算法契约（PYRo/Component/INS/pyro_ins_algo.h，新增）──────────────┐
│  attitude_algo_base : configure() / reset() / update(data, consumed) / output()       │
└──────────────▲──────────────────────────────────────────────────────────────────────┘
               │ 实现契约
┌──────────────┴──── 算法实现（PYRo/Algorithm/*）──────────────────────────────────────┐
│  vqf_algo_t（包装现有 vqf_t）    eskf_algo_t（未来）    mahony_algo_t（未来）          │
└─────────────────────────────────────────────────────────────────────────────────────┘
               ▲
┌──────────────┴──── 器件层（PYRo/Component/IMU，本轮不改）────────────────────────────┐
│  imu_base_drv : init / read / calibrate / assume_* / read_imu_data                   │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 坐标系约定（必须写进契约头）

| 环节 | 坐标系 | 谁负责转换 |
|---|---|---|
| 器件层输出 `imu_data_t` | **器件系** | — |
| 算法输入 | **器件系** | — |
| 算法输出 `attitude_state_t.quat` | 器件系 → **算法自定义世界系** | 算法自己 |
| 产品快照 `imu_snapshot_t` | **安装系**（+ 世界系可选项） | 产品层：`quat_to_install` / `mount_apply` |
| 对外 getter | 安装系 / 世界系 | 产品层 |

**关键论断：航向归零是"算法可自由替换"的前提，不是附加功能。**

算法在器件系工作，意味着不同算法的"世界系 yaw 零点"可以各不相同（VQF 用最小旋转规范化，EKF/Mahony 可能选别的）。是产品层的航向零点把这种任意性统一消除掉，因此换算法后对外数值口径不变。**如果没有归零层，"自由换算法"就不成立。**

### 2.3 为什么算法不放在安装系

那会让算法必须被告知装配方向（回到 P3 之前的耦合），且每换一个算法都要重接一次映射。P3 已证明：算法在器件系 + 产品层换系 + 归零，与算法在安装系的**数值结果完全一致**（见 `P3_HEADING_ZERO_PLAN.md`）。

---

## 3. 算法接入契约

### 3.1 新增 `PYRo/Component/INS/pyro_ins_algo.h`

```cpp
#ifndef PYRO_INS_ALGO_H
#define PYRO_INS_ALGO_H

#include <cstdint>

#include "imu_base_drv.h"
#include "pyro_core_def.h"

/* ============================================================================
 * 姿态算法接入契约
 *
 * 职责边界：
 *   算法实现者负责   —— 从两路带时间戳的样本流估计姿态与陀螺零偏
 *   算法实现者不负责 —— 器件型号、总线、任务/RTOS、装配方向、航向零点、
 *                       欧拉角、去重力、协议封包
 *
 * 坐标系：输入为【器件坐标系】（见 imu_base_drv.h）；输出 quat 为
 *         器件系 → 算法自定义世界系。算法可自选世界系 yaw 零点，产品层的
 *         航向归零会消除这种任意性，因此换算法不改变对外的数值口径。
 *
 * 数据可见性：可安全使用 acc / gyro 两路流与 timestamp_tick_s。
 *             cfg / cali 属器件层，算法【不应】依赖。
 * ========================================================================== */
namespace pyro
{

/* 算法输出的姿态结果。任何算法都必须填满全部字段；
 * 自身不产生的量按注释填保守值，不得留未初始化。 */
struct attitude_state_t
{
    float quat[4];  /* [w,x,y,z]，器件系 → 世界系 */
    float bias[3];  /* 陀螺零偏估计，rad/s，器件系 */
    bool  valid;    /* 已完成可信重力对齐，姿态可用 */
    bool  rest;     /* 当前判定静止；算法不支持则恒 false */
    float rest_t;   /* 连续静止计时，s；算法不支持则恒 0 */
};

/* 一次 update 实际消费的样本数，宿主据此归还环形缓冲。
 * 约定：consumed 表示"归还"的样本数，【不】代表算法用到了每一个样本。
 *       单样本算法通常取最新一样本但把 count 全部消费，否则缓冲无法归还。 */
struct algo_consumed_t
{
    uint8_t gyro;
    uint8_t acc;
};

class attitude_algo_base
{
  public:
    virtual ~attitude_algo_base() = default;

    /* 首次初始化或热更新参数。参数由派生类构造函数注入，基类不收参数——
     * 避免基类接口被具体算法的配置污染。 */
    virtual status_t configure() = 0;

    /* 重置时间基准、姿态、协方差与全部滤波历史。
     * 器件时间戳计数器重启、tick_s 改变或产品重新初始化时必须调用。 */
    virtual status_t reset() = 0;

    /* 消费 data 中尚未消费的样本，并在返回前更新 output()。
     *   PYRO_OK      —— 正常推进
     *   PYRO_WARNING —— 部分样本被门控跳过，批次继续（不终止）
     *   其他错误     —— 批次终止，未消费样本保留在流中，下拍重试
     * 支持两种风格：
     *   批量算法（如 VQF）—— 按自身时间戳规则合并两路，消费量由算法决定；
     *   单样本算法         —— 取 cur() 并消费整批（见 §3.3）。 */
    virtual status_t update(const imu_data_t &data, algo_consumed_t &consumed) = 0;

    /* 最近一次 update 的姿态结果；仅在 update() 之后调用。 */
    virtual const attitude_state_t &output() const = 0;
};

} // namespace pyro
#endif // PYRO_INS_ALGO_H
```

**设计取舍说明**

| 决定 | 理由 |
|---|---|
| `update` 吃 `imu_data_t` 而非 6 个标量 | 只有批量契约才能让"两路各有独立采样率 + 各自时间戳如何合并"由算法自己决定——这本就是姿态算法的核心职责（7680 Hz 双路必须由算法决定配对规则）。单样本算法不受影响（§3.3） |
| `consumed` 用出参而非额外的 `get_state()` | 一次调用完成，算法无需维护"上次消费量"状态 |
| `configure()` 无参 | 参数类型是算法特有的，基类不能收敛它们；由构造函数强类型注入 |
| 不把 `acc_confidence` / NIS / 拒绝计数放进契约 | VQF 特有诊断，强制所有算法填充会产生无意义值。诊断走调试观测或派生类非虚方法 |
| 不把 `roll/pitch/yaw` 放进契约 | 欧拉转换是纯数学且对所有算法相同，放产品层共享一份，避免 N 个算法 N 份实现 |
| 用纯虚基类而非鸭子类型 | 与现有 `imu_base_drv`（已是纯虚基类抽象驱动）风格一致；契约显式、可 mock。开销：1 kHz × 1 次虚调用，相对 VQF 单拍 ≈940 µs 可忽略 |

### 3.2 VQF 适配器：新增 `PYRo/Algorithm/VQF/vqf_algo.h`

**`pyro_algo_vqf.h` / `.cpp` 一行不改。**

```cpp
#ifndef PYRO_VQF_ALGO_H
#define PYRO_VQF_ALGO_H

#include "pyro_algo_vqf.h"
#include "pyro_ins_algo.h"

namespace pyro
{

class vqf_algo_t final : public attitude_algo_base
{
  public:
    explicit vqf_algo_t(const vqf_t::config_t &config) : _config(config) {}

    status_t configure() override { return _vqf.configure(_config); }
    status_t reset() override { return _vqf.reset(); }

    status_t update(const imu_data_t &data, algo_consumed_t &consumed) override
    {
        const status_t result = _vqf.update(data);
        _state = _vqf.get_state();
        for (uint8_t i = 0U; i < 4U; ++i) { _out.quat[i] = _state.quat[i]; }
        for (uint8_t i = 0U; i < 3U; ++i) { _out.bias[i] = _state.bias[i]; }
        _out.valid  = _state.initialized;
        _out.rest   = _state.rest_detected;
        _out.rest_t = _state.rest_t;
        consumed.gyro = _state.consumed_gyr;
        consumed.acc  = _state.consumed_acc;
        return result;
    }

    const attitude_state_t &output() const override { return _out; }

    /* VQF 特有的皮外诊断，不进契约（供调试观测与航向归零门控使用） */
    float acc_confidence() const { return _state.acc_confidence; }
    float acc_nis()        const { return _state.acc_nis; }
    bool  acc_rejected()   const { return _state.acc_rejected; }
    float bias_sigma()     const { return _state.bias_sigma; }

  private:
    vqf_t           _vqf{};
    vqf_t::config_t _config;
    vqf_t::state_t  _state{};   /* 诊断缓存，不进契约 */
    attitude_state_t _out{};
};

} // namespace pyro
#endif // PYRO_VQF_ALGO_H
```

> 注：`vqf_t::state_t` 含 `matrix_t<6,6>` + `matrix_t<3,3>`（约 200 B），`get_state()` 按值返回。当前 1 kHz 下可接受；若换更重算法后单拍预算吃紧，可给 `vqf_t` 增加轻量 `get_output()` 只返回姿态部分（届时再评估，本轮不做）。

### 3.3 单样本算法如何写（约定，不实现）

```cpp
status_t update(const imu_data_t &data, algo_consumed_t &consumed) override
{
    consumed.gyro = 0U;
    consumed.acc  = 0U;
    const bool has_gyro = data.gyro.count != 0U;
    const bool has_acc  = data.acc.count  != 0U;
    if (!has_gyro && !has_acc) { return PYRO_OK; }   /* 无新数据，不动 count */

    /* 只用最新一样本；dt 由相邻时间戳差推得 */
    if (has_gyro) { const gyro_sample_t &g = data.gyro.cur(); /* … */ }
    if (has_acc)  { const acc_sample_t  &a = data.acc.cur();  /* … */ }

    /* 关键约定：全部消费，否则环形缓冲无法归还，满后会持续覆盖最旧 */
    consumed.gyro = data.gyro.count;
    consumed.acc  = data.acc.count;
    return PYRO_OK;
}
```

---

## 4. 顶层接口

### 4.1 一致性语义（本规划最重要的裁决）

参考实现的 13 个 getter 全部"各读各的私有缓存"，**没有一致性保证**。协议封包要求"四元数 + 角速度 + 加速度来自同一结果快照"，逐个 getter 取数会差 1 拍。

因此设两级接口：

| 级别 | 接口 | 一致性 | 适用场合 |
|---|---|---|---|
| **主接口** | `ins_get()` 一次取全量快照 | **seqlock 保证同一拍、不撕裂** | 协议封包、控制环、任何需要多字段协同的场合 |
| 便捷接口 | 语义化 getter（§4.3） | 各取一次快照，**跨字段不保证同拍** | 只关心单个量的场合（显示、单点查询） |

头文件必须把这条写清楚，避免调用方误用。

### 4.2 快照结构：新增 `Module/IMU/imu_snapshot.h`

零 RTOS / 零驱动依赖，通信层只 include 这个头。

```cpp
#ifndef IMU_SNAPSHOT_H
#define IMU_SNAPSHOT_H

#include <cstdint>

/* 一次完整更新的结果快照。所有字段同坐标系（安装系）、同航向零点、同发布序号。
 * 由采集任务在 seqlock 保护下整体更新，读方拿到的必是某一次完整更新的结果。 */
struct imu_snapshot_t
{
    /* ---- 姿态（安装系） ---- */
    float quat[4];       /* [w,x,y,z]，安装系 → 世界系 */
    float roll;          /* °，ZYX */
    float pitch;         /* ° */
    float yaw;           /* °，已按产品零点归零 */
    float yaw_total;     /* °，解卷绕后的连续 yaw */
    float bias[3];       /* 陀螺零偏估计，rad/s，安装系 */

    /* ---- 运动量（安装系） ---- */
    float rates[3];      /* rad/s */
    float accel[3];      /* m/s²，含重力 */
    float linear_accel[3]; /* m/s²，去重力；valid 为假时无意义 */

    /* ---- 运动量（世界系，控制环用） ---- */
    float accel_n[3];
    float linear_accel_n[3];

    /* ---- 时序与状态 ---- */
    uint32_t sequence;        /* 发布序号，1 kHz 递增 */
    uint32_t tick_device;     /* 最近样本的器件时间戳 */
    uint32_t tick_rounds;     /* 采集轮次 */
    bool     valid;           /* 姿态已可信 */
    bool     heading_zeroed;  /* yaw 零点已建立 */
    uint8_t  heading_quality; /* 0=未建立 1=静止+零偏收敛 2=超时兜底 */
    bool     rest;
    float    rest_t;
    int8_t   algo_status;     /* 最近一次 update 的 status_t，便于定位 */
};

#endif // IMU_SNAPSHOT_H
```

> 字段集可按需裁剪：`accel_n` / `linear_accel_n` / `yaw_total` 为可选（参考实现有前两者）。
> 不存 `gravity[3]`——可由 `accel - linear_accel` 得到，避免冗余。

### 4.3 getter 清单：参考 13 个 → 我们 11 个

| 我们的接口 | 参考对应 | 坐标系 | 说明 |
|---|---|---|---|
| `ins_get()` | — | — | **新增**，主接口，一次取全量 |
| `ins_get_quaternion(float q[4])` | `get_quaternion` | 安装系→世界系 | |
| `ins_get_angles(float *yaw, float *pitch, float *roll)` | `get_angles_n` / `get_angles_b` | 安装系 | 参考的 `_b` 版未实现（返回 `_angle_n`），合并为一套 |
| `ins_get_angles_rad(float *yaw, float *pitch, float *roll)` | `get_rads_n` / `get_rads_b` | 安装系 | 同上合并 |
| `ins_get_rates(float g[3])` | `get_gyro_b` | 安装系 | 陀螺是机体系量，**世界系版无物理意义**（参考的 `get_gyro_n` 正是直接返回 `_gyro_b`），取消 |
| `ins_get_accel(float a[3])` | `get_accel_b` | 安装系 | 含重力 |
| `ins_get_accel_n(float a[3])` | `get_accel_n` | 世界系 | 含重力 |
| `ins_get_linear_accel(float a[3])` | `get_accel_without_g_b` | 安装系 | `valid=false` 或 `g_norm<=0` 时返回错误 |
| `ins_get_linear_accel_n(float a[3])` | `get_accel_without_g_n` | 世界系 | 同上 |
| `ins_get_status(imu_status_t *s)` | — | — | **新增**：`valid` / `heading_zeroed` / `heading_quality` / `rest` / `algo_status` |
| `ins_zero_heading()` | — | — | **新增（预留）**：命令归零，与自动归零同一套 `yaw0` 机制，`t0` 由调用方指定 |

所有接口返回 `pyro::status_t`，空指针 / 未初始化 / 数据无效一律返回错误，**不假装 `PYRO_OK`**。

### 4.4 去重力量的正确性门槛（参考实现缺失）

参考实现直接 `acc_b - gravity_b`，用的是上一拍姿态；**姿态未收敛时结果是错的却不报错**。

约定：
```text
linear_accel 有效 ⟺ snapshot.valid && cali.g_norm > 0
```
- 当前 `imu_cali_t::g_norm` 只被 `calibrate()` 写入、**从未被读取**，默认值 0。因此必须显式校验。
- `g_norm <= 0` 时用 `9.80665` 兜底并把 `linear_accel` 置零 + 返回 `PYRO_NOT_FOUND`（或按产品决定用兜底值并置一个 `quality` 位）。

---

## 5. 文件清单

| 类型 | 路径 | 动作 |
|---|---|---|
| **新增** | `PYRo/Component/INS/pyro_ins_algo.h` | 算法契约（`attitude_state_t` / `algo_consumed_t` / `attitude_algo_base`） |
| **新增** | `PYRo/Algorithm/VQF/vqf_algo.h` | VQF 适配器（纯头文件） |
| **新增** | `Module/IMU/imu_snapshot.h` | 对外快照类型（零 RTOS/驱动依赖） |
| 修改 | `Module/IMU/imu_service.h` | 接口重组：`ins_get()` + 语义化 getter；`imu_attitude_t` 内嵌 `attitude_state_t`；`imu_service_data()` 收为内部 |
| 修改 | `Module/IMU/imu_service.cpp` | 持有 `attitude_algo_base&`；任务循环改走契约；补 `linear_accel` / `yaw_total` / `algo_status` |
| 修改 | `Module/CMakeLists.txt` | 无新增源文件（三个新头文件）；确认 `Config` 已在包含路径 |
| 修改 | `PYRo/CMakeLists.txt` | **需加** `Component/INS` 到 `target_include_directories`；**无需**新增源文件（契约与适配器均为纯头文件） |
| 修改 | `Module/PROTOCOL.md` | 补 `linear_accel` 语义与有效性门槛（若对外发布） |
| 修改 | `README.md` | 目录职责与数据流补"算法契约"一层 |
| **不动** | `PYRo/Algorithm/VQF/pyro_algo_vqf.{h,cpp}` | 零改动 |
| **不动** | `PYRo/Component/IMU/**` | 零改动（含 `imu_base_drv`、`lsm6dsv16x_drv`） |
| **不动** | `Module/Config/imu_mount.h` | 零改动 |
| **不动** | `Module/Application/imu_module.cpp` | 仅入口调用，接口签名保持兼容 |

---

## 6. 分阶段实施

每阶段都可独立编译、独立验证、独立回滚。

### 阶段 1：契约 + 适配器（**零行为变化**）

| 项 | 内容 |
|---|---|
| 改动 | 新增 `pyro_ins_algo.h`、`vqf_algo.h`；`PYRo/CMakeLists.txt` 加包含路径 |
| 行为 | 不接入，仅编译 |
| 验收 | Debug/Release 编译通过；Flash/RAM 不变（纯头文件未被引用时链接器会丢弃） |
| 风险 | 极低 |

### 阶段 2：产品层改走契约（**仍用 VQF，数值应不变**）

| 项 | 内容 |
|---|---|
| 改动 | `imu_service.cpp` 把 `pyro::vqf_t s_vqf` 换成 `pyro::vqf_algo_t s_algo{k_vqf_cfg}`，任务循环改为 `s_algo.update(data, consumed)` → `s_algo.output()` → `consumed` 归还 |
| 行为 | **不应变化**（同一算法、同一参数、同一消费语义） |
| 验收 | 与阶段 1 构建逐字段比对：`accel`/`gyro` **逐位相同**；`quat`/`bias`/`rest` ≤1e-6；`heading_zeroed`/`heading_quality` 相同；`err_*` 全 0；`heap_free_min` 不变 |
| 风险 | 中：`consumed` 的归还时机与现有 `state.consumed_gyr/acc` 等价，但必须确认签名未变（`algo_consumed_t` 是 2 字节结构，无对齐问题） |

### 阶段 3：顶层接口重组（**新增字段，旧字段保留兼容一个版本**）

| 项 | 内容 |
|---|---|
| 改动 | 新增 `imu_snapshot.h`；`ins_get()` + 11 个 getter；`imu_attitude_t` 内嵌 `attitude_state_t`；`imu_service_data()` 收回内部；补 `linear_accel` / `yaw_total` / `algo_status` |
| 行为 | 数值不变；新增输出字段 |
| 验收 | 与阶段 2 比对既有字段全部 ≤1e-6；新增字段做独立正确性校验（见 §7 第 2、3 项）；Flash/RAM 增量 < 200 B |
| 风险 | 中：`imu_attitude_t` 字段路径变化。**已确认当前无外部调用者**（`imu_service_attitude()` / `imu_service_data()` 全工程零调用），因此可自由重构 |

### 阶段 4（可选）：第二个算法验证插拔

| 项 | 内容 |
|---|---|
| 改动 | 新增 `PYRo/Algorithm/Attitude/mahony_algo.h`（或直接接 `pyro_algo_kf` 做一个简化 ESKF 适配器） |
| 目的 | 证明"换算法只改一行"成立，且归零后对外数值口径不变 |
| 验收 | 切换算法后：`heading_zeroed` 仍为 1、`quat` 收敛、`roll/pitch` 与 VQF 结果同量级差异（不要求相同）；**`yaw` 归零后仍从 0 起算**（这是"算法可替换"的核心验证点） |
| 风险 | 低（纯新增文件） |

---

## 7. 验收判据（阶段 3 新增字段）

| 字段 | 校验方法 |
|---|---|
| `linear_accel` | 静止时应 ≈ 0（\|a\| < 0.1 m/s²）；缓慢倾斜过程中应 ≈ 手算的切向加速度 |
| `linear_accel_n` | 静止时 z ≈ 0；由 `linear_accel` 按快照四元数旋转后应一致（≤1e-5） |
| `accel_n` | `rotate(quat, accel)` 的 z 分量应 ≈ `g_norm`（本次实测 9.81–9.88，与 \|a\| 一致到 5e-5） |
| `yaw_total` | 连续转过 ±180° 边界时不应跳变；与 `yaw` 的差应为 360° 的整数倍 |
| `g_norm` 无效时 | `linear_accel` 应置零并返回非 `PYRO_OK`；不得输出错误值 |
| `algo_status` | 稳态应恒为 `PYRO_OK`；若出现 `PYRO_WARNING`，应与 `rest`/门控行为可解释 |

---

## 8. 风险与待决项

| # | 项 | 说明与建议 |
|---|---|---|
| 1 | `imu_service_data()` 对外泄漏驱动内部 | 返回 `imu_data_t&`，把驱动私有状态（含 `cfg`/`cali`/环形缓冲）暴露给任意调用方。**已确认零外部调用者**，建议直接收为 `imu_service.cpp` 内部静态函数或删除 |
| 2 | `g_norm` 从未被读取 | `calibrate()` 写入、无人使用，默认 0。去重力量必须先校验 `> 0`，否则用 `9.80665` 兜底并显式标记 |
| 3 | VQF 单拍耗时已接近预算 | `imu_service_vqf_us_max` 实测 **940–941 µs / 1000 µs**（Release）。若换更重算法或加 9 轴，必须重测并考虑降 ODR 或分频送融合（.cpp 头部已有论证，不要改回双任务） |
| 4 | 单样本算法的"消费"语义 | 必须写进契约（已写入 §3.1 注释）：消费 = 归还缓冲，不代表算法用到每个样本；否则环形缓冲无法归还、持续覆盖最旧 |
| 5 | `imu_attitude_t` vs `attitude_state_t` 并存 | 两个 90% 相同的类型是后续 bug 源。建议 `imu_attitude_t` **内嵌** `attitude_state_t`（`attitude.algo.quat`），而不是并列 |
| 6 | 术语：是否把 `imu_*` 改为 `ins_*` | 建议**保留 `imu_` 前缀**，与 `PROTOCOL.md` / `README.md` / 上位机口径一致，INS 语义靠注释表达。若统一改名，需同步改协议文档与上位机 |
| 7 | 算法是否需要"请求重初始化" | 例如检测到陀螺饱和、温度突变。当前契约没有此通道，可先用 `valid=false` 表达（产品层据此降级）。若将来需要，加 `virtual bool needs_reset() const` 一个方法即可，不必现在加 |
| 8 | 命令归零的接口位置 | `ins_zero_heading()` 属产品层命令，应由通信层解析命令后调用。本轮只预留声明，不实现命令解析 |
| 9 | 世界系定义随算法变化 | 已在 §2.2 说明：由航向归零消除。但**若将来某算法内部使用世界系做别的用途**（如世界系速度积分），换算法时该量不可比——契约目前不输出世界系状态量，可规避 |

---

## 9. 与参考实现对照总结

| 维度 | 参考 `PYRo-wl/Component/INS` | 本规划 | 理由 |
|---|---|---|---|
| 顶层取数 | 13 个 getter，各读私有缓存 | 1 个全量快照 + 11 个便捷 getter | 协议要求同拍一致；参考实现无此保证 |
| 有效性 | 恒 `PYRO_OK` | 快照带 `valid` / `heading_zeroed` / `algo_status` | 姿态未收敛时去重力量是错的，必须可判别 |
| 算法接口 | 具体类 `ins_attitude_t`，`init(5 float)` + `update(6 float, dt)` | 纯虚基类 `attitude_algo_base`，`configure()` / `reset()` / `update(imu_data_t, consumed)` | 契约显式、可 mock；批量语义才能容纳双路独立采样率 |
| 算法输出 | `Roll/Pitch/Yaw` 度 + 30+ public 字段 | `attitude_state_t`（quat/bias/valid/rest） | 欧拉角共享一份；漏填字段编译期报错 |
| 世界系 yaw 任意性 | 无处理 | 产品层航向归零 | 6 轴 yaw 不可观测，这是"算法可替换"的前提 |
| 装配方向 | INS 层 `switch` 手写 8 方向 | `Module/Config/imu_mount.h` 表 + 逆表导出 | 已是当前状态（P3），参考做法与我们一致 |
| 驱动耦合 | `pyro_ins.h` include `BMI088_driver.h` + FreeRTOS | 接口头零 RTOS/驱动依赖 | 通信层只需快照头 |
| 坐标系 | `_b` / `_n` 双套（`_b` 未实现） | 按物理意义保留唯一有意义的一套 | 消除冗余与实现歧义 |

---

*本规划基于仓库当前状态（2026-11-16）核对：`imu_service_data()` / `imu_service_attitude()` 全工程零调用者；`g_norm` 仅被 `calibrate()` 写入、从未读取；`vqf_us_max` 实测 940–941 µs。执行前若源码有变动，请以 grep 结果重新定位。*
