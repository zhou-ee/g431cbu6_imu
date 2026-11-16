# IMU 模块改造规划书：P3（装配方向上移）+ 航向归零

| 项 | 内容 |
|---|---|
| 目标工程 | `IMU_Module`（STM32G431CBU6 / Cortex-M4F / FreeRTOS / LSM6DSV16X + VQF） |
| 改造范围 | `PYRo/Component/IMU`、`PYRo/Algorithm` 不改、`Module` |
| 是否改变对外行为 | 第 1 步改变（仅 yaw 通道）；第 2、3 步**不应**改变任何数值 |
| 前置条件 | 当前 `P1` 固件（`direct` 由驱动逐样本应用）已上板测试通过 |

---

## 0. 执行记录（2026-11-16）

**状态：第 1、2、4 步已完成并通过上板验证；第 3 步按计划未做（通信层尚未实现）。**

### 改动落点

| 文件 | 改动 | 行数 |
|---|---|---|
| `Module/Config/imu_mount.h` | **新增**：`imu_direct_t`、`k_imu_direct`、`mount_indices()`、`mount_apply()`、`mount_quat_inverse()`、`quat_to_install()`（头文件 `inline`） | +137 |
| `PYRo/Component/IMU/imu_base_drv.h` | 删除 `imu_direct_t` 与 `imu_cfg_t::direct`；注释标注"器件坐标系" | −25 / +8 |
| `PYRo/Component/IMU/imu_base_drv.cpp` | 删除 `calibrate()` 中 4 行 `previous_direct` 保存/恢复 | −7 / +3 |
| `PYRo/Component/IMU/lsm6dsv16x/lsm6dsv16x_drv.cpp` | 删除 `direct_indices()` / `remap_axis()` 及 3 处调用 | −47 |
| `Module/IMU/imu_service.h` | `imu_attitude_t` 增 `heading_zeroed` / `heading_quality`；增 2 个调试量 | +12 |
| `Module/IMU/imu_service.cpp` | 航向归零 + 装配归一化接入 | +132 / −20 |
| `Module/CMakeLists.txt` | 包含路径加 `Config` | +1 |
| `Module/PROTOCOL.md`、`README.md` | 数据路径与航向零点 | +59 / −24 |

### 验证结果

**① 主机数学自检**（覆盖 `imu_mount.h` 全部新增数值代码，8 方向 × 200 组随机向量/四元数）

| 检查 | 结果 |
|---|---|
| `m_inv` 为单位四元数 | ✓ |
| `rotate(m, v) == mount_apply(v)` | ✓ 证明 `rotation_to_quat` 与 3×3 构造正确 |
| 正向 → 逆向往返恒等 | ✓ 证明逆表由正向表导出正确 |
| `rotate(quat_to_install(q), mount_apply(v)) == rotate(q, v)` | ✓ **姿态换系与向量换系同源** |
| 归零后 yaw=0、roll/pitch 不变、范数不变（2 万组四元数） | ✓ |

**② 上板验证**（STM32G431 + LSM6DSV16X，Release 构建）

| 项 | 结果 |
|---|---|
| 航向归零触发 | round 1010 建立，`heading_zeroed=1`、`heading_quality=1` |
| 归零捕获值 | `yaw0 = −0.1151°`，与按板上倾角（roll 26.8°）手算的最小旋转 yaw −0.1144° 吻合到 0.0007° |
| 归零前后 roll/pitch | 连续无跳变（26.98° → 27.10° → … → 26.87°），yaw 原点被搬移 |
| DIRECT_1（恒等映射） | `rotate(q, accel)` 的世界 Z 分量 == `\|a\|`（差 3e−5）→ 四元数与向量同系 |
| DIRECT_2（换轴 + 取反） | accel / gyro / **bias** 均为 DIRECT_1 的精确轴置换；roll↔pitch 按 90° 平面内安装互换（残差 0.004° / 0.05°）；yaw 仍 ≈ 0 |
| 链路健康 | `err_bus / err_data / err_timeout = 0`；`heap_free_min = 3176 B`（与改造前一致） |
| 发布自洽 | `quat` 独立换算出的 roll/pitch/yaw 与上报值逐位一致 |

**③ 尺寸增量（Release）**：Flash 65040 → 65792 B（**+752**），RAM 20336 → 20392 B（**+56**）。

### 与规划书的偏差

| 偏差 | 原因 |
|---|---|
| 未实现 `mount_map_inverse()` / `apply_mount_inverse()` | 通信层尚未实现。按"不新增无使用者接口"原则略去，改为在 `imu_mount.h` 头部注释写明 Mᵀ 的坑（正向表不可直接复用，DIRECT_2/DIRECT_4 会错） |
| 未引入独立的 `imu_mount_t` 结构体 | 单字段结构体属冗余；将来需要杆臂等参数时再引入 |
| `k_imu_direct` 放在 `imu_mount.h` 而非 `imu_service.cpp` | 将来通信层反向封包需引用同一份配置，避免两处定义 |

### 开机时序实测（2026-11-16 补充，高采样率复测）

用 round 计数作时基（`imu_service_algo_rounds`，1 kHz）重采上电过程：

| 阶段 | round | 现象 |
|---|---|---|
| ① 倾斜未建立 | < 200 | `tilt_initialized=false`，`acc_quat` 为单位阵 → 发布姿态 ≈ (0,0,0)，`valid=false` |
| ② 倾斜初始化 | ≈ 200 | 姿态**阶跃**到真实值，yaw 即解析最小旋转值 |
| ③ ESKF 收敛 | 200→1000 | 亚度级漂移：roll +0.58°，pitch −0.29°，yaw −0.15° |
| ④ rest 观测接入 | ≈ 1000 | rest-bias 观测生效，姿态小幅修正；归零同拍生效 |
| ⑤ 稳定 | > 1100 | 发布 yaw ≈ 0，roll/pitch 稳定 |

**⚠ 更正本文件初版的错误结论。** 初版写"上电后第一秒内 yaw 可摆动到 ±10° 量级"，该结论**错误**。
原因是初版采样时部分 `read_memory` 调用失败，每轮实际间隔未知、无法建立时基，于是把
"初始化前的 0°"与"初始化后的真实 yaw"混在同一序列里，**误将阶跃读成了摆动**。
高采样率复测表明：初始化那一刻的 yaw 就是解析最小旋转值（按当时加速度方向手算 −5.14°，
实测 −5.18°，差 0.04°），**不存在 yaw 暂态**。

### 归零重复性实测（同一姿态 6 次重启）

实测姿态 roll ≈ 42.3°、pitch ≈ −13.1°、|a| ≈ 9.88 m/s²。

| 指标 | 结果 |
|---|---|
| `yaw0`（归零捕获值）散布 | **0.035°**（−5.093° … −5.128°） |
| 发布 yaw 散布 | 0.009° |
| roll / pitch 散布 | 0.067° / 0.016° |
| 手算最小旋转 yaw | −5.06°（与实测一致） |

结论：**归零零点高度可重复，算法与归零逻辑无需修复。**

### 遗留项

1. **`valid=false` 窗口**：上电后约 0.2 s 内模块内部姿态为单位四元数（看起来像"水平"）。
   `imu_attitude_t::valid` 已如实标记；已在 `PROTOCOL.md` 明确"该窗口须按无效规则输出零四元数、
   不可直接封包"。通信层实现时挂上 `valid` 即可——**属协议约定，不是算法缺陷**。
2. **`bias` 与航向自由度的耦合未观察到差异**：DIRECT_1 / DIRECT_2 两次运行的 bias 精确互为轴置换，
   未出现 §2.4 中担心的耦合偏差。
3. **跨姿态的可比性未验证**：以上只验证了**同一姿态**的重复性。不同姿态之间的 yaw 零点差异由
   "六轴航向不可观测"决定，属物理本质而非缺陷；若产品需要绝对航向，只能靠命令归零或磁力计（§8）。
4. **初版"板卡始终平放"的说法有误**：实测姿态是倾斜的（roll 26.9°），只是多次运行之间**姿态未变**。
   本次已在新姿态（roll 42.3°）下完成复测。

---

## 1. 目标与非目标

### 1.1 目标

| 编号 | 目标 | 判定方式 |
|---|---|---|
| G1 | **航向零点由产品显式定义**，不再由开机摆放姿态隐式决定 | 归零时刻 yaw 恰为 0，且 `heading_zeroed` 状态位可观测 |
| G2 | **器件驱动不再感知板级装配参数**，其输出契约收缩为"器件坐标系的物理量" | `PYRo` 全目录搜索 `imu_direct_t` 零命中 |
| G3 | 8 方向映射表在 MCU 上**只存在一份**，且正/逆表由同一真源导出 | 新增 `Module/Config/imu_mount.h` 为唯一实现 |
| G4 | 归一化与归零发生**同一处**（快照写入点），对外单一口径 | `imu_attitude_t` 全字段同坐标系、同零点 |

### 1.2 非目标（本次不做）

- **不引入算法层抽象基类**（`attitude_algo_base`）。换算法的抽象是独立议题，与本次改造正交，避免把两件事混在一起无法归因。
- **不实现通信层**（CAN/UART 协议 v1）。反向封包所需的 `mount` 参数在本次由 `Module/Config/imu_mount.h` 的编译期常量提供（见 §5 第 3 步的说明）。
- **不引入运行期参数**。安装方向仍为编译期常量。
- **不改 `PYRo/Algorithm/VQF`**。VQF 源码零改动。

---

## 2. 改造前后的数据流

### 2.1 现状（P1）

```text
器件
 └→ lsm6dsv16x_drv_t::read()
      ├ 单位换算 + 减器件零偏
      ├ apply direct（【逐样本】，7680 Hz × 2 路）
      └ push 到 sample_stream
 └→ imu_data_t          ……… 安装坐标系
 └→ vqf_t::update()
 └→ get_state().quat    ……… 安装系 → 世界系
 └→ imu_attitude_t
 └→ 对外（yaw 零点 = 开机摆放姿态，隐式）
```

**问题**：`imu_cfg_t`（器件电气配置）里混入了板级装配量 `direct`；`calibrate()` 必须临时保存/恢复它；8 方向映射表在驱动内实现，换器件要重写一遍。

### 2.2 改造后（P3 + 航向归零）

```text
器件
 └→ lsm6dsv16x_drv_t::read()
      ├ 单位换算 + 减器件零偏
      └ push 到 sample_stream
 └→ imu_data_t          ……… 【器件坐标系】
 └→ vqf_t::update()
 └→ get_state().quat    ……… 器件系 → 世界系
 └──────────────────────────── Module 层，快照写入点 ────────────────────────────
     ① 装配归一化：apply_mount(accel/gyro/bias)、quat ⊗ m⁻¹
     ② 航向归零  ：Rz(-yaw0) ⊗ quat
     ③ 欧拉角从【归零后】的四元数导出
 └───────────────────────────────────────────────────────────────────────────────
 └→ imu_attitude_t      ……… 安装坐标系、yaw 已归零
 └→ 对外
```

### 2.3 等价性依据（决定了本方案成立）

`direct` 覆盖的 8 种方向均为**符号置换矩阵**（行列式经核算全为 +1，均为真旋转）。而 VQF 内部所有运算对符号置换不变：

| VQF 内部操作 | 位置 | 是否与符号置换可交换 |
|---|---|---|
| 3 通道 Butterworth 低通 | `_acc_lp` / `_rest_gyr_lp` / `_rest_acc_lp` | ✅ 逐通道，换轴仅重排通道状态 |
| 逐轴零偏限幅 `bias_clip` | `_correct_rest` 等 | ✅ 逐轴 |
| 模长判据（`norm_error`、振动能量、静止门限、NIS） | `_update_acc` | ✅ 模长 / 平方和 |
| 姿态积分与倾斜初始化 | `_Initialize_tilt` | ✅ 旋转协变 |

因此"驱动端 remap"与"输出端 remap"**数学等价**。

> ⚠ **该等价性只在符号置换下成立，是硬约束。** 若将来支持任意安装角（一般旋转或非正交安装矩阵），逐通道低通不再与旋转可交换、逐轴限幅不再旋转不变，两种放法结果**不同**。该约束必须写在 `imu_mount.h` 头部注释里。

### 2.4 已知的、可预期的行为差异

| 差异 | 量级 | 是否可接受 |
|---|---|---|
| yaw 零点定义改变 | 一个受开机姿态影响的常量 | ✅ 本次的**目的** |
| `accel` / `gyro` 通道 | 逐位相同（remap 只做复制与取负，IEEE754 下精确） | ✅ |
| `quat` / `bias` / `rest` / `acc_confidence` | 浮点舍入级（运算顺序改变） | ✅ 期望 ≤ 1e-6 |
| `initialized` 翻 true 的拍号 | 应完全相同（门控是旋转不变量） | ⚠ 必须实测确认 |
| `bias` 的长期收敛行为 | 理论上一致（零偏耦合的是不可观的航向自由度） | ⚠ **必须实测确认，不能仅靠推理** |

---

## 3. 术语与约定

| 记号 | 含义 |
|---|---|
| 器件系 | LSM6DSV16X 的 X/Y/Z，与原码一一对应 |
| 安装系 | 产品定义的参考系，`direct` 描述器件系到安装系的映射 |
| `M` | 器件系 → 安装系的映射，`v_inst = M · v_dev` |
| `m` | `M` 对应的单位四元数 |
| 正向映射 | `out[j] = s[j] · in[p[j]]`，由 `(idx, neg)` 描述 |
| 逆向映射 | `out[i] = s[p⁻¹(i)] · in[p⁻¹(i)]`，即 `Mᵀ` |
| `yaw0` | 归零时刻的 yaw，归零后 `yaw_pub(t) = yaw(t) − yaw0` |
| P1 / P3 | 装配方向归一化的两个落点：P1 = 驱动内（逐样本）；P3 = 快照处（逐发布） |

---

## 4. 步骤总览

| 步 | 内容 | 涉及文件 | 改变行为 | 验收 |
|---|---|---|---|---|
| 0 | 基线记录 | — | — | 存档尺寸与录波 |
| **1** | **航向归零**（落在现有 P1 上） | `Module/IMU/imu_service.h`、`.cpp` | ✅ 仅 yaw 通道 | §6.1 |
| **2** | **P3 重构** | `PYRo`×3、新增 `Module/Config/imu_mount.h`、`Module/IMU/imu_service.cpp`、`Module/CMakeLists.txt` | ❌ 不应改变 | §6.2 |
| 3 | 统一出口（可选，通信层落地时做） | 新增 `Module/IMU/imu_snapshot.h`、`imu_service.h/.cpp` | ❌ | §6.3 |
| 4 | 文档同步 | `Module/PROTOCOL.md`、`README.md` | — | — |

**为什么归零先做**：归零是本次唯一的**行为变更**；把它单独落在已上板验证过的 P1 上，异常可立即归因。第 2 步随后应表现为纯重构（零数值差异），此时任何差异都是真 bug。

---

## 5. 分步实施明细

### 第 0 步：基线记录

1. 记录当前 Debug / Release 的 Flash / RAM 尺寸（对照 `README.md` 第 72-77 行）。
2. 录一次基准波形并**连同固件 hash 一起存档**，用于第 2 步逐字段比对。建议场景：
   - 场景 A：按产品正常姿态平放上电，静止 60 s
   - 场景 B：倾斜约 30° 上电，静止 30 s 后做一次三轴转动
3. 记录 `imu_service_*` 全部调试量的起始值。

> 场景 B 是必需的：平放开机时 P1 的 yaw 本来就是 0（见 §7 风险 R1），只有倾斜开机才能显出零点差异。

---

### 第 1 步：航向归零（在现有 P1 上）

**改动文件：`Module/IMU/imu_service.h`、`Module/IMU/imu_service.cpp`**

#### 1-A `imu_service.h` —— `imu_attitude_t` 增字段（第 23-33 行）

```cpp
struct imu_attitude_t
{
    float quat[4];     /* [w,x,y,z] */
    float bias[3];     /* 陀螺零偏估计，rad/s */
    float roll;        /* ° */
    float pitch;       /* ° */
    float yaw;         /* °，已按产品零点归零 */
    float rest_t;      /* 连续静止计时，s */
    bool  rest;        /* 当前判定为静止 */
    bool  valid;       /* VQF 已完成可信重力方向初始化 */

    /* ---- 新增 ---- */
    bool    heading_zeroed;   /* yaw 已由产品零点定义；false 时 yaw 无产品含义 */
    uint8_t heading_quality;  /* 0=未建立 1=静止且零偏收敛 2=超时兜底 */
};
```

#### 1-B `imu_service.h` —— 调试观测增 2 个（第 51-80 行的 `extern "C"` 块内）

```cpp
extern volatile uint32_t imu_service_heading_zeroed;  /* 1=yaw 零点已建立 */
extern volatile uint32_t imu_service_heading_quality; /* 0/1/2 */
```

#### 1-C `imu_service.h` —— 文件头注释（第 1-12 行）

在头部说明中补一句：`yaw 为产品零点；heading_zeroed=false 期间 yaw 不可用`。

#### 1-D `imu_service.cpp` —— 调试量定义（第 53-81 行的 `extern "C"` 块内）

```cpp
volatile uint32_t imu_service_heading_zeroed  = 0U;
volatile uint32_t imu_service_heading_quality = 0U;
```

#### 1-E `imu_service.cpp` —— 归零门控常量（插在 `k_vqf_cfg` 之后，即第 118 行之后）

```cpp
/* ======================= 航向归零门控 =======================
 * 建立零点需要三件事：
 *   1) state.initialized  —— 可信重力方向已建立，四元数才有意义；
 *   2) state.rest_detected —— 已含 rest_min_t 的连续静止确认（VQF 内部判定）；
 *   3) bias_sigma 低于门限 —— 该量是零偏协方差三轴的上界，而竖直轴只在静止观测下可观测，
 *      也正是产生 yaw 漂移的那一轴。未收敛就归零，零点会带上截至该时刻的累积漂移，
 *      且后续静止观测会追溯修正 yaw 误差，使发布 yaw 在收敛过程中继续移动。
 *      （零偏收敛后漂移即停止，并非长期漂移。）
 * 三者缺一不可；再加超时兜底，避免产品在运动中被开机时永远不输出可信 yaw。 */
constexpr float    k_bias_sigma_ok_rps  = 0.005f;   /* rad/s，≈0.29 °/s */
constexpr uint32_t k_zero_timeout_round = 5000U;    /* 5 s @1 kHz */
```

> ⚠ 单位陷阱：`vqf_t::state_t::bias_sigma` 是 **rad/s**（`pyro_algo_vqf.cpp:130` 用 `bias_sigma_init * deg_to_rad` 建协方差，`:630` 取 `sqrt`），而 `vqf_config_t::bias_sigma_init` 是 **°/s**。门限别搞错量纲。

#### 1-F `imu_service.cpp` —— 归零实现（插在 `quat_to_euler` 之后，即第 159 行之后）

```cpp
/* ======================= 航向归零 =======================
 * 语义：只把 yaw 归零，roll/pitch 不动（重力参考不丢）。
 *   ❌ q_out = q0⁻¹ ⊗ q  —— 会把整个姿态打平，产品失去"水平"概念
 *   ✅ q_out = Rz(-yaw0) ⊗ q —— 左乘纯偏航，roll/pitch 逐项不变
 * 成立前提：欧拉角采用 ZYX 约定（Rz(yaw)Ry(pitch)Rx(roll)），
 *          此时左乘 Rz 恰好只加在 yaw 上。
 * 位置：纯输出端后处理，【不】修改 VQF 内部状态——内部 acc_reference /
 *       last_acc_lp 活在 gyr_quat 张成的近惯性系里，动内部姿态会破坏门控历史。
 * 失效：VQF 重建时间基准（reset / tick_s 改变）后世界系重建，必须重新归零。 */
namespace {

struct heading_ref_t
{
    float   yaw0    = 0.0f;   /* 归零时刻的 yaw，rad */
    bool    zeroed  = false;
    uint8_t quality = 0U;
};

heading_ref_t s_heading{};
uint32_t      s_zero_wait = 0U;

/* ZYX 偏航角（rad）。与 quat_to_euler 用同一公式，保持唯一真源。 */
float quat_yaw(const float q[4])
{
    return std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

/* 左乘 Rz(-yaw0)，只改 yaw 分量。 */
void apply_heading_zero(float q[4], float yaw0)
{
    const float h = 0.5f * yaw0;
    const float c = std::cos(h), s = std::sin(h);
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    q[0] = c * w + s * z;
    q[1] = c * x + s * y;
    q[2] = c * y - s * x;
    q[3] = c * z - s * w;
}

} /* namespace */
```

#### 1-G `imu_service.cpp` —— `run_loop` 快照写入段（现第 250-288 行）

现结构：

```cpp
/* ---- ④ 姿态快照 ---- */
float q[4];
float sq = 0.0f;
for (uint8_t i = 0U; i < 4U; ++i) { q[i] = state.quat[i]; sq += q[i] * q[i]; }

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
quat_to_euler(q, roll, pitch, yaw);

imu_attitude_t next{};
...
```

改为：

```cpp
/* ---- ④ 姿态快照 ---- */
float q[4];
float sq = 0.0f;
for (uint8_t i = 0U; i < 4U; ++i) { q[i] = state.quat[i]; sq += q[i] * q[i]; }

/* ④-1 建立航向零点（一次性，当拍即生效） */
if (!s_heading.zeroed)
{
    const bool settled = state.initialized && state.rest_detected &&
                         (state.bias_sigma < k_bias_sigma_ok_rps);
    const bool fallback = (++s_zero_wait >= k_zero_timeout_round) && state.initialized;
    if (settled || fallback)
    {
        s_heading.yaw0    = quat_yaw(q);
        s_heading.zeroed  = true;
        s_heading.quality = settled ? 1U : 2U;
    }
}

/* ④-2 归零。放在欧拉角导出【之前】，保证 quat 与 euler 同源 */
if (s_heading.zeroed)
{
    apply_heading_zero(q, s_heading.yaw0);
}

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
quat_to_euler(q, roll, pitch, yaw);

imu_attitude_t next{};
/* …原有赋值保持不变… */
next.heading_zeroed  = s_heading.zeroed;                 /* 新增 */
next.heading_quality = s_heading.quality;                /* 新增 */
```

> 因为 `s_heading.yaw0` 取自**同一拍**的 `q`，归零那一拍的输出 yaw 恰好为 0（不是"接近 0"）。

#### 1-H `imu_service.cpp` —— 调试镜像同步（现第 276-288 行）

在调试镜像块内追加：

```cpp
imu_service_heading_zeroed  = s_heading.zeroed ? 1U : 0U;
imu_service_heading_quality = s_heading.quality;
```

#### 1-I VQF 重建点的配套清理

当前代码尚未调用 `s_vqf.reset()`（`README.md` 第 129 行规划在时间戳重启时调用）。**将来加 reset 时必须同步清零**，否则旧 `yaw0` 会作用在新世界系上：

```cpp
s_vqf.reset(...);
s_heading   = heading_ref_t{};   /* ← 与 reset 成对出现 */
s_zero_wait = 0U;
```

---

### 第 2 步：P3 重构

#### 2.1 新增 `Module/Config/imu_mount.h`

唯一真源：方向表、逆向表、向量换系、四元数换系。**头文件内 `inline` 实现**，不新增 .cpp，供 `Module/IMU` 与将来 `Module/Communication` 共用。

```cpp
#ifndef IMU_MOUNT_H
#define IMU_MOUNT_H

#include <cmath>
#include <cstdint>

/* ============================================================================
 * 安装方向：板级装配参数
 *
 * 【归属】产品装配层。器件驱动不感知本文件——驱动的输出契约是"器件坐标系
 *         的物理量（单位换算 + 器件零偏已扣除）"，不感知产品如何装配。
 *
 * 【为什么可以搬到输出端】8 种方向均为符号置换矩阵（行列式全为 +1）。
 *         符号置换与 VQF 内部全部运算可交换：逐通道 Butterworth 低通、
 *         逐轴零偏限幅、模长/NIS 判据。因此"驱动端 remap（逐样本）"与
 *         "输出端 remap（逐发布）"数学等价，只是浮点舍入路径不同。
 *
 * 【硬约束】该等价性【只在符号置换下成立】。一旦支持任意安装角（一般旋转，
 *         或非正交的安装标定矩阵），逐通道低通不再与旋转可交换、逐轴限幅
 *         不再旋转不变，两种放法结果【不同】。届时必须重新评估算法输入
 *         坐标系，而不是简单扩展本文件。
 * ========================================================================== */

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

struct imu_mount_t {
    imu_direct_t direct = imu_direct_t::NONE;
};

/* ---- 正向：安装系[j] = neg[j] ? -器件系[idx[j]] : +器件系[idx[j]] ----
 * 与 PYRo INS 的 DIRECT_1..8 约定一致。
 * 实现自 lsm6dsv16x_drv.cpp 的 direct_indices() 原样搬运（原为文件局部函数）。 */
inline void mount_map(imu_direct_t d, uint8_t idx[3], bool neg[3])
{
    idx[0] = 0U; idx[1] = 1U; idx[2] = 2U;
    neg[0] = false; neg[1] = false; neg[2] = false;

    switch (d) {
        case imu_direct_t::DIRECT_2: idx[0] = 1U; idx[1] = 0U; neg[1] = true; break;
        case imu_direct_t::DIRECT_3: neg[0] = true; neg[1] = true; break;
        case imu_direct_t::DIRECT_4: idx[0] = 1U; idx[1] = 0U; neg[0] = true; break;
        case imu_direct_t::DIRECT_5: neg[1] = true; neg[2] = true; break;
        case imu_direct_t::DIRECT_6: idx[0] = 1U; idx[1] = 0U; neg[0] = true; neg[1] = true; neg[2] = true; break;
        case imu_direct_t::DIRECT_7: neg[0] = true; neg[2] = true; break;
        case imu_direct_t::DIRECT_8: idx[0] = 1U; idx[1] = 0U; neg[2] = true; break;
        case imu_direct_t::DIRECT_1:
        case imu_direct_t::NONE:
        default: break;   /* 单位映射 */
    }
}

/* ---- 逆向：器件系[i] = neg[i] ? -安装系[idx[i]] : +安装系[idx[i]] ----
 * ⚠ 必须由正向表【导出】，不能直接把正向结果再用一遍。
 *
 *   正向：out[j] = s[j] · in[p(j)]
 *   逆向：out[i] = s[p⁻¹(i)] · in[p⁻¹(i)]      ← 负号跟着 p⁻¹ 走
 *
 * 反向就是 Mᵀ。对 DIRECT_2 与 DIRECT_4 这类"换轴 + 非对称取反"的方向，
 * 照搬正向 (idx,neg) 会算错（s[0] ≠ s[1] 时符号位置不对）。
 * 由同一张表导出可保证两者永不漂移。 */
inline void mount_map_inverse(imu_direct_t d, uint8_t idx[3], bool neg[3])
{
    uint8_t fp[3];
    bool    fs[3];
    mount_map(d, fp, fs);
    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            if (fp[j] == i) { idx[i] = j; neg[i] = fs[j]; break; }
        }
    }
}

/* ---- 热路径：就地换系（参数在初始化时算好并缓存，这里只做复制与取负） ---- */
inline void apply_mount(float v[3], const uint8_t idx[3], const bool neg[3])
{
    const float t[3] = { v[idx[0]], v[idx[1]], v[idx[2]] };  /* 先取出，避免原地覆盖 */
    v[0] = neg[0] ? -t[0] : t[0];
    v[1] = neg[1] ? -t[1] : t[1];
    v[2] = neg[2] ? -t[2] : t[2];
}

/* ---- 3x3 旋转矩阵 → 单位四元数 [w,x,y,z]（Shepperd 分支法，数值稳定） ----
 * 只在初始化时调用一次。 */
inline void rotation_to_quat(const float R[3][3], float q[4])
{
    const float tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0f) {
        const float s = std::sqrt(tr + 1.0f) * 2.0f;
        q[0] = 0.25f * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        const float s = std::sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25f * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        const float s = std::sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25f * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        const float s = std::sqrt(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25f * s;
    }
}

/* ---- 初始化期：由同一张正向表构造 m 的逆四元数并缓存 ----
 * 姿态换系：q_inst = q_dev ⊗ m⁻¹。
 * 推导：v_w = q_dev ⊗ v_dev ⊗ q_dev*，而 v_dev = m⁻¹ ⊗ v_inst ⊗ m，
 *       故 v_w = (q_dev ⊗ m⁻¹) ⊗ v_inst ⊗ (q_dev ⊗ m⁻¹)*  ⇒ q_inst = q_dev ⊗ m⁻¹。
 * 注意是【右乘】。 */
inline void mount_inverse_quat(imu_direct_t d, float m_inv[4])
{
    uint8_t idx[3];
    bool    neg[3];
    mount_map(d, idx, neg);

    float R[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    for (uint8_t i = 0U; i < 3U; ++i) {
        R[i][idx[i]] = neg[i] ? -1.0f : 1.0f;   /* out_i = s_i · v_{idx[i]} */
    }

    float m[4];
    rotation_to_quat(R, m);
    m_inv[0] =  m[0];   /* 单位四元数的逆 = 共轭 */
    m_inv[1] = -m[1];
    m_inv[2] = -m[2];
    m_inv[3] = -m[3];
}

/* ---- 热路径：姿态换系，右乘缓存的 m⁻¹（16 次乘法） ---- */
inline void quat_to_install(float q[4], const float m_inv[4])
{
    const float aw = q[0], ax = q[1], ay = q[2], az = q[3];
    const float bw = m_inv[0], bx = m_inv[1], by = m_inv[2], bz = m_inv[3];
    q[0] = aw * bw - ax * bx - ay * by - az * bz;
    q[1] = aw * bx + ax * bw + ay * bz - az * by;
    q[2] = aw * by - ax * bz + ay * bw + az * bx;
    q[3] = aw * bz + ax * by - ay * bx + az * bw;
}

#endif /* IMU_MOUNT_H */
```

#### 2.2 `PYRo/Component/IMU/imu_base_drv.h`

| 位置 | 现状 | 改动 |
|---|---|---|
| 第 15-25 行 | `enum class imu_direct_t : uint8_t {…}` | **整段删除**（移至 `Module/Config/imu_mount.h`） |
| 第 35 行 | `imu_direct_t direct = imu_direct_t::NONE;` | **删除**（连同行尾注释） |
| 第 7 行 | `float Accel[3]; /* 单位 m/s² */` | 注释改为 `/* 器件坐标系，单位 m/s² */` |
| 第 12 行 | `float Gyro[3];` | 注释改为 `/* 器件坐标系，单位 rad/s */` |
| 第 105 行附近 | `struct imu_data_t {` | 头部注释补充：**"物理量均为器件坐标系；装配方向映射由产品装配层在输出端完成"** |
| 第 45-51 行 | `sample_stream_t` 文档 | 不改 |

改完后 `imu_cfg_t` 的字段（第 27-36 行）只剩纯电气配置：`calibrate_flag` / `calibrate_times` / `acc_range` / `acc_freq` / `gyro_range` / `gyro_freq` / `fifo_en`。

#### 2.3 `PYRo/Component/IMU/imu_base_drv.cpp`

| 位置 | 现状 | 改动 |
|---|---|---|
| 第 19 行 | `const imu_direct_t previous_direct = _data.cfg.direct;` | **删除** |
| 第 25 行 | `_data.cfg.direct = imu_direct_t::NONE;` | **删除** |
| 第 66 行 | `_data.cfg.direct = previous_direct;` | **删除**（停滞保护分支内） |
| 第 77 行 | `_data.cfg.direct = previous_direct;` | **删除**（成功后恢复） |
| 第 21 行 | `/* 偏置存储在传感器原坐标系。标定读取未扣偏置、未换轴的新样本。 */` | 改为 `/* 偏置存储在传感器原坐标系。驱动的输出契约即为器件坐标系，标定天然读到的就是未扣偏置的新样本。 */` |

> 这 4 行删除是"驱动不该知道装配方向"最直接的收益：`calibrate()` 不再需要为了拿传感器系样本而临时篡改配置项。删除后 `_data.cfg` 在标定期间完全不被修改，`previous_direct` 的保存/恢复逻辑整体消失。

#### 2.4 `PYRo/Component/IMU/lsm6dsv16x/lsm6dsv16x_drv.cpp`

| 位置 | 现状 | 改动 |
|---|---|---|
| 第 32 行 | `void direct_indices(imu_direct_t d, uint8_t idx[3], bool neg[3]);` | **删除**（前向声明） |
| 第 185-187 行 | `uint8_t idx[3] = …; bool neg[3] = …; direct_indices(_data.cfg.direct, idx, neg);` | **删除**（`read_direct()` 内） |
| 第 200 行 | `remap_axis(g.Gyro, idx, neg);` | **删除** |
| 第 209 行 | `remap_axis(a.Accel, idx, neg);` | **删除** |
| 第 321-324 行 | `idx/neg` 局部量 + `direct_indices(_data.cfg.direct, idx, neg);` | **删除**（`fifo_parse_word()` GYRO 分支） |
| 第 324 行 | `remap_axis(s.Gyro, idx, neg);` | **删除** |
| 第 338-341 行 | `idx/neg` 局部量 + `direct_indices(…)` | **删除**（`fifo_parse_word()` XL 分支） |
| 第 341 行 | `remap_axis(s.Accel, idx, neg);` | **删除** |
| 第 430-448 行 | `namespace { void direct_indices(…) {…} }` 及其小节注释 | **整段删除** |
| 第 450-457 行 | `void remap_axis(…) {…}` 及其小节注释 | **整段删除** |

删除后该文件的三条解包路径统一为"单位换算 → 减零偏 → push 到流"，不再有任何坐标系变换。

`lsm6dsv16x_drv.h` **不改**（构造函数仍收 `imu_cfg_t`）。

#### 2.5 `Module/IMU/imu_service.cpp`

**(a) 配置区（第 88-100 行）**

```cpp
const imu_cfg_t k_imu_cfg = {
    false,                  /* calibrate_flag */
    500U,                   /* calibrate_times */
    4U,                     /* acc_range  ：±4 g */
    1920U,                  /* acc_freq   ：1920 Hz */
    2000U,                  /* gyro_range ：±2000 dps */
    1920U,                  /* gyro_freq  ：1920 Hz */
    true,                   /* fifo_en */
};                          /* ← 删除原第 99 行的 imu_direct_t::DIRECT_1 */
```

**(b) 新增装配配置（紧随其后）**

```cpp
/* ======================= 产品装配参数 =======================
 * 安装方向属板级装配，归产品层。驱动不再感知它，映射在快照写入点完成。
 * 换方向只改这一行。 */
constexpr imu_direct_t k_imu_direct = imu_direct_t::DIRECT_1;
```

**(c) 装配区注释（第 120-132 行）**

把"换 SPI/I²C 只需换这一行"的注释保留，并补一句：装配方向见 `k_imu_direct`。

**(d) 初始化期缓存（`imu_task_t::init()` 内，`s_vqf.configure()` 之后，约第 186 行）**

```cpp
/* 装配归一化参数只算一次：热路径只做复制/取负/16 次乘法 */
mount_map(k_imu_direct, s_mount_idx, s_mount_neg);
mount_inverse_quat(k_imu_direct, s_m_inv);
```

配套静态量（放在 `s_vqf` 定义之后，约第 132 行）：

```cpp
uint8_t s_mount_idx[3] = {0U, 1U, 2U};
bool    s_mount_neg[3] = {false, false, false};
float   s_m_inv[4]     = {1.0f, 0.0f, 0.0f, 0.0f};
```

**(e) 调试镜像改为同源（第 207-221 行）**

现状：直接把 `cur()` 的值写进 `imu_service_gyro_rps` / `imu_service_accel_mps2`。
改为：先存局部量，**在归一化之后**再写全局，保证调试量与发出去的快照同坐标系。

```cpp
float last_gyro_dev[3] = {0.0f, 0.0f, 0.0f};
float last_acc_dev[3]  = {0.0f, 0.0f, 0.0f};
if (ng != 0U) { const gyro_sample_t &g = data.gyro.cur();
                last_gyro_dev[0] = g.Gyro[0]; /* …1,2… */ }
if (na != 0U) { const acc_sample_t  &a = data.acc.cur();
                last_acc_dev[0]  = a.Accel[0]; /* …1,2… */ }
```

**(f) 快照写入段（现第 250-288 行）整体替换为**

```cpp
/* ---- ④ 姿态快照 ---- */
const pyro::vqf_t::state_t &state = …;   /* 沿用已有的 state */

/* ④-1 装配归一化：器件系 → 安装系
 * 换系后再归零，因为航向零点是【安装系】下的产品定义。 */
float q[4] = {state.quat[0], state.quat[1], state.quat[2], state.quat[3]};
quat_to_install(q, s_m_inv);

float gyro_inst[3] = {last_gyro_dev[0], last_gyro_dev[1], last_gyro_dev[2]};
float acc_inst[3]  = {last_acc_dev[0],  last_acc_dev[1],  last_acc_dev[2]};
float bias_inst[3] = {state.bias[0], state.bias[1], state.bias[2]};
apply_mount(gyro_inst, s_mount_idx, s_mount_neg);
apply_mount(acc_inst,  s_mount_idx, s_mount_neg);
apply_mount(bias_inst, s_mount_idx, s_mount_neg);   /* ← 零偏也是机体系向量，最容易漏 */

/* ④-2 航向归零（同第 1 步，位置不变，只是输入已换成安装系四元数） */
if (!s_heading.zeroed) { /* …同 §1-G… */ }
if (s_heading.zeroed)  { apply_heading_zero(q, s_heading.yaw0); }

/* ④-3 欧拉角必须从【换系 + 归零之后】的四元数导出。
 * 欧拉角不是线性量，不能先算再换系。 */
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
quat_to_euler(q, roll, pitch, yaw);

/* ④-4 写快照 */
imu_attitude_t next{};
for (uint8_t i = 0U; i < 4U; ++i) { next.quat[i] = q[i]; }
for (uint8_t i = 0U; i < 3U; ++i) { next.bias[i] = bias_inst[i]; }
next.roll = roll; next.pitch = pitch; next.yaw = yaw;
next.rest_t = state.rest_t;
next.rest   = state.rest_detected;
next.valid  = state.initialized;
next.heading_zeroed  = s_heading.zeroed;
next.heading_quality = s_heading.quality;

++s_attitude_seq;
s_attitude = next;
++s_attitude_seq;

/* ④-5 调试镜像：与快照同源 */
for (uint8_t i = 0U; i < 4U; ++i) { imu_service_quat[i] = q[i]; }
for (uint8_t i = 0U; i < 3U; ++i) {
    imu_service_bias_rps[i]  = bias_inst[i];
    imu_service_gyro_rps[i]  = gyro_inst[i];
    imu_service_accel_mps2[i]= acc_inst[i];
}
imu_service_quat_norm = …;   /* 归零后仍是单位四元数，保持原算法 */
/* …其余调试量保持原样… */
```

**归一化清单（漏一个就会真的不一致）：**

```text
quat       → quat_to_install（右乘 m⁻¹）
accel      → apply_mount
gyro       → apply_mount
bias       → apply_mount        ← 最容易漏
roll/pitch/yaw → 必须从换系后的 quat 导出
调试镜像   → 与快照同源
将来 velocity/position（世界系）→ 【不】换系
```

**(g) 头文件 `/ 文件头注释`

顶部架构说明（第 1-37 行）补一节"装配归一化与航向零点"，把 §2.3 的可交换性依据与约束写进去。

#### 2.6 `Module/CMakeLists.txt`

第 5-7 行：

```cmake
target_include_directories(IMUProduct PUBLIC
        Application
        IMU
        Config)          # ← 新增
```

`imu_mount.h` 为头文件，无需加入源列表。`PYRo/CMakeLists.txt` **不改**（`Component/IMU` 已在包含路径中；本步只做删除）。

---

### 第 3 步：统一出口（可选，通信层落地时执行）

**当前不必做。** 通信层尚未实现，而安装方向是编译期常量，通信层届时直接
`#include "imu_mount.h"` 引用同一个 `k_imu_direct` 即可完成反向封包。

当**运行期参数**（`README.md` 第 22、134 行规划）落地后，`mount` / 量程 / 灵敏度
需要在快照里随样本一起冻结，此时再新增 `Module/IMU/imu_snapshot.h`：

```cpp
struct imu_measurement_t { float accel_mps2[3]; float gyro_rps[3]; float temperature;
                           uint32_t acc_timestamp; uint32_t gyro_timestamp; };
struct imu_params_t      { imu_cfg_t cfg; imu_cali_t cali; imu_mount_t mount;
                           float acc_mps2_per_lsb; float gyro_rps_per_lsb; float timestamp_tick_s; };
struct imu_snapshot_t    { imu_attitude_t attitude; imu_measurement_t measurement;
                           imu_params_t params; uint32_t sequence; };
imu_snapshot_t imu_service_snapshot();   /* 唯一跨任务 API，值返回 + seqlock */
```

> ⚠ 届时注意：陀螺灵敏度 `gyro_rps_per_lsb` 目前是 `lsm6dsv16x_drv_t` 的 **private** 成员
> （`lsm6dsv16x_drv.h` 第 54 行 `_gyro_dps_per_lsb`），通信层取不到；且**不能**用
> `cfg.gyro_range / 32768` 反推——±2000 dps 按量程算是 0.06104 °/s/LSB，而数据手册典型值
> 是 0.070 °/s/LSB，差 14.7%。它必须进入通用器件契约。

---

### 第 4 步：文档同步

#### 4.1 `Module/PROTOCOL.md`

| 位置 | 现状 | 改为 |
|---|---|---|
| 第 13 行 | "加速度和角速度从两路最新解包物理量反向恢复为器件 X/Y/Z 轴的 int16 输出码" | 保持（结论不变，但说明来源是快照的安装系统一量） |
| 第 15 行 | "航向参考由初始化确定" | "**航向零点由产品归零建立**（模块上电后建立，或由上位机命令建立）；`heading_zeroed=false` 期间 yaw 无产品含义" |
| 第 35-42 行 | "驱动正向处理为单位换算、静态偏置校正、**安装方向映射**" | 安装方向映射**移到快照层**；驱动只做单位换算 + 静态偏置校正，输出器件坐标系 |
| 第 77-78 行 | "原始值的坐标系为器件 X/Y/Z，四元数沿用产品配置的安装坐标系" | 说明模块内部快照已统一为安装系；线上原始码由快照"先逆安装方向"恢复，公式不变 |
| §四元数编码与原始样本封包 | — | 补一段"航向零点"说明，含 `heading_quality` 语义 |

**不变的部分**：第 43-64 行的反向公式 `raw = (Mᵀ·gyro_installed + b) / Sg` **一字不改**。因为快照仍是安装系，且 `Mᵀ` 本来就在 `+b` 之前。主机侧 34603008 次往返校核继续有效。

#### 4.2 `README.md`

| 位置 | 改动 |
|---|---|
| 第 35 行（目录职责表） | `PYRo/Component/IMU`：改为"IMU 数据结构、标定与 LSM6DSV16X 驱动（**输出器件坐标系**）" |
| 第 41-49 行（`Module/` 目录树） | 增加 `Config/  板级装配参数（安装方向）` |
| 第 51 行（依赖方向） | 保持 `Module → PYRo`；补充"装配方向映射位于 `Module`，PYRo 不感知" |
| 第 86-91 行（数据流第 2、3 步） | 第 2 步"配置标定与零偏估计策略"之后补"缓存装配归一化参数"；新增一步"快照写入前完成装配归一化与航向归零" |
| 第 94 行（内部数据约定） | 补充"对外数据统一为安装坐标系；yaw 零点由归零建立" |
| 第 134 行（Flash 与参数） | "安装方向由编译期产品配置提供"改为"位于 `Module/Config/imu_mount.h`" |

---

## 6. 验收判据

### 6.1 第 1 步（航向归零）

| 检查 | 期望 |
|---|---|
| 归零时刻的 yaw | **恰为 0.0**（当拍即生效，非"接近 0"） |
| 归零前后 roll / pitch | 连续，无跳变 |
| 归零前后 `bias` / `rest` / `rest_t` / `acc_confidence` | 完全不变 |
| `heading_zeroed` / `heading_quality` | 平放静止上电 → 数秒内 1 / 1；运动上电 → 5 s 后 1 / 2 |
| 倾斜 30° 上电（场景 B） | 归零前 yaw ≈ 某个非零值；归零后为 0 |
| `quat` 的范数 | 归零后仍为 1（±1e-6） |
| `quat` 与 `yaw` 的自洽 | 对 `quat` 独立算 ZYX yaw，应等于上报 `yaw` |

### 6.2 第 2 步（P3 重构）—— 与第 1 步基线逐字段比对

| 通道 | 判据 | 不等时的含义 |
|---|---|---|
| `accel` / `gyro` | **逐位相同** | remap 是精确操作，不等 = 归一化写错或漏写 |
| `initialized` 翻 true 的拍号 | **完全相同** | 门控是旋转不变量，不等 = 破坏了 VQF 初始化条件 |
| `quat` / `roll` / `pitch` / `yaw` | ≤ 1e-6 | — |
| `bias` | ≤ 1e-6 | **明显不等 → §2.4 那行"零偏与航向自由度耦合"真实存在**，须评估是否保留 P1 |
| `rest` / `rest_t` / `acc_confidence` / `acc_nis` | ≤ 1e-6 | — |
| `heading_zeroed` / `heading_quality` | 相同 | — |
| Flash / RAM | 记录并对照，回填 `README.md` 第 72-77 行 | — |

### 6.3 编译期自检（建议同时加入）

```text
1. PYRo 全目录搜索 "imu_direct_t" / "direct_indices" / "remap_axis" → 零命中
2. 对比 mount_map 与 mount_map_inverse 的输出：
   对 8 个方向，验证 apply_mount → apply_mount_inverse 恒等（往返恢复）
3. 对 8 个方向，验证 quat_to_install 与逐轴 apply_mount 一致：
   取任意向量 v，rotate(q ⊗ m⁻¹, apply_mount(v)) == apply_mount(rotate(q, v))
4. 主机侧：8 方向 × 全 int16 范围的往返校核（可复用 PROTOCOL.md 第 63-64 行
   已有的 34603008 次校核脚本）
```

第 2 项就是上一轮识别出的那个坑（DIRECT_2 / DIRECT_4 的正向表不能直接复用为逆向），
把它变成自检项比写在注释里可靠。

---

## 7. 风险与坑 

| 编号 | 风险 | 说明 | 对策 |
|---|---|---|---|
| R1 | **平放开机时看不出差异** | 平放（安装系重力沿 ±ẑ）时 `_initialize_tilt` 只产生 `I` 或 `Rx(180°)`，两者 ZYX yaw 都是 0。于是 P1 的 yaw 本来就是 0，归零前后"看起来一样" | 验证必须包含**倾斜开机**场景（§5 第 0 步场景 B） |
| R2 | 漏换 `bias` | `bias` 是机体系向量，与 `accel`/`gyro` 同类，但不在"明显是测量量"的直觉里 | 列入 §2.5(f) 清单；回归判据中单列一行 |
| R3 | 先算欧拉角再换系 | 欧拉角不是线性量，`euler(M·v) ≠ M·euler(v)` | 强制顺序：归一化 → 归零 → `quat_to_euler` |
| R4 | 逆向映射照搬正向表 | 反向是 `Mᵀ`；`DIRECT_2` / `DIRECT_4` 这类"换轴 + 非对称取反"照搬会错（`s[0] ≠ s[1]` 时符号位置不对） | `mount_map_inverse` 由正向表导出；§6.3 自检项 2 |
| R5 | 调试量与快照不同源 | 调试器看到的和发出去的不是一个坐标系 | §2.5(e)(g)：调试镜像在归一化之后统一写 |
| R6 | VQF reset 未清零点 | 时间戳重启后世界系重建，旧 `yaw0` 失效 | §1-I：`s_vqf.reset()` 与清零点必须成对 |
| R7 | yaw 回绕 | `atan2` 值域 (−π, π]，yaw 从 +180° 跳到 −180° | MCU 侧不加状态；解卷绕放在上位机（它才能维护连续计数器）。写入 PROTOCOL.md |
| R8 | 单位量纲 | `state.bias_sigma` 是 rad/s，`config.bias_sigma_init` 是 °/s | §1-E 已标注 |
| R9 | 任意安装角 | 一般旋转下"输出端 remap"与"驱动端 remap"**不再等价** | 写进 `imu_mount.h` 头部注释作为硬约束 |
| R10 | 归零在振动环境永不触发 | `rest_detected` 依赖静止判定 | 已有 `quality=2` 超时兜底；长期方案是命令归零或磁力计（见 §8） |

---

## 8. 后续演进（不在本次范围）

| 议题 | 说明 |
|---|---|
| **命令归零** | 协议 v1 现只有 `message_type = 0x01`（测量）。增加 `0x02`（命令）+ `CMD_ZERO_YAW`，把零点定义权交给上位机，获得可重复的航向基准。这是产品化的正解。 |
| **磁力计 / 9 轴** | 航向变为可观测量后，人为归零反而有害。届时用编译期开关（如 `k_heading_zero_enable`）一键关闭本机制。 |
| **算法层抽象** | `attitude_algo_base`（换 VQF / ESKF / Mahony）。与本次改造正交，独立推进。 |
| **运行期参数** | 安装方向、量程、灵敏度改为可在线更新，届时执行第 3 步。 |
| **CAN 发送忙处理** | `README.md` 第 115-116 行已记录该缺陷，属通信层。 |

---

## 9. 回滚

| 步 | 回滚方式 |
|---|---|
| 第 1 步 | 纯 `Module` 层增量；删除 `imu_attitude_t` 的两个新字段与 §1-E/F/G/H 的代码块即可。`PYRo` 未被动过。 |
| 第 2 步 | `PYRo` 三处改动均为**删除**，恢复即为重新加回；`Module/Config/imu_mount.h` 为新增文件，删除即可。**建议第 2 步单独提交为一个 commit，便于整体回退。** |
| 全部 | 因两步分离，可只回退第 2 步而保留航向归零（归零本身是独立收益）。 |

---

## 10. 交付物清单

| 类型 | 路径 | 动作 |
|---|---|---|
| 新增 | `Module/Config/imu_mount.h` | 新建 |
| 新增 | `Module/IMU/imu_snapshot.h` | 第 3 步（可选） |
| 修改 | `PYRo/Component/IMU/imu_base_drv.h` | 删枚举、删字段、改注释 |
| 修改 | `PYRo/Component/IMU/imu_base_drv.cpp` | 删 4 行 |
| 修改 | `PYRo/Component/IMU/lsm6dsv16x/lsm6dsv16x_drv.cpp` | 删 2 个函数 + 3 处调用 |
| 修改 | `Module/IMU/imu_service.h` | 增字段、增调试量、改注释 |
| 修改 | `Module/IMU/imu_service.cpp` | 归零 + 归一化 |
| 修改 | `Module/CMakeLists.txt` | 包含路径加 `Config` |
| 修改 | `Module/PROTOCOL.md` | 数据路径与航向零点说明 |
| 修改 | `README.md` | 目录职责与数据流 |
| **不改** | `PYRo/Algorithm/**`、`PYRo/Component/IMU/lsm6dsv16x/lsm6dsv16x_drv.h`、`PYRo/CMakeLists.txt`、`Module/Application/imu_module.cpp` | — |

---

*本规划书基于仓库当前状态（2026-11-16）核对：文件行号与代码片段均取自实际源码。执行前若源码有变动，请以 grep 结果重新定位，不要直接按行号套用。*
