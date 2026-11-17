# IMU_Module 结构说明

> 本文档描述 2026-11-17 整理后的代码结构。所有图示为实际代码结构的对应，
> 不是规划。规模数据取自 Release 构建实测。

## 1. 分层与依赖方向

```mermaid
graph TB
    subgraph MOD["Module — 产品装配层"]
        APP["Application<br/>imu_module.cpp<br/>入口 · 启动定序"]
        CFG["Config<br/>imu_config.h · imu_mount.h<br/>编译期唯一真源"]
        IMU["IMU<br/>imu_capture · imu_fusion<br/>采集 / 融合+归零+快照"]
        COM["Communication<br/>imu_protocol · imu_publisher<br/>编解码 + 1kHz 发布"]
    end
    subgraph PYRO["PYRo — 通用组件层"]
        CORE["Core<br/>状态码 · 任务 · 锁 · DMA堆"]
        PERI["Peripheral<br/>SPI · UART · CAN · DWT"]
        DEV["Device<br/>SPI 中间件"]
        COMP["Component/IMU<br/>驱动 + 环形缓冲"]
        ALGO["Algorithm<br/>Matrix · KF · Butterworth<br/>VQF · CRC"]
    end
    subgraph VEND["第三方与生成代码"]
        CMX["CubeMX<br/>HAL + FreeRTOS + 启动"]
        DSP["third_party<br/>CMSIS-DSP"]
    end

    APP --> IMU
    APP --> COM
    IMU --> CFG
    COM --> CFG
    IMU --> COMP
    IMU --> ALGO
    COM --> ALGO
    COMP --> DEV
    DEV --> PERI
    COMP --> CORE
    PERI --> CMX
    ALGO --> DSP
    CORE --> CMX
```

**单向依赖 `Module → PYRo → CubeMX/CMSIS-DSP`**，无反向引用。
`Module` 持有全部产品参数与协议定义，`PYRo` 不感知产品如何装配。

| 层 | 规模 | 职责边界 |
|---|---|---|
| `Module` | 11 个源文件 / 1482 行 | 产品参数、装配顺序、协议 v1、对外快照 |
| `PYRo` | 驱动 / 算法 / 外设 / 核心设施 | 可复用，不含产品语义 |
| `CubeMX` | HAL / FreeRTOS / 启动 | CubeMX 生成，手改处均有 `USER CODE` 标记 |

## 2. 运行时数据流

```mermaid
flowchart TD
    DEV["LSM6DSV16X<br/>1920 Hz · ±4g · ±2000dps"]
    FIFO["器件 FIFO<br/>continuous · 时间戳批入"]

    subgraph CAP["ins_cap 任务（HIGH，1 ms 硬周期）"]
        RD["read()<br/>①读FIFO状态 ②判溢出<br/>③读≤36字 ④逐7字节TAG解包"]
        PUSH["reserve → 写满 → publish<br/>器件坐标系<br/>单位换算 + 扣静态偏置"]
    end

    subgraph RING["SPSC 环形缓冲（2×64 帧）"]
        direction LR
        R1["tail 只由生产者写"]
        R2["head 只由消费者写"]
    end

    subgraph ALGO["ins_algo 任务（ABOVE_NORMAL，尽力而为）"]
        UPD["vqf.update()<br/>双路时间戳归并<br/>陀螺逐样本传播<br/>倾斜/静止观测 ≈500 Hz"]
        NORM["装配归一化<br/>q⊗m⁻¹ + 向量查表"]
        HDG["航向归零<br/>Rz(-yaw0)⊗q"]
        EUL["欧拉角导出 ZYX"]
        SNAP["快照写入<br/>seqlock 发布"]
    end

    subgraph PUB["imu_pub 任务（NORMAL，1 ms）"]
        ENC["encode_payload<br/>四元数 int16×32767<br/>加速度/角速度还原器件原码"]
        CAN["CAN 三帧<br/>0x310..0x312"]
        UART["UART 26B<br/>A5 5A + len + payload + CRC"]
    end

    DEV --> FIFO --> RD --> PUSH --> RING
    RING --> UPD --> NORM --> HDG --> EUL --> SNAP
    SNAP -.seqlock.-> ENC
    ENC --> CAN
    ENC --> UART
```

关键点：

- **`read()` 与 `update()` 分属不同任务**，中间只靠 SPSC 环形缓冲交接，
  全程**不使用任何临界区**（索引所有权分离 + publish 屏障）。
- `ins_cap` 的周期是硬约束：`read()` 内部两段 DMA 等待会挂起任务，
  融合耗时再高也不挤占这个周期。
- 快照是**值拷贝 + seqlock**，`imu_pub` 与融合任务零共享。

## 3. 坐标系变换链

```mermaid
flowchart LR
    D["器件坐标系<br/>X/Y/Z 与原码一一对应"]
    A["算法世界系<br/>Z-up"]
    I["安装坐标系<br/>产品定义"]
    Z["航向已归零<br/>相对产品零点"]

    D -->|"vqf.update()<br/>四元数积分"| A
    D -->|"mount_apply()<br/>符号置换 M"| I
    A -->|"q ⊗ m⁻¹<br/>右乘换系"| I
    I -->|"Rz(-yaw0) ⊗ q<br/>左乘只改 yaw"| Z
```

**为什么换系放在输出端而非驱动端**：8 种安装方向全是符号置换矩阵
（行列式 +1），而 VQF 内部的逐通道低通、逐轴限幅、模长判据对该置换不变，
因此两种放法数学等价。该等价性**只在符号置换下成立**，若将来支持任意
安装角必须重新设计（见 `imu_mount.h` 头部注释）。

反向（安装系 → 器件系，用于封包）**必须用导出的逆表**，
不能直接复用正向表——`DIRECT_2`/`DIRECT_4` 这类"换轴+非对称取反"会算错。

## 4. 任务与资源

```mermaid
graph LR
    subgraph HEAP["FreeRTOS 堆 12288 B"]
        T1["defaultTask<br/>1024 词 · prio 3<br/>初始化后自删"]
        T2["ins_cap<br/>256 词 · prio 5"]
        T3["ins_algo<br/>768 词 · prio 4"]
        T4["imu_pub<br/>384 词 · prio 3"]
        T5["IDLE<br/>128 词 · prio 0"]
    end
    subgraph SRAM["SRAM 32 KB"]
        BSS[".data + .bss<br/>约 21.9 KB"]
        DMAH["DMA 堆 2048 B<br/>UART 2×512"]
        VQF["VQF 实例 3120 B"]
    end
```

| 任务 | 栈 | 优先级 | 周期 | 实测高水位余量 |
|---|---|---|---|---|
| `ins_cap` | 256 词 | 5 (HIGH) | 1 ms 硬 | 165 词 |
| `ins_algo` | 768 词 | 4 (ABOVE_NORMAL) | 尽力而为 | 169 词 |
| `imu_pub` | 384 词 | 3 (NORMAL) | 1 ms | 269 词 |

## 5. 协议编解码（纯函数层）

```mermaid
flowchart TD
    S["imu_snapshot_t（imu_fusion.h）<br/>安装系 + 已归零 + 解包参数"]
    S --> Q["四元数 ×32767 int16<br/>无效姿态置零"]
    S --> AC["加速度：Mᵀ → /Sa → 取整限幅"]
    S --> GY["角速度：Mᵀ → +b → /Sg → 取整限幅"]
    Q --> P["payload_t<br/>10×int16 = 20 B"]
    AC --> P
    GY --> P
    P --> U["UART 26 B<br/>A5 5A · len · payload · CRC16"]
    P --> C["CAN 3×8 B<br/>0x310 qw qx qy qz<br/>0x311 ax ay az _<br/>0x312 gx gy gz _"]
```

**协议层零硬件依赖**，可在主机上逐字节验证。UART 帧 CRC 直接复用
`PYRo/Algorithm/CRC` 的 `append_crc16_check_sum()`（反射查表族，
poly 0x1021 反射、init 0xFFFF、无 xorout，校验向量 `123456789` → `0x6F91`）。
产品尚未定型，最终对端参数以正式发布的协议文档为准；帧结构与覆盖范围不变。

## 6. 目录职责速查

| 路径 | 职责 | 关键约束 |
|---|---|---|
| `Module/Application/` | 启动定序：INS → 发布 | 顺序不可交换 |
| `Module/Config/imu_config.h` | 器件/融合/发布/任务参数 | 编译期唯一真源 |
| `Module/Config/imu_mount.h` | 8 方向映射 + 逆表导出 | 正向表不可复用于反向 |
| `Module/IMU/imu_capture.*` | 器件对象、read()、SPSC 生产、采集统计 | 唯一持有硬件的地方 |
| `Module/IMU/imu_fusion.*` | VQF/ESKF、归一化、归零、快照 | 只消费缓冲，不碰总线 |

| `Module/Communication/imu_protocol.*` | 协议 v1 编解码 | 纯函数，主机可测 |
| `Module/Communication/imu_publisher.*` | 1 kHz CAN/UART 发布 | 忙则丢整组，不重试 |
| `PYRo/Component/IMU/` | 驱动 + SPSC 环形缓冲 | 输出器件坐标系 |
| `PYRo/Algorithm/VQF/` | VQF + 6 维误差状态 ESKF | 器件系工作，yaw 不可观测 |
| `PYRo/Algorithm/CRC/` | CRC16（反射查表族）+ CRC8 | UART 帧复用其 `append_crc16_check_sum` |
| `PYRo/Peripheral/` | SPI/UART/CAN/DWT 封装 | ISR 路由用定容 map |

## 7. 已验证的判据

| 验证 | 方法 | 结果 |
|---|---|---|
| 协议字节向量 | 主机 g++ 编译 imu_protocol.cpp + PYRo CRC | 32 checks / 0 fail；3775 万组原码回环误差恒 0（测试文件已随 Module/tests 移除，判据留档）|
| VQF/ESKF 数值 | `python PYRo/tests/run_vqf_eskf.py` | 8 个用例全通过，`sizeof(vqf_t)=3120 B` |
| SPSC 并发 | `python PYRo/tests/run_spsc_test.py` | 400 万样本无撕裂/乱序/丢失；变异测试确认判据有效 |
| 采集解耦 | 上板实测 | `cap_rounds` 与 `algo_rounds` 独立，采集稳 1000.4 Hz |
| UART CRC | 上板抓帧 + 独立复算 | `0xB0BE` 与线上帧尾一致 |

## 8. 关键设计取舍（有实测支撑）

| 决策 | 依据 |
|---|---|
| **单生产者/单消费者 + 无锁** | 400 万样本并发测试；旧实现的 `count` 双方读改写是竞争源 |
| **满则丢弃新样本**（不覆盖最旧） | 覆盖需移动 `head`，而 `head` 属消费者；A1 使 `err_data` 成为纯背压计数 |
| **采集与融合拆双任务** | 1920 Hz 下 `vqf_us` 峰值 835 µs 曾串在 1 ms 周期里；拆分后采集不受影响 |
| **`rest_filter_tau = 0.15 s`** | 实测：τ=0.5 时静止判决建立瞬间 `rest_lp_z` 符号都错（−0.0011 vs 真值 +0.0022），yaw 突变 0.45~1.6°；τ=0.15 后误差降到 0.000065 rad/s，突变消失 |
| **`motion_bias_est_enabled = false`** | 开启时多一条 `bias_z` 修改通道，逻辑更复杂；实测其贡献极小（8 s 仅 0.000047 rad/s） |
| **ODR 保持 1920 Hz** | 7680 Hz 实测 `vqf_us` 达 8.3 ms、`err_data` 每秒涨 1000，链路饱和 |
| **CRC 复用 PYRo 既有实现** | 产品未定型，对端参数由正式发布协议指定；直接复用 `append_crc16_check_sum`，不在协议层新增算法实现。实测 6/6 帧 CRC 一致 |

## 9. 后续待办

1. **7680 Hz**：需先压低单批成本（矩阵块化、Butterworth 递推改 float），
   再重测峰值；当前架构已支持"融合慢不影响采集"。
2. **CAN 物理层**：台架无第二节点，未验证 ACK 与总线负载下的丢组率。
3. **发布优先级**：`imu_pub`(3) 低于 `ins_algo`(4)，融合满载时发布约 845 Hz，
   需按产品需求决定是否提升。
4. **`.ioc` 脱钩**：`CubeMX/IMU_Module.ioc` 里 `TOTAL_HEAP_SIZE`/`Tasks01`
   仍是旧值，重新 Generate 会覆盖手改配置。
