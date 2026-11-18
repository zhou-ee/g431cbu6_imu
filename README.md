# IMU_Module

IMU 模块正式产品固件的主开发仓库。目录与构建分层参考 PYRo-wl，产品主要代码集中在根目录 `Module`。

## 当前工程

| 项目 | 当前配置 |
|---|---|
| MCU | STM32G431CBU6，Cortex-M4F |
| 系统时钟 | 170 MHz（HSI 16 MHz → PLL，M=4/N=85/P=2；HSE 未启用） |
| 链接空间 | Flash 128 KB，RAM 32 KB |
| IMU | LSM6DSV16X，已有 SPI 驱动、FIFO 和器件时间戳适配 |
| SPI1 | PA5/PA6/PA7，5.3125 Mbit/s，DMA 收发 |
| 片选 | PA4，高电平初始化，产品装配层负责绑定 |
| UART | USART2，PA2/PA3，921600 波特率，DMA 收发（DMA1_Ch3/Ch4，NVIC 5） |
| CAN | FDCAN1，PA11/PA12，当前为经典 CAN、1 Mbit/s |
| RTOS | FreeRTOS，1 kHz tick，主堆 12288 B |
| DSP | 根目录 `third_party/CMSIS-DSP` 本地快照 |
| IMU 采样率 | gyro / acc 各 1920 Hz（产品目标 7680 Hz，见"采样与发布频率"） |
| 测量发布 | CAN、UART 同时 1000 Hz，发布四元数、加速度与角速度，共用快照 |
| 采集触发 | FIFO 轮询 |
| 参数生命周期 | 编译期默认值与运行期参数，复位后恢复默认值 |

2026-11-15 完成依赖与构建接入。现有源码及硬件配置共 1150 个文件通过迁移前后的 SHA-256 一致性检查。
固件入口 `StartDefaultTask` 已调用 `imu_module_init()`，采集与 VQF/ESKF 融合链路随调度器启动并常驻。
CAN / UART 通信层尚未实现，`Module/Communication` 目录尚未建立，协议 v1 目前只有
[协议文档](Module/PROTOCOL.md)。

## 目录职责

| 目录 | 职责 |
|---|---|
| `CubeMX` | `.ioc`、启动文件、链接脚本、HAL、FreeRTOS 及生成的初始化代码 |
| `PYRo/Core` | 状态码、任务、锁、固定容量容器和内存管理 |
| `PYRo/Peripheral` | SPI、UART、CAN、DWT 等通用外设封装 |
| `PYRo/Device` | 器件寄存器与总线适配 |
| `PYRo/Component/IMU` | IMU 数据结构、标定与 LSM6DSV16X 驱动（输出器件坐标系） |
| `PYRo/Algorithm` | Matrix、KF、Butterworth、VQF 等通用算法 |
| `Module` | 产品装配、INS 服务、装配方向映射、协议编解码与发布（结构见下） |
| `third_party` | 固定快照的第三方依赖、许可证、来源与哈希记录 |
| `cmake` | 根目录构建使用的 G431 工具链和链接路径适配 |

依赖方向为 `Module → PYRo → CubeMX / CMSIS-DSP`。产品配置与协议由 `Module` 持有，
`PYRo` 不感知产品如何装配。

### Module 分层（2026-11-16 重构）

结构参考 `Wheel_leg/PYRo-wl` 的 `Component/INS`：一个组件同时持有"器件、姿态模型、
常驻任务"，对外只给结果。对应关系与差异：

| 参考实现 | 本工程 | 说明 |
|---|---|---|
| `ins_drv_t` | `IMU/ins_service_t` | 器件注入 + 常驻任务 + 装配归一化 + 航向归零 + 统一快照 |
| `ins_attitude_t` | `PYRo/Algorithm/VQF/pyro_algo_vqf.*` | 姿态 + ESKF 已在 PYRo，Module 不再重复一层 |
| `ins_config_t` | `Config/imu_config.h` + `imu_mount.h` | 编译期产品参数，不运行时传递 |
| 13 个语义化 getter | `IMU/imu_snapshot_t` 单一快照 | 参考实现的 getter 各自读缓存，两次调用跨拍；快照带 `valid`/`heading_zeroed` |
| — | `Communication/imu_protocol.*` | 纯函数编解码，可在主机上逐字节验证 |
| — | `Communication/imu_publisher` | 1 kHz CAN/UART 发布任务 |
| — | `Application/imu_module.*` | 初始化顺序与产品入口 |

```text
Module/
├── Application/   imu_module.h/.cpp      产品入口：INS → 发布 的装配顺序
├── Config/        imu_config.h           器件/融合/发布/任务参数（唯一真源）
│                  imu_mount.h            装配方向映射（正/逆表 + 换系）
├── IMU/           ins_service.h/.cpp     INS 服务（采集+融合+归一化+归零+快照）
│                  imu_snapshot.h         统一输出快照（测量 + 解包参数）
├── Communication/ imu_protocol.h/.cpp    协议 v1 编解码（纯函数）
│                  imu_publisher.h/.cpp   1 kHz CAN/UART 发布
└── tests/         test_protocol.cpp      协议字节向量与原码回环验证
                   run_protocol_test.py
```

单任务而非"采集/算法"双任务：双任务实测会导致环形缓冲 head 在处理期间漂移（读到错位
样本）、临界区屏蔽 SPI/DMA 中断（采集从 1 kHz 掉到 254 Hz）、双缓冲计数清零时机不当
导致时间戳倒退。代价是融合耗时计入 1 ms 周期，余量见"耗时分布"。

旧实现（`Module/IMU/imu_service.*` 等）已在本次重构中删除：协议编解码、发布调度与
采集融合原本混在同一文件里，外部只见调试镜像、没有可测的协议层。

### 验证状态（2026-11-16）

主机侧（不接硬件，`g++` 即可）：

```powershell
$env:PATH = "<mingw64>\bin;" + $env:PATH
python Module/tests/run_protocol_test.py     # 协议 v1：字节向量 + 原码回环
python <备份目录>/PYRo_tests/run_vqf_eskf.py   # VQF/ESKF 数值回归
```

`run_protocol_test.py` 覆盖四类结论，全部通过：

1. PROTOCOL.md 的字节向量逐字节一致（UART 26 B 含 CRC `0xBD87`、CAN 三帧 `0x310/0x311/0x312`）；
2. **3775 万组**原码回环（8 个装配方向 × 4/6 组量程 × 3 组静态偏置 × 全 16 位码值）
   恢复误差恒为 0——这是接口层最硬的判据；
3. 姿态无效时四元数四字置零；
4. 取整/限幅边界（±0.5 远离零、越界饱和、NaN/±Inf 归零）。

上板侧（DAPLink + `openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg`）：

- `imu_module_init_result = 0`、`imu_service_vqf_ready = 1`、`imu_service_err_*` 全 0；
- 1 kHz 采集/融合（`imu_service_algo_rounds` 与 `uwTick` 同步增长）、姿态有效、航向已归零；
- 实际发出的 26 字节与协议一致，读回的帧尾 CRC `0x3D1A` 与独立计算值相同；
  解出的 `acc_z = 9.789 m/s²`（重力）、`gyro ≈ 0.005 rad/s`（静止）；
- CAN 三帧内容与 UART 负载逐字节对应（`imu_pub_can_last[3][8]`），`imu_pub_can_err = 0`；
  **物理层交付未能验证**——台架没有第二个 CAN 节点（无 ACK），需在真实总线上复测。

尚未验证：CAN 在带对端的终止总线上的丢组率与中断开销；UART 接收方向（本版只发不收）。

## 构建

环境需要 CMake 3.22 及以上、Ninja、ARM GNU Toolchain，命令行能够找到 `arm-none-eabi-gcc/g++`。
在仓库根目录执行以下命令。

```sh
cmake --preset Debug
cmake --build --preset Debug --parallel 4

cmake --preset Release
cmake --build --preset Release --parallel 4
```

产物分别位于 `build/Debug/IMU_Module.elf` 和 `build/Release/IMU_Module.elf`。
构建参数为 Cortex-M4、FPv4 单精度 FPU、hard-float ABI、C11、C++17。
DSP 使用 `FASTBUILD=ON`、`DISABLEFLOAT16=ON`，具体导入记录见 [依赖清单](third_party/dependencies.json)。

2026-11-15 基础固件构建结果如下。

| 配置 | Flash | RAM |
|---|---|---|
| Debug | 32364 B | 6416 B |
| Release | 17756 B | 6416 B |

这些数字对应当前默认任务及依赖接入。VQF 和驱动已参与编译，链接器按实际引用回收未使用的函数。
完整产品的解算、通信、任务栈和缓冲资源需要在业务接入后重新测量。
矩阵 Cholesky、上下三角求解、矩阵运算与四元数接口另外完成了 M4F 链接检查。

2026-11-16 采集与解算链路接入 `Module` 后重新测量（含装配归一化与航向归零）。

| 配置 | Flash | RAM |
|---|---|---|
| Debug | 98528 B | 22576 B |
| Release | 67612 B | 22544 B |

本次改造（装配方向上移至 `Module/Config`、增加航向归零）相对改造前的增量为 Release +752 B Flash、+56 B RAM。
同日"协议与健壮性修复"轮的增量为 Debug +296 B Flash / +1284 B RAM、Release +68 B Flash / +1280 B RAM，
其中 RAM 增量几乎全部来自双路样本缓冲由 24 帧增至 64 帧（每路 +640 B）——原值在 7680 Hz 下只有
3.1 ms 余量，不足一个读周期的两段 DMA 超时时间。

## 数据流规划

1. HAL 完成时钟、GPIO、DMA、SPI、UART 和 CAN 初始化。
2. 产品装配绑定 SPI1 与 PA4 片选，初始化 IMU，读取时间戳时钟校正值，配置标定与零偏估计策略，并缓存装配方向映射参数。
3. 采集任务批量读取 FIFO，驱动输出**器件坐标系**的最新解包物理量，融合按实际消费量归还环形缓冲。
4. VQF 按器件时间戳合并 gyro/acc，按各路实际时间差更新姿态与零偏（工作坐标系为器件坐标系）。
5. 快照写入前完成**装配归一化**（器件系 → 安装系，四元数、加速度、角速度、零偏一并换系）与**航向归零**，欧拉角由归零后的四元数导出。
6. IMU 服务提供四元数、欧拉角、零偏、两路最新解包物理量，以及航向零点与运行状态。
7. 通信任务每 1 ms 取得快照，将四元数编码为 int16，将加速度和角速度按解包逆操作恢复为器件码值，为 CAN 和 UART 封包。
8. 运行参数更新在采集链路外校验，输出状态反映初始化、运行和故障阶段。

采集与姿态融合由同一任务顺序执行，DMA 中断完成通知与总线状态更新。
内部数据沿用四元数 `[w,x,y,z]`、角速度 rad/s、加速度 m/s²，统一为安装坐标系与同一航向零点。
装配方向映射配置与映射表位于 `Module/Config/imu_mount.h`，驱动不感知产品安装方式。
线上四元数采用 32767 倍 int16 编码，加速度和角速度由最新解包物理量反向恢复为器件 X/Y/Z 轴的 int16 码值。
通信先逆安装方向，对角速度加回驱动静态偏置，再除以正向解包采用的转换系数，取整限幅后封包。
接收方按当前 ±4 g、±2000 °/s 量程的器件灵敏度转换原始码，按产品安装方向与四元数配合使用。
完整帧定义、UART CRC 参数、CAN 三帧布局与测试向量见 [协议 v1](Module/PROTOCOL.md)。

## 产品约束与接入问题

### 采样与发布频率

产品目标是 gyro/acc 各 7680 Hz、CAN 与 UART 每路 1000 Hz 发布。**当前固件按 1920 Hz 运行**：
VQF/ESKF 把倾斜与静止观测限制在 2 ms 一次（约 500 Hz），陀螺仍按每个样本传播。
降速依据是 1 ms 周期预算的实测结果（G431/M4F，目标板静止，`imu_service_vqf_us_max` 为单批峰值）：

| 构建 | 配置 | update() 平均 | update() 峰值 | 整拍平均 |
|---|---|---|---|---|
| Release | 仅 INS（发布关闭） | 515 µs | 791 µs | 671 µs |
| Release | INS + UART 发布 | 520 µs | 845 µs | 678 µs |
| Release | **INS + CAN + UART 发布（最终）** | **521 µs** | **831 µs** | **678 µs** |
| Debug（Algorithm 仍单独 -O2） | INS + CAN + UART | 560 µs | ~830 µs | 788 µs |

前两行是隔离测量：UART 发布只多花约 5 µs 平均、50 µs 峰值（26 B / 921600 走 TX DMA）；
**CAN 发布最初的 320 µs 峰值几乎全是中断开销**：驱动曾打开 `FDCAN_IT_ARB_PROTOCOL_ERROR`
驱动曾打开 `FDCAN_IT_ARB_PROTOCOL_ERROR`/
`DATA_PROTOCOL_ERROR`——它们是**逐帧**事件，台架没有第二个节点、每帧都等不到 ACK，于是 3 帧/ms 各触发一次 HAL 错误分支；裁剪这两个通知后峰值从 1157 µs 回到 831 µs。裁剪前用 Debug 符号读到的是真相（`hfdcan1.Instance->CCCR=0x00001040` → **DAR=1，自动重传本就关闭**；`PSR.LEC=5` ACK 错误；`ECR.TEC=160` 错误被动），此前按裸地址读到的 `CCCR=0/PSR=0/TEC=0` 是错的，不要再用裸地址判断 FDCAN 状态。**在有对端确认的终止总线上该开销应显著下降，需在真实
总线上复测后再定 CAN 发布频率。**

不发布时峰值 791 µs、距 1 ms 还有约 200 µs，三轮优化（矩阵暂存复用 / 块结构协方差传播 /
模块重构）前后一致；开启 CAN 发布后峰值会越界，所以"1 ms 周期"的结论以第一行为准。
实测循环速率：仅 INS 时 997–999 Hz。
`uwTick` 在调试器 halt 期间停止计数（已实测：halt 2 s 前后读数不变），所以速率测量不受读数干扰。

7680 Hz 下每拍约 7.7 个样本/路，按同比例估算远超预算，因此在上板实测通过前不提高 ODR。
若要提高到 7680 Hz，必须先压低单批成本，再重新测量峰值与循环速率。

融合消费全部 IMU 样本，发布调度每 1 ms 取得四元数、最新解包物理量和对应解包参数快照。
可变步长低通在 dt 变化时计算 double 指数和三角函数，需要继续跟踪最坏批次耗时、任务调度抖动及丢帧计数。
跨批次迟到的加速度当前缺少历史姿态补偿，输入批次需要按共同时间轴提交。

两路各 1920 Hz、每路 6 字节原始值时，裸数据量约 23040 B/s；7680 Hz 时为 92160 B/s。
当前协议每组为 26 B UART 帧（帧头 2 + 长度 2 + 负载 20 + CRC 2），1 kHz 下占 921600、8N1 链路约 28.2%。
经典 CAN 每组为三个 8 B 标准帧，按每帧含位填充和帧间隔 135 bit 保守预算，1 kHz 占 1 Mbit/s 约 40.5%。
需要在目标总线上核对其余节点负载与仲裁延迟。

### 耗时分布与优化空间（2026-11-16 实测）

Release 构建、DAPLink 停机采样 PC 600 次（按符号/模块归并）。下表是**优化前**的基线：

| 模块 | 占比 | 说明 |
|---|---|---|
| 矩阵层（`arm_mat_*` + `pyro::matrix`） | **35%** | 6×6 协方差传播、Joseph 更新、3×3 雅可比 |
| 内存操作（`memset`/`memcpy`） | **16%** | `matrix_t` 临时对象与工作区的零初始化/拷贝 |
| double 软浮点（`__aeabi_dadd`/`__muldf3`） | **13%** | Butterworth 的状态与系数递推 |
| IDLE / FreeRTOS | 9% | 空闲余量 |
| KF 核心（`kf_t`） | 6% | 预测/修正簿记、`_covariance_valid`、`_finish_covariance` |
| VQF 逻辑 / Butterworth 本体 | 6% / 3% | |
| 驱动与 SPI | 1% | 采集不在关键路径上 |

两轮优化后（同一采样方法）变为：矩阵层 34.3%、double 软浮点 16.8%、内存操作 9.2%、
IDLE 10.7%、KF 核心 8.2%、VQF 逻辑 9.3%、Butterworth 本体 4.3%、驱动 1.8%。
每轮的具体变化见下方"已实施"两节。

`memset` 的调用点（按返回地址统计）：`_update_gyr` 38%、`matrix_t::operator*` 25%、
`_inject_error` 9%、`kf_t::reset` 4%——热路径上反复构造/清零矩阵临时量。
该版 `memset` 是逐字节循环（`strb.w` + 回跳，约 4.5 周期/字节），11% CPU 对应约 4 KB/ms 的零初始化。

**已实施，第一轮：复用矩阵暂存（2026-11-16）**
把 `_update_gyr` / `_inject_error` 里的 7~9 个矩阵临时量提为 `vqf_eskf_t` 成员复用，
不再逐样本构造/清零；同时把原来被 `operator*` 静默吞掉的矩阵运算返回码补上检查。

**已实施，第二轮：块结构协方差传播（2026-11-16）**
误差状态转移 `F = [[I, -A·dt], [0, I]]` 与误差注入 `G = [[J_l, 0], [0, I]]` 都是块结构，
不再构造 6×6 的 F/G，直接用 3×3 块运算（利用 P 对称，各只需 **3 次 3×3 乘法 = 81 MAC**，
通用路径是 2 次 6×6 = 432 MAC 加一次 6×6 转置）。算好的 `F·P·Fᵀ` 交给新增的
`kf_t::predict_covariance(x_pred, P_propagated, Q_d)`——通用 KF 仍负责 +Q_d、数值检查、
对称化/方差下限与 `PREDICTED` 提交语义，`predict(x_pred,F,Q_d)` 原样保留。

两轮的累计效果（Release，同一块板、静止、同条件）：

| 指标 | 原始 | 第一轮后 | 第二轮后 | 累计 |
|---|---|---|---|---|
| `update()` 平均 | 616 µs | 555 µs | **518 µs** | −15.9% |
| `update()` 峰值 | 940 µs | 849 µs | **793 µs** | −15.6% |
| 整拍平均（读+融合+发布） | 765 µs | 705 µs | **668 µs** | −12.7% |
| `update()` 峰值（Debug） | 1130 µs | 850 µs | **789 µs** | −30.2% |
| `arm_mat_mult_f32` 占比 | 24.0% | 30.5% | **17.3%** | — |
| `memset` 占比 | 11.3% | 2.7% | **4.8%** | — |

代价：`vqf_eskf_t` 实例 2184 → 3304 → **3048 B**（第二轮块暂存替代了 6×6 的 F/G，净减 256 B），
Flash 67084 → 67612 B（第一轮 +1120，第二轮 +528）。
第一轮主机回归数值逐字相同；第二轮因块结构改变浮点分组，只有 `motion_bias` 第 6 位
有效数字与 `tilt 0.040563°→0.040582°` 有变化，其余逐字一致，回归全部通过。

**已试过并否决的优化**：把 `matrix::multiply` 的小矩阵换成编译期定长、寄存器累加的手写内核。
主机基准 6×6 快 3.87×、3×3 快 2.69×，且 18000 组随机矩阵逐位一致；但**上板 A/B 反而略差**
（`update()` 平均 616→623 µs，峰值 940→1128 µs）。原因是 Release 下 CMSIS 的通用实现已经
优化成"单累加器 + 廉价 load"，而无 cache 的 M4F 上非连续访问并不昂贵。该改动已回退，
主机与上板数据都留档在此，避免重复踩坑。

后续可选方向（按收益/风险排序，均未实施）：

1. **压缩最坏拍**：限制每拍从 FIFO 取的字数（当前一次最多 36 字 = 12 对样本，够 6 ms 用），
   把尾部耗时摊到后续拍。只改驱动常量、不动算法，可进一步压低峰值。
2. **Butterworth 递推改 float**（系数仍用 double）：double 软浮点仍占 17%（块化后份额上升），
   可吃掉其中大部分，但会改变 VQF 的低通数值，需主机回放对比后再决定。
3. **观测侧 Joseph 更新块化**：`kf_t::correct` 的 `H` 只有前 3 列非零，Joseph 的
   `K·H`、`(I-KH)·P·(I-KH)ᵀ` 也能按块化简。收益小于前两轮，但方向相同。

G431 的 FDCAN 发送 FIFO 固定为 3 个元素（消息 RAM 布局由 G4 HAL 的编译期宏固定，`TXBC.TFQS` 不参与配置），
一组三帧即可占满。`PYRo/Peripheral/CAN/pyro_can_drv.cpp` 已改为**队列满只返回 `PYRO_BUSY`、不撤销在发请求**：
撤销会把同组已入队的前几帧一起丢掉，接收端无法区分"丢组"与"错序"。
发布层（`Communication/imu_publisher.cpp`）据此实现整组推进：任一阵列返回忙就放弃本组剩余帧，
不重试（下一拍有更新的快照，重试会把旧组插到新组前面，反而破坏接收端按到达顺序配对）。

**本版已接入 CAN**：`bsp_can::init_all()` 完成 `init()` + `start()`，实测 `FDCAN1->CCCR.INIT=0`
（此前从未初始化，收发都未使能）。但台架无第二个节点，帧等不到 ACK，物理层交付无法验证；
已验证的是"驱动接受 + 字节正确"：`imu_pub_can_err=0`、`imu_pub_can_base=0x310`、
`imu_pub_can_last[3][8]` 与协议逐字节一致（见 `Module/tests`）。
`MX_FDCAN1_Init()` 里 `AutoRetransmission = DISABLE` 对应 `CCCR.DAR=1`，
已用 `hfdcan1.Instance->CCCR=0x00001040` 复核确认 DAR=1（单次发送、不重传），与 CubeMX 配置一致；无对端时每帧只尝试一次并报 ACK 错误。

### 内存与任务

FreeRTOS 主堆 12288 B。两个常驻任务：`ins`（1024 词 = 4 KB，HIGH）与 `imu_pub`
（384 词 = 1.5 KB，NORMAL）。实测运行后 `xPortGetFreeHeapSize()` = **1504 B**；
栈余量分别为 446 词（ins）与 257 词（pub），可从 `imu_service_ins_stack_free` /
`imu_pub_stack_free` 持续观测。DMA 专用堆默认 2048 B（USART2 的 2×512 从这里出）。
`defaultTask` 初始化完即进入 `osDelay(1)` 空转，其 4 KB 栈长期不释放；
若后续不需要它承担复位或看护职责，可在 `imu_module_init()` 后 `vTaskDelete(NULL)` 释放。
静态占用实测：`ucHeap` 12288 B、VQF 实例 3048 B、`lsm6dsv16x_drv_t` 1120 B、SPI 中间件 1080 B，
当前 RAM 合计 **24984 B（Release）/ 25040 B（Debug）**，占 32 KB 的 76%；
Flash **72936 B / 55.7%**（Release）、**112052 B / 85.5%**（Debug）。
RAM 从重构前的 22544 B 涨 2432 B，主要是 UART/CAN 驱动实例与其容器、发布任务栈；
Debug 的 Flash 涨到 112 KB 是因为 UART 驱动的 `std::map`/`std::function` 在 -O0 下展开。

### 硬件与时间基准

产品本阶段采用 FIFO 轮询，PA4 为片选。
1 kHz 发布采用固定周期调度（`INCLUDE_vTaskDelayUntil=1` 已配置），采集与计算耗时计入每 1 ms 的预算。
实测循环速率：仅 INS 时 997–999 Hz（Release），即 1 ms 周期基本被融合打满；
开启 CAN 发布后跌到约 996 Hz（`vTaskDelayUntil` 难以真正挂起），原因与判据见"采样与发布频率"。
时间戳计数器重启后需要重置 VQF 时间基准，运行状态应明确反映这次重新初始化。

### Flash 与参数

本阶段产品配置为完整应用使用 128 KB Flash，标定与运行参数在 RAM 中管理。
节点号、安装方向及滤波默认值由编译期产品配置提供，复位后重新初始化。
安装方向位于 `Module/Config/imu_mount.h`，换装配方向只改其中的 `k_imu_direct`。

### 通信协议

主机同时使用 CAN 与 UART。协议 v1 使用小端序，四元数为缩放 int16，加速度和角速度为原始 int16。
UART 采用 `A5 5A` 帧头 + 长度 + 20 B 负载 + CRC16/CCITT-FALSE，完整帧 26 B，CRC 覆盖帧头与长度。
CAN 使用控制器帧级 CRC，第一帧携带完整四元数，第二、三帧分别携带加速度与角速度，末两字节为保留字段。
CAN 默认节点号为 1，ID 为 `0x310..0x312`，节点号由编译期配置调整。
本版不带发布序号：加速度帧与角速度帧按到达顺序配对，5 ms 内未配齐即丢弃该组。
丢帧会让其后各组的配对整体错位（直到窗口超时才重新对齐），因此 CAN 四元数与原始向量的严格组关联
只在无丢帧时成立；需要严格对应时应在后续协议版本恢复序号字段。

## 后续实施顺序

1. 在 1920 Hz 下完成 RAM/CPU/总线预算复核；提高到 7680 Hz 前必须先压低单批成本并重测（见"采样与发布频率"）。
2. ~~修正 CAN 发送忙处理、配置固定周期任务、接入采集解算链路~~（已完成：驱动在队列满时只返回忙，
   固定周期任务与姿态快照已接入 `Module/Application` 与 `Module/IMU`）。
3. 测量栈高水位与丢帧，验证时间戳与故障恢复（当前 `err_bus / err_data / err_timeout / vqf_err` 全 0）。
4. 在 `Module/Communication` 实现协议 v1：先调 `can_drv_t::init()/start()`（当前 FDCAN 仍处初始化态），
   再验证四元数编码、正反向换算的原码恢复、UART CRC、CAN 三帧到达顺序配对与拥塞处理。
5. 配置编译期默认值与运行参数，验证双接口同步快照和持续 1 kHz 发布。
6. 完成静止、动态运动、温度变化和持续运行的板级验证，记录发布固件版本与依赖快照。
