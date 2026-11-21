# IMU Module（产品装配层）

**三任务流水线：采集 → 融合 → 发布，1 kHz 硬周期**

本目录是产品层，决定「这个产品怎么装、怎么采、怎么发」。驱动（`PYRo/Component/IMU`）与算法（`PYRo/Algorithm`）都不读这里的配置——参数由 `capture::init()` / `fusion::init()` 在启动时装配进去，换产品只改本目录。

```
Module/
├── Config/         编译期配置：采样、容差、任务栈、装配方向
├── IMU/            capture（读器件） + fusion（姿态解算）
├── Communication/  协议编码 + CAN/UART 发布
└── Application/    任务编排与生命周期
```

---

## Part 1: 代码全解 (Code Deep Dive)

### 1. 三个任务的职责与优先级

| 任务 | 职责 | 周期 | 优先级 |
|---|---|---|---|
| `ins_cap` | `read()`：FIFO 状态 → 等 DMA → 读数据 → TAG 解包 → SPSC 发布 | 1 ms 硬周期 | HIGH |
| `ins_algo` | `update()`：消费环形缓冲 → ESKF 融合 → 快照发布 | 尽力而为 | ABOVE_NORMAL |
| `imu_pub` | 取快照 → 协议编码 → CAN/UART 提交 | 由 `s_algo_sem` 唤醒 | NORMAL |

**为什么采集与融合拆开**：`update()` 峰值可达 760 µs，若与采集同任务，会把 1 ms 周期打穿。拆开后融合慢只降低姿态更新率，不影响采集。

拆分的安全前提是 `sample_stream_t` 的 SPSC 设计（索引所有权分离 + publish 屏障），因此**全程不需要 `taskENTER_CRITICAL`**。旧实现曾在算法侧用临界区把 BASEPRI 提到 5，与 SPI1/DMA 的 NVIC 优先级相同，导致 DMA 完成中断被屏蔽，采集从 1 kHz 掉到 254 Hz。

### 2. 环形缓冲的所有权

`sample_stream_t` 是单生产者/单消费者回绕缓冲，每个索引只有一个写者：

```
tail —— 只由生产者（capture 的 read()）写，指向下一个可写槽
head —— 只由消费者（fusion 的 assume_*_point）写，指向最旧未消费样本
```

生产者 `reserve()` 拿槽 → 原地写满 → `publish()` 才移动 `tail`。屏障保证数据先于索引可见，消费者绝不会看到半写的样本；最坏情况只是暂时看不到最新样本。

**满时丢弃新样本**，不覆盖最旧——覆盖必须移动 `head`，而 `head` 属于消费者，让生产者写它就退回了需要加锁的设计。副作用：`err_data` 成为纯粹的「背压丢样」计数，与器件 FIFO 溢出不再混同。

### 3. 批次原子交接

融合逐轴读取采集侧共享数组时，采集抢占可能让单个三轴向量混入两个样本。因此用很短的调度器挂起窗口固定两路索引与丢样代次：

```C++
// imu_fusion.cpp  process_batch()
vTaskSuspendAll();
const imu_data_t &live = capture::data();
batch.gyro = live.gyro;                 // 只拷贝 64 字节描述符，不含样本数组
batch.acc = live.acc;
batch.timestamp_tick_s = live.timestamp_tick_s;
discontinuities = capture::discontinuities();
(void)xTaskResumeAll();
```

窗口内只做 64 字节拷贝，**无阻塞调用、不屏蔽中断**（`vTaskSuspendAll` 只禁止任务切换，SPI/DMA 完成中断照常）。完成融合后再从受所有权保护的槽拷贝，最后归还。

### 4. 快照有效性

`imu_fusion` 对外发布的是 seqlock 保护的快照。写侧同样在 `vTaskSuspendAll` 窗口内完成「置奇 → 写入 → 置偶」，因为更高优先级的读者读到奇序号会空转，若写方被抢占则双方互等。

有效性由两路最后推进器件时间的样本决定：任一路超过 `kSampleStaleMs`（20 ms）无更新即失效。有效转无效时生成一次零四元数快照并暂停发布，真实样本恢复后继续。

### 5. 为什么 ODR 定在 1920 Hz

1920 Hz 是实测 1 ms 预算下的取值：`update()` 平均约 553 µs、峰值约 835 µs（Release）。

7680 Hz 曾做过 A/B 实测，结论是不可用：

| 指标 | 1920 Hz | 7680 Hz |
|---|---|---|
| `cap_us` max | 358 µs | 358 µs（采集不受影响，双任务拆分生效） |
| `update()` avg | 553 µs | **6 528 µs**（约 12 倍，非 4 倍） |
| `update()` max | 835 µs | **8 333 µs** |
| 结果 | 正常 | SPSC 缓冲被打爆，`err_data` 约 1000 次/秒，融合掉到约 120 Hz |

耗时随样本数呈非线性增长（陀螺逐样本传播），因此提高 ODR 的前提是先压低单批成本。该实验同时否证了「算力不足来自运动零偏校准」的猜测——当时运动零偏估计已关闭，耗时仍为 8.3 ms。

### 6. 丢样代次与恢复

FIFO 积压丢弃、硬件溢出或软件背压后，恢复首帧的角速度若被用于整个采样缺口，会产生严重错误的姿态。因此采集层维护丢样代次（`discontinuities`），融合发现代次变化时：

- 丢弃此时已排队的批次
- 重建滤波时间基准，重新确认重力与航向零点

另按器件时间限制单路最大间隔为标称采样周期的 6 倍，超限时保留首帧用于建立新时间基准。

---

## Part 2: 快速上手 (Quick Start)

### 1. 一份配置驱动全部行为

改产品行为只动 `Config/imu_config.h`，常用项：

```C++
// 采样
constexpr uint16_t kAccOdrHz     = 1920U;
constexpr uint16_t kGyroOdrHz    = 1920U;
constexpr uint16_t kFifoMaxWordsPerRead = 16U;   // 单次回读上限，超限整批丢弃

// 发布
constexpr uint8_t  kNodeId        = 1U;    // CAN base_id = 0x300 + id*0x10
constexpr uint32_t kPublishPeriodMs = 1U;
constexpr uint32_t kSampleStaleMs   = 20U;  // 快照失效期限

// 任务栈（改动后须复核 *_stack_free 是否仍 > 100 词）
constexpr uint16_t kCapTaskStackWords       = 256U;
constexpr uint16_t kInsTaskStackWords       = 768U;
constexpr uint16_t kPublisherTaskStackWords = 384U;
```

装配方向在 `Config/imu_mount.h`，与采样配置分离。

### 2. 启动顺序

`imu_module_init()` 的顺序不可调换：先同步完成器件与融合配置，再建立信号量、启动等待样本的融合任务，最后启动采集。这样保证高优先级任务跑起来时配置已就绪。

```C++
// Application/imu_module.cpp
capture::init();      // 器件自检、量程/ODR、FIFO
fusion::init();       // 读采集侧解包参数与零偏
s_cap_sem  = xSemaphoreCreateBinary();
s_algo_sem = xSemaphoreCreateBinary();
s_algo_task.start();  // 先起等待方
s_cap_task.start();   // 再起生产者
```

前两个任务创建失败时清理已分配信号量与已启动的等待任务；发布初始化失败则保留已运行的采集与融合，再次调用只重试发布。

### 3. 读取姿态数据

外部通过 `imu_module` 的访问器取数据，不需要知道内部任务：

```C++
// 姿态（四元数 + 欧拉角）
const pyro::quat_t &q = imu_module::attitude();

// 器件坐标系物理量（最新一帧，允许陈旧一拍）
const float *acc  = imu_module::last_acc_dev();
const float *gyro = imu_module::last_gyro_dev();
```

### 4. 诊断观察量

所有 `imu_service_*` 变量都是 `extern "C"` 全局，可直接用调试器按裸名字读取：

| 变量 | 含义 |
|---|---|
| `imu_service_cap_rounds` | 采集轮次（1 kHz 硬周期） |
| `imu_service_algo_rounds` | 融合轮次 |
| `imu_service_cap_us_avg/max` | 单拍 `read()` 耗时 |
| `imu_service_vqf_us_avg/max` | 单批 `update()` 耗时 |
| `imu_service_err_*` | 分类错误计数（总线/超时/溢出/背压） |
| `imu_service_snapshot_valid` | 当前快照是否有效 |
| `imu_service_heap_free_min` | 启动期最低剩余堆 |

**正常基线**（Release，1920 Hz）：`cap_us_avg ≈ 127 µs`、`vqf_us_max ≈ 760 µs`、`err_timeout = err_bus = 0`、采集与融合轮次 1:1。

### 5. 台架自测

CAN 可切内部回环验证编码与提交路径：

```C++
constexpr bool kCanLoopbackSelfTest = true;  // 仅台架，正常运行必须 false
```

发布层会轮询取回自己发的三帧并逐字节比对，用 `imu_pub_can_rx` / `imu_pub_can_rx_bad` 观察。**回环模式下帧不出芯片**。
