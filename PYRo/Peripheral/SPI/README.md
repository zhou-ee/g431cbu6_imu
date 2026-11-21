# PYRo SPI Driver

**单总线、多从机共享的 SPI 驱动，DMA 异步收发 + 中断内释放片选**

该 `pyro_spi_drv` 模块把一条 SPI 外设抽象为「一条总线」，一个 HAL 句柄对应一个驱动实例，由 BSP 独占持有。总线通过互斥量串行化多个从机的访问；事务完成点在中段上下文中释放片选，确保 CS 上升沿紧贴数据搬运结束。

---

## Part 1: 代码全解 (Code Deep Dive)

### 1. 为什么一个句柄只有一个实例

一条 SPI 外设（如 `SPI1`）在物理上就是一条总线，可挂多颗从机。若让每颗从机各自持有一个驱动实例，会出现两个后果：

- DMA 完成中断的回调只带 `SPI_HandleTypeDef*`，无权知道「是哪颗从机」，路由表按句柄索引只能命中一个；
- 完成信号量与完成钩子都变成「每设备一份」，ISR 无法确定该给哪一份。

因此 `spi_drv_t` 属于**总线**而非设备。构造函数被声明为 `private`，只有 `bsp_spi` 是友元，从结构上杜绝第二个实例：

```C++
// pyro_spi_drv.h
class spi_drv_t
{
    friend class bsp_spi;   // 唯一构造入口
  public:
    ~spi_drv_t();
    spi_drv_t(const spi_drv_t &)            = delete;
    spi_drv_t &operator=(const spi_drv_t &) = delete;
  private:
    explicit spi_drv_t(SPI_HandleTypeDef *hspi);
};
```

### 2. 总线所有权：忙则挂起，而非报错

多从机并发访问时，早期写法是「查一下 `dma_busy()`，忙就返回失败」。这有两个问题：检查与占用之间可被抢占（TOCTOU），且把「排队」误报成「总线故障」，污染诊断计数。

现改为阻塞在互斥量上等待：

```C++
// pyro_spi_drv.cpp
status_t spi_drv_t::acquire_bus(const uint32_t waittime)
{
    return (xSemaphoreTake(_bus_mutex, pdMS_TO_TICKS(waittime)) == pdTRUE)
             ? PYRO_OK : PYRO_BUSY;
}
```

**为什么锁不藏在 `tx_dma()` 内部**：一笔 DMA 事务在设备层是「发起 → 等待 → 收取」三段调用，锁必须跨越这三个函数保持。若在 `tx_dma` 内加锁、返回前解锁，则等待期间总线无人持有，别的设备可抢入，导致完成信号量上出现两个等待者（二值信号量只能唤醒一个），且完成钩子被后发起者覆盖。

### 3. 完成钩子：CS 上升沿是事务边界

`set_cplt_hook()` 注册的回调在 DMA 完成/错误时、给完成信号量之前执行。当前唯一用途是拉高片选。

这不是「省掉唤醒延迟」的性能优化，而是协议要求：**CS 上升沿是器件唯一的原子事务边界**——器件在此沿提交写入、复位相位序列器、释放 MISO 为高阻。若 CS 迟迟不拉高，下一笔事务的 `cs_low()` 对已是低电平的引脚是 no-op，器件会把两笔独立事务合并成一笔（地址字节被当数据、FIFO 指针少推进一次）。

### 4. 钩子的绑定生命周期

钩子是**单槽**的，被同总线的多颗从机共享，因此绑定必须与总线所有权严格对齐：

```
acquire_bus()              ← 取得总线
  set_cplt_hook(self)      ← 绑定（只可能被自己看到）
  ...发起、等待、收取...
  set_cplt_hook(nullptr)   ← 解绑（业务已完成）
release_bus()              ← 归还总线
```

绑定在取锁之后，别设备改不到它；解绑在放锁之前，总线交出去时钩子已清空，即使随后出现迟到的完成中断（超时/`abort` 后），ISR 也不会误调上一个设备的释放函数。

设备层把这对操作下沉到 `bus_acquire()` / `bus_release()` 中，因此**所有路径（含发起失败）自动覆盖**，无需逐条路径手工解绑。

### 5. 为什么用裸函数指针而非 `std::function`

本回调在 ISR 中执行，且每笔事务前都要重设。实测替换为 `std::function` 后：

| 指标 | 裸指针 | `std::function` | 增量 |
|---|---|---|---|
| Flash | 116 052 B | 117 364 B | +1 312 B |
| 采集平均耗时 | 157 µs | 173 µs | +10 % |

裸指针的调用是一次已知地址的跳转，赋值是两条字写入，对 1 ms 采集周期最友好。

### 6. 锁的参考开销

以下数据取自 STM32G431（Cortex-M4 @170 MHz，Release `-Os`），**仅作量级参考**，实际值随内核、主频与 FreeRTOS 配置变化：

| 操作 | 平均周期 | 换算 |
|---|---|---|
| `acquire_bus()`（无竞争） | 约 211 | 约 1.2 µs |
| `release_bus()` | 约 324 | 约 1.9 µs |
| 每拍 2 对合计 | — | 约 6.3 µs |

开销主要不在算法，而在 FreeRTOS 自身的固定成本：`configASSERT` 全部启用（含 `xTaskGetSchedulerState()` 调用）、`vPortEnterCritical`/`vPortExitCritical` 为非内联函数、以及 mutex 的优先级继承记录。

因此代码注释中不写具体耗时——那是平台属性，不是接口契约。

### 7. ISR 路由

`USE_HAL_SPI_REGISTER_CALLBACKS = 0`，`HAL_SPI_RegisterCallback` 不可用，因此用弱符号覆盖三条 HAL 回调，全部收敛到 `notify_from_isr()`：

```C++
// pyro_spi_drv.cpp
extern "C" PYRO_ISR_TEXT void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) { spi_notify(hspi); }
extern "C" PYRO_ISR_TEXT void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)    { spi_notify(hspi); }
extern "C" PYRO_ISR_TEXT void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)     { spi_notify(hspi); }
```

路由表用定容 `map_t` 而非 `std::map`：后者会把 libstdc++ 的红黑树与异常路径链进固件（实测 5 784 B）。

完成点执行顺序不可交换：

```C++
state.busy = 0;                       // ① 先清标志，防迟到信号
_err       = HAL_SPI_GetError(_hspi); // ② 快照错误码
if (_cplt_hook) { _cplt_hook(_cplt_ctx); }  // ③ 放 CS（硬件时序最紧）
xSemaphoreGiveFromISR(_done, &woken);       // ④ 再唤醒任务
portYIELD_FROM_ISR(woken);
```

---

## Part 2: 快速上手 (Quick Start)

### 1. 取得总线引用

外部不可自行构造 `spi_drv_t`，必须经 BSP：

```C++
// imu_capture.cpp
pyro::spi_drv_t &s_spi_bus = pyro::bsp_spi::get_spi1();
```

`get_spi1()` 内部是函数局部静态，首次调用时才构造，必然晚于 `main()` 中的 `MX_SPI1_Init()`。**注意不要写成文件作用域对象**，否则会在 C++ 静态初始化期（早于 `main`）构造，此时 HAL 句柄尚未初始化、FreeRTOS 堆也未就绪。

### 2. 挂载从机

每颗从机一个设备对象，共用同一个总线引用，只差片选引脚：

```C++
// 从机 A（IMU，CS = PA4）
s_bus = new (std::nothrow) lsm6dsv16x_spi_t(*s_spi_bus, GPIOA, GPIO_PIN_4, true);

// 从机 B（另一颗器件，CS = PA15），共用同一条总线
s_bus2 = new (std::nothrow) lsm6dsv16x_spi_t(*s_spi_bus, GPIOA, GPIO_PIN_15, true);
```

### 3. 一笔 DMA 事务的完整流程

设备层已经把锁与钩子封装好，上层按三段式使用：

```C++
// ① 发起：内部取得总线、绑定钩子、拉低 CS、启动 DMA
if (!_device.read_regs_request(FIFO_STATUS1, 2U)) { /* 失败 */ }

// ② 让出 CPU 等待完成通知（此期间融合任务可运行）
if (!_device.wait_dma(LSM6DSV16X_CPLT_TIMEOUT_MS)) { /* 超时 */ }

// ③ 收取：只读本设备缓冲，不再触碰共享 DMA 状态
if (!_device.read_regs_collect(st, 2U)) { /* 失败 */ }
```

发起与等待分离是为了**释放 CPU**——等待发生在 `xSemaphoreTake(_done)` 上，任务挂起，不轮询寄存器。

### 4. 阻塞式收发

配置期的小读写可用轮询版本。它由任务侧释放 CS，不触发完成钩子：

```C++
// 阻塞读（HAL 轮询）
_spi_drv.txrx(tx, rx, size, LSM6DSV16X_TIMEOUT_MS);

// 阻塞写
_spi_drv.tx(tx, size, LSM6DSV16X_TIMEOUT_MS);
```

**阻塞路径期间 SCK 一直被本设备占用**，故同样需要持有总线锁，否则别的设备此时拉低自己的 CS 会与本设备的 MISO 驱动冲突。

### 5. 超时与恢复

`wait_dma()` 超时会中止 DMA 并清零 `state.busy`。此时 ISR 再进来会因 `busy == 0` 提前返回、不再调用完成钩子，因此 **CS 必须由任务侧补一次释放**：

```C++
// lsm6dsv16x_middleware.cpp
bool lsm6dsv16x_spi_t::wait_dma(const uint32_t timeout_ms)
{
    if (_spi_drv.wait_dma(timeout_ms) == pyro::PYRO_OK) { bus_release(); return true; }
    (void)_spi_drv.abort();
    cs_high();          // 任务侧兜底释放
    _result_ready = false;
    bus_release();
    return false;
}
```
