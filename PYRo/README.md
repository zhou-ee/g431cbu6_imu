# PYRo（移植子集）使用与移植说明

本目录是 PYRo 库（上游位置 `E:\Software\Develop\RM\Wheel_leg\PYRo-wl\PYRo`，目标 STM32H723 / Cortex-M7）向本工程
（**STM32G431CB / Cortex-M4F，128KB Flash / 32KB SRAM**，CubeMX + CMake + Ninja + FreeRTOS V10.3.1）的移植子集。

| 上游模块 | 本目录 | 说明 |
| --- | --- | --- |
| `Peripheral/CAN` | `Peripheral/CAN/` | FDCAN 驱动 + BSP（`can_drv_t` / `bsp_can`） |
| `Peripheral/UART` | `Peripheral/UART/` | DMA 双缓冲串口驱动 + BSP（`uart_drv_t` / `bsp_uart`） |
| `Peripheral/DWT` | `Peripheral/DWT/` | CYCCNT 高精度计时（`dwt_drv_t`） |
| `Core/Lock` | `Core/Lock/` | `mutex_t` / `scoped_mutex_t` / `rw_lock` / `read_scope_lock` / `write_scope_lock` |
| `Core/Task` | `Core/Task/` | `task_base_t`（两阶段初始化任务基类） |
| 上游依赖 | `Core/Def`、`Core/ETL`、`Core/Memory` | 状态码 `pyro_core_def.h`、`map_t`、DMA 专用堆 `pyro_core_dma_heap.c`、全局 new/delete 重定向 `pyro_core_mem.cpp`（UART/CAN 直接依赖，必须一起移植） |
| — | `Core/Config/` | `pyro_core_config.h`（实例宏）、`pyro_platform.h`（**本工程新增的平台适配层**） |

未移植：`Algorithm`、`Component`、`Device`、`Debug`、`Core/FSM`、`Module` 以及上游对 dsppp/CMSIS-DSP 的依赖。

## 1. 编译接入

顶层 `CMakeLists.txt` 已接好：

```cmake
add_subdirectory(PYRo)                       # PYRo 是 OBJECT 库
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx PYRo)
set(CMAKE_CXX_STANDARD 17)                   # PYRo 需要 C++17（DWT 用 inline static 成员）
```

`PYRo/CMakeLists.txt` 通过 `target_link_libraries(PYRo PUBLIC stm32cubemx)` 拿到 HAL / FreeRTOS / CMSIS 的头文件与芯片宏，
无需手工再写 include 路径。

## 2. 平台适配层 `Core/Config/pyro_platform.h`

把上游 H7 与本工程 M4 的差异集中成 3 个宏，其余源码与上游逐行一致，便于后续同步上游：

| 宏 | H7（上游） | G4（本工程） | 影响 |
| --- | --- | --- | --- |
| `PYRO_ISR_TEXT` | `__attribute__((section(".itcm_text")))` | 空（留在 Flash） | CAN/UART 的中断服务代码段；M4 无 ITCM |
| `PYRO_CAN_HAS_FIFO_WATERMARK` | 1 | 0 | G4 HAL 无 `HAL_FDCAN_ConfigFifoWatermark()` |
| `PYRO_DMA_HEAP_ATTR` | `section(".dma_heap")` | 空（留在 `.bss`） | G4 的 32KB SRAM 全部可被 DMA 访问，不需要专用段 |

> 该头文件依赖芯片宏（`STM32H7xx` / `STM32G4xx`），必须在包含 `main.h` 之后包含。

## 3. 上板前必须补的工程侧配置

### 3.1 FDCAN1：标准滤波器数量必须 ≥ 1（**已代为配置**）

`can_drv_t::init()` 会往 `FilterIndex = 0` 写一条掩码滤波器。若 CubeMX 里
`FDCAN1 → Basic Parameters → Std Filters Nbr = 0`，则硬件侧标准滤波器列表长度为 0：
写入的滤波器元素会被硬件忽略，叠加驱动设置的全局滤波（非匹配报文一律 `FDCAN_REJECT`），
结果是 **CAN 收不到任何报文**（发送不受影响）。

已修改（可随时回退）：
- `lsm6dsv_test.ioc`：`FDCAN1.StdFiltersNbr=1`（并加入 `FDCAN1.IPParameters`）
- `Core/Src/fdcan.c`：`hfdcan1.Init.StdFiltersNbr = 1;`

### 3.2 USART2：RX/TX DMA 与中断（**已配置**）

UART 驱动用 `HAL_UARTEx_ReceiveToIdle_DMA()` 做不定长接收，要求该串口有 **RX DMA 通道**；
非阻塞发送 `write(p, size)` 还要求 **TX DMA 通道**。当前 CubeMX 生成代码**已经配好**：

| 项 | 位置 | 配置 |
|---|---|---|
| `hdma_usart2_rx` | `Core/Src/usart.c` `HAL_UART_MspInit()` | `DMA1_Channel3` + `DMA_REQUEST_USART2_RX`，Normal，Byte |
| `hdma_usart2_tx` | 同上 | `DMA1_Channel4` + `DMA_REQUEST_USART2_TX`，Normal，Byte |
| 句柄绑定 | 同上 | `__HAL_LINKDMA(uartHandle, hdmarx/hdmatx, ...)` |
| 中断 | `Core/Src/usart.c` / `Core/Src/dma.c` / `Core/Src/stm32g4xx_it.c` | `USART2_IRQn`、`DMA1_Channel3/4_IRQn` 均使能，优先级 5 |

优先级 5 恰好等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`，满足
FreeRTOS "ISR 不得高于该值"的要求（数值 ≥ 5 才允许调用 `FromISR` API）。

历史上本节写的是"尚未配置、需要手工加 DMA"。该结论已过期：动手前请以
`Core/Src/usart.c` 的 `HAL_UART_MspInit()` 实际内容为准；若 CubeMX 重新生成后
`huart2.hdmarx/hdmatx` 又变回 `NULL`，驱动侧仍会**提前返回 `PYRO_ERROR`** 而不会跑飞
（见 §5 第 5、15 条），现象是"UART 完全不通但没有 HardFault"。

### 3.3 调度器必须启动，否则 CAN/Lock/Task 会把中断永久关掉

`Core/Src/main.c` 里的 `MX_FREERTOS_Init()` / `osKernelStart()` **已经启用**，入口
`StartDefaultTask` 调用 `imu_module_init()` 拉起采集链路。下面这些约束仍然成立——
任何"临时把调度器关掉调试"的做法都会踩到：

- FreeRTOS 的 Cortex-M4 端口里 `uxCriticalNesting` 在调度器启动前是哨兵值 `0xaaaaaaaa`
  （`port.c`），只有 `xPortStartScheduler()` 才会把它清 0；
- 所以调度器启动前调用 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`，退出时嵌套计数
  回不到 0，**`portENABLE_INTERRUPTS()` 永远不会执行**：PRIMASK 一直为 1，
  TIM6 时基停摆、`HAL_GetTick()` 冻结、DMA/FDCAN 中断全部失效（而且 `configASSERT` 查不出来）；
- 受影响 API：`can_drv_t::send_msg()` / `register_rx_msg()`、`can_msg_buffer_t::get_data()`、
  以及 `Lock` / `Task` 全部功能；UART 的 DMA 接收中断里也会调 `portYIELD_FROM_ISR`，同样要求调度器在跑。
- 调度器启动**之前**只能安全使用：`dwt_drv_t`、UART 的阻塞式 `write(p, size, waittime)`（轮询）。

该工程可以安全启动调度器：`SVC/PendSV/SysTick` 由 FreeRTOS 端口提供，HAL 时基已经切到 TIM6，两者不冲突。

### 3.4 初始化顺序

所有 `bsp_*::init_all()`、`dwt_drv_t::init()` 都必须在 CubeMX 的外设初始化之后调用
（即 `MX_FDCAN1_Init()` / `MX_USART2_UART_Init()` / `MX_DMA_Init()` 之后），否则驱动的 HAL 句柄尚未填充。
`can_drv_t::init()` 依赖 `HAL_FDCAN_Init()` 已经设好 `msgRam` 地址且 `State == READY`，并且只能成功执行一次。

### 3.5 堆空间

- FreeRTOS 主堆 `configTOTAL_HEAP_SIZE = 12288`（`Core/Inc/FreeRTOSConfig.h`）。
  UART 的 `std::map`/`std::vector`/`std::function` 节点、任务栈、信号量都从这里分配
  （每个互斥锁约 88B、`rw_lock` 一次要 3 个信号量约 264B、每个任务至少 512B）。
  当前 `defaultTask` 与 `imu` 任务各占 1024 词（4 KB），实测剩余最低 3176 B——
  再加任务或队列前需要扩堆。
- DMA 堆 `configTOTAL_DMA_HEAP_SIZE = 2048`（`Core/Memory/pyro_core_dma_heap.c`）。
  按 heap_4 规则 2048B 实际可用 2040B，每个 512B 缓冲实占 520B（含 8B 块头）：
  一个串口需要 2 个缓冲 = 1040B 够用，**再加一个串口就不够了**（需 ≥ 2×串口数×520B）。

## 4. 快速上手

> 一个把 4.1~4.4 全部用起来的完整可运行例子：`Core/Src/periph_test.cpp`
> （IMU/SPI + UART DMA 收发 + CAN 收发，结果用串口文本行与 CAN 帧同时输出）。

### 4.0 初始化顺序（`main.c` 的 `USER CODE BEGIN 2` 段）

```cpp
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  pyro::dwt_drv_t::init(170);      /* 时钟配置完成后即可，越早越好 */
  pyro::bsp_can::init_all();       /* 需要 StdFiltersNbr >= 1（见 3.1） */
  pyro::bsp_uart::init_all();      /* 需要 USART2 RX/TX DMA（见 3.2），失败时返回 PYRO_ERROR */

  /* 要用 CAN 收发 / Lock / Task，必须先让调度器跑起来（见 3.3）：
     MX_FREERTOS_Init();
     osKernelStart();                 // 不再返回
  */
  /* USER CODE END 2 */
```

### 4.1 DWT（高精度计时）

```cpp
#include "pyro_dwt_drv.h"

pyro::dwt_drv_t::init(170);              /* 本工程 SYSCLK = 170MHz，必须在时钟配置之后调用一次 */

uint32_t last = pyro::dwt_drv_t::get_current_ticks();
// ... 待测代码 ...
float dt = pyro::dwt_drv_t::get_delta_t(&last);   /* 秒，自动更新 last */
pyro::dwt_drv_t::delay_us(50);                    /* 忙等微秒延时 */
float ms = pyro::dwt_drv_t::get_timeline_ms();    /* 自 init() 以来的毫秒数 */
```

> 注意：`get_timeline_*()` 靠轮询检测 32 位计数器溢出（170MHz 下约 25.3s 一圈），
> 长时间不调用会丢圈数。

### 4.2 CAN

```cpp
#include "pyro_bsp_can.h"

pyro::can_msg_buffer_t rx_buf(0x201);            /* 想收哪个 ID 就建哪个缓冲 */

void can_init() {
    if (pyro::PYRO_OK != pyro::bsp_can::init_all()) { /* 检查 StdFiltersNbr 是否为 1 */ }
    pyro::bsp_can::get_can1().register_rx_msg(&rx_buf);
}

std::array<uint8_t, 8> data{};
if (rx_buf.get_data(data)) {                     /* 返回 true = 自上次 mark_read() 后有新数据 */
    rx_buf.mark_read();
}
uint8_t tx[8] = {0};
pyro::bsp_can::get_can1().send_msg(0x201, tx);   /* 经典帧，固定 8 字节 */
```

### 4.3 UART（DMA 双缓冲 + 回调分发）

```cpp
#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"

pyro::bsp_uart::init_all();                      /* 启动 USART2 的 DMA 接收（需先满足 3.2） */

void MyModule::attach() {
    PYRO_UART2.add_rx_event_callback(            /* 等价于 pyro::bsp_uart::get_uart2() */
        [this](uint8_t *buf, uint16_t len, BaseType_t &woken) -> bool {
            return this->parse(buf, len, woken); /* 返回 true = 已消费，不再传给后面的回调 */
        },
        reinterpret_cast<uint32_t>(this));       /* owner：注销时用同一个值 */
}

PYRO_UART2.write(tx, sizeof(tx), 100);           /* 阻塞发送（超时 100ms） */
PYRO_UART2.write(tx, sizeof(tx));                /* DMA 非阻塞发送 */
```

> 回调在中断上下文执行，不要做耗时操作；`tx` 缓冲区必须位于 DMA 可访问的内存（本工程全部 SRAM 均可）。

### 4.4 Lock / Task

```cpp
#include "pyro_mutex.h"
#include "pyro_rw_lock.h"
#include "pyro_task.h"

pyro::mutex_t g_mutex;
pyro::rw_lock g_cfg_lock;

void reader() {
    pyro::read_scope_lock lock(g_cfg_lock);       /* 构造即加锁，析构自动解锁 */
    /* ... 读共享数据 ... */
}

class ImuTask : public pyro::task_base_t {
public:
    ImuTask() : task_base_t("imu", 256, 256, priority_t::ABOVE_NORMAL) {}
protected:
    pyro::status_t init() override { return pyro::PYRO_OK; }
    void run_loop() override { for (;;) { /* ... */ vTaskDelay(1); } }
};
ImuTask g_imu_task;
g_imu_task.start();                              /* 先跑 init()，再创建常驻 run_loop() 任务 */
```

> `priority_t` 共 7 级（IDLE…REALTIME），内部按 `configMAX_PRIORITIES = 7` 映射到 0…6。

## 5. 与上游的差异清单

有意修改（每条都已在上游对应位置加中文注释）：

| # | 文件 | 改动 | 原因 |
| --- | --- | --- | --- |
| 1 | `Core/Config/pyro_platform.h`（新增） | 平台宏集中层 | 见 §2 |
| 2 | `Peripheral/CAN/pyro_can_drv.cpp` | `.itcm_text` → `PYRO_ISR_TEXT`；`HAL_FDCAN_ConfigFifoWatermark` 用 `PYRO_CAN_HAS_FIFO_WATERMARK` 包住 | G4 无 ITCM、无该 HAL API |
| 3 | `Peripheral/UART/pyro_uart_drv.{h,cpp}` | H7 HAL 头 → `main.h`；按平台包含 `*_hal_uart_ex.h` | G4 的 `hal_uart.h` 不连带包含 `_ex`，串口扩展 API 找不到 |
| 4 | `Peripheral/UART/pyro_uart_drv.cpp` | 析构 `vPortFree` → `vPortDmaFree` | 缓冲区来自 `pvPortDmaMalloc`，上游此处会破坏堆 |
| 5 | `Peripheral/UART/pyro_uart_drv.cpp` | `enable_rx_dma()` 增加 `_huart->hdmarx == nullptr` 提前返回 | 无 RX DMA 时 HAL 内部会解引用空指针 |
| 6 | `Peripheral/UART/pyro_uart_drv.{h,cpp}` | 4 个回调注册接口用 `#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)` 包住 | 本工程该项为 0，G4 HAL 把该 API 整体编译掉了；打开该选项后自动恢复 |
| 7 | `Peripheral/CAN/pyro_bsp_can.{h,cpp}` | 只留 `get_can1()`，`can2/can3` 返回 `nullptr` | G431 只有 1 个 FDCAN，且 CubeMX 只配了 FDCAN1 |
| 8 | `Peripheral/UART/pyro_bsp_uart.{h,cpp}` | 只留 `get_uart2()` | 同上，CubeMX 只配了 USART2 |
| 9 | `Core/Config/pyro_core_config.h` | 只留 `PYRO_UART2` | 上游的 UART1/5/7/10 在本 MCU 上不存在 |
| 10 | `Core/ETL/map.h` | `_size` 类内初始化为 0；`erase()` 仅在找到键时移动元素并递减 | 上游未初始化 `_size`（非静态对象上是 UB）、`erase()` 找不到键时也会递减计数 |
| 11 | `Core/Lock/pyro_rw_lock.h` | `"freertos.h"` → `"FreeRTOS.h"` | 大小写敏感文件系统上上游无法编译 |
| 12 | `Core/Memory/pyro_core_dma_heap.c` | 默认堆大小 24576 → 2048；`.dma_heap` 段属性 → `PYRO_DMA_HEAP_ATTR` | 本 MCU 只有 32KB SRAM；无需专用段 |
| 13 | `Core/Task/pyro_task.cpp` | 构造函数加 `(void)init_stack;` | 上游该参数未使用（保留原设计，仅消除告警） |
| 14 | `Core/Memory/pyro_core_mem.cpp`（**补移植**） | 上游的全局 `operator new/delete` → `pvPortMalloc/vPortFree` 重定向 | **首轮移植遗漏项**。UART 驱动的 `std::map/std::function/std::vector` 走全局 `new`，不重定向就会落到 newlib 的 `_sbrk` 堆：不受 FreeRTOS 记账、无锁（newlib 的 `__malloc_lock` 在本工程是空桩）、失败时 `abort`+stdio |
| 15 | `Peripheral/UART/pyro_uart_drv.cpp` | `write(p, size)` 增加 `hdmatx == nullptr` 提前返回 | G4 HAL 在无 TX DMA 时**静默返回 `HAL_OK`**，却把 `gState` 永久留在 `BUSY_TX` → 之后所有发送（含阻塞式）都只能得到 `HAL_BUSY` |
| 16 | `Peripheral/UART/pyro_uart_drv.cpp` | ISR 的 `xHigherPriorityTaskWoken` 由 `static` 改为每次复位的局部量 | 上游 static 变量永不复位，一旦被置位，此后每次中断退出都会强制一次 PendSV |
| 17 | `Peripheral/UART/pyro_uart_drv.cpp` | 构造函数改为“先分配缓冲、成功后才注册到 `uart_map`”，部分失败时回收已分配块 | 上游会注册一个半初始化实例（`init_flag=false`、缓冲为 NULL），且半失败时泄漏一块缓冲 |
| 18 | `Core/ETL/map.h` | 存储改 C 数组；`operator[]` 增加容量检查 + `full()`；ctor/dtor 改 `= default`；删除无法实例化的 `operator[] const` 重载 | 去掉 libstdc++ 断言 → `fprintf`/`abort`/stdio 链（Debug 实测 −4.9KB Flash，Release 下该分支能被优化掉）与 `__cxa_atexit` 注册链；容量耗尽不再越界写 |
| 19 | `Peripheral/CAN/pyro_can_drv.cpp` | `init()` 前置校验：`StdFiltersNbr` 与 `ExtFiltersNbr` 全为 0 时返回 `PYRO_PARAM_ERROR` | 把“CubeMX 没配滤波器 → 一帧也收不到”的静默失效变成显式错误 |
| 20 | `Peripheral/CAN/pyro_can_drv.cpp` | `register_rx_msg()` 增加容量检查，满时返回 `PYRO_NO_MEMORY` | `map_t` 容量上限是 10（见下），上游无检查会越界写 |
| 21 | `Core/Def/pyro_core_def.h` | 修好 `PYRO_ASSERT_RET` 在未定义 `USE_LOG` 分支里缺少的右括号 | 上游该宏本身是语法错误，一旦使用即编译失败 |
| 22 | `Core/Task/pyro_task.cpp` | `xTaskCreate` 失败时返回 `PYRO_NO_MEMORY`（上游继续往下走并返回 `PYRO_OK`） | “任务没建成”被静默吞掉（多半是 FreeRTOS 堆不够），调用者以为启动成功 |

已知但**未修改**的上游缺陷 / 使用注意事项：

- `Core/Task/pyro_task.cpp`：`start()` 实际是在**调用者上下文**直接执行 `init()`（并非创建临时任务），
  `init_stack` 不生效；`init()` 里不要放阻塞/依赖调度器的代码。
- `Core/Task/pyro_task.cpp`：`_loop_stack_depth == 0` 时不创建常驻任务（这是上游语义，不是错误）。
- `Core/ETL/map.h`：容量固定为 `_max_size = 10`，而 `pyro_can_drv.h` 里的 `MAX_ID_REGIST_NUM = 32` 只是残留常量。
  需要注册更多 CAN ID 时请改大 `_max_size`（代价：每个 `map_t` 实例 `_max_size × (sizeof(K)+sizeof(V))` 静态 RAM）。
- `Core/Lock/pyro_rw_lock.cpp`：`write_lock(timeout)` 返回 `false` 后**不得**再调 `write_unlock()`，
  否则 `_writer_waiting_count` 会多减，导致 `_read_gate` 错误归还或永久不归还。
- `Peripheral/CAN/pyro_can_drv.cpp`：`send_msg()` 在 BusOff 刚恢复的那一帧会返回 `PYRO_BUSY` 并丢弃（上游行为）。
- `Peripheral/CAN/pyro_can_drv.cpp`：`HAL_FDCAN_RxFifo0Callback()` 未按 `RxFifo0ITs` 掩码过滤；
  G4 的 Rx FIFO0 固定 3 个元素且**没有水位寄存器**（`FDCAN_IT_RX_FIFO0_MESSAGE_LOST` 也会进这个回调），
  高负载下可能丢帧（`HAL_FDCAN_GetRxMessage` 在 FIFO 空时返回 `HAL_ERROR`，不会读脏数据）。
- `Core/Memory/pyro_core_mem.cpp`：`operator new` 失败返回 `NULL` 而不是抛异常；
  在 `-fno-exceptions` 下由编译器生成的检查转为 `abort()`，即分配失败就是死机（不会静默继续）。

## 6. 资源开销（实测，arm-none-eabi-g++ 15.3）

以移植前的工程为基线（RAM 3040B）：

| 配置 | Debug RAM | Debug Flash | Release RAM | Release Flash |
| --- | --- | --- | --- | --- |
| 移植前 | 3040 B | 30344 B | 3040 B | 19144 B |
| 移植后（**不引用任何移植代码**） | 3176 B | 32620 B | 3176 B | 20220 B |
| 移植后（全量引用：`tests/pyro_port_link_check.cpp`） | 8832 B | 61796 B | 8832 B | 31996 B |

几点解读：

- **只编译、不使用**：Debug/Release 都只多约 **0.1KB RAM / 1~2.3KB Flash**。
  这部分来自 `HAL_FDCAN_RxFifo0Callback()`——它是对 HAL 弱符号的强覆盖，只要编译进来就会被
  `HAL_FDCAN_IRQHandler()` 引用，连带整条 CAN 驱动链路都无法被 `--gc-sections` 回收。
  这是驱动设计的固有代价，与是否调用无关（若在意，可改用 `HAL_FDCAN_RegisterRxFifo0Callback()`
  注册回调而不是提供强符号，需要 CubeMX 打开 `USE_HAL_FDCAN_REGISTER_CALLBACKS`）。
- **全量使用**：比基线多约 **5.8KB RAM / 12.9KB Flash（Release）/ 31.5KB（Debug -O0）**。
  RAM 里约 5KB 是两个堆（FreeRTOS 主堆 3KB + DMA 堆 2KB），驱动状态对象只占几百字节。
- Flash 增量主要来自 FreeRTOS 任务/信号量内核路径与 C++ 标准库模板
  （`std::map` / `std::function` / `std::vector`，UART 回调机制用到）。评估体积请以 **Release** 数字为准：
  `-O0` 的 Debug 构建会把模板与断言路径放大一倍多（§5 第 18 条就是被 Debug 放大出来的一条）。

## 7. 移植期自检工具

`tests/pyro_port_link_check.cpp` 会把所有移植模块都引用一遍，用于验证“能编过 + 能链接 + 体积开销”。
默认不参与编译；需要时把该文件加入 `PYRo/CMakeLists.txt` 的 `target_sources` 再构建即可（用完删掉该行）。
它靠一个 `.init_array` 里的锚点对象保持符号不被 `--gc-sections` 回收，锚点只取函数地址、不调用驱动，
因此不会产生运行期副作用。

## 8. 与上游同步的建议

1. 上游文件尽量**整文件覆盖**，然后按 §5 表格逐条重放本工程的适配改动（改动点都带 `移植`/`移植说明` 注释，便于检索）。
2. 平台相关差异只改 `Core/Config/pyro_platform.h` 与各文件头部的 HAL include 块，不要散落到业务逻辑里。
3. 新增/删除外设实例时，同步改 `pyro_bsp_*.{h,cpp}` 与 `pyro_core_config.h`。
4. 若上游模块本身有改动，优先看 `map.h`、`pyro_uart_drv.cpp`、`pyro_can_drv.cpp`、`pyro_core_dma_heap.c`
   这几处——它们在本移植里带了**修正性**改动（§5 的 #4、#5、#10、#15~#20），合并时不要被上游版本覆盖回去。
5. `Core/Memory/pyro_core_mem.cpp` 必须始终留在编译列表里：少了它，UART 里的 `std::map` 会把动态分配
   悄悄引到 newlib 堆上（不受 FreeRTOS 记账、无锁）。
