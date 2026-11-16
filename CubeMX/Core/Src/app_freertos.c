/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_module.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask
   * 移植调整：256 词（1 KB）时，imu_module_init() 的调用链
   *   imu_module_init → imu_service_start → capture.init() → s_imu.init()
   *   → calibrate() → read() → SPI 中间件 → 驱动解包
   * 在 -O0 下累计栈帧超过 1 KB，会踩坏 FreeRTOS 堆中相邻的 TCB / 任务栈，
   * 表现为运行约 1 秒后 PendSV 恢复上下文时 r3 为垃圾值 → HardFault。
   * 这里给到 1024 词（4 KB），留足余量。 */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* IMU 链路启动：调度器已运行，满足信号量 / HAL_Delay / DMA 等待前提。
   * 本调用内部完成器件初始化，并拉起【采集】与【融合+发布】两个常驻任务
   * （拆分原因见 Module/IMU/ins_service.cpp 文件头）。
   * 失败原因见 imu_module_init_result 与 imu_service_* 诊断量。 */
  (void)imu_module_init();

  /* 回收引导任务：初始化完成后本任务无事可做，自删归还 1024 词（4 KB）栈与 TCB。
   *
   * 为什么需要这一步（实测数据）：12 KB 堆里原本放
   *   defaultTask 4K + ins 4K + pub 1.5K + IDLE 0.5K + TCB ≈ 10 KB，余 3176 B。
   * 拆出采集任务后多出 ins_cap，若不回收引导任务则峰值超过 12288 B，
   * imu_pub 创建失败 -> imu_module_init_result = -4（已实测到该值）。
   *
   * 注意：vTaskDelete(NULL) 只是把本任务标记为待删，真正的栈回收发生在
   * idle 任务执行 prvCheckTasksWaitingTermination 时，因此【不能】指望
   * "删完立刻用回这 4 KB 去创建别的任务"。所以创建期的堆峰值必须靠
   * 另一条路压下来 —— 见 Module/Config/imu_config.h 里 ins_cap 的栈取值。
   * 这里自删的意义是：稳态运行期把 4 KB 还给系统（实测 heap_free 由
   * 创建期的紧张值回到约 4.7 KB）。 */
  vTaskDelete(NULL);

  /* 以下不可达，保留以明确语义 */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

