/**
 * @file    ins_service.h
 * @brief   INS 服务：器件采集 + 姿态融合 + 装配归一化 + 航向归零 + 统一快照。
 *
 * ============================ 结构参考 ============================
 * 参考 Wheel_leg/PYRo-wl 的 `Component/INS`：
 *   pyro_ins.h/.cpp          —— ins_drv_t：持有器件、姿态模型、常驻任务，对外给结果
 *   pyro_ins_attitude.h/.cpp —— 姿态模型（自研四元数 EKF）
 *
 * 本工程对应关系：
 *   ins_drv_t         → ins_service_t（本文件）
 *   ins_attitude_t    → pyro::vqf_t（PYRo/Algorithm/VQF，已有主机回放回归）
 *   ins_config_t      → Module/Config/imu_config.h + imu_mount.h
 * 姿态模型不在本层重复实现——`pyro::vqf_t` 已经是"算法 + ESKF"，再包一层只是冗余。
 *
 * ============================ 与参考的有意差异 ============================
 * · 不提供 _b/_n 两套共 13 个 getter：输出只有安装坐标系的一份快照。
 *   参考实现的 getter 各自读私有缓存，两次调用之间任务可能翻拍，跨字段不同拍；
 *   协议要求"同组"时那正是错数据。
 * · 快照带 valid / heading_zeroed：参考实现所有 getter 恒返回 OK，
 *   调用方无法判断姿态是否已收敛（此时"去重力加速度"之类的派生量是错的）。
 * · 装配方向映射在本层完成，驱动只输出器件坐标系（见 Config/imu_mount.h）。
 * · 航向零点由产品定义（6 轴融合的 yaw 不可观测），参考实现没有这一层。
 *
 * ============================ 为什么是单任务 ============================
 * 采集与融合同一个任务顺序执行。双任务（采集 + 算法）实测有三类问题：环形缓冲
 * head 在处理期间漂移导致读到错位样本、用临界区保护会把 SPI/DMA 中断一并屏蔽
 * （采集从 1 kHz 掉到 254 Hz）、双缓冲计数清零时机不当导致时间戳倒退。
 * 代价是融合耗时计入 1 ms 周期：实测 update() 平均 518 µs、峰值 793 µs（Release），
 * 峰值距 1 ms 还有约 200 µs 留给两段 SPI/DMA 与快照发布。
 */

#ifndef IMU_MODULE_IMU_INS_SERVICE_H
#define IMU_MODULE_IMU_INS_SERVICE_H

#include <cstdint>

#include "imu_snapshot.h"
#include "pyro_core_def.h"

namespace imu_module
{

class ins_service_t
{
  public:
    static ins_service_t &instance();

    /** 启动：绑定总线与器件、初始化、配置算法、创建常驻任务。
     *  仅可在调度器启动后的任务上下文调用。
     *  @retval PYRO_OK           成功
     *  @retval PYRO_ERROR        器件初始化失败
     *  @retval PYRO_PARAM_ERROR  算法配置失败
     *  @retval PYRO_NO_MEMORY    任务创建失败（FreeRTOS 堆不足）
     *  @retval PYRO_ALREADY_INIT 重复调用 */
    pyro::status_t init();

    /** 取一次完整快照（seqlock）。
     *  任意【任务】上下文可调用；**不可在 ISR 调用**——读侧依赖"写方一定能拿到
     *  CPU 把序号置回偶数"，写方在挂起调度器的窗口内发布，任务级读者不会空转。 */
    imu_snapshot_t snapshot() const;

    [[nodiscard]] bool started() const { return _started; }

  private:
    ins_service_t() = default;

    /* 任务类需要驱动文件内的硬件静态对象与 _round()。 */
    friend class ins_task_t;
    friend class cap_task_t;

    /** 采集任务体：只做 read()（FIFO → SPSC 环形缓冲），1 ms 固定周期。 */
    void _capture_loop();

    void _round();   /**< 一次融合 + 发布，由算法任务每 1 ms 尽力调用 */

    /** 航向零点门控：三项条件同时成立才建立，另加超时兜底。
     *  @param yaw_install  本拍安装系 yaw，rad（只读，用于捕获零点）
     *  @param initialized  可信重力方向已建立
     *  @param rest         连续静止已确认
     *  @param bias_sigma   零偏标准差上界，rad/s */
    void _update_heading_zero(float yaw_install, bool initialized, bool rest,
                              float bias_sigma_rps);

    struct heading_ref_t
    {
        float   yaw0    = 0.0f;  /**< 归零时刻的 yaw，rad */
        bool    zeroed  = false;
        uint8_t quality = 0U;    /**< 0 未建立 / 1 静止收敛 / 2 超时兜底 */
    };

    heading_ref_t _heading{};
    uint32_t      _zero_wait = 0U;  /**< 归零门控已等待的融合轮次 */
    uint32_t      _sequence  = 0U;
    bool          _started   = false;

    /* 快照发布（seqlock）：写方在 vTaskSuspendAll 窗口内完成"置奇→写→置偶"。 */
    mutable imu_snapshot_t    _snapshot{};
    mutable volatile uint32_t _snapshot_seq = 0U;
};

} // namespace imu_module

/* ============================ 调试观测 ============================
 * 调试器 watch 用，零依赖，可随时删除。前缀沿用 imu_service_*（历史命名），
 * 以免破坏既有的调试脚本与文档引用。 */
extern "C" {
extern volatile int32_t  imu_service_init_result;      /* 0=就绪 1=未运行 <0=失败 */
extern volatile int32_t  imu_service_vqf_ready;        /* 1=算法配置成功 */
extern volatile uint32_t imu_service_read_ok;          /* read() 返回 true 的累计次数 */
extern volatile uint32_t imu_service_algo_rounds;      /* 实际执行融合的轮数 */
extern volatile uint32_t imu_service_vqf_consumed;     /* 算法累计消费样本数（两路合计） */
extern volatile uint32_t imu_service_err_bus;          /* 总线失败次数 */
extern volatile uint32_t imu_service_err_data;         /* FIFO 溢出 / 未知 TAG / 背压丢样 合计 */
extern volatile uint32_t imu_service_err_ovr;          /* 其中：器件 FIFO 溢出（整拍未被读走） */
extern volatile uint32_t imu_service_err_backpressure; /* 其中：环形缓冲满、丢弃新样本 */
extern volatile uint32_t imu_service_err_timeout;      /* 等完成通知超时次数 */
extern volatile uint32_t imu_service_vqf_err;          /* 算法返回 ERROR 的次数 */
extern volatile uint32_t imu_service_vqf_warn;         /* 算法返回 WARNING 的次数 */
extern volatile int32_t  imu_service_vqf_status;       /* 最近一次 update() 返回值 */
extern volatile uint32_t imu_service_vqf_us_max;       /* 单批融合耗时峰值（µs，每秒刷新）*/
extern volatile uint32_t imu_service_vqf_us_avg;       /* 单批融合平均耗时（µs，每秒刷新）*/
extern volatile uint32_t imu_service_loop_us_avg;      /* 整拍平均耗时（读+融合+发布，µs）*/
extern volatile uint32_t imu_service_heap_free_min;    /* 任务拉起后的 FreeRTOS 堆余量 */
extern volatile uint32_t imu_service_ins_stack_free;    /* 融合任务栈余量（词，1 s 刷新） */
extern volatile uint32_t imu_service_cap_stack_free;    /* 采集任务栈余量（词，1 s 刷新） */
extern volatile uint32_t imu_service_cap_rounds;        /* 采集轮次（1 kHz 硬周期） */
extern volatile uint32_t imu_service_cap_us_max;        /* 单拍 read() 峰值耗时（µs） */
extern volatile uint32_t imu_service_cap_us_avg;        /* 单拍 read() 平均耗时（µs） */
extern volatile float    imu_service_quat[4];          /* 安装系 [w,x,y,z] */
extern volatile float    imu_service_quat_norm;        /* 应恒 ≈ 1.0 */
extern volatile float    imu_service_accel_mps2[3];    /* 安装系加速度 */
extern volatile float    imu_service_gyro_rps[3];      /* 安装系角速度 */
extern volatile float    imu_service_bias_rps[3];      /* 安装系零偏估计 */
extern volatile float    imu_service_roll;             /* ° */
extern volatile float    imu_service_pitch;            /* ° */
extern volatile float    imu_service_yaw;              /* °（已归零） */
extern volatile uint32_t imu_service_rest;             /* 1=当前静止 */
extern volatile float    imu_service_rest_t;           /* 静止计时，s */
extern volatile uint32_t imu_service_heading_zeroed;   /* 1=yaw 零点已建立 */
extern volatile uint32_t imu_service_heading_quality;  /* 0/1/2 */
extern volatile float    imu_service_acc_confidence;   /* 加速度可信度，0..1 */
extern volatile float    imu_service_acc_nis;          /* 最近两维倾斜观测 NIS */
extern volatile uint32_t imu_service_acc_rejected;     /* 1=最近加速度受门控/恢复等待 */
}


#endif /* IMU_MODULE_IMU_INS_SERVICE_H */
