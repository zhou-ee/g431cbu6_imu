/* ============================================================================
 * lsm6dsv16x_reg.h —— LSM6DSV16X 寄存器映射（C++17 constexpr 版）
 *
 * 依据 ST 数据手册 DS13510 Rev 4；地址/位域/复位值与手册「Register mapping」各表
 * 逐条核对，并与 ST 官方驱动 lsm6dsv16x_reg.h 交叉比对。
 *
 * 命名（对齐数据手册）：
 *   <寄存器名>                寄存器地址            例 CTRL1、FIFO_CTRL3、FSM_OUTS1
 *   <寄存器名>_<字段名>_POS   字段起始位            例 CTRL1_ODR_XL_POS
 *   <寄存器名>_<字段名>_MSK   字段掩码（多比特）     例 CTRL1_ODR_XL_MSK
 *   <寄存器名>_<字段名>       单比特字段的值        例 CTRL3_BDU
 *   <寄存器名>_<字段名>_<值>  多取值字段的值        例 CTRL6_FS_G_500dps、CTRL8_FS_XL_4g
 *   字段名去掉与寄存器名尾部重复的前缀（INT1_/INT2_、FSM_OUTS<n>_），
 *   例 FSM_INT1 的 INT1_FSM3 → FSM_INT1_FSM3。
 *
 * 寄存器配置使用量程（4g / 500dps）与采样率（480Hz）编码；
 * 原始值换算见第 8 节，陀螺仪使用对应量程的 Sensitivity 常数（dps/LSB）。
 *
 * 目录
 *   0 通用            1 主页面         2 嵌入功能
 *   3 高级功能页 0    4 高级功能页 1   5 高级功能页 2
 *   6 传感器集线器    7 SPI2/OIS 侧    8 满量程与换算    9 用法示例
 *
 * ---------------------------------------------------------------------------
 * 4 个互相独立的地址空间，同一地址在不同空间含义完全不同！
 *   主页面         默认
 *   嵌入功能       FUNC_CFG_ACCESS = SPACE_EMBEDDED（0x80）
 *   高级功能页     在嵌入功能空间下再写 PAGE_SEL = PAGE_SEL_PAGE_0/1/2
 *   传感器集线器   FUNC_CFG_ACCESS = SPACE_SENSOR_HUB（0x40）
 * 例：0xD0 在页 0 是 FSM_EXT_MATRIX_ZZ_L，在页 1 却是 PEDO_SC_DELTAT_L。
 * ---------------------------------------------------------------------------
 * 本文件使用 namespace / constexpr，只能由 C++ 编译。
 * ========================================================================== */

#ifndef LSM6DSV16X_REG_H
#define LSM6DSV16X_REG_H

#if !defined(__cplusplus)
#error "lsm6dsv16x_reg.h 使用 namespace/constexpr，请用 C++ 编译"
#endif

#include <cstdint>

namespace LSM6DSV16X {

/* ============================== 0. 通用 ============================== */
constexpr uint8_t WHO_AM_I_ID = 0x70;      /* WHO_AM_I(0Fh) 固定返回该值 */
constexpr uint8_t SPI_READ_BIT = 0x80;     /* 地址字节 bit7 = 1 表示读 */
constexpr uint8_t SPI_WRITE_MASK = 0x7F;   /* 写操作：地址 & 0x7F */


/* ==================== 1. 主页面寄存器（默认地址空间，直接访问） ==================== */
/* ---------------- 1.1 接口与全局配置（01h~0Fh） ---------------- */
constexpr uint8_t FUNC_CFG_ACCESS = 0x01;                                /* 接口/嵌入功能空间访问控制（R/W, 复位 0x00） */
constexpr uint8_t FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS_POS = 7;           /* 位7 使能对嵌入功能配置寄存器的访问 */
constexpr uint8_t FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS = 0x1u << FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS_POS;
constexpr uint8_t FUNC_CFG_ACCESS_SHUB_REG_ACCESS_POS = 6;               /* 位6 使能对传感器集线器（I²C 主器件）配置寄存器的访问 */
constexpr uint8_t FUNC_CFG_ACCESS_SHUB_REG_ACCESS = 0x1u << FUNC_CFG_ACCESS_SHUB_REG_ACCESS_POS;
constexpr uint8_t FUNC_CFG_ACCESS_FSM_WR_CTRL_EN_POS = 3;                /* 位3 使能 FSM 对 CTRL 寄存器的控制（FSM 可以自主更改器件的某些配置） */
constexpr uint8_t FUNC_CFG_ACCESS_FSM_WR_CTRL_EN = 0x1u << FUNC_CFG_ACCESS_FSM_WR_CTRL_EN_POS;
constexpr uint8_t FUNC_CFG_ACCESS_SW_POR_POS = 2;                        /* 位2 器件全局复位 */
constexpr uint8_t FUNC_CFG_ACCESS_SW_POR = 0x1u << FUNC_CFG_ACCESS_SW_POR_POS;
constexpr uint8_t FUNC_CFG_ACCESS_SPI2_RESET_POS = 1;                    /* 位1 从主接口复位 SPI2 的控制寄存器 */
constexpr uint8_t FUNC_CFG_ACCESS_SPI2_RESET = 0x1u << FUNC_CFG_ACCESS_SPI2_RESET_POS;
constexpr uint8_t FUNC_CFG_ACCESS_OIS_CTRL_FROM_UI_POS = 0;              /* 位0 使能从主接口对 OIS 配置的完全控制 */
constexpr uint8_t FUNC_CFG_ACCESS_OIS_CTRL_FROM_UI = 0x1u << FUNC_CFG_ACCESS_OIS_CTRL_FROM_UI_POS;
constexpr uint8_t SPACE_MAIN = 0x00u;                                    /* 主页面（默认地址空间） */
constexpr uint8_t SPACE_EMBEDDED = FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS;  /* 嵌入功能寄存器空间 */
constexpr uint8_t SPACE_SENSOR_HUB = FUNC_CFG_ACCESS_SHUB_REG_ACCESS;    /* 传感器集线器空间 */

constexpr uint8_t PIN_CTRL = 0x02;                                       /* SDO/OCS_Aux/SDO_Aux 引脚上拉控制（R/W, 复位 0x23） */
constexpr uint8_t PIN_CTRL_OIS_PU_DIS_POS = 7;                           /* 位7 禁用 OCS_Aux 和 SDO_Aux 引脚上的上拉 */
constexpr uint8_t PIN_CTRL_OIS_PU_DIS = 0x1u << PIN_CTRL_OIS_PU_DIS_POS;
constexpr uint8_t PIN_CTRL_SDO_PU_EN_POS = 6;                            /* 位6 使能 SDO 引脚上的上拉 */
constexpr uint8_t PIN_CTRL_SDO_PU_EN = 0x1u << PIN_CTRL_SDO_PU_EN_POS;
constexpr uint8_t PIN_CTRL_IBHR_POR_EN_POS = 5;                          /* 位5 选择器件在 "reset whole chip" I3C 模式之后执行的操作 */
constexpr uint8_t PIN_CTRL_IBHR_POR_EN = 0x1u << PIN_CTRL_IBHR_POR_EN_POS;

constexpr uint8_t IF_CFG = 0x03;                                         /* 接口配置寄存器（R/W, 复位 0x00） */
constexpr uint8_t IF_CFG_SDA_PU_EN_POS = 7;                              /* 位7 使能 SDA 引脚上的上拉 */
constexpr uint8_t IF_CFG_SDA_PU_EN = 0x1u << IF_CFG_SDA_PU_EN_POS;
constexpr uint8_t IF_CFG_SHUB_PU_EN_POS = 6;                             /* 位6 使能主器件 I²C 上拉 */
constexpr uint8_t IF_CFG_SHUB_PU_EN = 0x1u << IF_CFG_SHUB_PU_EN_POS;
constexpr uint8_t IF_CFG_ASF_CTRL_POS = 5;                               /* 位5 使能抗尖峰滤波器 */
constexpr uint8_t IF_CFG_ASF_CTRL = 0x1u << IF_CFG_ASF_CTRL_POS;
constexpr uint8_t IF_CFG_H_LACTIVE_POS = 4;                              /* 位4 中断激活电平 */
constexpr uint8_t IF_CFG_H_LACTIVE = 0x1u << IF_CFG_H_LACTIVE_POS;
constexpr uint8_t IF_CFG_PP_OD_POS = 3;                                  /* 位3 INT1 和 INT2 引脚上的推挽/开漏选择 */
constexpr uint8_t IF_CFG_PP_OD = 0x1u << IF_CFG_PP_OD_POS;
constexpr uint8_t IF_CFG_SIM_POS = 2;                                    /* 位2 SPI 串行接口模式选择 */
constexpr uint8_t IF_CFG_SIM = 0x1u << IF_CFG_SIM_POS;
constexpr uint8_t IF_CFG_I2C_I3C_DISABLE_POS = 0;                        /* 位0 禁用 I²C 和 MIPI I3C® 接口 */
constexpr uint8_t IF_CFG_I2C_I3C_DISABLE = 0x1u << IF_CFG_I2C_I3C_DISABLE_POS;

constexpr uint8_t ODR_TRIG_CFG = 0x06;                                   /* ODR 触发模式配置寄存器（R/W, 复位 0x00） */
/* ODR_TRIG_NODR[7:0] 合法值为 0 及 4~255（1~3 非法） */
constexpr uint8_t ODR_TRIG_CFG_ODR_TRIG_NODR_POS = 0;                    /* 位[7:0] */
constexpr uint8_t ODR_TRIG_CFG_ODR_TRIG_NODR_MSK = 0xFF;                 /* 位[7:0] 掩码 */

constexpr uint8_t FIFO_CTRL1 = 0x07;                                     /* FIFO 控制寄存器 1（R/W, 复位 0x00） */
/* WTM 1 LSB = 1 个 TAG + 1 个传感器数据（6 字节）= 7 字节 */
constexpr uint8_t FIFO_CTRL1_WTM_POS = 0;                                /* 位[7:0] */
constexpr uint8_t FIFO_CTRL1_WTM_MSK = 0xFF;                             /* 位[7:0] 掩码 */

constexpr uint8_t FIFO_CTRL2 = 0x08;                                     /* FIFO 控制寄存器 2（R/W, 复位 0x00） */
constexpr uint8_t FIFO_CTRL2_STOP_ON_WTM_POS = 7;                        /* 位7 在阈值电平时停止传感链 FIFO 的值存储 */
constexpr uint8_t FIFO_CTRL2_STOP_ON_WTM = 0x1u << FIFO_CTRL2_STOP_ON_WTM_POS;
constexpr uint8_t FIFO_CTRL2_FIFO_COMPR_RT_EN_POS = 6;                   /* 位6 使能/禁用压缩算法运行 */
constexpr uint8_t FIFO_CTRL2_FIFO_COMPR_RT_EN = 0x1u << FIFO_CTRL2_FIFO_COMPR_RT_EN_POS;
constexpr uint8_t FIFO_CTRL2_ODR_CHG_EN_POS = 4;                         /* 位4 使能 ODR CHANGE 虚拟传感器在 FIFO 中批处理 */
constexpr uint8_t FIFO_CTRL2_ODR_CHG_EN = 0x1u << FIFO_CTRL2_ODR_CHG_EN_POS;
constexpr uint8_t FIFO_CTRL2_UNCOMPR_RATE_POS = 1;                       /* 位[2:1] */
constexpr uint8_t FIFO_CTRL2_UNCOMPR_RATE_MSK = 0x06;                    /* 位[2:1] 掩码 */
constexpr uint8_t FIFO_CTRL2_UNCOMPR_RATE_RATE_8 = 0x1u << FIFO_CTRL2_UNCOMPR_RATE_POS; /* 每 8 个批处理速率写 1 次未压缩数据 */
constexpr uint8_t FIFO_CTRL2_UNCOMPR_RATE_RATE_16 = 0x2u << FIFO_CTRL2_UNCOMPR_RATE_POS; /* 每 16 个批处理速率写 1 次 */
constexpr uint8_t FIFO_CTRL2_UNCOMPR_RATE_RATE_32 = 0x3u << FIFO_CTRL2_UNCOMPR_RATE_POS; /* 每 32 个批处理速率写 1 次 */
constexpr uint8_t FIFO_CTRL2_XL_DUALC_BATCH_FROM_FSM_POS = 0;            /* 位0 当使能双通道模式时，该位使能加速度计通道 2 在 FIFO 中的 FSM 触发批 */
constexpr uint8_t FIFO_CTRL2_XL_DUALC_BATCH_FROM_FSM = 0x1u << FIFO_CTRL2_XL_DUALC_BATCH_FROM_FSM_POS;

constexpr uint8_t FIFO_CTRL3 = 0x09;                                     /* FIFO 控制寄存器 3（R/W, 复位 0x00） */
constexpr uint8_t FIFO_CTRL3_BDR_GY_POS = 4;                             /* 位[7:4] */
constexpr uint8_t FIFO_CTRL3_BDR_GY_MSK = 0xF0;                          /* 位[7:4] 掩码 */
constexpr uint8_t FIFO_CTRL3_BDR_GY_NOT_BATCHED = 0x0u << FIFO_CTRL3_BDR_GY_POS; /* 不批处理（默认） */
constexpr uint8_t FIFO_CTRL3_BDR_GY_1Hz875 = 0x1u << FIFO_CTRL3_BDR_GY_POS; /* 1.875 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_7Hz5 = 0x2u << FIFO_CTRL3_BDR_GY_POS; /* 7.5 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_15Hz = 0x3u << FIFO_CTRL3_BDR_GY_POS; /* 15 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_30Hz = 0x4u << FIFO_CTRL3_BDR_GY_POS; /* 30 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_60Hz = 0x5u << FIFO_CTRL3_BDR_GY_POS; /* 60 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_120Hz = 0x6u << FIFO_CTRL3_BDR_GY_POS; /* 120 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_240Hz = 0x7u << FIFO_CTRL3_BDR_GY_POS; /* 240 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_480Hz = 0x8u << FIFO_CTRL3_BDR_GY_POS; /* 480 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_960Hz = 0x9u << FIFO_CTRL3_BDR_GY_POS; /* 960 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_1920Hz = 0xAu << FIFO_CTRL3_BDR_GY_POS; /* 1.92 kHz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_3840Hz = 0xBu << FIFO_CTRL3_BDR_GY_POS; /* 3.84 kHz */
constexpr uint8_t FIFO_CTRL3_BDR_GY_7680Hz = 0xCu << FIFO_CTRL3_BDR_GY_POS; /* 7.68 kHz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_POS = 0;                             /* 位[3:0] */
constexpr uint8_t FIFO_CTRL3_BDR_XL_MSK = 0x0F;                          /* 位[3:0] 掩码 */
constexpr uint8_t FIFO_CTRL3_BDR_XL_NOT_BATCHED = 0x0u << FIFO_CTRL3_BDR_XL_POS; /* 不批处理（默认） */
constexpr uint8_t FIFO_CTRL3_BDR_XL_1Hz875 = 0x1u << FIFO_CTRL3_BDR_XL_POS; /* 1.875 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_7Hz5 = 0x2u << FIFO_CTRL3_BDR_XL_POS; /* 7.5 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_15Hz = 0x3u << FIFO_CTRL3_BDR_XL_POS; /* 15 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_30Hz = 0x4u << FIFO_CTRL3_BDR_XL_POS; /* 30 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_60Hz = 0x5u << FIFO_CTRL3_BDR_XL_POS; /* 60 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_120Hz = 0x6u << FIFO_CTRL3_BDR_XL_POS; /* 120 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_240Hz = 0x7u << FIFO_CTRL3_BDR_XL_POS; /* 240 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_480Hz = 0x8u << FIFO_CTRL3_BDR_XL_POS; /* 480 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_960Hz = 0x9u << FIFO_CTRL3_BDR_XL_POS; /* 960 Hz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_1920Hz = 0xAu << FIFO_CTRL3_BDR_XL_POS; /* 1.92 kHz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_3840Hz = 0xBu << FIFO_CTRL3_BDR_XL_POS; /* 3.84 kHz */
constexpr uint8_t FIFO_CTRL3_BDR_XL_7680Hz = 0xCu << FIFO_CTRL3_BDR_XL_POS; /* 7.68 kHz */

constexpr uint8_t FIFO_CTRL4 = 0x0A;                                     /* FIFO 控制寄存器 4（R/W, 复位 0x00） */
constexpr uint8_t FIFO_CTRL4_DEC_TS_BATCH_POS = 6;                       /* 位[7:6] */
constexpr uint8_t FIFO_CTRL4_DEC_TS_BATCH_MSK = 0xC0;                    /* 位[7:6] 掩码 */
constexpr uint8_t FIFO_CTRL4_DEC_TS_BATCH_DEC_1 = 0x1u << FIFO_CTRL4_DEC_TS_BATCH_POS; /* 抽取 1（= max(BDR_XL, BDR_GY)） */
constexpr uint8_t FIFO_CTRL4_DEC_TS_BATCH_DEC_8 = 0x2u << FIFO_CTRL4_DEC_TS_BATCH_POS; /* 抽取 8 */
constexpr uint8_t FIFO_CTRL4_DEC_TS_BATCH_DEC_32 = 0x3u << FIFO_CTRL4_DEC_TS_BATCH_POS; /* 抽取 32 */
constexpr uint8_t FIFO_CTRL4_ODR_T_BATCH_POS = 4;                        /* 位[5:4] */
constexpr uint8_t FIFO_CTRL4_ODR_T_BATCH_MSK = 0x30;                     /* 位[5:4] 掩码 */
constexpr uint8_t FIFO_CTRL4_ODR_T_BATCH_1Hz875 = 0x1u << FIFO_CTRL4_ODR_T_BATCH_POS; /* 1.875 Hz */
constexpr uint8_t FIFO_CTRL4_ODR_T_BATCH_15Hz = 0x2u << FIFO_CTRL4_ODR_T_BATCH_POS; /* 15 Hz */
constexpr uint8_t FIFO_CTRL4_ODR_T_BATCH_60Hz = 0x3u << FIFO_CTRL4_ODR_T_BATCH_POS; /* 60 Hz */
constexpr uint8_t FIFO_CTRL4_G_EIS_FIFO_EN_POS = 3;                      /* 位3 使能增强型 EIS 陀螺仪输出值的 FIFO 批处理 */
constexpr uint8_t FIFO_CTRL4_G_EIS_FIFO_EN = 0x1u << FIFO_CTRL4_G_EIS_FIFO_EN_POS;
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_POS = 0;                          /* 位[2:0] */
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_MSK = 0x07;                       /* 位[2:0] 掩码 */
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_BYPASS = 0x0u << FIFO_CTRL4_FIFO_MODE_POS; /* Bypass：FIFO 关闭（默认） */
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_FIFO = 0x1u << FIFO_CTRL4_FIFO_MODE_POS; /* FIFO：满后停止采集 */
/* continuousWTM-to-full */
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_CONTINUOUS_WTM_TO_FULL = 0x2u << FIFO_CTRL4_FIFO_MODE_POS;
/* continuous-to-FIFO */
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_CONTINUOUS_TO_FIFO = 0x3u << FIFO_CTRL4_FIFO_MODE_POS;
/* Bypass-to-continuous */
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_BYPASS_TO_CONTINUOUS = 0x4u << FIFO_CTRL4_FIFO_MODE_POS;
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_CONTINUOUS = 0x6u << FIFO_CTRL4_FIFO_MODE_POS; /* continuous：满后覆盖最旧 */
/* Bypass-to-FIFO */
constexpr uint8_t FIFO_CTRL4_FIFO_MODE_BYPASS_TO_FIFO = 0x7u << FIFO_CTRL4_FIFO_MODE_POS;

constexpr uint8_t COUNTER_BDR_REG1 = 0x0B;                               /* 计数器批处理数据速率寄存器 1（R/W, 复位 0x00） */
/* CNT_BDR_TH[9:8] 是高 2 位，低 8 位在 COUNTER_BDR_REG2(0Ch) */
constexpr uint8_t COUNTER_BDR_REG1_TRIG_COUNTER_BDR_POS = 5;             /* 位[6:5] */
constexpr uint8_t COUNTER_BDR_REG1_TRIG_COUNTER_BDR_MSK = 0x60;          /* 位[6:5] 掩码 */
/* 计数器触发源 = 加速度计批处理事件 */
constexpr uint8_t COUNTER_BDR_REG1_TRIG_COUNTER_BDR_XL_EVENT = 0x0u << COUNTER_BDR_REG1_TRIG_COUNTER_BDR_POS;
/* 计数器触发源 = 陀螺仪批处理事件 */
constexpr uint8_t COUNTER_BDR_REG1_TRIG_COUNTER_BDR_GY_EVENT = 0x1u << COUNTER_BDR_REG1_TRIG_COUNTER_BDR_POS;
/* 计数器触发源 = EIS 陀螺批处理事件 */
constexpr uint8_t COUNTER_BDR_REG1_TRIG_COUNTER_BDR_GY_EIS_EVENT = 0x2u << COUNTER_BDR_REG1_TRIG_COUNTER_BDR_POS;
constexpr uint8_t COUNTER_BDR_REG1_CNT_BDR_TH_POS = 0;                   /* 位[1:0] */
constexpr uint8_t COUNTER_BDR_REG1_CNT_BDR_TH_MSK = 0x03;                /* 位[1:0] 掩码 */

constexpr uint8_t COUNTER_BDR_REG2 = 0x0C;                               /* 计数器批处理数据速率寄存器 2（R/W, 复位 0x00） */
constexpr uint8_t COUNTER_BDR_REG2_CNT_BDR_TH_POS = 0;                   /* 位[7:0] */
constexpr uint8_t COUNTER_BDR_REG2_CNT_BDR_TH_MSK = 0xFF;                /* 位[7:0] 掩码 */

constexpr uint8_t INT1_CTRL = 0x0D;                                      /* INT1 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t INT1_CTRL_INT1_CNT_BDR_POS = 6;                        /* 位6 使能 INT1 引脚上的 COUNTER_BDR_IA 中断 */
constexpr uint8_t INT1_CTRL_INT1_CNT_BDR = 0x1u << INT1_CTRL_INT1_CNT_BDR_POS;
constexpr uint8_t INT1_CTRL_INT1_FIFO_FULL_POS = 5;                      /* 位5 */
constexpr uint8_t INT1_CTRL_INT1_FIFO_FULL = 0x1u << INT1_CTRL_INT1_FIFO_FULL_POS;
constexpr uint8_t INT1_CTRL_INT1_FIFO_OVR_POS = 4;                       /* 位4 使能 INT1 引脚上的 FIFO 溢出中断 */
constexpr uint8_t INT1_CTRL_INT1_FIFO_OVR = 0x1u << INT1_CTRL_INT1_FIFO_OVR_POS;
constexpr uint8_t INT1_CTRL_INT1_FIFO_TH_POS = 3;                        /* 位3 使能 INT1 引脚上的 FIFO 阈值中断 */
constexpr uint8_t INT1_CTRL_INT1_FIFO_TH = 0x1u << INT1_CTRL_INT1_FIFO_TH_POS;
constexpr uint8_t INT1_CTRL_INT1_DRDY_G_POS = 1;                         /* 位1 使能 INT1 引脚上的陀螺仪数据准备就绪中断 */
constexpr uint8_t INT1_CTRL_INT1_DRDY_G = 0x1u << INT1_CTRL_INT1_DRDY_G_POS;
constexpr uint8_t INT1_CTRL_INT1_DRDY_XL_POS = 0;                        /* 位0 使能 INT1 引脚上的加速度计数据准备就绪中断 */
constexpr uint8_t INT1_CTRL_INT1_DRDY_XL = 0x1u << INT1_CTRL_INT1_DRDY_XL_POS;

constexpr uint8_t INT2_CTRL = 0x0E;                                      /* INT2 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t INT2_CTRL_INT2_EMB_FUNC_ENDOP_POS = 7;                 /* 位7 */
constexpr uint8_t INT2_CTRL_INT2_EMB_FUNC_ENDOP = 0x1u << INT2_CTRL_INT2_EMB_FUNC_ENDOP_POS;
constexpr uint8_t INT2_CTRL_INT2_CNT_BDR_POS = 6;                        /* 位6 使能 INT2 上的 COUNTER_BDR_IA 中断 */
constexpr uint8_t INT2_CTRL_INT2_CNT_BDR = 0x1u << INT2_CTRL_INT2_CNT_BDR_POS;
constexpr uint8_t INT2_CTRL_INT2_FIFO_FULL_POS = 5;                      /* 位5 使能 INT2 引脚上的 FIFO 满标志中断 */
constexpr uint8_t INT2_CTRL_INT2_FIFO_FULL = 0x1u << INT2_CTRL_INT2_FIFO_FULL_POS;
constexpr uint8_t INT2_CTRL_INT2_FIFO_OVR_POS = 4;                       /* 位4 使能 INT2 引脚上的 FIFO 溢出中断 */
constexpr uint8_t INT2_CTRL_INT2_FIFO_OVR = 0x1u << INT2_CTRL_INT2_FIFO_OVR_POS;
constexpr uint8_t INT2_CTRL_INT2_FIFO_TH_POS = 3;                        /* 位3 使能 INT2 引脚上的 FIFO 阈值中断 */
constexpr uint8_t INT2_CTRL_INT2_FIFO_TH = 0x1u << INT2_CTRL_INT2_FIFO_TH_POS;
constexpr uint8_t INT2_CTRL_INT2_DRDY_G_EIS_POS = 2;                     /* 位2 使能 INT2 引脚上的陀螺仪 EIS 数据准备就绪中断 */
constexpr uint8_t INT2_CTRL_INT2_DRDY_G_EIS = 0x1u << INT2_CTRL_INT2_DRDY_G_EIS_POS;
constexpr uint8_t INT2_CTRL_INT2_DRDY_G_POS = 1;                         /* 位1 INT2 引脚上的陀螺仪数据准备就绪中断 */
constexpr uint8_t INT2_CTRL_INT2_DRDY_G = 0x1u << INT2_CTRL_INT2_DRDY_G_POS;
constexpr uint8_t INT2_CTRL_INT2_DRDY_XL_POS = 0;                        /* 位0 INT2 引脚上的加速度计数据准备就绪中断 */
constexpr uint8_t INT2_CTRL_INT2_DRDY_XL = 0x1u << INT2_CTRL_INT2_DRDY_XL_POS;

constexpr uint8_t WHO_AM_I = 0x0F;                                       /* WHO_AM_I 只读器件 ID（R, 复位 0x70） */

/* ---------------- 1.2 传感器控制与自检（10h~19h） ---------------- */
constexpr uint8_t CTRL1 = 0x10;                                          /* 加速度计控制寄存器 1（R/W, 复位 0x00） */
constexpr uint8_t CTRL1_OP_MODE_XL_POS = 4;                              /* 位[6:4] */
constexpr uint8_t CTRL1_OP_MODE_XL_MSK = 0x70;                           /* 位[6:4] 掩码 */
constexpr uint8_t CTRL1_OP_MODE_XL_HIGH_PERFORMANCE = 0x0u << CTRL1_OP_MODE_XL_POS; /* 高性能模式（默认） */
constexpr uint8_t CTRL1_OP_MODE_XL_HIGH_ACCURACY_ODR = 0x1u << CTRL1_OP_MODE_XL_POS; /* 高精度 ODR 模式 */
constexpr uint8_t CTRL1_OP_MODE_XL_ODR_TRIGGERED = 0x3u << CTRL1_OP_MODE_XL_POS; /* ODR 触发模式 */
constexpr uint8_t CTRL1_OP_MODE_XL_LOW_POWER_2AVG = 0x4u << CTRL1_OP_MODE_XL_POS; /* 低功耗 1（2 次平均） */
constexpr uint8_t CTRL1_OP_MODE_XL_LOW_POWER_4AVG = 0x5u << CTRL1_OP_MODE_XL_POS; /* 低功耗 2（4 次平均） */
constexpr uint8_t CTRL1_OP_MODE_XL_LOW_POWER_8AVG = 0x6u << CTRL1_OP_MODE_XL_POS; /* 低功耗 3（8 次平均） */
constexpr uint8_t CTRL1_OP_MODE_XL_NORMAL = 0x7u << CTRL1_OP_MODE_XL_POS; /* 正常模式 */
constexpr uint8_t CTRL1_ODR_XL_POS = 0;                                  /* 位[3:0] */
constexpr uint8_t CTRL1_ODR_XL_MSK = 0x0F;                               /* 位[3:0] 掩码 */
constexpr uint8_t CTRL1_ODR_XL_POWER_DOWN = 0x0u << CTRL1_ODR_XL_POS;    /* 掉电（默认） */
constexpr uint8_t CTRL1_ODR_XL_1Hz875 = 0x1u << CTRL1_ODR_XL_POS;        /* 1.875 Hz（仅低功耗模式） */
constexpr uint8_t CTRL1_ODR_XL_7Hz5 = 0x2u << CTRL1_ODR_XL_POS;          /* 7.5 Hz */
constexpr uint8_t CTRL1_ODR_XL_15Hz = 0x3u << CTRL1_ODR_XL_POS;          /* 15 Hz */
constexpr uint8_t CTRL1_ODR_XL_30Hz = 0x4u << CTRL1_ODR_XL_POS;          /* 30 Hz */
constexpr uint8_t CTRL1_ODR_XL_60Hz = 0x5u << CTRL1_ODR_XL_POS;          /* 60 Hz */
constexpr uint8_t CTRL1_ODR_XL_120Hz = 0x6u << CTRL1_ODR_XL_POS;         /* 120 Hz */
constexpr uint8_t CTRL1_ODR_XL_240Hz = 0x7u << CTRL1_ODR_XL_POS;         /* 240 Hz */
constexpr uint8_t CTRL1_ODR_XL_480Hz = 0x8u << CTRL1_ODR_XL_POS;         /* 480 Hz */
constexpr uint8_t CTRL1_ODR_XL_960Hz = 0x9u << CTRL1_ODR_XL_POS;         /* 960 Hz */
constexpr uint8_t CTRL1_ODR_XL_1920Hz = 0xAu << CTRL1_ODR_XL_POS;        /* 1.92 kHz */
constexpr uint8_t CTRL1_ODR_XL_3840Hz = 0xBu << CTRL1_ODR_XL_POS;        /* 3.84 kHz */
constexpr uint8_t CTRL1_ODR_XL_7680Hz = 0xCu << CTRL1_ODR_XL_POS;        /* 7.68 kHz */

constexpr uint8_t CTRL2 = 0x11;                                          /* 陀螺仪控制寄存器 2（R/W, 复位 0x00） */
constexpr uint8_t CTRL2_OP_MODE_G_POS = 4;                               /* 位[6:4] */
constexpr uint8_t CTRL2_OP_MODE_G_MSK = 0x70;                            /* 位[6:4] 掩码 */
constexpr uint8_t CTRL2_OP_MODE_G_HIGH_PERFORMANCE = 0x0u << CTRL2_OP_MODE_G_POS; /* 高性能模式（默认） */
constexpr uint8_t CTRL2_OP_MODE_G_HIGH_ACCURACY_ODR = 0x1u << CTRL2_OP_MODE_G_POS; /* 高精度 ODR 模式 */
constexpr uint8_t CTRL2_OP_MODE_G_ODR_TRIGGERED = 0x3u << CTRL2_OP_MODE_G_POS; /* ODR 触发模式 */
constexpr uint8_t CTRL2_OP_MODE_G_SLEEP = 0x4u << CTRL2_OP_MODE_G_POS;   /* 睡眠模式 */
constexpr uint8_t CTRL2_OP_MODE_G_LOW_POWER = 0x5u << CTRL2_OP_MODE_G_POS; /* 低功耗模式 */
constexpr uint8_t CTRL2_ODR_G_POS = 0;                                   /* 位[3:0] */
constexpr uint8_t CTRL2_ODR_G_MSK = 0x0F;                                /* 位[3:0] 掩码 */
constexpr uint8_t CTRL2_ODR_G_POWER_DOWN = 0x0u << CTRL2_ODR_G_POS;      /* 掉电（默认） */
/* 注意：陀螺仪 ODR 表与加速度计表在 >=7.5 Hz 段完全一致，代码 0001 未使用（保留）。
 *       手册表 55：0010=7.5Hz, 0011=15Hz, 0100=30Hz, 0101=60Hz, 0110=120Hz,
 *                  0111=240Hz, 1000=480Hz, 1001=960Hz, 1010=1.92kHz,
 *                  1011=3.84kHz, 1100=7.68kHz
 *       已用 GDA 门控实测验证：0x8 -> 477Hz(480Hz), 0x7 -> 238Hz(240Hz), 0x6 -> 119Hz(120Hz) */
constexpr uint8_t CTRL2_ODR_G_7Hz5 = 0x2u << CTRL2_ODR_G_POS;            /* 7.5 Hz */
constexpr uint8_t CTRL2_ODR_G_15Hz = 0x3u << CTRL2_ODR_G_POS;            /* 15 Hz */
constexpr uint8_t CTRL2_ODR_G_30Hz = 0x4u << CTRL2_ODR_G_POS;            /* 30 Hz */
constexpr uint8_t CTRL2_ODR_G_60Hz = 0x5u << CTRL2_ODR_G_POS;            /* 60 Hz */
constexpr uint8_t CTRL2_ODR_G_120Hz = 0x6u << CTRL2_ODR_G_POS;           /* 120 Hz */
constexpr uint8_t CTRL2_ODR_G_240Hz = 0x7u << CTRL2_ODR_G_POS;           /* 240 Hz */
constexpr uint8_t CTRL2_ODR_G_480Hz = 0x8u << CTRL2_ODR_G_POS;           /* 480 Hz */
constexpr uint8_t CTRL2_ODR_G_960Hz = 0x9u << CTRL2_ODR_G_POS;           /* 960 Hz */
constexpr uint8_t CTRL2_ODR_G_1920Hz = 0xAu << CTRL2_ODR_G_POS;          /* 1.92 kHz */
constexpr uint8_t CTRL2_ODR_G_3840Hz = 0xBu << CTRL2_ODR_G_POS;          /* 3.84 kHz */
constexpr uint8_t CTRL2_ODR_G_7680Hz = 0xCu << CTRL2_ODR_G_POS;          /* 7.68 kHz */

constexpr uint8_t CTRL3 = 0x12;                                          /* 控制寄存器 3（R/W, 复位 0x44） */
constexpr uint8_t CTRL3_BOOT_POS = 7;                                    /* 位7 重启存储器内容 */
constexpr uint8_t CTRL3_BOOT = 0x1u << CTRL3_BOOT_POS;
constexpr uint8_t CTRL3_BDU_POS = 6;                                     /* 位6 块数据更新 */
constexpr uint8_t CTRL3_BDU = 0x1u << CTRL3_BDU_POS;
constexpr uint8_t CTRL3_IF_INC_POS = 2;                                  /* 位2 在使用串行接口（I²C、MIPI I3C 或 SPI）进行多字节访问期间 */
constexpr uint8_t CTRL3_IF_INC = 0x1u << CTRL3_IF_INC_POS;
constexpr uint8_t CTRL3_SW_RESET_POS = 0;                                /* 位0 软件复位，将所有控制寄存器复位为其 */
constexpr uint8_t CTRL3_SW_RESET = 0x1u << CTRL3_SW_RESET_POS;
constexpr uint8_t CTRL3_DEFAULT_BDU_IF_INC = CTRL3_BDU | CTRL3_IF_INC;   /* 复位值 = BDU(1) + IF_INC(1) = 0x44 */

constexpr uint8_t CTRL4 = 0x13;                                          /* 控制寄存器 4（R/W, 复位 0x00） */
constexpr uint8_t CTRL4_INT2_ON_INT1_POS = 4;                            /* 位4 使能将嵌入功能中断信号路由到 INT1 引脚 */
constexpr uint8_t CTRL4_INT2_ON_INT1 = 0x1u << CTRL4_INT2_ON_INT1_POS;
constexpr uint8_t CTRL4_DRDY_MASK_POS = 3;                               /* 位3 使能/屏蔽数据准备就绪信号 */
constexpr uint8_t CTRL4_DRDY_MASK = 0x1u << CTRL4_DRDY_MASK_POS;
constexpr uint8_t CTRL4_INT2_DRDY_TEMP_POS = 2;                          /* 位2 使能 INT2 引脚上的温度传感器数据准备就绪中断 */
constexpr uint8_t CTRL4_INT2_DRDY_TEMP = 0x1u << CTRL4_INT2_DRDY_TEMP_POS;
constexpr uint8_t CTRL4_DRDY_PULSED_POS = 1;                             /* 位1 使能脉冲式数据准备就绪模式 */
constexpr uint8_t CTRL4_DRDY_PULSED = 0x1u << CTRL4_DRDY_PULSED_POS;
constexpr uint8_t CTRL4_INT2_IN_LH_POS = 0;                              /* 位0 置为 1 可更改 DEN 或嵌入功能的 INT2 引脚输入触发的极性 */
constexpr uint8_t CTRL4_INT2_IN_LH = 0x1u << CTRL4_INT2_IN_LH_POS;

constexpr uint8_t CTRL5 = 0x14;                                          /* 控制寄存器 5（R/W, 复位 0x00） */
constexpr uint8_t CTRL5_BUS_ACT_SEL_POS = 1;                             /* 位[2:1] */
constexpr uint8_t CTRL5_BUS_ACT_SEL_MSK = 0x06;                          /* 位[2:1] 掩码 */
constexpr uint8_t CTRL5_BUS_ACT_SEL_2us = 0x0u << CTRL5_BUS_ACT_SEL_POS; /* IBI 总线可用时间 2 µs */
constexpr uint8_t CTRL5_BUS_ACT_SEL_50us = 0x1u << CTRL5_BUS_ACT_SEL_POS; /* 50 µs（默认） */
constexpr uint8_t CTRL5_BUS_ACT_SEL_1ms = 0x2u << CTRL5_BUS_ACT_SEL_POS; /* 1 ms */
constexpr uint8_t CTRL5_BUS_ACT_SEL_25ms = 0x3u << CTRL5_BUS_ACT_SEL_POS; /* 25 ms */
constexpr uint8_t CTRL5_INT_EN_I3C_POS = 0;                              /* 位0 当使能 I3C 时使能 INT 引脚 */
constexpr uint8_t CTRL5_INT_EN_I3C = 0x1u << CTRL5_INT_EN_I3C_POS;

constexpr uint8_t CTRL6 = 0x15;                                          /* 控制寄存器 6（R/W, 复位 0x00） */
constexpr uint8_t CTRL6_LPF1_G_BW_POS = 4;                               /* 位[6:4] */
constexpr uint8_t CTRL6_LPF1_G_BW_MSK = 0x70;                            /* 位[6:4] 掩码 */
constexpr uint8_t CTRL6_FS_G_POS = 0;                                    /* 位[3:0] */
constexpr uint8_t CTRL6_FS_G_MSK = 0x0F;                                 /* 位[3:0] 掩码 */
constexpr uint8_t CTRL6_FS_G_125dps = 0x0u << CTRL6_FS_G_POS;            /* ±125 dps */
constexpr uint8_t CTRL6_FS_G_250dps = 0x1u << CTRL6_FS_G_POS;            /* ±250 dps */
constexpr uint8_t CTRL6_FS_G_500dps = 0x2u << CTRL6_FS_G_POS;            /* ±500 dps */
constexpr uint8_t CTRL6_FS_G_1000dps = 0x3u << CTRL6_FS_G_POS;           /* ±1000 dps */
constexpr uint8_t CTRL6_FS_G_2000dps = 0x4u << CTRL6_FS_G_POS;           /* ±2000 dps */
constexpr uint8_t CTRL6_FS_G_4000dps = 0xCu << CTRL6_FS_G_POS;           /* ±4000 dps */

constexpr uint8_t CTRL7 = 0x16;                                          /* 控制寄存器 7（R/W, 复位 0x00） */
constexpr uint8_t CTRL7_AH_QVAR_EN_POS = 7;                              /* 位7 使能模拟集线器和 Qvar 链 */
constexpr uint8_t CTRL7_AH_QVAR_EN = 0x1u << CTRL7_AH_QVAR_EN_POS;
constexpr uint8_t CTRL7_INT2_DRDY_AH_QVAR_POS = 6;                       /* 位6 INT2 引脚上的模拟集线器和 Qvar 数据准备就绪中断 */
constexpr uint8_t CTRL7_INT2_DRDY_AH_QVAR = 0x1u << CTRL7_INT2_DRDY_AH_QVAR_POS;
constexpr uint8_t CTRL7_AH_QVAR_C_ZIN_POS = 4;                           /* 位[5:4] */
constexpr uint8_t CTRL7_AH_QVAR_C_ZIN_MSK = 0x30;                        /* 位[5:4] 掩码 */
constexpr uint8_t CTRL7_AH_QVAR_C_ZIN_2G4 = 0x0u << CTRL7_AH_QVAR_C_ZIN_POS; /* 等效输入阻抗 2.4 GΩ（默认） */
constexpr uint8_t CTRL7_AH_QVAR_C_ZIN_730M = 0x1u << CTRL7_AH_QVAR_C_ZIN_POS; /* 730 MΩ */
constexpr uint8_t CTRL7_AH_QVAR_C_ZIN_300M = 0x2u << CTRL7_AH_QVAR_C_ZIN_POS; /* 300 MΩ */
constexpr uint8_t CTRL7_AH_QVAR_C_ZIN_235M = 0x3u << CTRL7_AH_QVAR_C_ZIN_POS; /* 235 MΩ */
constexpr uint8_t CTRL7_LPF1_G_EN_POS = 0;                               /* 位0 使能陀螺仪数字 LPF1 滤波器 */
constexpr uint8_t CTRL7_LPF1_G_EN = 0x1u << CTRL7_LPF1_G_EN_POS;

constexpr uint8_t CTRL8 = 0x17;                                          /* 控制寄存器 8（R/W, 复位 0x00） */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_POS = 5;                           /* 位[7:5] */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_MSK = 0xE0;                        /* 位[7:5] 掩码 */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_4 = 0x0u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/4（高通路径为 SLOPE (ODR/4)） */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_10 = 0x1u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/10 */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_20 = 0x2u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/20 */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_45 = 0x3u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/45 */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_100 = 0x4u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/100 */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_200 = 0x5u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/200 */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_400 = 0x6u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/400 */
constexpr uint8_t CTRL8_HP_LPF2_XL_BW_ODR_800 = 0x7u << CTRL8_HP_LPF2_XL_BW_POS; /* ODR/800 */
constexpr uint8_t CTRL8_XL_DUALC_EN_POS = 3;                             /* 位3 使能双通道模式 */
constexpr uint8_t CTRL8_XL_DUALC_EN = 0x1u << CTRL8_XL_DUALC_EN_POS;
constexpr uint8_t CTRL8_FS_XL_POS = 0;                                   /* 位[1:0] */
constexpr uint8_t CTRL8_FS_XL_MSK = 0x03;                                /* 位[1:0] 掩码 */
constexpr uint8_t CTRL8_FS_XL_2g = 0x0u << CTRL8_FS_XL_POS;              /* ±2 g */
constexpr uint8_t CTRL8_FS_XL_4g = 0x1u << CTRL8_FS_XL_POS;              /* ±4 g */
constexpr uint8_t CTRL8_FS_XL_8g = 0x2u << CTRL8_FS_XL_POS;              /* ±8 g */
constexpr uint8_t CTRL8_FS_XL_16g = 0x3u << CTRL8_FS_XL_POS;             /* ±16 g */

constexpr uint8_t CTRL9 = 0x18;                                          /* 控制寄存器 9（R/W, 复位 0x00） */
constexpr uint8_t CTRL9_HP_REF_MODE_XL_POS = 6;                          /* 位6 使能加速度计高通滤波器参考模式 */
constexpr uint8_t CTRL9_HP_REF_MODE_XL = 0x1u << CTRL9_HP_REF_MODE_XL_POS;
constexpr uint8_t CTRL9_XL_FASTSETTL_MODE_POS = 5;                       /* 位5 使能加速度计 LPF2 与 HPF 快速稳定模式 */
constexpr uint8_t CTRL9_XL_FASTSETTL_MODE = 0x1u << CTRL9_XL_FASTSETTL_MODE_POS;
constexpr uint8_t CTRL9_HP_SLOPE_XL_EN_POS = 4;                          /* 位4 加速度计斜率滤波器/高通滤波器选择 */
constexpr uint8_t CTRL9_HP_SLOPE_XL_EN = 0x1u << CTRL9_HP_SLOPE_XL_EN_POS;
constexpr uint8_t CTRL9_LPF2_XL_EN_POS = 3;                              /* 位3 加速度计高分辨率选择 */
constexpr uint8_t CTRL9_LPF2_XL_EN = 0x1u << CTRL9_LPF2_XL_EN_POS;
constexpr uint8_t CTRL9_USR_OFF_W_POS = 1;                               /* 位1 寄存器 X_OFS_USR (73h)、Y_OFS_USR (74h) */
constexpr uint8_t CTRL9_USR_OFF_W = 0x1u << CTRL9_USR_OFF_W_POS;
constexpr uint8_t CTRL9_USR_OFF_ON_OUT_POS = 0;                          /* 位0 使能加速度计用户偏移校正模块；该模块对低通路径有效 */
constexpr uint8_t CTRL9_USR_OFF_ON_OUT = 0x1u << CTRL9_USR_OFF_ON_OUT_POS;

constexpr uint8_t CTRL10 = 0x19;                                         /* 控制寄存器 10（R/W, 复位 0x00） */
constexpr uint8_t CTRL10_EMB_FUNC_DEBUG_POS = 6;                         /* 位6 使能嵌入功能的调试模式 */
constexpr uint8_t CTRL10_EMB_FUNC_DEBUG = 0x1u << CTRL10_EMB_FUNC_DEBUG_POS;
constexpr uint8_t CTRL10_ST_G_POS = 2;                                   /* 位[3:2] */
constexpr uint8_t CTRL10_ST_G_MSK = 0x0C;                                /* 位[3:2] 掩码 */
constexpr uint8_t CTRL10_ST_G_NORMAL = 0x0u << CTRL10_ST_G_POS;          /* 正常模式（默认） */
constexpr uint8_t CTRL10_ST_G_POSITIVE = 0x1u << CTRL10_ST_G_POS;        /* 正号自检 */
constexpr uint8_t CTRL10_ST_G_NEGATIVE = 0x2u << CTRL10_ST_G_POS;        /* 负号自检 */
constexpr uint8_t CTRL10_ST_XL_POS = 0;                                  /* 位[1:0] */
constexpr uint8_t CTRL10_ST_XL_MSK = 0x03;                               /* 位[1:0] 掩码 */
constexpr uint8_t CTRL10_ST_XL_NORMAL = 0x0u << CTRL10_ST_XL_POS;        /* 正常模式（默认） */
constexpr uint8_t CTRL10_ST_XL_POSITIVE = 0x1u << CTRL10_ST_XL_POS;      /* 正号自检 */
constexpr uint8_t CTRL10_ST_XL_NEGATIVE = 0x2u << CTRL10_ST_XL_POS;      /* 负号自检 */

/* ---------------- 1.3 状态与中断源（1Ah~1Fh） ---------------- */
constexpr uint8_t CTRL_STATUS = 0x1A;                                    /* 控制状态寄存器（R, 输出） */
constexpr uint8_t CTRL_STATUS_FSM_WR_CTRL_STATUS_POS = 2;                /* 位2 该标志指示器件配置寄存器的当前控制者 */
constexpr uint8_t CTRL_STATUS_FSM_WR_CTRL_STATUS = 0x1u << CTRL_STATUS_FSM_WR_CTRL_STATUS_POS;

constexpr uint8_t FIFO_STATUS1 = 0x1B;                                   /* FIFO 状态寄存器 1（R, 输出） */
constexpr uint8_t FIFO_STATUS1_DIFF_FIFO_POS = 0;                        /* 位[7:0] */
constexpr uint8_t FIFO_STATUS1_DIFF_FIFO_MSK = 0xFF;                     /* 位[7:0] 掩码 */

constexpr uint8_t FIFO_STATUS2 = 0x1C;                                   /* FIFO 状态寄存器 2（R, 输出） */
constexpr uint8_t FIFO_STATUS2_FIFO_WTM_IA_POS = 7;                      /* 位7 FIFO 水印状态 */
constexpr uint8_t FIFO_STATUS2_FIFO_WTM_IA = 0x1u << FIFO_STATUS2_FIFO_WTM_IA_POS;
constexpr uint8_t FIFO_STATUS2_FIFO_OVR_IA_POS = 6;                      /* 位6 FIFO 溢出状态 */
constexpr uint8_t FIFO_STATUS2_FIFO_OVR_IA = 0x1u << FIFO_STATUS2_FIFO_OVR_IA_POS;
constexpr uint8_t FIFO_STATUS2_FIFO_FULL_IA_POS = 5;                     /* 位5 智能 FIFO 满状态 */
constexpr uint8_t FIFO_STATUS2_FIFO_FULL_IA = 0x1u << FIFO_STATUS2_FIFO_FULL_IA_POS;
constexpr uint8_t FIFO_STATUS2_COUNTER_BDR_IA_POS = 4;                   /* 位4 计数器 BDR 达到 COUNTER_BDR_REG1 (0Bh) 和 COUN */
constexpr uint8_t FIFO_STATUS2_COUNTER_BDR_IA = 0x1u << FIFO_STATUS2_COUNTER_BDR_IA_POS;
constexpr uint8_t FIFO_STATUS2_FIFO_OVR_LATCHED_POS = 3;                 /* 位3 锁存的 FIFO 溢出状态 */
constexpr uint8_t FIFO_STATUS2_FIFO_OVR_LATCHED = 0x1u << FIFO_STATUS2_FIFO_OVR_LATCHED_POS;
constexpr uint8_t FIFO_STATUS2_DIFF_FIFO_8_POS = 0;                      /* 位0 FIFO 中存储的未读传感器数据（TAG + 6 字节）数量 */
constexpr uint8_t FIFO_STATUS2_DIFF_FIFO_8 = 0x1u << FIFO_STATUS2_DIFF_FIFO_8_POS; /* DIFF_FIFO[8:0] 的第 9 位（低 8 位在 FIFO_STATUS1）*/

constexpr uint8_t ALL_INT_SRC = 0x1D;                                    /* 所有中断的源寄存器（R, 输出） */
constexpr uint8_t ALL_INT_SRC_EMB_FUNC_IA_POS = 7;                       /* 位7 嵌入功能中断状态 */
constexpr uint8_t ALL_INT_SRC_EMB_FUNC_IA = 0x1u << ALL_INT_SRC_EMB_FUNC_IA_POS;
constexpr uint8_t ALL_INT_SRC_SHUB_IA_POS = 6;                           /* 位6 传感器集线器（I²C 主器件）中断状态 */
constexpr uint8_t ALL_INT_SRC_SHUB_IA = 0x1u << ALL_INT_SRC_SHUB_IA_POS;
constexpr uint8_t ALL_INT_SRC_SLEEP_CHANGE_IA_POS = 5;                   /* 位5 检测活动/静止状态中的变化事件 */
constexpr uint8_t ALL_INT_SRC_SLEEP_CHANGE_IA = 0x1u << ALL_INT_SRC_SLEEP_CHANGE_IA_POS;
constexpr uint8_t ALL_INT_SRC_D6D_IA_POS = 4;                            /* 位4 当竖向、横向、面朝上、面朝下位置发生改变时中断有效 */
constexpr uint8_t ALL_INT_SRC_D6D_IA = 0x1u << ALL_INT_SRC_D6D_IA_POS;
constexpr uint8_t ALL_INT_SRC_TAP_IA_POS = 2;                            /* 位2 单击或双击事件检测状态，取决于 SINGLE_DOUBLE_TAP_bit 的值 */
constexpr uint8_t ALL_INT_SRC_TAP_IA = 0x1u << ALL_INT_SRC_TAP_IA_POS;
constexpr uint8_t ALL_INT_SRC_WU_IA_POS = 1;                             /* 位1 唤醒事件状态 */
constexpr uint8_t ALL_INT_SRC_WU_IA = 0x1u << ALL_INT_SRC_WU_IA_POS;
constexpr uint8_t ALL_INT_SRC_FF_IA_POS = 0;                             /* 位0 自由落体事件状态 */
constexpr uint8_t ALL_INT_SRC_FF_IA = 0x1u << ALL_INT_SRC_FF_IA_POS;

constexpr uint8_t STATUS_REG = 0x1E;                                     /* STATUS_REG 寄存器由主接口 SPI/I²C 和（R, 输出） */
/* 由主接口与 SPI2/OIS 共享，读操作会复位相应位 */
constexpr uint8_t STATUS_REG_TIMESTAMP_ENDCOUNT_POS = 7;                 /* 位7 在 5.6 ms 内提示时间戳溢出 */
constexpr uint8_t STATUS_REG_TIMESTAMP_ENDCOUNT = 0x1u << STATUS_REG_TIMESTAMP_ENDCOUNT_POS;
constexpr uint8_t STATUS_REG_OIS_DRDY_POS = 5;                           /* 位5 加速度计 OIS 或陀螺仪 OIS 有新的输出数据可用 */
constexpr uint8_t STATUS_REG_OIS_DRDY = 0x1u << STATUS_REG_OIS_DRDY_POS;
constexpr uint8_t STATUS_REG_GDA_EIS_POS = 4;                            /* 位4 增强型 EIS 陀螺仪有新数据可用 */
constexpr uint8_t STATUS_REG_GDA_EIS = 0x1u << STATUS_REG_GDA_EIS_POS;
constexpr uint8_t STATUS_REG_AH_QVARDA_POS = 3;                          /* 位3 模拟集线器或 Qvar 有新数据可用 */
constexpr uint8_t STATUS_REG_AH_QVARDA = 0x1u << STATUS_REG_AH_QVARDA_POS;
constexpr uint8_t STATUS_REG_TDA_POS = 2;                                /* 位2 温度有新数据可用 */
constexpr uint8_t STATUS_REG_TDA = 0x1u << STATUS_REG_TDA_POS;
constexpr uint8_t STATUS_REG_GDA_POS = 1;                                /* 位1 陀螺仪有新数据可用 */
constexpr uint8_t STATUS_REG_GDA = 0x1u << STATUS_REG_GDA_POS;
constexpr uint8_t STATUS_REG_XLDA_POS = 0;                               /* 位0 加速度计有新数据可用 */
constexpr uint8_t STATUS_REG_XLDA = 0x1u << STATUS_REG_XLDA_POS;

/* ---------------- 1.4 输出数据区（20h~3Bh，20h 起连续无空洞） ---------------- */
constexpr uint8_t OUT_TEMP_L = 0x20;                                     /* 温度数据输出寄存器（R, 输出） */
/* 温度 = raw / 256 + 25（°C）；见 TEMP_LSB_PER_C / TEMP_ZERO_C */

constexpr uint8_t OUT_TEMP_H = 0x21;                                     /* 温度输出高字节（R, 输出） */

constexpr uint8_t OUTX_L_G = 0x22;                                       /* 陀螺仪 X 轴角速率输出低字节（R, 输出） */

constexpr uint8_t OUTX_H_G = 0x23;                                       /* 陀螺仪 X 轴角速率输出高字节（R, 输出） */

constexpr uint8_t OUTY_L_G = 0x24;                                       /* 陀螺仪 Y 轴角速率输出低字节（R, 输出） */

constexpr uint8_t OUTY_H_G = 0x25;                                       /* 陀螺仪 Y 轴角速率输出高字节（R, 输出） */

constexpr uint8_t OUTZ_L_G = 0x26;                                       /* 陀螺仪 Z 轴角速率输出低字节（R, 输出） */

constexpr uint8_t OUTZ_H_G = 0x27;                                       /* 陀螺仪 Z 轴角速率输出高字节（R, 输出） */

constexpr uint8_t OUTX_L_A = 0x28;                                       /* 加速度计 X 轴输出低字节（R, 输出） */

constexpr uint8_t OUTX_H_A = 0x29;                                       /* 加速度计 X 轴输出高字节（R, 输出） */

constexpr uint8_t OUTY_L_A = 0x2A;                                       /* 加速度计 Y 轴输出低字节（R, 输出） */

constexpr uint8_t OUTY_H_A = 0x2B;                                       /* 加速度计 Y 轴输出高字节（R, 输出） */

constexpr uint8_t OUTZ_L_A = 0x2C;                                       /* 加速度计 Z 轴输出低字节（R, 输出） */

constexpr uint8_t OUTZ_H_A = 0x2D;                                       /* 加速度计 Z 轴输出高字节（R, 输出） */

constexpr uint8_t UI_OUTX_L_G_OIS_EIS = 0x2E;                            /* 陀螺仪 EIS/OIS X 轴输出低字节（R, 输出） */

constexpr uint8_t UI_OUTX_H_G_OIS_EIS = 0x2F;                            /* 陀螺仪 EIS/OIS X 轴输出高字节（R, 输出） */

constexpr uint8_t UI_OUTY_L_G_OIS_EIS = 0x30;                            /* 陀螺仪 EIS/OIS Y 轴输出低字节（R, 输出） */

constexpr uint8_t UI_OUTY_H_G_OIS_EIS = 0x31;                            /* 陀螺仪 EIS/OIS Y 轴输出高字节（R, 输出） */

constexpr uint8_t UI_OUTZ_L_G_OIS_EIS = 0x32;                            /* 陀螺仪 EIS/OIS Z 轴输出低字节（R, 输出） */

constexpr uint8_t UI_OUTZ_H_G_OIS_EIS = 0x33;                            /* 陀螺仪 EIS/OIS Z 轴输出高字节（R, 输出） */

constexpr uint8_t UI_OUTX_L_A_OIS_DUALC = 0x34;                          /* 加速度计双通道 X 轴输出低字节（R, 输出） */

constexpr uint8_t UI_OUTX_H_A_OIS_DUALC = 0x35;                          /* 加速度计双通道 X 轴输出高字节（R, 输出） */

constexpr uint8_t UI_OUTY_L_A_OIS_DUALC = 0x36;                          /* 加速度计双通道 Y 轴输出低字节（R, 输出） */

constexpr uint8_t UI_OUTY_H_A_OIS_DUALC = 0x37;                          /* 加速度计双通道 Y 轴输出高字节（R, 输出） */

constexpr uint8_t UI_OUTZ_L_A_OIS_DUALC = 0x38;                          /* 加速度计双通道 Z 轴输出低字节（R, 输出） */

constexpr uint8_t UI_OUTZ_H_A_OIS_DUALC = 0x39;                          /* 加速度计双通道 Z 轴输出高字节（R, 输出） */

constexpr uint8_t AH_QVAR_OUT_L = 0x3A;                                  /* 模拟集线器/Qvar 数据低字节（R, 输出） */

constexpr uint8_t AH_QVAR_OUT_H = 0x3B;                                  /* 模拟集线器/Qvar 数据高字节（R, 输出） */

/* ---------------- 1.5 时间戳与事件源（40h~4Bh） ---------------- */
constexpr uint8_t TIMESTAMP0 = 0x40;                                     /* 时间戳输出寄存器 0（R, 输出） */
/* 32 位无符号：TIMESTAMP0 最低字节 … TIMESTAMP3 最高字节 */

constexpr uint8_t TIMESTAMP1 = 0x41;                                     /* 时间戳输出寄存器 1（R, 输出） */

constexpr uint8_t TIMESTAMP2 = 0x42;                                     /* 时间戳输出寄存器 2（R, 输出） */

constexpr uint8_t TIMESTAMP3 = 0x43;                                     /* 时间戳输出寄存器 3（R, 输出） */

constexpr uint8_t UI_STATUS_REG_OIS = 0x44;                              /* OIS 状态寄存器（R, 输出） */
constexpr uint8_t UI_STATUS_REG_OIS_GYRO_SETTLING_POS = 2;               /* 位2 当陀螺仪输出处于建立阶段时为高电平 */
constexpr uint8_t UI_STATUS_REG_OIS_GYRO_SETTLING = 0x1u << UI_STATUS_REG_OIS_GYRO_SETTLING_POS;
constexpr uint8_t UI_STATUS_REG_OIS_GDA_OIS_POS = 1;                     /* 位1 陀螺仪 OIS 数据可用（读取输出数据的高位部分之一时复位） */
constexpr uint8_t UI_STATUS_REG_OIS_GDA_OIS = 0x1u << UI_STATUS_REG_OIS_GDA_OIS_POS;
constexpr uint8_t UI_STATUS_REG_OIS_XLDA_OIS_POS = 0;                    /* 位0 加速度计 OIS 数据可用（读取输出数据的高位部分之一时复位） */
constexpr uint8_t UI_STATUS_REG_OIS_XLDA_OIS = 0x1u << UI_STATUS_REG_OIS_XLDA_OIS_POS;

constexpr uint8_t WAKE_UP_SRC = 0x45;                                    /* 唤醒中断源寄存器（R, 输出） */
constexpr uint8_t WAKE_UP_SRC_SLEEP_CHANGE_IA_POS = 6;                   /* 位6 检测活动/静止状态的变化事件 */
constexpr uint8_t WAKE_UP_SRC_SLEEP_CHANGE_IA = 0x1u << WAKE_UP_SRC_SLEEP_CHANGE_IA_POS;
constexpr uint8_t WAKE_UP_SRC_FF_IA_POS = 5;                             /* 位5 自由落体事件检测状态 */
constexpr uint8_t WAKE_UP_SRC_FF_IA = 0x1u << WAKE_UP_SRC_FF_IA_POS;
constexpr uint8_t WAKE_UP_SRC_SLEEP_STATE_POS = 4;                       /* 位4 睡眠状态位 */
constexpr uint8_t WAKE_UP_SRC_SLEEP_STATE = 0x1u << WAKE_UP_SRC_SLEEP_STATE_POS;
constexpr uint8_t WAKE_UP_SRC_WU_IA_POS = 3;                             /* 位3 唤醒事件检测状态 */
constexpr uint8_t WAKE_UP_SRC_WU_IA = 0x1u << WAKE_UP_SRC_WU_IA_POS;
constexpr uint8_t WAKE_UP_SRC_X_WU_POS = 2;                              /* 位2 X 轴上的唤醒事件检测状态 */
constexpr uint8_t WAKE_UP_SRC_X_WU = 0x1u << WAKE_UP_SRC_X_WU_POS;
constexpr uint8_t WAKE_UP_SRC_Y_WU_POS = 1;                              /* 位1 Y 轴上的唤醒事件检测状态 */
constexpr uint8_t WAKE_UP_SRC_Y_WU = 0x1u << WAKE_UP_SRC_Y_WU_POS;
constexpr uint8_t WAKE_UP_SRC_Z_WU_POS = 0;                              /* 位0 Z 轴上的唤醒事件检测状态 */
constexpr uint8_t WAKE_UP_SRC_Z_WU = 0x1u << WAKE_UP_SRC_Z_WU_POS;

constexpr uint8_t TAP_SRC = 0x46;                                        /* 敲击源寄存器（R, 输出） */
constexpr uint8_t TAP_SRC_TAP_IA_POS = 6;                                /* 位6 敲击事件检测状态 */
constexpr uint8_t TAP_SRC_TAP_IA = 0x1u << TAP_SRC_TAP_IA_POS;
constexpr uint8_t TAP_SRC_SINGLE_TAP_POS = 5;                            /* 位5 单击事件状态 */
constexpr uint8_t TAP_SRC_SINGLE_TAP = 0x1u << TAP_SRC_SINGLE_TAP_POS;
constexpr uint8_t TAP_SRC_DOUBLE_TAP_POS = 4;                            /* 位4 双击事件检测状态 */
constexpr uint8_t TAP_SRC_DOUBLE_TAP = 0x1u << TAP_SRC_DOUBLE_TAP_POS;
constexpr uint8_t TAP_SRC_TAP_SIGN_POS = 3;                              /* 位3 敲击事件检测到的加速度符号 */
constexpr uint8_t TAP_SRC_TAP_SIGN = 0x1u << TAP_SRC_TAP_SIGN_POS;
constexpr uint8_t TAP_SRC_X_TAP_POS = 2;                                 /* 位2 X 轴上的敲击事件检测状态 */
constexpr uint8_t TAP_SRC_X_TAP = 0x1u << TAP_SRC_X_TAP_POS;
constexpr uint8_t TAP_SRC_Y_TAP_POS = 1;                                 /* 位1 Y 轴上的敲击事件检测状态 */
constexpr uint8_t TAP_SRC_Y_TAP = 0x1u << TAP_SRC_Y_TAP_POS;
constexpr uint8_t TAP_SRC_Z_TAP_POS = 0;                                 /* 位0 Z 轴上的敲击事件检测状态 */
constexpr uint8_t TAP_SRC_Z_TAP = 0x1u << TAP_SRC_Z_TAP_POS;

constexpr uint8_t D6D_SRC = 0x47;                                        /* 纵向、横向、面朝上和面朝下源寄存器（R, 输出） */
constexpr uint8_t D6D_SRC_D6D_IA_POS = 6;                                /* 位6 纵向、横向、面朝上、面朝下位置变化的中断有效 */
constexpr uint8_t D6D_SRC_D6D_IA = 0x1u << D6D_SRC_D6D_IA_POS;
constexpr uint8_t D6D_SRC_ZH_POS = 5;                                    /* 位5 Z 轴高事件（高于阈值） */
constexpr uint8_t D6D_SRC_ZH = 0x1u << D6D_SRC_ZH_POS;
constexpr uint8_t D6D_SRC_ZL_POS = 4;                                    /* 位4 Z 轴低事件（低于阈值） */
constexpr uint8_t D6D_SRC_ZL = 0x1u << D6D_SRC_ZL_POS;
constexpr uint8_t D6D_SRC_YH_POS = 3;                                    /* 位3 Y 轴高事件（高于阈值） */
constexpr uint8_t D6D_SRC_YH = 0x1u << D6D_SRC_YH_POS;
constexpr uint8_t D6D_SRC_YL_POS = 2;                                    /* 位2 Y 轴低事件（低于阈值） */
constexpr uint8_t D6D_SRC_YL = 0x1u << D6D_SRC_YL_POS;
constexpr uint8_t D6D_SRC_XH_POS = 1;                                    /* 位1 X 轴高事件（超过阈值） */
constexpr uint8_t D6D_SRC_XH = 0x1u << D6D_SRC_XH_POS;
constexpr uint8_t D6D_SRC_XL_POS = 0;                                    /* 位0 X 轴低事件（低于阈值） */
constexpr uint8_t D6D_SRC_XL = 0x1u << D6D_SRC_XL_POS;

constexpr uint8_t STATUS_MASTER_MAINPAGE = 0x48;                         /* 传感器集线器源寄存器（R, 输出） */
constexpr uint8_t STATUS_MASTER_MAINPAGE_WR_ONCE_DONE_POS = 7;           /* 位7 当 MASTER_CONFIG (14h) 中的 WRITE_ONCE 位配置为 */
constexpr uint8_t STATUS_MASTER_MAINPAGE_WR_ONCE_DONE = 0x1u << STATUS_MASTER_MAINPAGE_WR_ONCE_DONE_POS;
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE3_NACK_POS = 6;            /* 位6 若从器件 3 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE3_NACK = 0x1u << STATUS_MASTER_MAINPAGE_SLAVE3_NACK_POS;
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE2_NACK_POS = 5;            /* 位5 若从器件 2 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE2_NACK = 0x1u << STATUS_MASTER_MAINPAGE_SLAVE2_NACK_POS;
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE1_NACK_POS = 4;            /* 位4 若从器件 1 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE1_NACK = 0x1u << STATUS_MASTER_MAINPAGE_SLAVE1_NACK_POS;
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE0_NACK_POS = 3;            /* 位3 若从器件 0 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_MAINPAGE_SLAVE0_NACK = 0x1u << STATUS_MASTER_MAINPAGE_SLAVE0_NACK_POS;
constexpr uint8_t STATUS_MASTER_MAINPAGE_SENS_HUB_ENDOP_POS = 0;         /* 位0 传感器集线器通信状态 */
constexpr uint8_t STATUS_MASTER_MAINPAGE_SENS_HUB_ENDOP = 0x1u << STATUS_MASTER_MAINPAGE_SENS_HUB_ENDOP_POS;

constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE = 0x49;                       /* 嵌入功能状态寄存器（R, 输出） */
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_FSM_LC_POS = 7;            /* 位7 FSM 长计数器超时中断事件的中断状态位 */
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_FSM_LC = 0x1u << EMB_FUNC_STATUS_MAINPAGE_IS_FSM_LC_POS;
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_SIGMOT_POS = 5;            /* 位5 大幅运动检测的中断状态位（1：检测到中断；0：无中断） */
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_SIGMOT = 0x1u << EMB_FUNC_STATUS_MAINPAGE_IS_SIGMOT_POS;
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_TILT_POS = 4;              /* 位4 倾斜检测的中断状态位（1：检测到中断；0：无中断） */
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_TILT = 0x1u << EMB_FUNC_STATUS_MAINPAGE_IS_TILT_POS;
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_STEP_DET_POS = 3;          /* 位3 步伐检测的中断状态位（1：检测到中断；0：无中断） */
constexpr uint8_t EMB_FUNC_STATUS_MAINPAGE_IS_STEP_DET = 0x1u << EMB_FUNC_STATUS_MAINPAGE_IS_STEP_DET_POS;

constexpr uint8_t FSM_STATUS_MAINPAGE = 0x4A;                            /* 有限状态机状态寄存器（R, 输出） */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM8_POS = 7;                   /* 位7 FSM8 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM8 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM8_POS;
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM7_POS = 6;                   /* 位6 FSM7 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM7 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM7_POS;
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM6_POS = 5;                   /* 位5 FSM6 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM6 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM6_POS;
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM5_POS = 4;                   /* 位4 FSM5 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM5 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM5_POS;
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM4_POS = 3;                   /* 位3 FSM4 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM4 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM4_POS;
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM3_POS = 2;                   /* 位2 FSM3 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM3 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM3_POS;
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM2_POS = 1;                   /* 位1 FSM2 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM2 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM2_POS;
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM1_POS = 0;                   /* 位0 FSM1 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_MAINPAGE_IS_FSM1 = 0x1u << FSM_STATUS_MAINPAGE_IS_FSM1_POS;

constexpr uint8_t MLC_STATUS_MAINPAGE = 0x4B;                            /* 机器学习内核状态寄存器（R, 输出） */
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC4_POS = 3;                   /* 位3 MLC4 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC4 = 0x1u << MLC_STATUS_MAINPAGE_IS_MLC4_POS;
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC3_POS = 2;                   /* 位2 MLC3 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC3 = 0x1u << MLC_STATUS_MAINPAGE_IS_MLC3_POS;
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC2_POS = 1;                   /* 位1 MLC2 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC2 = 0x1u << MLC_STATUS_MAINPAGE_IS_MLC2_POS;
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC1_POS = 0;                   /* 位0 MLC1 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_MAINPAGE_IS_MLC1 = 0x1u << MLC_STATUS_MAINPAGE_IS_MLC1_POS;

constexpr uint8_t INTERNAL_FREQ_FINE = 0x4F;                             /* 内部频率寄存器（R, 输出） */

/* ---------------- 1.6 中断与功能配置（50h~5Fh） ---------------- */
constexpr uint8_t FUNCTIONS_ENABLE = 0x50;                               /* 使能中断功能寄存器（RW, 复位 0x00） */
constexpr uint8_t FUNCTIONS_ENABLE_INTERRUPTS_ENABLE_POS = 7;            /* 位7 使能基本中断（6D/4D、自由落体、唤醒、单击、活动/静止） */
constexpr uint8_t FUNCTIONS_ENABLE_INTERRUPTS_ENABLE = 0x1u << FUNCTIONS_ENABLE_INTERRUPTS_ENABLE_POS;
constexpr uint8_t FUNCTIONS_ENABLE_TIMESTAMP_EN_POS = 6;                 /* 位6 使能时间戳计数器 */
constexpr uint8_t FUNCTIONS_ENABLE_TIMESTAMP_EN = 0x1u << FUNCTIONS_ENABLE_TIMESTAMP_EN_POS;
constexpr uint8_t FUNCTIONS_ENABLE_DIS_RST_LIR_ALL_INT_POS = 3;          /* 位3 当该位被置为 1 时，读取 ALL_INT_SRC (1Dh) 寄存器不会复位锁 */
constexpr uint8_t FUNCTIONS_ENABLE_DIS_RST_LIR_ALL_INT = 0x1u << FUNCTIONS_ENABLE_DIS_RST_LIR_ALL_INT_POS;
constexpr uint8_t FUNCTIONS_ENABLE_INACT_EN_POS = 0;                     /* 位[1:0] */
constexpr uint8_t FUNCTIONS_ENABLE_INACT_EN_MSK = 0x03;                  /* 位[1:0] 掩码 */
/* 仅产生静止/运动中断（默认） */
constexpr uint8_t FUNCTIONS_ENABLE_INACT_EN_EVENT_ONLY = 0x0u << FUNCTIONS_ENABLE_INACT_EN_POS;
/* 静止后加速度计转低功耗 1 */
constexpr uint8_t FUNCTIONS_ENABLE_INACT_EN_XL_LP1 = 0x1u << FUNCTIONS_ENABLE_INACT_EN_POS;
/* 加速度计低功耗 1 + 陀螺睡眠 */
constexpr uint8_t FUNCTIONS_ENABLE_INACT_EN_XL_LP1_G_SLEEP = 0x2u << FUNCTIONS_ENABLE_INACT_EN_POS;
/* 加速度计低功耗 1 + 陀螺掉电 */
constexpr uint8_t FUNCTIONS_ENABLE_INACT_EN_XL_LP1_G_POWER_DOWN = 0x3u << FUNCTIONS_ENABLE_INACT_EN_POS;

constexpr uint8_t DEN = 0x51;                                            /* DEN 配置寄存器（R/W, 复位 0x0E） */
constexpr uint8_t DEN_LVL1_EN_POS = 6;                                   /* 位6 使能 DEN 数据电平感应触发 */
constexpr uint8_t DEN_LVL1_EN = 0x1u << DEN_LVL1_EN_POS;
constexpr uint8_t DEN_LVL2_EN_POS = 5;                                   /* 位5 使能 DEN 电平感应锁存 */
constexpr uint8_t DEN_LVL2_EN = 0x1u << DEN_LVL2_EN_POS;
constexpr uint8_t DEN_DEN_XL_EN_POS = 4;                                 /* 位4 将 DEN 功能扩展到加速度计传感器 */
constexpr uint8_t DEN_DEN_XL_EN = 0x1u << DEN_DEN_XL_EN_POS;
constexpr uint8_t DEN_DEN_X_POS = 3;                                     /* 位3 DEN 值存储在 X 轴 LSB 中 */
constexpr uint8_t DEN_DEN_X = 0x1u << DEN_DEN_X_POS;
constexpr uint8_t DEN_DEN_Y_POS = 2;                                     /* 位2 DEN 值存储在 Y 轴 LSB 中 */
constexpr uint8_t DEN_DEN_Y = 0x1u << DEN_DEN_Y_POS;
constexpr uint8_t DEN_DEN_Z_POS = 1;                                     /* 位1 DEN 值存储在 Z 轴 LSB 中 */
constexpr uint8_t DEN_DEN_Z = 0x1u << DEN_DEN_Z_POS;
constexpr uint8_t DEN_DEN_XL_G_POS = 0;                                  /* 位0 DEN 标记传感器选择 */
constexpr uint8_t DEN_DEN_XL_G = 0x1u << DEN_DEN_XL_G_POS;

constexpr uint8_t INACTIVITY_DUR = 0x54;                                 /* 活动/静止配置寄存器（R/W, 复位 0x04） */
constexpr uint8_t INACTIVITY_DUR_SLEEP_STATUS_ON_INT_POS = 7;            /* 位7 活动/静止中断模式配置 */
constexpr uint8_t INACTIVITY_DUR_SLEEP_STATUS_ON_INT = 0x1u << INACTIVITY_DUR_SLEEP_STATUS_ON_INT_POS;
constexpr uint8_t INACTIVITY_DUR_WU_INACT_THS_W_POS = 4;                 /* 位[6:4] */
constexpr uint8_t INACTIVITY_DUR_WU_INACT_THS_W_MSK = 0x70;              /* 位[6:4] 掩码 */
constexpr uint8_t INACTIVITY_DUR_XL_INACT_ODR_POS = 2;                   /* 位[3:2] */
constexpr uint8_t INACTIVITY_DUR_XL_INACT_ODR_MSK = 0x0C;                /* 位[3:2] 掩码 */
/* 1.875 Hz */
constexpr uint8_t INACTIVITY_DUR_XL_INACT_ODR_1Hz875 = 0x0u << INACTIVITY_DUR_XL_INACT_ODR_POS;
/* 15 Hz（默认） */
constexpr uint8_t INACTIVITY_DUR_XL_INACT_ODR_15Hz = 0x1u << INACTIVITY_DUR_XL_INACT_ODR_POS;
/* 30 Hz */
constexpr uint8_t INACTIVITY_DUR_XL_INACT_ODR_30Hz = 0x2u << INACTIVITY_DUR_XL_INACT_ODR_POS;
/* 60 Hz */
constexpr uint8_t INACTIVITY_DUR_XL_INACT_ODR_60Hz = 0x3u << INACTIVITY_DUR_XL_INACT_ODR_POS;
constexpr uint8_t INACTIVITY_DUR_INACT_DUR_POS = 0;                      /* 位[1:0] */
constexpr uint8_t INACTIVITY_DUR_INACT_DUR_MSK = 0x03;                   /* 位[1:0] 掩码 */
constexpr uint8_t INACTIVITY_DUR_INACT_DUR_X1 = 0x0u << INACTIVITY_DUR_INACT_DUR_POS; /* 1 次超阈值即转运动（默认） */
constexpr uint8_t INACTIVITY_DUR_INACT_DUR_X2 = 0x1u << INACTIVITY_DUR_INACT_DUR_POS; /* 连续 2 次转运动 */
constexpr uint8_t INACTIVITY_DUR_INACT_DUR_X3 = 0x2u << INACTIVITY_DUR_INACT_DUR_POS; /* 连续 3 次转运动 */
constexpr uint8_t INACTIVITY_DUR_INACT_DUR_X4 = 0x3u << INACTIVITY_DUR_INACT_DUR_POS; /* 连续 4 次转运动 */

constexpr uint8_t INACTIVITY_THS = 0x55;                                 /* 活动/静止阈值设置寄存器（R/W, 复位 0x00） */
constexpr uint8_t INACTIVITY_THS_INACT_THS_POS = 0;                      /* 位[5:0] */
constexpr uint8_t INACTIVITY_THS_INACT_THS_MSK = 0x3F;                   /* 位[5:0] 掩码 */

constexpr uint8_t TAP_CFG0 = 0x56;                                       /* 单击配置寄存器 0（R/W, 复位 0x00） */
constexpr uint8_t TAP_CFG0_LOW_PASS_ON_6D_POS = 6;                       /* 位6 6D 功能上的 LPF2 滤波器选择 */
constexpr uint8_t TAP_CFG0_LOW_PASS_ON_6D = 0x1u << TAP_CFG0_LOW_PASS_ON_6D_POS;
constexpr uint8_t TAP_CFG0_HW_FUNC_MASK_XL_SETTL_POS = 5;                /* 位5 使能当加速度计数据处于建立过程时屏蔽基本中断功能 */
constexpr uint8_t TAP_CFG0_HW_FUNC_MASK_XL_SETTL = 0x1u << TAP_CFG0_HW_FUNC_MASK_XL_SETTL_POS;
constexpr uint8_t TAP_CFG0_SLOPE_FDS_POS = 4;                            /* 位4 唤醒和活动/静止功能上的 HPF 或斜率滤波器选择 */
constexpr uint8_t TAP_CFG0_SLOPE_FDS = 0x1u << TAP_CFG0_SLOPE_FDS_POS;
constexpr uint8_t TAP_CFG0_TAP_X_EN_POS = 3;                             /* 位3 使能单击识别中的 X 方向 */
constexpr uint8_t TAP_CFG0_TAP_X_EN = 0x1u << TAP_CFG0_TAP_X_EN_POS;
constexpr uint8_t TAP_CFG0_TAP_Y_EN_POS = 2;                             /* 位2 使能单击识别中的 Y 方向 */
constexpr uint8_t TAP_CFG0_TAP_Y_EN = 0x1u << TAP_CFG0_TAP_Y_EN_POS;
constexpr uint8_t TAP_CFG0_TAP_Z_EN_POS = 1;                             /* 位1 使能单击识别中的 Z 方向 */
constexpr uint8_t TAP_CFG0_TAP_Z_EN = 0x1u << TAP_CFG0_TAP_Z_EN_POS;
constexpr uint8_t TAP_CFG0_LIR_POS = 0;                                  /* 位0 锁存中断 */
constexpr uint8_t TAP_CFG0_LIR = 0x1u << TAP_CFG0_LIR_POS;

constexpr uint8_t TAP_CFG1 = 0x57;                                       /* 单击配置寄存器 1（R/W, 复位 0x00） */
constexpr uint8_t TAP_CFG1_TAP_PRIORITY_POS = 5;                         /* 位[7:5] */
constexpr uint8_t TAP_CFG1_TAP_PRIORITY_MSK = 0xE0;                      /* 位[7:5] 掩码 */
constexpr uint8_t TAP_CFG1_TAP_THS_X_POS = 0;                            /* 位[4:0] */
constexpr uint8_t TAP_CFG1_TAP_THS_X_MSK = 0x1F;                         /* 位[4:0] 掩码 */

constexpr uint8_t TAP_CFG2 = 0x58;                                       /* 单击配置寄存器 2（R/W, 复位 0x00） */
constexpr uint8_t TAP_CFG2_TAP_THS_Y_POS = 0;                            /* 位[4:0] */
constexpr uint8_t TAP_CFG2_TAP_THS_Y_MSK = 0x1F;                         /* 位[4:0] 掩码 */

constexpr uint8_t TAP_THS_6D = 0x59;                                     /* 纵向/横向位置和单击功能阈值寄存器（R/W, 复位 0x00） */
constexpr uint8_t TAP_THS_6D_D4D_EN_POS = 7;                             /* 位7 使能 4D 方向检测 */
constexpr uint8_t TAP_THS_6D_D4D_EN = 0x1u << TAP_THS_6D_D4D_EN_POS;
constexpr uint8_t TAP_THS_6D_SIXD_THS_POS = 5;                           /* 位[6:5] */
constexpr uint8_t TAP_THS_6D_SIXD_THS_MSK = 0x60;                        /* 位[6:5] 掩码 */
constexpr uint8_t TAP_THS_6D_SIXD_THS_80deg = 0x0u << TAP_THS_6D_SIXD_THS_POS; /* 阈值 80° */
constexpr uint8_t TAP_THS_6D_SIXD_THS_70deg = 0x1u << TAP_THS_6D_SIXD_THS_POS; /* 阈值 70° */
constexpr uint8_t TAP_THS_6D_SIXD_THS_60deg = 0x2u << TAP_THS_6D_SIXD_THS_POS; /* 阈值 60° */
constexpr uint8_t TAP_THS_6D_SIXD_THS_50deg = 0x3u << TAP_THS_6D_SIXD_THS_POS; /* 阈值 50° */
constexpr uint8_t TAP_THS_6D_TAP_THS_Z_POS = 0;                          /* 位[4:0] */
constexpr uint8_t TAP_THS_6D_TAP_THS_Z_MSK = 0x1F;                       /* 位[4:0] 掩码 */

constexpr uint8_t TAP_DUR = 0x5A;                                        /* 单击识别功能设置寄存器（R/W, 复位 0x00） */
constexpr uint8_t TAP_DUR_DUR_POS = 4;                                   /* 位[7:4] */
constexpr uint8_t TAP_DUR_DUR_MSK = 0xF0;                                /* 位[7:4] 掩码 */
constexpr uint8_t TAP_DUR_QUIET_POS = 2;                                 /* 位[3:2] */
constexpr uint8_t TAP_DUR_QUIET_MSK = 0x0C;                              /* 位[3:2] 掩码 */
constexpr uint8_t TAP_DUR_SHOCK_POS = 0;                                 /* 位[1:0] */
constexpr uint8_t TAP_DUR_SHOCK_MSK = 0x03;                              /* 位[1:0] 掩码 */

constexpr uint8_t WAKE_UP_THS = 0x5B;                                    /* 唤醒阈值寄存器（R/W, 复位 0x00） */
constexpr uint8_t WAKE_UP_THS_SINGLE_DOUBLE_TAP_POS = 7;                 /* 位7 使能单击/双击事件 */
constexpr uint8_t WAKE_UP_THS_SINGLE_DOUBLE_TAP = 0x1u << WAKE_UP_THS_SINGLE_DOUBLE_TAP_POS;
constexpr uint8_t WAKE_UP_THS_USR_OFF_ON_WU_POS = 6;                     /* 位6 将经过用户偏移校正的低通滤波数据（而非高通滤波数据）送至唤醒和活动/静止功能 */
constexpr uint8_t WAKE_UP_THS_USR_OFF_ON_WU = 0x1u << WAKE_UP_THS_USR_OFF_ON_WU_POS;
constexpr uint8_t WAKE_UP_THS_WK_THS_POS = 0;                            /* 位[5:0] */
constexpr uint8_t WAKE_UP_THS_WK_THS_MSK = 0x3F;                         /* 位[5:0] 掩码 */

constexpr uint8_t WAKE_UP_DUR = 0x5C;                                    /* 自由落体、唤醒和睡眠模式功能持续时间设置寄存器（R/W, 复位 0x00） */
constexpr uint8_t WAKE_UP_DUR_FF_DUR_POS = 7;                            /* 位7 */
constexpr uint8_t WAKE_UP_DUR_FF_DUR = 0x1u << WAKE_UP_DUR_FF_DUR_POS;
constexpr uint8_t WAKE_UP_DUR_WAKE_DUR_POS = 5;                          /* 位[6:5] */
constexpr uint8_t WAKE_UP_DUR_WAKE_DUR_MSK = 0x60;                       /* 位[6:5] 掩码 */
constexpr uint8_t WAKE_UP_DUR_SLEEP_DUR_POS = 0;                         /* 位[3:0] */
constexpr uint8_t WAKE_UP_DUR_SLEEP_DUR_MSK = 0x0F;                      /* 位[3:0] 掩码 */

constexpr uint8_t FREE_FALL = 0x5D;                                      /* 自由落体功能持续时间设置寄存器（R/W, 复位 0x00） */
constexpr uint8_t FREE_FALL_FF_DUR_POS = 3;                              /* 位[7:3] */
constexpr uint8_t FREE_FALL_FF_DUR_MSK = 0xF8;                           /* 位[7:3] 掩码 */
constexpr uint8_t FREE_FALL_FF_THS_POS = 0;                              /* 位[2:0] */
constexpr uint8_t FREE_FALL_FF_THS_MSK = 0x07;                           /* 位[2:0] 掩码 */
constexpr uint8_t FREE_FALL_FF_THS_156mg = 0x0u << FREE_FALL_FF_THS_POS; /* 156 mg */
constexpr uint8_t FREE_FALL_FF_THS_219mg = 0x1u << FREE_FALL_FF_THS_POS; /* 219 mg */
constexpr uint8_t FREE_FALL_FF_THS_250mg = 0x2u << FREE_FALL_FF_THS_POS; /* 250 mg */
constexpr uint8_t FREE_FALL_FF_THS_312mg = 0x3u << FREE_FALL_FF_THS_POS; /* 312 mg */
constexpr uint8_t FREE_FALL_FF_THS_344mg = 0x4u << FREE_FALL_FF_THS_POS; /* 344 mg */
constexpr uint8_t FREE_FALL_FF_THS_406mg = 0x5u << FREE_FALL_FF_THS_POS; /* 406 mg */
constexpr uint8_t FREE_FALL_FF_THS_469mg = 0x6u << FREE_FALL_FF_THS_POS; /* 469 mg */
constexpr uint8_t FREE_FALL_FF_THS_500mg = 0x7u << FREE_FALL_FF_THS_POS; /* 500 mg */

constexpr uint8_t MD1_CFG = 0x5E;                                        /* 事件路由到 INT1 寄存器（R/W, 复位 0x00） */
constexpr uint8_t MD1_CFG_INT1_SLEEP_CHANGE_POS = 7;                     /* 位7 将活动/静止识别事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_SLEEP_CHANGE = 0x1u << MD1_CFG_INT1_SLEEP_CHANGE_POS;
constexpr uint8_t MD1_CFG_INT1_SINGLE_TAP_POS = 6;                       /* 位6 将单击识别事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_SINGLE_TAP = 0x1u << MD1_CFG_INT1_SINGLE_TAP_POS;
constexpr uint8_t MD1_CFG_INT1_WU_POS = 5;                               /* 位5 将唤醒事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_WU = 0x1u << MD1_CFG_INT1_WU_POS;
constexpr uint8_t MD1_CFG_INT1_FF_POS = 4;                               /* 位4 将自由落体事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_FF = 0x1u << MD1_CFG_INT1_FF_POS;
constexpr uint8_t MD1_CFG_INT1_DOUBLE_TAP_POS = 3;                       /* 位3 将单击事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_DOUBLE_TAP = 0x1u << MD1_CFG_INT1_DOUBLE_TAP_POS;
constexpr uint8_t MD1_CFG_INT1_6D_POS = 2;                               /* 位2 将 6D 事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_6D = 0x1u << MD1_CFG_INT1_6D_POS;
constexpr uint8_t MD1_CFG_INT1_EMB_FUNC_POS = 1;                         /* 位1 将嵌入功能事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_EMB_FUNC = 0x1u << MD1_CFG_INT1_EMB_FUNC_POS;
constexpr uint8_t MD1_CFG_INT1_SHUB_POS = 0;                             /* 位0 将传感器集线器通信结束事件路由至 INT1 */
constexpr uint8_t MD1_CFG_INT1_SHUB = 0x1u << MD1_CFG_INT1_SHUB_POS;

constexpr uint8_t MD2_CFG = 0x5F;                                        /* 事件路由到 INT2 寄存器（R/W, 复位 0x00） */
constexpr uint8_t MD2_CFG_INT2_SLEEP_CHANGE_POS = 7;                     /* 位7 将活动/静止识别事件路由至 INT2 */
constexpr uint8_t MD2_CFG_INT2_SLEEP_CHANGE = 0x1u << MD2_CFG_INT2_SLEEP_CHANGE_POS;
constexpr uint8_t MD2_CFG_INT2_SINGLE_TAP_POS = 6;                       /* 位6 将单击识别路由至 INT2 */
constexpr uint8_t MD2_CFG_INT2_SINGLE_TAP = 0x1u << MD2_CFG_INT2_SINGLE_TAP_POS;
constexpr uint8_t MD2_CFG_INT2_WU_POS = 5;                               /* 位5 将唤醒事件路由至 INT2 */
constexpr uint8_t MD2_CFG_INT2_WU = 0x1u << MD2_CFG_INT2_WU_POS;
constexpr uint8_t MD2_CFG_INT2_FF_POS = 4;                               /* 位4 将自由落体事件路由至 INT2 */
constexpr uint8_t MD2_CFG_INT2_FF = 0x1u << MD2_CFG_INT2_FF_POS;
constexpr uint8_t MD2_CFG_INT2_DOUBLE_TAP_POS = 3;                       /* 位3 将单击事件路由至 INT2 */
constexpr uint8_t MD2_CFG_INT2_DOUBLE_TAP = 0x1u << MD2_CFG_INT2_DOUBLE_TAP_POS;
constexpr uint8_t MD2_CFG_INT2_6D_POS = 2;                               /* 位2 将 6D 事件路由至 INT2 */
constexpr uint8_t MD2_CFG_INT2_6D = 0x1u << MD2_CFG_INT2_6D_POS;
constexpr uint8_t MD2_CFG_INT2_EMB_FUNC_POS = 1;                         /* 位1 将嵌入功能事件路由至 INT2 */
constexpr uint8_t MD2_CFG_INT2_EMB_FUNC = 0x1u << MD2_CFG_INT2_EMB_FUNC_POS;
constexpr uint8_t MD2_CFG_INT2_TIMESTAMP_POS = 0;                        /* 位0 使能将 5.6 ms 内时间戳溢出的告警路由至 INT2 引脚 */
constexpr uint8_t MD2_CFG_INT2_TIMESTAMP = 0x1u << MD2_CFG_INT2_TIMESTAMP_POS;

/* ---------------- 1.7 功能与 OIS 配置（62h~75h） ---------------- */
constexpr uint8_t HAODR_CFG = 0x62;                                      /* HAODR 数据速率配置寄存器（R/W, 复位 0x00） */
constexpr uint8_t HAODR_CFG_HAODR_SEL_POS = 0;                           /* 位[1:0] */
constexpr uint8_t HAODR_CFG_HAODR_SEL_MSK = 0x03;                        /* 位[1:0] 掩码 */
constexpr uint8_t HAODR_CFG_HAODR_SEL_ODR_SET_0 = 0x0u << HAODR_CFG_HAODR_SEL_POS; /* ODR 组 1：15/30/60/120/240/480/960/1920/3840/7680 Hz（见表 20） */
constexpr uint8_t HAODR_CFG_HAODR_SEL_ODR_SET_1 = 0x1u << HAODR_CFG_HAODR_SEL_POS; /* ODR 组 2：15.625~8000 Hz */
constexpr uint8_t HAODR_CFG_HAODR_SEL_ODR_SET_2 = 0x2u << HAODR_CFG_HAODR_SEL_POS; /* ODR 组 3：12.5~6400 Hz */

constexpr uint8_t EMB_FUNC_CFG = 0x63;                                   /* 嵌入功能配置寄存器（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_CFG_XL_DUALC_BATCH_FROM_IF_POS = 7;           /* 位7 使能双通道模式时，该位使能对加速度计通道 2 进行 FIFO 批处理 */
constexpr uint8_t EMB_FUNC_CFG_XL_DUALC_BATCH_FROM_IF = 0x1u << EMB_FUNC_CFG_XL_DUALC_BATCH_FROM_IF_POS;
constexpr uint8_t EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_G_SETTL_POS = 5;        /* 位5 当陀螺仪数据稳定时，使能/屏蔽嵌入功能的执行触发 */
constexpr uint8_t EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_G_SETTL = 0x1u << EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_G_SETTL_POS;
constexpr uint8_t EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_XL_SETTL_POS = 4;       /* 位4 当加速度计数据稳定时，使能/屏蔽嵌入功能的执行触发 */
constexpr uint8_t EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_XL_SETTL = 0x1u << EMB_FUNC_CFG_EMB_FUNC_IRQ_MASK_XL_SETTL_POS;
constexpr uint8_t EMB_FUNC_CFG_EMB_FUNC_DISABLE_POS = 3;                 /* 位3 禁止嵌入功能的执行 */
constexpr uint8_t EMB_FUNC_CFG_EMB_FUNC_DISABLE = 0x1u << EMB_FUNC_CFG_EMB_FUNC_DISABLE_POS;

constexpr uint8_t UI_HANDSHAKE_CTRL = 0x64;                              /* UI / SPI2 共享寄存器的控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t UI_HANDSHAKE_CTRL_UI_SHARED_ACK_POS = 1;               /* 位1 主接口侧 */
constexpr uint8_t UI_HANDSHAKE_CTRL_UI_SHARED_ACK = 0x1u << UI_HANDSHAKE_CTRL_UI_SHARED_ACK_POS;
constexpr uint8_t UI_HANDSHAKE_CTRL_UI_SHARED_REQ_POS = 0;               /* 位0 该位由主接口主器件用于请求访问 UI_SPI2_SHARED_0 (65h) 至 */
constexpr uint8_t UI_HANDSHAKE_CTRL_UI_SHARED_REQ = 0x1u << UI_HANDSHAKE_CTRL_UI_SHARED_REQ_POS;

constexpr uint8_t UI_SPI2_SHARED_0 = 0x65;                               /* UI/SPI2 共享寄存器 0（R/W, 复位 0x00） */

constexpr uint8_t UI_SPI2_SHARED_1 = 0x66;                               /* UI/SPI2 共享寄存器 1（R/W, 复位 0x00） */

constexpr uint8_t UI_SPI2_SHARED_2 = 0x67;                               /* UI/SPI2 共享寄存器 2（R/W, 复位 0x00） */

constexpr uint8_t UI_SPI2_SHARED_3 = 0x68;                               /* UI/SPI2 共享寄存器 3（R/W, 复位 0x00） */

constexpr uint8_t UI_SPI2_SHARED_4 = 0x69;                               /* UI/SPI2 共享寄存器 4（R/W, 复位 0x00） */

constexpr uint8_t UI_SPI2_SHARED_5 = 0x6A;                               /* UI/SPI2 共享寄存器 5（R/W, 复位 0x00） */

constexpr uint8_t CTRL_EIS = 0x6B;                                       /* 陀螺仪 EIS 通道控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t CTRL_EIS_ODR_G_EIS_POS = 6;                            /* 位[7:6] */
constexpr uint8_t CTRL_EIS_ODR_G_EIS_MSK = 0xC0;                         /* 位[7:6] 掩码 */
constexpr uint8_t CTRL_EIS_ODR_G_EIS_OFF = 0x0u << CTRL_EIS_ODR_G_EIS_POS; /* EIS 通道关闭（默认） */
constexpr uint8_t CTRL_EIS_ODR_G_EIS_1920Hz = 0x1u << CTRL_EIS_ODR_G_EIS_POS; /* 1.92 kHz */
constexpr uint8_t CTRL_EIS_ODR_G_EIS_960Hz = 0x2u << CTRL_EIS_ODR_G_EIS_POS; /* 960 Hz */
constexpr uint8_t CTRL_EIS_LPF_G_EIS_BW_POS = 4;                         /* 位4 陀螺仪数字 LPF_EIS 滤波器带宽选择 */
constexpr uint8_t CTRL_EIS_LPF_G_EIS_BW = 0x1u << CTRL_EIS_LPF_G_EIS_BW_POS;
constexpr uint8_t CTRL_EIS_G_EIS_ON_G_OIS_OUT_REG_POS = 3;               /* 位3 使能将陀螺仪 EIS 输出从 UI 输出地址 (2Eh – 33h) 路由至 O */
constexpr uint8_t CTRL_EIS_G_EIS_ON_G_OIS_OUT_REG = 0x1u << CTRL_EIS_G_EIS_ON_G_OIS_OUT_REG_POS;
constexpr uint8_t CTRL_EIS_FS_G_EIS_POS = 0;                             /* 位[2:0] */
constexpr uint8_t CTRL_EIS_FS_G_EIS_MSK = 0x07;                          /* 位[2:0] 掩码 */
constexpr uint8_t CTRL_EIS_FS_G_EIS_125dps = 0x0u << CTRL_EIS_FS_G_EIS_POS; /* ±125 dps（默认） */
constexpr uint8_t CTRL_EIS_FS_G_EIS_250dps = 0x1u << CTRL_EIS_FS_G_EIS_POS; /* ±250 dps */
constexpr uint8_t CTRL_EIS_FS_G_EIS_500dps = 0x2u << CTRL_EIS_FS_G_EIS_POS; /* ±500 dps */
constexpr uint8_t CTRL_EIS_FS_G_EIS_1000dps = 0x3u << CTRL_EIS_FS_G_EIS_POS; /* ±1000 dps */
constexpr uint8_t CTRL_EIS_FS_G_EIS_2000dps = 0x4u << CTRL_EIS_FS_G_EIS_POS; /* ±2000 dps */

constexpr uint8_t UI_INT_OIS = 0x6F;                                     /* OIS 中断配置寄存器（R/W 主控 / R SPI2, 复位 0x00） */
/* bit[3:0] 保留；ST_XL_OIS/ST_G_OIS 只在 SPI2_INT_OIS 侧视图可见 */
constexpr uint8_t UI_INT_OIS_INT2_DRDY_OIS_POS = 7;                      /* 位7 使能从 UI 接口在 INT2 引脚上输出 OIS 链 DRDY */
constexpr uint8_t UI_INT_OIS_INT2_DRDY_OIS = 0x1u << UI_INT_OIS_INT2_DRDY_OIS_POS;
constexpr uint8_t UI_INT_OIS_DRDY_MASK_OIS_POS = 6;                      /* 位6 使能 / 屏蔽 OIS 数据可用 */
constexpr uint8_t UI_INT_OIS_DRDY_MASK_OIS = 0x1u << UI_INT_OIS_DRDY_MASK_OIS_POS;
constexpr uint8_t UI_INT_OIS_ST_OIS_CLAMPDIS_POS = 4;                    /* 位4 禁用自检期间的 OIS 链钳位 */
constexpr uint8_t UI_INT_OIS_ST_OIS_CLAMPDIS = 0x1u << UI_INT_OIS_ST_OIS_CLAMPDIS_POS;

constexpr uint8_t UI_CTRL1_OIS = 0x70;                                   /* OIS 配置寄存器（R/W 主控 / R SPI2, 复位 0x00） */
constexpr uint8_t UI_CTRL1_OIS_SIM_OIS_POS = 5;                          /* 位5 SPI2 3 线或 4 线接口 */
constexpr uint8_t UI_CTRL1_OIS_SIM_OIS = 0x1u << UI_CTRL1_OIS_SIM_OIS_POS;
constexpr uint8_t UI_CTRL1_OIS_OIS_XL_EN_POS = 2;                        /* 位2 使能加速度计 OIS 链 */
constexpr uint8_t UI_CTRL1_OIS_OIS_XL_EN = 0x1u << UI_CTRL1_OIS_OIS_XL_EN_POS;
constexpr uint8_t UI_CTRL1_OIS_OIS_G_EN_POS = 1;                         /* 位1 使能陀螺仪 OIS 链 */
constexpr uint8_t UI_CTRL1_OIS_OIS_G_EN = 0x1u << UI_CTRL1_OIS_OIS_G_EN_POS;
constexpr uint8_t UI_CTRL1_OIS_SPI2_READ_EN_POS = 0;                     /* 位0 在主接口完全控制模式下，使能辅助 SPI 用于读取 OIS 数据 */
constexpr uint8_t UI_CTRL1_OIS_SPI2_READ_EN = 0x1u << UI_CTRL1_OIS_SPI2_READ_EN_POS;

constexpr uint8_t UI_CTRL2_OIS = 0x71;                                   /* OIS 配置寄存器（R/W 主控 / R SPI2, 复位 0x00） */
constexpr uint8_t UI_CTRL2_OIS_LPF1_G_OIS_BW_POS = 3;                    /* 位[4:3] */
constexpr uint8_t UI_CTRL2_OIS_LPF1_G_OIS_BW_MSK = 0x18;                 /* 位[4:3] 掩码 */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_POS = 0;                         /* 位[2:0] */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_MSK = 0x07;                      /* 位[2:0] 掩码 */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_125dps = 0x0u << UI_CTRL2_OIS_FS_G_OIS_POS; /* ±125 dps */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_250dps = 0x1u << UI_CTRL2_OIS_FS_G_OIS_POS; /* ±250 dps */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_500dps = 0x2u << UI_CTRL2_OIS_FS_G_OIS_POS; /* ±500 dps */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_1000dps = 0x3u << UI_CTRL2_OIS_FS_G_OIS_POS; /* ±1000 dps */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_2000dps = 0x4u << UI_CTRL2_OIS_FS_G_OIS_POS; /* ±2000 dps */
constexpr uint8_t UI_CTRL2_OIS_FS_G_OIS_4000dps = 0x5u << UI_CTRL2_OIS_FS_G_OIS_POS; /* ±4000 dps */

constexpr uint8_t UI_CTRL3_OIS = 0x72;                                   /* OIS 配置寄存器（R/W 主控 / R SPI2, 复位 0x00） */
constexpr uint8_t UI_CTRL3_OIS_LPF_XL_OIS_BW_POS = 3;                    /* 位[5:3] */
constexpr uint8_t UI_CTRL3_OIS_LPF_XL_OIS_BW_MSK = 0x38;                 /* 位[5:3] 掩码 */
constexpr uint8_t UI_CTRL3_OIS_FS_XL_OIS_POS = 0;                        /* 位[1:0] */
constexpr uint8_t UI_CTRL3_OIS_FS_XL_OIS_MSK = 0x03;                     /* 位[1:0] 掩码 */
constexpr uint8_t UI_CTRL3_OIS_FS_XL_OIS_2g = 0x0u << UI_CTRL3_OIS_FS_XL_OIS_POS; /* ±2 g */
constexpr uint8_t UI_CTRL3_OIS_FS_XL_OIS_4g = 0x1u << UI_CTRL3_OIS_FS_XL_OIS_POS; /* ±4 g */
constexpr uint8_t UI_CTRL3_OIS_FS_XL_OIS_8g = 0x2u << UI_CTRL3_OIS_FS_XL_OIS_POS; /* ±8 g */
constexpr uint8_t UI_CTRL3_OIS_FS_XL_OIS_16g = 0x3u << UI_CTRL3_OIS_FS_XL_OIS_POS; /* ±16 g */

constexpr uint8_t X_OFS_USR = 0x73;                                      /* 加速度计 X 轴用户偏移寄存器（R/W, 复位 0x00） */

constexpr uint8_t Y_OFS_USR = 0x74;                                      /* 加速度计 Y 轴用户偏移寄存器（R/W, 复位 0x00） */

constexpr uint8_t Z_OFS_USR = 0x75;                                      /* 加速度计 Z 轴用户偏移寄存器（R/W, 复位 0x00） */

/* ---------------- 1.8 FIFO 输出（78h~7Eh，78h 起连续 7 字节 = TAG + 数据） ---------------- */
constexpr uint8_t FIFO_DATA_OUT_TAG = 0x78;                              /* FIFO 标签寄存器（R, 输出） */
/* 读 FIFO：从本寄存器一次突发读 7 字节（TAG + 3 轴 16 位数据） */
constexpr uint8_t FIFO_DATA_OUT_TAG_TAG_SENSOR_POS = 3;                  /* 位[7:3] */
constexpr uint8_t FIFO_DATA_OUT_TAG_TAG_SENSOR_MSK = 0xF8;               /* 位[7:3] 掩码 */
constexpr uint8_t FIFO_DATA_OUT_TAG_TAG_CNT_POS = 1;                     /* 位[2:1] */
constexpr uint8_t FIFO_DATA_OUT_TAG_TAG_CNT_MSK = 0x06;                  /* 位[2:1] 掩码 */
constexpr uint8_t FIFO_DATA_OUT_TAG_GYRO = 0x01u << FIFO_DATA_OUT_TAG_TAG_SENSOR_POS; /* TAG 码 0x01：陀螺仪 */
constexpr uint8_t FIFO_DATA_OUT_TAG_XL = 0x02u << FIFO_DATA_OUT_TAG_TAG_SENSOR_POS; /* TAG 码 0x02：加速度计 */
constexpr uint8_t FIFO_DATA_OUT_TAG_TEMP = 0x03u << FIFO_DATA_OUT_TAG_TAG_SENSOR_POS; /* TAG 码 0x03：温度 */
/* TAG 码 0x04：时间戳 */
constexpr uint8_t FIFO_DATA_OUT_TAG_TIMESTAMP = 0x04u << FIFO_DATA_OUT_TAG_TAG_SENSOR_POS;
constexpr uint8_t FIFO_DATA_OUT_TAG_MLC = 0x1Au << FIFO_DATA_OUT_TAG_TAG_SENSOR_POS; /* TAG 码 0x1A：MLC 结果 */
/* TAG 码 0x1B：MLC 滤波器 */
constexpr uint8_t FIFO_DATA_OUT_TAG_MLC_FILTER = 0x1Bu << FIFO_DATA_OUT_TAG_TAG_SENSOR_POS;
/* TAG 码 0x1C：MLC 特征 */
constexpr uint8_t FIFO_DATA_OUT_TAG_MLC_FEATURE = 0x1Cu << FIFO_DATA_OUT_TAG_TAG_SENSOR_POS;

constexpr uint8_t FIFO_DATA_OUT_X_L = 0x79;                              /* FIFO 数据 X 轴低字节（R, 输出） */

constexpr uint8_t FIFO_DATA_OUT_X_H = 0x7A;                              /* FIFO 数据 X 轴高字节（R, 输出） */

constexpr uint8_t FIFO_DATA_OUT_Y_L = 0x7B;                              /* FIFO 数据 Y 轴低字节（R, 输出） */

constexpr uint8_t FIFO_DATA_OUT_Y_H = 0x7C;                              /* FIFO 数据 Y 轴高字节（R, 输出） */

constexpr uint8_t FIFO_DATA_OUT_Z_L = 0x7D;                              /* FIFO 数据 Z 轴低字节（R, 输出） */

constexpr uint8_t FIFO_DATA_OUT_Z_H = 0x7E;                              /* FIFO 数据 Z 轴高字节（R, 输出） */


/* ==================== 2. 嵌入功能寄存器（先写 FUNC_CFG_ACCESS = SPACE_EMBEDDED） ==================== */
/* ---------------- 2.1 页访问与嵌入功能控制（02h~17h；PAGE_RW 在 17h） ---------------- */
constexpr uint8_t PAGE_SEL = 0x02;                                       /* 高级功能专用页选择寄存器（R/W, 复位 0x01） */
constexpr uint8_t PAGE_SEL_POS = 4;                                      /* 位[7:4] */
constexpr uint8_t PAGE_SEL_MSK = 0xF0;                                   /* 位[7:4] 掩码 */
constexpr uint8_t PAGE_SEL_PAGE_0 = 0x0u << PAGE_SEL_POS;                /* 页 0：SFLP 零偏、FSM 外部传感器/矩阵、EXT_CFG */
constexpr uint8_t PAGE_SEL_PAGE_1 = 0x1u << PAGE_SEL_POS;                /* 页 1：FSM 程序/起始地址、计步器、MLC 灵敏度 */
constexpr uint8_t PAGE_SEL_PAGE_2 = 0x2u << PAGE_SEL_POS;                /* 页 2：EXT_FORMAT、外部传感器 3 字节配置 */

constexpr uint8_t EMB_FUNC_EN_A = 0x04;                                  /* 使能嵌入功能寄存器（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_EN_A_MLC_BEFORE_FSM_EN_POS = 7;               /* 位7 使能机器学习内核功能 */
constexpr uint8_t EMB_FUNC_EN_A_MLC_BEFORE_FSM_EN = 0x1u << EMB_FUNC_EN_A_MLC_BEFORE_FSM_EN_POS;
constexpr uint8_t EMB_FUNC_EN_A_SIGN_MOTION_EN_POS = 5;                  /* 位5 使能大幅运动检测功能 */
constexpr uint8_t EMB_FUNC_EN_A_SIGN_MOTION_EN = 0x1u << EMB_FUNC_EN_A_SIGN_MOTION_EN_POS;
constexpr uint8_t EMB_FUNC_EN_A_TILT_EN_POS = 4;                         /* 位4 使能倾斜计算 */
constexpr uint8_t EMB_FUNC_EN_A_TILT_EN = 0x1u << EMB_FUNC_EN_A_TILT_EN_POS;
constexpr uint8_t EMB_FUNC_EN_A_PEDO_EN_POS = 3;                         /* 位3 使能计步器算法 */
constexpr uint8_t EMB_FUNC_EN_A_PEDO_EN = 0x1u << EMB_FUNC_EN_A_PEDO_EN_POS;
constexpr uint8_t EMB_FUNC_EN_A_SFLP_GAME_EN_POS = 1;                    /* 位1 使能用于 6 轴（加速度计 + 陀螺仪）游戏旋转向量的传感器融合低功耗算法 */
constexpr uint8_t EMB_FUNC_EN_A_SFLP_GAME_EN = 0x1u << EMB_FUNC_EN_A_SFLP_GAME_EN_POS;

constexpr uint8_t EMB_FUNC_EN_B = 0x05;                                  /* 使能嵌入功能寄存器（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_EN_B_MLC_EN_POS = 4;                          /* 位4 使能机器学习内核功能 */
constexpr uint8_t EMB_FUNC_EN_B_MLC_EN = 0x1u << EMB_FUNC_EN_B_MLC_EN_POS;
constexpr uint8_t EMB_FUNC_EN_B_FIFO_COMPR_EN_POS = 3;                   /* 位3 使能 FIFO 压缩功能 */
constexpr uint8_t EMB_FUNC_EN_B_FIFO_COMPR_EN = 0x1u << EMB_FUNC_EN_B_FIFO_COMPR_EN_POS;
constexpr uint8_t EMB_FUNC_EN_B_FSM_EN_POS = 0;                          /* 位0 使能有限状态机 (FSM) 功能 */
constexpr uint8_t EMB_FUNC_EN_B_FSM_EN = 0x1u << EMB_FUNC_EN_B_FSM_EN_POS;

constexpr uint8_t EMB_FUNC_EXEC_STATUS = 0x07;                           /* 嵌入功能执行状态寄存器（R, 输出） */
constexpr uint8_t EMB_FUNC_EXEC_STATUS_EMB_FUNC_EXEC_OVR_POS = 1;        /* 位1 */
constexpr uint8_t EMB_FUNC_EXEC_STATUS_EMB_FUNC_EXEC_OVR = 0x1u << EMB_FUNC_EXEC_STATUS_EMB_FUNC_EXEC_OVR_POS;
constexpr uint8_t EMB_FUNC_EXEC_STATUS_EMB_FUNC_ENDOP_POS = 0;           /* 位0 此位置为 1 时，没有嵌入功能正在运行 */
constexpr uint8_t EMB_FUNC_EXEC_STATUS_EMB_FUNC_ENDOP = 0x1u << EMB_FUNC_EXEC_STATUS_EMB_FUNC_ENDOP_POS;

constexpr uint8_t PAGE_ADDRESS = 0x08;                                   /* 页地址寄存器（R/W, 复位 0x00） */
/* 高级功能页内地址，配合 PAGE_RW_PAGE_WRITE / PAGE_RW_PAGE_READ 使用 */

constexpr uint8_t PAGE_VALUE = 0x09;                                     /* 页值寄存器（R/W, 复位 0x00） */
/* 高级功能页数据；连续地址可连续读写本寄存器 */

constexpr uint8_t EMB_FUNC_INT1 = 0x0A;                                  /* INT1 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_INT1_FSM_LC_POS = 7;                          /* 位7 */
constexpr uint8_t EMB_FUNC_INT1_FSM_LC = 0x1u << EMB_FUNC_INT1_FSM_LC_POS;
constexpr uint8_t EMB_FUNC_INT1_SIG_MOT_POS = 5;                         /* 位5 */
constexpr uint8_t EMB_FUNC_INT1_SIG_MOT = 0x1u << EMB_FUNC_INT1_SIG_MOT_POS;
constexpr uint8_t EMB_FUNC_INT1_TILT_POS = 4;                            /* 位4 */
constexpr uint8_t EMB_FUNC_INT1_TILT = 0x1u << EMB_FUNC_INT1_TILT_POS;
constexpr uint8_t EMB_FUNC_INT1_STEP_DETECTOR_POS = 3;                   /* 位3 */
constexpr uint8_t EMB_FUNC_INT1_STEP_DETECTOR = 0x1u << EMB_FUNC_INT1_STEP_DETECTOR_POS;

constexpr uint8_t FSM_INT1 = 0x0B;                                       /* INT1 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t FSM_INT1_FSM8_POS = 7;                                 /* 位7 */
constexpr uint8_t FSM_INT1_FSM8 = 0x1u << FSM_INT1_FSM8_POS;
constexpr uint8_t FSM_INT1_FSM7_POS = 6;                                 /* 位6 */
constexpr uint8_t FSM_INT1_FSM7 = 0x1u << FSM_INT1_FSM7_POS;
constexpr uint8_t FSM_INT1_FSM6_POS = 5;                                 /* 位5 */
constexpr uint8_t FSM_INT1_FSM6 = 0x1u << FSM_INT1_FSM6_POS;
constexpr uint8_t FSM_INT1_FSM5_POS = 4;                                 /* 位4 */
constexpr uint8_t FSM_INT1_FSM5 = 0x1u << FSM_INT1_FSM5_POS;
constexpr uint8_t FSM_INT1_FSM4_POS = 3;                                 /* 位3 */
constexpr uint8_t FSM_INT1_FSM4 = 0x1u << FSM_INT1_FSM4_POS;
constexpr uint8_t FSM_INT1_FSM3_POS = 2;                                 /* 位2 */
constexpr uint8_t FSM_INT1_FSM3 = 0x1u << FSM_INT1_FSM3_POS;
constexpr uint8_t FSM_INT1_FSM2_POS = 1;                                 /* 位1 */
constexpr uint8_t FSM_INT1_FSM2 = 0x1u << FSM_INT1_FSM2_POS;
constexpr uint8_t FSM_INT1_FSM1_POS = 0;                                 /* 位0 */
constexpr uint8_t FSM_INT1_FSM1 = 0x1u << FSM_INT1_FSM1_POS;

constexpr uint8_t MLC_INT1 = 0x0D;                                       /* INT1 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t MLC_INT1_MLC4_POS = 3;                                 /* 位3 */
constexpr uint8_t MLC_INT1_MLC4 = 0x1u << MLC_INT1_MLC4_POS;
constexpr uint8_t MLC_INT1_MLC3_POS = 2;                                 /* 位2 */
constexpr uint8_t MLC_INT1_MLC3 = 0x1u << MLC_INT1_MLC3_POS;
constexpr uint8_t MLC_INT1_MLC2_POS = 1;                                 /* 位1 */
constexpr uint8_t MLC_INT1_MLC2 = 0x1u << MLC_INT1_MLC2_POS;
constexpr uint8_t MLC_INT1_MLC1_POS = 0;                                 /* 位0 */
constexpr uint8_t MLC_INT1_MLC1 = 0x1u << MLC_INT1_MLC1_POS;

constexpr uint8_t EMB_FUNC_INT2 = 0x0E;                                  /* INT2 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_INT2_FSM_LC_POS = 7;                          /* 位7 */
constexpr uint8_t EMB_FUNC_INT2_FSM_LC = 0x1u << EMB_FUNC_INT2_FSM_LC_POS;
constexpr uint8_t EMB_FUNC_INT2_SIG_MOT_POS = 5;                         /* 位5 */
constexpr uint8_t EMB_FUNC_INT2_SIG_MOT = 0x1u << EMB_FUNC_INT2_SIG_MOT_POS;
constexpr uint8_t EMB_FUNC_INT2_TILT_POS = 4;                            /* 位4 */
constexpr uint8_t EMB_FUNC_INT2_TILT = 0x1u << EMB_FUNC_INT2_TILT_POS;
constexpr uint8_t EMB_FUNC_INT2_STEP_DETECTOR_POS = 3;                   /* 位3 */
constexpr uint8_t EMB_FUNC_INT2_STEP_DETECTOR = 0x1u << EMB_FUNC_INT2_STEP_DETECTOR_POS;

constexpr uint8_t FSM_INT2 = 0x0F;                                       /* INT2 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t FSM_INT2_FSM8_POS = 7;                                 /* 位7 */
constexpr uint8_t FSM_INT2_FSM8 = 0x1u << FSM_INT2_FSM8_POS;
constexpr uint8_t FSM_INT2_FSM7_POS = 6;                                 /* 位6 */
constexpr uint8_t FSM_INT2_FSM7 = 0x1u << FSM_INT2_FSM7_POS;
constexpr uint8_t FSM_INT2_FSM6_POS = 5;                                 /* 位5 */
constexpr uint8_t FSM_INT2_FSM6 = 0x1u << FSM_INT2_FSM6_POS;
constexpr uint8_t FSM_INT2_FSM5_POS = 4;                                 /* 位4 */
constexpr uint8_t FSM_INT2_FSM5 = 0x1u << FSM_INT2_FSM5_POS;
constexpr uint8_t FSM_INT2_FSM4_POS = 3;                                 /* 位3 */
constexpr uint8_t FSM_INT2_FSM4 = 0x1u << FSM_INT2_FSM4_POS;
constexpr uint8_t FSM_INT2_FSM3_POS = 2;                                 /* 位2 */
constexpr uint8_t FSM_INT2_FSM3 = 0x1u << FSM_INT2_FSM3_POS;
constexpr uint8_t FSM_INT2_FSM2_POS = 1;                                 /* 位1 */
constexpr uint8_t FSM_INT2_FSM2 = 0x1u << FSM_INT2_FSM2_POS;
constexpr uint8_t FSM_INT2_FSM1_POS = 0;                                 /* 位0 */
constexpr uint8_t FSM_INT2_FSM1 = 0x1u << FSM_INT2_FSM1_POS;

constexpr uint8_t MLC_INT2 = 0x11;                                       /* INT2 引脚控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t MLC_INT2_MLC4_POS = 3;                                 /* 位3 */
constexpr uint8_t MLC_INT2_MLC4 = 0x1u << MLC_INT2_MLC4_POS;
constexpr uint8_t MLC_INT2_MLC3_POS = 2;                                 /* 位2 */
constexpr uint8_t MLC_INT2_MLC3 = 0x1u << MLC_INT2_MLC3_POS;
constexpr uint8_t MLC_INT2_MLC2_POS = 1;                                 /* 位1 */
constexpr uint8_t MLC_INT2_MLC2 = 0x1u << MLC_INT2_MLC2_POS;
constexpr uint8_t MLC_INT2_MLC1_POS = 0;                                 /* 位0 */
constexpr uint8_t MLC_INT2_MLC1 = 0x1u << MLC_INT2_MLC1_POS;

constexpr uint8_t EMB_FUNC_STATUS = 0x12;                                /* 嵌入功能状态寄存器（R, 输出） */
constexpr uint8_t EMB_FUNC_STATUS_IS_FSM_LC_POS = 7;                     /* 位7 FSM 长计数器超时中断事件的中断状态位 */
constexpr uint8_t EMB_FUNC_STATUS_IS_FSM_LC = 0x1u << EMB_FUNC_STATUS_IS_FSM_LC_POS;
constexpr uint8_t EMB_FUNC_STATUS_IS_SIGMOT_POS = 5;                     /* 位5 大幅运动检测的中断状态位（1：检测到中断；0：无中断） */
constexpr uint8_t EMB_FUNC_STATUS_IS_SIGMOT = 0x1u << EMB_FUNC_STATUS_IS_SIGMOT_POS;
constexpr uint8_t EMB_FUNC_STATUS_IS_TILT_POS = 4;                       /* 位4 倾斜检测的中断状态位（1：检测到中断；0：无中断） */
constexpr uint8_t EMB_FUNC_STATUS_IS_TILT = 0x1u << EMB_FUNC_STATUS_IS_TILT_POS;
constexpr uint8_t EMB_FUNC_STATUS_IS_STEP_DET_POS = 3;                   /* 位3 步伐检测的中断状态位（1：检测到中断；0：无中断） */
constexpr uint8_t EMB_FUNC_STATUS_IS_STEP_DET = 0x1u << EMB_FUNC_STATUS_IS_STEP_DET_POS;

constexpr uint8_t FSM_STATUS = 0x13;                                     /* 有限状态机状态寄存器（R, 输出） */
constexpr uint8_t FSM_STATUS_IS_FSM8_POS = 7;                            /* 位7 FSM8 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM8 = 0x1u << FSM_STATUS_IS_FSM8_POS;
constexpr uint8_t FSM_STATUS_IS_FSM7_POS = 6;                            /* 位6 FSM7 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM7 = 0x1u << FSM_STATUS_IS_FSM7_POS;
constexpr uint8_t FSM_STATUS_IS_FSM6_POS = 5;                            /* 位5 FSM6 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM6 = 0x1u << FSM_STATUS_IS_FSM6_POS;
constexpr uint8_t FSM_STATUS_IS_FSM5_POS = 4;                            /* 位4 FSM5 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM5 = 0x1u << FSM_STATUS_IS_FSM5_POS;
constexpr uint8_t FSM_STATUS_IS_FSM4_POS = 3;                            /* 位3 FSM4 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM4 = 0x1u << FSM_STATUS_IS_FSM4_POS;
constexpr uint8_t FSM_STATUS_IS_FSM3_POS = 2;                            /* 位2 FSM3 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM3 = 0x1u << FSM_STATUS_IS_FSM3_POS;
constexpr uint8_t FSM_STATUS_IS_FSM2_POS = 1;                            /* 位1 FSM2 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM2 = 0x1u << FSM_STATUS_IS_FSM2_POS;
constexpr uint8_t FSM_STATUS_IS_FSM1_POS = 0;                            /* 位0 FSM1 中断事件的中断状态位 */
constexpr uint8_t FSM_STATUS_IS_FSM1 = 0x1u << FSM_STATUS_IS_FSM1_POS;

constexpr uint8_t MLC_STATUS = 0x15;                                     /* 机器学习内核状态寄存器（R, 输出） */
constexpr uint8_t MLC_STATUS_IS_MLC4_POS = 3;                            /* 位3 MLC4 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_IS_MLC4 = 0x1u << MLC_STATUS_IS_MLC4_POS;
constexpr uint8_t MLC_STATUS_IS_MLC3_POS = 2;                            /* 位2 MLC3 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_IS_MLC3 = 0x1u << MLC_STATUS_IS_MLC3_POS;
constexpr uint8_t MLC_STATUS_IS_MLC2_POS = 1;                            /* 位1 MLC2 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_IS_MLC2 = 0x1u << MLC_STATUS_IS_MLC2_POS;
constexpr uint8_t MLC_STATUS_IS_MLC1_POS = 0;                            /* 位0 MLC1 中断事件的中断状态位 */
constexpr uint8_t MLC_STATUS_IS_MLC1 = 0x1u << MLC_STATUS_IS_MLC1_POS;

constexpr uint8_t PAGE_RW = 0x17;                                        /* 页读写使能寄存器（R/W, 复位 0x00） */
constexpr uint8_t PAGE_RW_EMB_FUNC_LIR_POS = 7;                          /* 位7 嵌入功能的锁存中断模式 */
constexpr uint8_t PAGE_RW_EMB_FUNC_LIR = 0x1u << PAGE_RW_EMB_FUNC_LIR_POS;
constexpr uint8_t PAGE_RW_PAGE_WRITE_POS = 6;                            /* 位6 使能对所选高级功能专用页的写入 */
constexpr uint8_t PAGE_RW_PAGE_WRITE = 0x1u << PAGE_RW_PAGE_WRITE_POS;
constexpr uint8_t PAGE_RW_PAGE_READ_POS = 5;                             /* 位5 使能对所选高级功能专用页的读取 */
constexpr uint8_t PAGE_RW_PAGE_READ = 0x1u << PAGE_RW_PAGE_READ_POS;

/* ---------------- 2.2 嵌入功能 FIFO/FSM 使能（44h~46h） ---------------- */
constexpr uint8_t EMB_FUNC_FIFO_EN_A = 0x44;                             /* 嵌入功能 FIFO 配置寄存器 A（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_FIFO_EN_A_MLC_FIFO_EN_POS = 7;                /* 位7 使能将机器学习内核结果批处理到 FIFO 缓冲器中 */
constexpr uint8_t EMB_FUNC_FIFO_EN_A_MLC_FIFO_EN = 0x1u << EMB_FUNC_FIFO_EN_A_MLC_FIFO_EN_POS;
constexpr uint8_t EMB_FUNC_FIFO_EN_A_STEP_COUNTER_FIFO_EN_POS = 6;       /* 位6 使能将步数计数器值批处理到 FIFO 缓冲器中 */
constexpr uint8_t EMB_FUNC_FIFO_EN_A_STEP_COUNTER_FIFO_EN = 0x1u << EMB_FUNC_FIFO_EN_A_STEP_COUNTER_FIFO_EN_POS;
constexpr uint8_t EMB_FUNC_FIFO_EN_A_SFLP_GBIAS_FIFO_EN_POS = 5;         /* 位5 使能将 SFLP 算法计算得到的陀螺仪偏差值批处理到 FIFO缓冲器中 */
constexpr uint8_t EMB_FUNC_FIFO_EN_A_SFLP_GBIAS_FIFO_EN = 0x1u << EMB_FUNC_FIFO_EN_A_SFLP_GBIAS_FIFO_EN_POS;
constexpr uint8_t EMB_FUNC_FIFO_EN_A_SFLP_GRAVITY_FIFO_EN_POS = 4;       /* 位4 使能将 SFLP 算法计算得到的重力值批处理到 FIFO 缓冲器中 */
constexpr uint8_t EMB_FUNC_FIFO_EN_A_SFLP_GRAVITY_FIFO_EN = 0x1u << EMB_FUNC_FIFO_EN_A_SFLP_GRAVITY_FIFO_EN_POS;
constexpr uint8_t EMB_FUNC_FIFO_EN_A_SFLP_GAME_FIFO_EN_POS = 1;          /* 位1 使能将 SFLP算法计算得到的游戏旋转向量（四元数）值批处理到 FIFO 缓冲器 */
constexpr uint8_t EMB_FUNC_FIFO_EN_A_SFLP_GAME_FIFO_EN = 0x1u << EMB_FUNC_FIFO_EN_A_SFLP_GAME_FIFO_EN_POS;

constexpr uint8_t EMB_FUNC_FIFO_EN_B = 0x45;                             /* 嵌入功能 FIFO 配置寄存器 B（R/W, 输出） */
constexpr uint8_t EMB_FUNC_FIFO_EN_B_MLC_FILTER_FEATURE_FIFO_EN_POS = 1; /* 位1 使能将机器学习内核滤波器和特征批处理到 FIFO 缓冲器中 */
constexpr uint8_t EMB_FUNC_FIFO_EN_B_MLC_FILTER_FEATURE_FIFO_EN = 0x1u << EMB_FUNC_FIFO_EN_B_MLC_FILTER_FEATURE_FIFO_EN_POS;

constexpr uint8_t FSM_ENABLE = 0x46;                                     /* FSM 使能寄存器（R/W, 复位 0x00） */
constexpr uint8_t FSM_ENABLE_FSM8_EN_POS = 7;                            /* 位7 使能 FSM8 */
constexpr uint8_t FSM_ENABLE_FSM8_EN = 0x1u << FSM_ENABLE_FSM8_EN_POS;
constexpr uint8_t FSM_ENABLE_FSM7_EN_POS = 6;                            /* 位6 使能 FSM7 */
constexpr uint8_t FSM_ENABLE_FSM7_EN = 0x1u << FSM_ENABLE_FSM7_EN_POS;
constexpr uint8_t FSM_ENABLE_FSM6_EN_POS = 5;                            /* 位5 使能 FSM6 */
constexpr uint8_t FSM_ENABLE_FSM6_EN = 0x1u << FSM_ENABLE_FSM6_EN_POS;
constexpr uint8_t FSM_ENABLE_FSM5_EN_POS = 4;                            /* 位4 使能 FSM5 */
constexpr uint8_t FSM_ENABLE_FSM5_EN = 0x1u << FSM_ENABLE_FSM5_EN_POS;
constexpr uint8_t FSM_ENABLE_FSM4_EN_POS = 3;                            /* 位3 使能 FSM4 */
constexpr uint8_t FSM_ENABLE_FSM4_EN = 0x1u << FSM_ENABLE_FSM4_EN_POS;
constexpr uint8_t FSM_ENABLE_FSM3_EN_POS = 2;                            /* 位2 使能 FSM3 */
constexpr uint8_t FSM_ENABLE_FSM3_EN = 0x1u << FSM_ENABLE_FSM3_EN_POS;
constexpr uint8_t FSM_ENABLE_FSM2_EN_POS = 1;                            /* 位1 使能 FSM2 */
constexpr uint8_t FSM_ENABLE_FSM2_EN = 0x1u << FSM_ENABLE_FSM2_EN_POS;
constexpr uint8_t FSM_ENABLE_FSM1_EN_POS = 0;                            /* 位0 使能 FSM1 */
constexpr uint8_t FSM_ENABLE_FSM1_EN = 0x1u << FSM_ENABLE_FSM1_EN_POS;

/* ---------------- 2.3 FSM 长计数器与中断应答（48h~4Bh） ---------------- */
constexpr uint8_t FSM_LONG_COUNTER_L = 0x48;                             /* FSM 长计数器低字节（R/W, 复位 0x00） */
/* 32 位无符号长计数器，本寄存器为低字节 */

constexpr uint8_t FSM_LONG_COUNTER_H = 0x49;                             /* FSM 长计数器高字节（R/W, 复位 0x00） */

constexpr uint8_t INT_ACK_MASK = 0x4B;                                   /* 复位状态寄存器（R/W, 复位 0x00） */
constexpr uint8_t INT_ACK_MASK_IACK_MASK_POS = 0;                        /* 位[7:0] */
constexpr uint8_t INT_ACK_MASK_IACK_MASK_MSK = 0xFF;                     /* 位[7:0] 掩码 */

/* ---------------- 2.4 FSM 输出（4Ch~53h） ---------------- */
constexpr uint8_t FSM_OUTS1 = 0x4C;                                      /* FSM1 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS1_P_X_POS = 7;                                 /* 位7 FSM1 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS1_P_X = 0x1u << FSM_OUTS1_P_X_POS;
constexpr uint8_t FSM_OUTS1_N_X_POS = 6;                                 /* 位6 FSM1 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS1_N_X = 0x1u << FSM_OUTS1_N_X_POS;
constexpr uint8_t FSM_OUTS1_P_Y_POS = 5;                                 /* 位5 FSM1 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS1_P_Y = 0x1u << FSM_OUTS1_P_Y_POS;
constexpr uint8_t FSM_OUTS1_N_Y_POS = 4;                                 /* 位4 FSM1 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS1_N_Y = 0x1u << FSM_OUTS1_N_Y_POS;
constexpr uint8_t FSM_OUTS1_P_Z_POS = 3;                                 /* 位3 FSM1 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS1_P_Z = 0x1u << FSM_OUTS1_P_Z_POS;
constexpr uint8_t FSM_OUTS1_N_Z_POS = 2;                                 /* 位2 FSM1 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS1_N_Z = 0x1u << FSM_OUTS1_N_Z_POS;
constexpr uint8_t FSM_OUTS1_P_V_POS = 1;                                 /* 位1 FSM1 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS1_P_V = 0x1u << FSM_OUTS1_P_V_POS;
constexpr uint8_t FSM_OUTS1_N_V_POS = 0;                                 /* 位0 FSM1 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS1_N_V = 0x1u << FSM_OUTS1_N_V_POS;

constexpr uint8_t FSM_OUTS2 = 0x4D;                                      /* FSM2 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS2_P_X_POS = 7;                                 /* 位7 FSM2 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS2_P_X = 0x1u << FSM_OUTS2_P_X_POS;
constexpr uint8_t FSM_OUTS2_N_X_POS = 6;                                 /* 位6 FSM2 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS2_N_X = 0x1u << FSM_OUTS2_N_X_POS;
constexpr uint8_t FSM_OUTS2_P_Y_POS = 5;                                 /* 位5 FSM2 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS2_P_Y = 0x1u << FSM_OUTS2_P_Y_POS;
constexpr uint8_t FSM_OUTS2_N_Y_POS = 4;                                 /* 位4 FSM2 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS2_N_Y = 0x1u << FSM_OUTS2_N_Y_POS;
constexpr uint8_t FSM_OUTS2_P_Z_POS = 3;                                 /* 位3 FSM2 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS2_P_Z = 0x1u << FSM_OUTS2_P_Z_POS;
constexpr uint8_t FSM_OUTS2_N_Z_POS = 2;                                 /* 位2 FSM2 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS2_N_Z = 0x1u << FSM_OUTS2_N_Z_POS;
constexpr uint8_t FSM_OUTS2_P_V_POS = 1;                                 /* 位1 FSM2 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS2_P_V = 0x1u << FSM_OUTS2_P_V_POS;
constexpr uint8_t FSM_OUTS2_N_V_POS = 0;                                 /* 位0 FSM2 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS2_N_V = 0x1u << FSM_OUTS2_N_V_POS;

constexpr uint8_t FSM_OUTS3 = 0x4E;                                      /* FSM3 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS3_P_X_POS = 7;                                 /* 位7 FSM3 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS3_P_X = 0x1u << FSM_OUTS3_P_X_POS;
constexpr uint8_t FSM_OUTS3_N_X_POS = 6;                                 /* 位6 FSM3 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS3_N_X = 0x1u << FSM_OUTS3_N_X_POS;
constexpr uint8_t FSM_OUTS3_P_Y_POS = 5;                                 /* 位5 FSM3 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS3_P_Y = 0x1u << FSM_OUTS3_P_Y_POS;
constexpr uint8_t FSM_OUTS3_N_Y_POS = 4;                                 /* 位4 FSM3 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS3_N_Y = 0x1u << FSM_OUTS3_N_Y_POS;
constexpr uint8_t FSM_OUTS3_P_Z_POS = 3;                                 /* 位3 FSM3 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS3_P_Z = 0x1u << FSM_OUTS3_P_Z_POS;
constexpr uint8_t FSM_OUTS3_N_Z_POS = 2;                                 /* 位2 FSM3 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS3_N_Z = 0x1u << FSM_OUTS3_N_Z_POS;
constexpr uint8_t FSM_OUTS3_P_V_POS = 1;                                 /* 位1 FSM3 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS3_P_V = 0x1u << FSM_OUTS3_P_V_POS;
constexpr uint8_t FSM_OUTS3_N_V_POS = 0;                                 /* 位0 FSM3 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS3_N_V = 0x1u << FSM_OUTS3_N_V_POS;

constexpr uint8_t FSM_OUTS4 = 0x4F;                                      /* FSM4 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS4_P_X_POS = 7;                                 /* 位7 FSM4 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS4_P_X = 0x1u << FSM_OUTS4_P_X_POS;
constexpr uint8_t FSM_OUTS4_N_X_POS = 6;                                 /* 位6 FSM4 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS4_N_X = 0x1u << FSM_OUTS4_N_X_POS;
constexpr uint8_t FSM_OUTS4_P_Y_POS = 5;                                 /* 位5 FSM4 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS4_P_Y = 0x1u << FSM_OUTS4_P_Y_POS;
constexpr uint8_t FSM_OUTS4_N_Y_POS = 4;                                 /* 位4 FSM4 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS4_N_Y = 0x1u << FSM_OUTS4_N_Y_POS;
constexpr uint8_t FSM_OUTS4_P_Z_POS = 3;                                 /* 位3 FSM4 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS4_P_Z = 0x1u << FSM_OUTS4_P_Z_POS;
constexpr uint8_t FSM_OUTS4_N_Z_POS = 2;                                 /* 位2 FSM4 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS4_N_Z = 0x1u << FSM_OUTS4_N_Z_POS;
constexpr uint8_t FSM_OUTS4_P_V_POS = 1;                                 /* 位1 FSM4 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS4_P_V = 0x1u << FSM_OUTS4_P_V_POS;
constexpr uint8_t FSM_OUTS4_N_V_POS = 0;                                 /* 位0 FSM4 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS4_N_V = 0x1u << FSM_OUTS4_N_V_POS;

constexpr uint8_t FSM_OUTS5 = 0x50;                                      /* FSM5 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS5_P_X_POS = 7;                                 /* 位7 FSM5 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS5_P_X = 0x1u << FSM_OUTS5_P_X_POS;
constexpr uint8_t FSM_OUTS5_N_X_POS = 6;                                 /* 位6 FSM5 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS5_N_X = 0x1u << FSM_OUTS5_N_X_POS;
constexpr uint8_t FSM_OUTS5_P_Y_POS = 5;                                 /* 位5 FSM5 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS5_P_Y = 0x1u << FSM_OUTS5_P_Y_POS;
constexpr uint8_t FSM_OUTS5_N_Y_POS = 4;                                 /* 位4 FSM5 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS5_N_Y = 0x1u << FSM_OUTS5_N_Y_POS;
constexpr uint8_t FSM_OUTS5_P_Z_POS = 3;                                 /* 位3 FSM5 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS5_P_Z = 0x1u << FSM_OUTS5_P_Z_POS;
constexpr uint8_t FSM_OUTS5_N_Z_POS = 2;                                 /* 位2 FSM5 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS5_N_Z = 0x1u << FSM_OUTS5_N_Z_POS;
constexpr uint8_t FSM_OUTS5_P_V_POS = 1;                                 /* 位1 FSM5 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS5_P_V = 0x1u << FSM_OUTS5_P_V_POS;
constexpr uint8_t FSM_OUTS5_N_V_POS = 0;                                 /* 位0 FSM5 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS5_N_V = 0x1u << FSM_OUTS5_N_V_POS;

constexpr uint8_t FSM_OUTS6 = 0x51;                                      /* FSM6 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS6_P_X_POS = 7;                                 /* 位7 FSM6 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS6_P_X = 0x1u << FSM_OUTS6_P_X_POS;
constexpr uint8_t FSM_OUTS6_N_X_POS = 6;                                 /* 位6 FSM6 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS6_N_X = 0x1u << FSM_OUTS6_N_X_POS;
constexpr uint8_t FSM_OUTS6_P_Y_POS = 5;                                 /* 位5 FSM6 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS6_P_Y = 0x1u << FSM_OUTS6_P_Y_POS;
constexpr uint8_t FSM_OUTS6_N_Y_POS = 4;                                 /* 位4 FSM6 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS6_N_Y = 0x1u << FSM_OUTS6_N_Y_POS;
constexpr uint8_t FSM_OUTS6_P_Z_POS = 3;                                 /* 位3 FSM6 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS6_P_Z = 0x1u << FSM_OUTS6_P_Z_POS;
constexpr uint8_t FSM_OUTS6_N_Z_POS = 2;                                 /* 位2 FSM6 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS6_N_Z = 0x1u << FSM_OUTS6_N_Z_POS;
constexpr uint8_t FSM_OUTS6_P_V_POS = 1;                                 /* 位1 FSM6 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS6_P_V = 0x1u << FSM_OUTS6_P_V_POS;
constexpr uint8_t FSM_OUTS6_N_V_POS = 0;                                 /* 位0 FSM6 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS6_N_V = 0x1u << FSM_OUTS6_N_V_POS;

constexpr uint8_t FSM_OUTS7 = 0x52;                                      /* FSM7 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS7_P_X_POS = 7;                                 /* 位7 FSM7 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS7_P_X = 0x1u << FSM_OUTS7_P_X_POS;
constexpr uint8_t FSM_OUTS7_N_X_POS = 6;                                 /* 位6 FSM7 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS7_N_X = 0x1u << FSM_OUTS7_N_X_POS;
constexpr uint8_t FSM_OUTS7_P_Y_POS = 5;                                 /* 位5 FSM7 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS7_P_Y = 0x1u << FSM_OUTS7_P_Y_POS;
constexpr uint8_t FSM_OUTS7_N_Y_POS = 4;                                 /* 位4 FSM7 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS7_N_Y = 0x1u << FSM_OUTS7_N_Y_POS;
constexpr uint8_t FSM_OUTS7_P_Z_POS = 3;                                 /* 位3 FSM7 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS7_P_Z = 0x1u << FSM_OUTS7_P_Z_POS;
constexpr uint8_t FSM_OUTS7_N_Z_POS = 2;                                 /* 位2 FSM7 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS7_N_Z = 0x1u << FSM_OUTS7_N_Z_POS;
constexpr uint8_t FSM_OUTS7_P_V_POS = 1;                                 /* 位1 FSM7 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS7_P_V = 0x1u << FSM_OUTS7_P_V_POS;
constexpr uint8_t FSM_OUTS7_N_V_POS = 0;                                 /* 位0 FSM7 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS7_N_V = 0x1u << FSM_OUTS7_N_V_POS;

constexpr uint8_t FSM_OUTS8 = 0x53;                                      /* FSM8 输出寄存器（R, 输出） */
constexpr uint8_t FSM_OUTS8_P_X_POS = 7;                                 /* 位7 FSM8 输出：在 X 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS8_P_X = 0x1u << FSM_OUTS8_P_X_POS;
constexpr uint8_t FSM_OUTS8_N_X_POS = 6;                                 /* 位6 FSM8 输出：在 X 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS8_N_X = 0x1u << FSM_OUTS8_N_X_POS;
constexpr uint8_t FSM_OUTS8_P_Y_POS = 5;                                 /* 位5 FSM8 输出：在 Y 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS8_P_Y = 0x1u << FSM_OUTS8_P_Y_POS;
constexpr uint8_t FSM_OUTS8_N_Y_POS = 4;                                 /* 位4 FSM8 输出：在 Y 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS8_N_Y = 0x1u << FSM_OUTS8_N_Y_POS;
constexpr uint8_t FSM_OUTS8_P_Z_POS = 3;                                 /* 位3 FSM8 输出：在 Z 轴上检测到正向事件 */
constexpr uint8_t FSM_OUTS8_P_Z = 0x1u << FSM_OUTS8_P_Z_POS;
constexpr uint8_t FSM_OUTS8_N_Z_POS = 2;                                 /* 位2 FSM8 输出：在 Z 轴上检测到负向事件 */
constexpr uint8_t FSM_OUTS8_N_Z = 0x1u << FSM_OUTS8_N_Z_POS;
constexpr uint8_t FSM_OUTS8_P_V_POS = 1;                                 /* 位1 FSM8 输出：在向量上检测到正向事件 */
constexpr uint8_t FSM_OUTS8_P_V = 0x1u << FSM_OUTS8_P_V_POS;
constexpr uint8_t FSM_OUTS8_N_V_POS = 0;                                 /* 位0 FSM8 输出：在向量上检测到负向事件 */
constexpr uint8_t FSM_OUTS8_N_V = 0x1u << FSM_OUTS8_N_V_POS;

/* ---------------- 2.5 嵌入功能 ODR（5Eh~60h） ---------------- */
constexpr uint8_t SFLP_ODR = 0x5E;                                       /* 传感器融合低功耗输出数据速率配置寄存器（R/W, 复位 0x5B） */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_POS = 3;                        /* 位[5:3] */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_MSK = 0x38;                     /* 位[5:3] 掩码 */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_15Hz = 0x0u << SFLP_ODR_SFLP_GAME_ODR_POS; /* 15 Hz */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_30Hz = 0x1u << SFLP_ODR_SFLP_GAME_ODR_POS; /* 30 Hz */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_60Hz = 0x2u << SFLP_ODR_SFLP_GAME_ODR_POS; /* 60 Hz */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_120Hz = 0x3u << SFLP_ODR_SFLP_GAME_ODR_POS; /* 120 Hz（默认） */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_240Hz = 0x4u << SFLP_ODR_SFLP_GAME_ODR_POS; /* 240 Hz */
constexpr uint8_t SFLP_ODR_SFLP_GAME_ODR_480Hz = 0x5u << SFLP_ODR_SFLP_GAME_ODR_POS; /* 480 Hz */

constexpr uint8_t FSM_ODR = 0x5F;                                        /* 有限状态机输出数据速率配置寄存器（R/W, 复位 0x4B） */
constexpr uint8_t FSM_ODR_POS = 3;                                       /* 位[5:3] */
constexpr uint8_t FSM_ODR_MSK = 0x38;                                    /* 位[5:3] 掩码 */
constexpr uint8_t FSM_ODR_15Hz = 0x0u << FSM_ODR_POS;                    /* 15 Hz */
constexpr uint8_t FSM_ODR_30Hz = 0x1u << FSM_ODR_POS;                    /* 30 Hz（默认） */
constexpr uint8_t FSM_ODR_60Hz = 0x2u << FSM_ODR_POS;                    /* 60 Hz */
constexpr uint8_t FSM_ODR_120Hz = 0x3u << FSM_ODR_POS;                   /* 120 Hz */
constexpr uint8_t FSM_ODR_240Hz = 0x4u << FSM_ODR_POS;                   /* 240 Hz */
constexpr uint8_t FSM_ODR_480Hz = 0x5u << FSM_ODR_POS;                   /* 480 Hz */
constexpr uint8_t FSM_ODR_960Hz = 0x6u << FSM_ODR_POS;                   /* 960 Hz */

constexpr uint8_t MLC_ODR = 0x60;                                        /* 机器学习内核输出数据速率配置寄存器（R/W, 复位 0x15） */
constexpr uint8_t MLC_ODR_POS = 4;                                       /* 位[6:4] */
constexpr uint8_t MLC_ODR_MSK = 0x70;                                    /* 位[6:4] 掩码 */
constexpr uint8_t MLC_ODR_15Hz = 0x0u << MLC_ODR_POS;                    /* 15 Hz */
constexpr uint8_t MLC_ODR_30Hz = 0x1u << MLC_ODR_POS;                    /* 30 Hz（默认） */
constexpr uint8_t MLC_ODR_60Hz = 0x2u << MLC_ODR_POS;                    /* 60 Hz */
constexpr uint8_t MLC_ODR_120Hz = 0x3u << MLC_ODR_POS;                   /* 120 Hz */
constexpr uint8_t MLC_ODR_240Hz = 0x4u << MLC_ODR_POS;                   /* 240 Hz */

/* ---------------- 2.6 计步器与嵌入功能源（62h~64h） ---------------- */
constexpr uint8_t STEP_COUNTER_L = 0x62;                                 /* 步数计数器低字节（R, 输出） */
/* 16 位无符号步数，本寄存器为低字节 */

constexpr uint8_t STEP_COUNTER_H = 0x63;                                 /* 步数计数器高字节（R, 输出） */

constexpr uint8_t EMB_FUNC_SRC = 0x64;                                   /* 嵌入功能源寄存器（R/W, 输出） */
constexpr uint8_t EMB_FUNC_SRC_PEDO_RST_STEP_POS = 7;                    /* 位7 复位计步器步数计数器 */
constexpr uint8_t EMB_FUNC_SRC_PEDO_RST_STEP = 0x1u << EMB_FUNC_SRC_PEDO_RST_STEP_POS;
constexpr uint8_t EMB_FUNC_SRC_STEP_DETECTED_POS = 5;                    /* 位5 步伐检测器事件检测状态 */
constexpr uint8_t EMB_FUNC_SRC_STEP_DETECTED = 0x1u << EMB_FUNC_SRC_STEP_DETECTED_POS;
constexpr uint8_t EMB_FUNC_SRC_STEP_COUNT_DELTA_IA_POS = 4;              /* 位4 计步器在 delta 时间内的步伐识别状态 */
constexpr uint8_t EMB_FUNC_SRC_STEP_COUNT_DELTA_IA = 0x1u << EMB_FUNC_SRC_STEP_COUNT_DELTA_IA_POS;
constexpr uint8_t EMB_FUNC_SRC_STEP_OVERFLOW_POS = 3;                    /* 位3 步数计数器溢出状态 */
constexpr uint8_t EMB_FUNC_SRC_STEP_OVERFLOW = 0x1u << EMB_FUNC_SRC_STEP_OVERFLOW_POS;
constexpr uint8_t EMB_FUNC_SRC_STEPCOUNTER_BIT_SET_POS = 2;              /* 位2 当步数增加时，该位等于 1 */
constexpr uint8_t EMB_FUNC_SRC_STEPCOUNTER_BIT_SET = 0x1u << EMB_FUNC_SRC_STEPCOUNTER_BIT_SET_POS;

/* ---------------- 2.7 嵌入功能初始化请求（66h~67h） ---------------- */
constexpr uint8_t EMB_FUNC_INIT_A = 0x66;                                /* 嵌入功能初始化寄存器（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_INIT_A_MLC_BEFORE_FSM_INIT_POS = 7;           /* 位7 机器学习内核初始化请求（MLC 在 FSM 之前执行） */
constexpr uint8_t EMB_FUNC_INIT_A_MLC_BEFORE_FSM_INIT = 0x1u << EMB_FUNC_INIT_A_MLC_BEFORE_FSM_INIT_POS;
constexpr uint8_t EMB_FUNC_INIT_A_SIG_MOT_INIT_POS = 5;                  /* 位5 大幅运动检测算法初始化请求 */
constexpr uint8_t EMB_FUNC_INIT_A_SIG_MOT_INIT = 0x1u << EMB_FUNC_INIT_A_SIG_MOT_INIT_POS;
constexpr uint8_t EMB_FUNC_INIT_A_TILT_INIT_POS = 4;                     /* 位4 倾斜算法初始化请求 */
constexpr uint8_t EMB_FUNC_INIT_A_TILT_INIT = 0x1u << EMB_FUNC_INIT_A_TILT_INIT_POS;
constexpr uint8_t EMB_FUNC_INIT_A_STEP_DET_INIT_POS = 3;                 /* 位3 计步器步数计数器/检测器算法初始化请求 */
constexpr uint8_t EMB_FUNC_INIT_A_STEP_DET_INIT = 0x1u << EMB_FUNC_INIT_A_STEP_DET_INIT_POS;
constexpr uint8_t EMB_FUNC_INIT_A_SFLP_GAME_INIT_POS = 1;                /* 位1 SFLP 游戏算法初始化请求 */
constexpr uint8_t EMB_FUNC_INIT_A_SFLP_GAME_INIT = 0x1u << EMB_FUNC_INIT_A_SFLP_GAME_INIT_POS;

constexpr uint8_t EMB_FUNC_INIT_B = 0x67;                                /* 嵌入功能初始化寄存器（R/W, 复位 0x00） */
constexpr uint8_t EMB_FUNC_INIT_B_MLC_INIT_POS = 4;                      /* 位4 机器学习内核初始化请求（MLC 在 FSM 之后执行） */
constexpr uint8_t EMB_FUNC_INIT_B_MLC_INIT = 0x1u << EMB_FUNC_INIT_B_MLC_INIT_POS;
constexpr uint8_t EMB_FUNC_INIT_B_FIFO_COMPR_INIT_POS = 3;               /* 位3 FIFO 压缩特征初始化请求 */
constexpr uint8_t EMB_FUNC_INIT_B_FIFO_COMPR_INIT = 0x1u << EMB_FUNC_INIT_B_FIFO_COMPR_INIT_POS;
constexpr uint8_t EMB_FUNC_INIT_B_FSM_INIT_POS = 0;                      /* 位0 FSM 初始化请求 */
constexpr uint8_t EMB_FUNC_INIT_B_FSM_INIT = 0x1u << EMB_FUNC_INIT_B_FSM_INIT_POS;

/* ---------------- 2.8 MLC 决策树输出（70h~73h） ---------------- */
constexpr uint8_t MLC1_SRC = 0x70;                                       /* MLC1 决策树输出寄存器（R, 输出） */

constexpr uint8_t MLC2_SRC = 0x71;                                       /* MLC2 决策树输出寄存器（R, 输出） */

constexpr uint8_t MLC3_SRC = 0x72;                                       /* MLC3 决策树输出寄存器（R, 输出） */

constexpr uint8_t MLC4_SRC = 0x73;                                       /* MLC4 决策树输出寄存器（R, 输出） */


/* ==================== 3. 高级功能页 0（PAGE_SEL = PAGE_SEL_PAGE_0） ==================== */
constexpr uint8_t SFLP_GAME_GBIASX_L = 0x6E;                             /* SFLP 游戏算法 X 轴陀螺零偏低字节（R/W, 复位 0x00） */

constexpr uint8_t SFLP_GAME_GBIASX_H = 0x6F;                             /* SFLP 游戏算法 X 轴陀螺零偏高字节（R/W, 复位 0x00） */

constexpr uint8_t SFLP_GAME_GBIASY_L = 0x70;                             /* SFLP 游戏算法 Y 轴陀螺零偏低字节（R/W, 复位 0x00） */

constexpr uint8_t SFLP_GAME_GBIASY_H = 0x71;                             /* SFLP 游戏算法 Y 轴陀螺零偏高字节（R/W, 复位 0x00） */

constexpr uint8_t SFLP_GAME_GBIASZ_L = 0x72;                             /* SFLP 游戏算法 Z 轴陀螺零偏低字节（R/W, 复位 0x00） */

constexpr uint8_t SFLP_GAME_GBIASZ_H = 0x73;                             /* SFLP 游戏算法 Z 轴陀螺零偏高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_SENSITIVITY_L = 0xBA;                          /* FSM 外部传感器灵敏度低字节（R/W, 复位 0x24） */

constexpr uint8_t FSM_EXT_SENSITIVITY_H = 0xBB;                          /* FSM 外部传感器灵敏度高字节（R/W, 复位 0x16） */

constexpr uint8_t FSM_EXT_OFFX_L = 0xC0;                                 /* 外部传感器 X 轴偏移低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_OFFX_H = 0xC1;                                 /* 外部传感器 X 轴偏移高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_OFFY_L = 0xC2;                                 /* 外部传感器 Y 轴偏移低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_OFFY_H = 0xC3;                                 /* 外部传感器 Y 轴偏移高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_OFFZ_L = 0xC4;                                 /* 外部传感器 Z 轴偏移低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_OFFZ_H = 0xC5;                                 /* 外部传感器 Z 轴偏移高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_XX_L = 0xC6;                            /* 外部传感器变换矩阵 XX 低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_XX_H = 0xC7;                            /* 外部传感器变换矩阵 XX 高字节（R/W, 复位 0x3C） */

constexpr uint8_t FSM_EXT_MATRIX_XY_L = 0xC8;                            /* 外部传感器变换矩阵 XY 低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_XY_H = 0xC9;                            /* 外部传感器变换矩阵 XY 高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_XZ_L = 0xCA;                            /* 外部传感器变换矩阵 XZ 低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_XZ_H = 0xCB;                            /* 外部传感器变换矩阵 XZ 高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_YY_L = 0xCC;                            /* 外部传感器变换矩阵 YY 低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_YY_H = 0xCD;                            /* 外部传感器变换矩阵 YY 高字节（R/W, 复位 0x3C） */

constexpr uint8_t FSM_EXT_MATRIX_YZ_L = 0xCE;                            /* 外部传感器变换矩阵 YZ 低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_YZ_H = 0xCF;                            /* 外部传感器变换矩阵 YZ 高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_ZZ_L = 0xD0;                            /* 外部传感器变换矩阵 ZZ 低字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_EXT_MATRIX_ZZ_H = 0xD1;                            /* 外部传感器变换矩阵 ZZ 高字节（R/W, 复位 0x3C） */

constexpr uint8_t EXT_CFG_A = 0xD4;                                      /* 外部传感器坐标（R/W, 复位 0x05） */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_POS = 4;                          /* 位[6:4] */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_MSK = 0x70;                       /* 位[6:4] 掩码 */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_EQ_Y = 0x0u << EXT_CFG_A_EXT_Y_AXIS_POS; /* Y = Y */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_EQ_MIN_Y = 0x1u << EXT_CFG_A_EXT_Y_AXIS_POS; /* Y = -Y */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_EQ_X = 0x2u << EXT_CFG_A_EXT_Y_AXIS_POS; /* Y = X */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_EQ_MIN_X = 0x3u << EXT_CFG_A_EXT_Y_AXIS_POS; /* Y = -X */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_EQ_MIN_Z = 0x4u << EXT_CFG_A_EXT_Y_AXIS_POS; /* Y = -Z */
constexpr uint8_t EXT_CFG_A_EXT_Y_AXIS_EQ_Z = 0x5u << EXT_CFG_A_EXT_Y_AXIS_POS; /* Y = Z */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_POS = 0;                          /* 位[2:0] */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_MSK = 0x07;                       /* 位[2:0] 掩码 */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_EQ_Y = 0x0u << EXT_CFG_A_EXT_Z_AXIS_POS; /* Z = Y（默认） */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_EQ_MIN_Y = 0x1u << EXT_CFG_A_EXT_Z_AXIS_POS; /* Z = -Y */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_EQ_X = 0x2u << EXT_CFG_A_EXT_Z_AXIS_POS; /* Z = X */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_EQ_MIN_X = 0x3u << EXT_CFG_A_EXT_Z_AXIS_POS; /* Z = -X */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_EQ_MIN_Z = 0x4u << EXT_CFG_A_EXT_Z_AXIS_POS; /* Z = -Z */
constexpr uint8_t EXT_CFG_A_EXT_Z_AXIS_EQ_Z = 0x5u << EXT_CFG_A_EXT_Z_AXIS_POS; /* Z = Z */

constexpr uint8_t EXT_CFG_B = 0xD5;                                      /* 外部传感器坐标（R/W, 复位 0x02） */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_POS = 0;                          /* 位[2:0] */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_MSK = 0x07;                       /* 位[2:0] 掩码 */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_EQ_Y = 0x0u << EXT_CFG_B_EXT_X_AXIS_POS; /* X = Y */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_EQ_MIN_Y = 0x1u << EXT_CFG_B_EXT_X_AXIS_POS; /* X = -Y */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_EQ_X = 0x2u << EXT_CFG_B_EXT_X_AXIS_POS; /* X = X（默认） */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_EQ_MIN_X = 0x3u << EXT_CFG_B_EXT_X_AXIS_POS; /* X = -X */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_EQ_MIN_Z = 0x4u << EXT_CFG_B_EXT_X_AXIS_POS; /* X = -Z */
constexpr uint8_t EXT_CFG_B_EXT_X_AXIS_EQ_Z = 0x5u << EXT_CFG_B_EXT_X_AXIS_POS; /* X = Z */


/* ==================== 4. 高级功能页 1（PAGE_SEL = PAGE_SEL_PAGE_1；FSM 程序与计步器在此页） ==================== */
constexpr uint8_t FSM_LC_TIMEOUT_L = 0x7A;                               /* FSM 长计数器超时低字节（R/W, 复位 0x00） */
/* 16 位长计数器超时值，本寄存器为低字节 */

constexpr uint8_t FSM_LC_TIMEOUT_H = 0x7B;                               /* FSM 长计数器超时高字节（R/W, 复位 0x00） */

constexpr uint8_t FSM_PROGRAMS = 0x7C;                                   /* FSM 程序数量寄存器（R/W, 复位 0x00） */
/* FSM 程序数量（每程序 16 条指令），程序区首个可用地址 0x35C */
constexpr uint8_t FSM_PROGRAMS_FSM_N_PROG_POS = 0;                       /* 位[7:0] */
constexpr uint8_t FSM_PROGRAMS_FSM_N_PROG_MSK = 0xFF;                    /* 位[7:0] 掩码 */

constexpr uint8_t FSM_START_ADD_L = 0x7E;                                /* FSM 起始地址低字节（R/W, 复位 0x00） */
/* 16 位 FSM 起始地址，本寄存器为低字节 */

constexpr uint8_t FSM_START_ADD_H = 0x7F;                                /* FSM 起始地址高字节（R/W, 复位 0x00） */

constexpr uint8_t PEDO_CMD_REG = 0x83;                                   /* 计步器配置寄存器（R/W, 复位 0x00） */
constexpr uint8_t PEDO_CMD_REG_CARRY_COUNT_EN_POS = 3;                   /* 位3 当用户希望仅在计数溢出事件产生中断时置为 1 */
constexpr uint8_t PEDO_CMD_REG_CARRY_COUNT_EN = 0x1u << PEDO_CMD_REG_CARRY_COUNT_EN_POS;
constexpr uint8_t PEDO_CMD_REG_FP_REJECTION_EN_POS = 2;                  /* 位2 使能假阳性抑制功能 */
constexpr uint8_t PEDO_CMD_REG_FP_REJECTION_EN = 0x1u << PEDO_CMD_REG_FP_REJECTION_EN_POS;

constexpr uint8_t PEDO_DEB_STEPS_CONF = 0x84;                            /* 计步器去抖配置寄存器（R/W, 复位 0x0A） */
constexpr uint8_t PEDO_DEB_STEPS_CONF_DEB_STEP_POS = 0;                  /* 位[7:0] */
constexpr uint8_t PEDO_DEB_STEPS_CONF_DEB_STEP_MSK = 0xFF;               /* 位[7:0] 掩码 */

constexpr uint8_t PEDO_SC_DELTAT_L = 0xD0;                               /* 计步器应达时间低字节（R/W, 复位 0x00） */
/* 16 位计步时间周期，1 LSB = 6.4 ms */

constexpr uint8_t PEDO_SC_DELTAT_H = 0xD1;                               /* 计步器应达时间高字节（R/W, 复位 0x00） */

constexpr uint8_t MLC_EXT_SENSITIVITY_L = 0xE8;                          /* MLC 外部传感器灵敏度低字节（R/W, 复位 0x00） */
/* 16 位无符号灵敏度换算值，本寄存器为低字节 */

constexpr uint8_t MLC_EXT_SENSITIVITY_H = 0xE9;                          /* MLC 外部传感器灵敏度高字节（R/W, 复位 0x3C） */


/* ==================== 5. 高级功能页 2（PAGE_SEL = PAGE_SEL_PAGE_2） ==================== */
constexpr uint8_t EXT_FORMAT = 0x00;                                     /* AH/Qvar/外部传感器数据格式选择（R/W, 复位 0x00） */
constexpr uint8_t EXT_FORMAT_EXT_FORMAT_SEL_POS = 2;                     /* 位2 选择用于 FSM 和 MLC 处理的 AH / Qvar / 外部传感器数据格式 */
/* AH/Qvar/外部传感器 2 字节格式（默认） */
constexpr uint8_t EXT_FORMAT_EXT_FORMAT_SEL_2BYTE = 0x0u << EXT_FORMAT_EXT_FORMAT_SEL_POS;
/* 3 字节格式 */
constexpr uint8_t EXT_FORMAT_EXT_FORMAT_SEL_3BYTE = 0x1u << EXT_FORMAT_EXT_FORMAT_SEL_POS;

constexpr uint8_t EXT_3BYTE_SENSITIVITY_L = 0x02;                        /* 外部传感器(3 字节)灵敏度低字节（R/W, 复位 0x00） */

constexpr uint8_t EXT_3BYTE_SENSITIVITY_H = 0x03;                        /* 外部传感器(3 字节)灵敏度高字节（R/W, 复位 0x0C） */

constexpr uint8_t EXT_3BYTE_OFFSET_XL = 0x06;                            /* 外部传感器(3 字节)偏移最低字节（R/W, 复位 0x00） */

constexpr uint8_t EXT_3BYTE_OFFSET_L = 0x07;                             /* 外部传感器(3 字节)偏移低字节（R/W, 复位 0x54） */

constexpr uint8_t EXT_3BYTE_OFFSET_H = 0x08;                             /* 外部传感器(3 字节)偏移高字节（R/W, 复位 0x3F） */


/* ==================== 6. 传感器集线器寄存器（先写 FUNC_CFG_ACCESS = SPACE_SENSOR_HUB） ==================== */
constexpr uint8_t SENSOR_HUB_1 = 0x02;                                   /* 传感器集线器输出寄存器 1（R, 输出） */
/* SENSOR_HUB_1~18 依次对应外部传感器读回的连续字节 */

constexpr uint8_t SENSOR_HUB_2 = 0x03;                                   /* 传感器集线器输出寄存器 2（R, 输出） */

constexpr uint8_t SENSOR_HUB_3 = 0x04;                                   /* 传感器集线器输出寄存器 3（R, 输出） */

constexpr uint8_t SENSOR_HUB_4 = 0x05;                                   /* 传感器集线器输出寄存器 4（R, 输出） */

constexpr uint8_t SENSOR_HUB_5 = 0x06;                                   /* 传感器集线器输出寄存器 5（R, 输出） */

constexpr uint8_t SENSOR_HUB_6 = 0x07;                                   /* 传感器集线器输出寄存器 6（R, 输出） */

constexpr uint8_t SENSOR_HUB_7 = 0x08;                                   /* 传感器集线器输出寄存器 7（R, 输出） */

constexpr uint8_t SENSOR_HUB_8 = 0x09;                                   /* 传感器集线器输出寄存器 8（R, 输出） */

constexpr uint8_t SENSOR_HUB_9 = 0x0A;                                   /* 传感器集线器输出寄存器 9（R, 输出） */

constexpr uint8_t SENSOR_HUB_10 = 0x0B;                                  /* 传感器集线器输出寄存器 10（R, 输出） */

constexpr uint8_t SENSOR_HUB_11 = 0x0C;                                  /* 传感器集线器输出寄存器 11（R, 输出） */

constexpr uint8_t SENSOR_HUB_12 = 0x0D;                                  /* 传感器集线器输出寄存器 12（R, 输出） */

constexpr uint8_t SENSOR_HUB_13 = 0x0E;                                  /* 传感器集线器输出寄存器 13（R, 输出） */

constexpr uint8_t SENSOR_HUB_14 = 0x0F;                                  /* 传感器集线器输出寄存器 14（R, 输出） */

constexpr uint8_t SENSOR_HUB_15 = 0x10;                                  /* 传感器集线器输出寄存器 15（R, 输出） */

constexpr uint8_t SENSOR_HUB_16 = 0x11;                                  /* 传感器集线器输出寄存器 16（R, 输出） */

constexpr uint8_t SENSOR_HUB_17 = 0x12;                                  /* 传感器集线器输出寄存器 17（R, 输出） */

constexpr uint8_t SENSOR_HUB_18 = 0x13;                                  /* 传感器集线器输出寄存器 18（R, 输出） */

constexpr uint8_t MASTER_CONFIG = 0x14;                                  /* 主器件配置寄存器（R/W, 复位 0x00） */
constexpr uint8_t MASTER_CONFIG_RST_MASTER_REGS_POS = 7;                 /* 位7 复位主器件逻辑和输出寄存器 */
constexpr uint8_t MASTER_CONFIG_RST_MASTER_REGS = 0x1u << MASTER_CONFIG_RST_MASTER_REGS_POS;
constexpr uint8_t MASTER_CONFIG_WRITE_ONCE_POS = 6;                      /* 位6 仅在第一个传感器集线器周期执行从器件 0 写操作 */
constexpr uint8_t MASTER_CONFIG_WRITE_ONCE = 0x1u << MASTER_CONFIG_WRITE_ONCE_POS;
constexpr uint8_t MASTER_CONFIG_START_CONFIG_POS = 5;                    /* 位5 传感器集线器触发信号选择 */
constexpr uint8_t MASTER_CONFIG_START_CONFIG = 0x1u << MASTER_CONFIG_START_CONFIG_POS;
constexpr uint8_t MASTER_CONFIG_PASS_THROUGH_MODE_POS = 4;               /* 位4 I²C 接口直通 */
constexpr uint8_t MASTER_CONFIG_PASS_THROUGH_MODE = 0x1u << MASTER_CONFIG_PASS_THROUGH_MODE_POS;
constexpr uint8_t MASTER_CONFIG_MASTER_ON_POS = 2;                       /* 位2 使能传感器集线器 I²C 主器件 */
constexpr uint8_t MASTER_CONFIG_MASTER_ON = 0x1u << MASTER_CONFIG_MASTER_ON_POS;
constexpr uint8_t MASTER_CONFIG_AUX_SENS_ON_POS = 0;                     /* 位[1:0] */
constexpr uint8_t MASTER_CONFIG_AUX_SENS_ON_MSK = 0x03;                  /* 位[1:0] 掩码 */
constexpr uint8_t MASTER_CONFIG_AUX_SENS_ON_ONE = 0x0u << MASTER_CONFIG_AUX_SENS_ON_POS; /* 读 1 个外部传感器（默认） */
constexpr uint8_t MASTER_CONFIG_AUX_SENS_ON_TWO = 0x1u << MASTER_CONFIG_AUX_SENS_ON_POS; /* 读 2 个 */
/* 读 3 个 */
constexpr uint8_t MASTER_CONFIG_AUX_SENS_ON_THREE = 0x2u << MASTER_CONFIG_AUX_SENS_ON_POS;
/* 读 4 个 */
constexpr uint8_t MASTER_CONFIG_AUX_SENS_ON_FOUR = 0x3u << MASTER_CONFIG_AUX_SENS_ON_POS;

constexpr uint8_t SLV0_ADD = 0x15;                                       /* 第一个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV0_ADD_SLAVE0_ADD_POS = 1;                           /* 位[7:1] */
constexpr uint8_t SLV0_ADD_SLAVE0_ADD_MSK = 0xFE;                        /* 位[7:1] 掩码 */
constexpr uint8_t SLV0_ADD_RW_0_POS = 0;                                 /* 位0 */
constexpr uint8_t SLV0_ADD_RW_0_WRITE = 0x0u << SLV0_ADD_RW_0_POS;       /* 对从器件 0 执行写操作 */
constexpr uint8_t SLV0_ADD_RW_0_READ = 0x1u << SLV0_ADD_RW_0_POS;        /* 对从器件 0 执行读操作 */

constexpr uint8_t SLV0_SUBADD = 0x16;                                    /* 第一个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV0_SUBADD_SLAVE0_REG_POS = 0;                        /* 位[7:0] */
constexpr uint8_t SLV0_SUBADD_SLAVE0_REG_MSK = 0xFF;                     /* 位[7:0] 掩码 */

constexpr uint8_t SLV0_CONFIG = 0x17;                                    /* 第一个外部传感器（R/W, 复位 0x80） */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_POS = 5;                          /* 位[7:5] */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_MSK = 0xE0;                       /* 位[7:5] 掩码 */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_1Hz875 = 0x0u << SLV0_CONFIG_SHUB_ODR_POS; /* 1.875 Hz */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_15Hz = 0x1u << SLV0_CONFIG_SHUB_ODR_POS; /* 15 Hz */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_30Hz = 0x2u << SLV0_CONFIG_SHUB_ODR_POS; /* 30 Hz */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_60Hz = 0x3u << SLV0_CONFIG_SHUB_ODR_POS; /* 60 Hz */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_120Hz = 0x4u << SLV0_CONFIG_SHUB_ODR_POS; /* 120 Hz（默认） */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_240Hz = 0x5u << SLV0_CONFIG_SHUB_ODR_POS; /* 240 Hz */
constexpr uint8_t SLV0_CONFIG_SHUB_ODR_480Hz = 0x6u << SLV0_CONFIG_SHUB_ODR_POS; /* 480 Hz */
constexpr uint8_t SLV0_CONFIG_BATCH_EXT_SENS_0_EN_POS = 3;               /* 位3 使能第一个从器件的 FIFO 数据批处理 */
constexpr uint8_t SLV0_CONFIG_BATCH_EXT_SENS_0_EN = 0x1u << SLV0_CONFIG_BATCH_EXT_SENS_0_EN_POS;
constexpr uint8_t SLV0_CONFIG_SLAVE0_NUMOP_POS = 0;                      /* 位[2:0] */
constexpr uint8_t SLV0_CONFIG_SLAVE0_NUMOP_MSK = 0x07;                   /* 位[2:0] 掩码 */

constexpr uint8_t SLV1_ADD = 0x18;                                       /* 第二个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV1_ADD_SLAVE1_ADD_POS = 1;                           /* 位[7:1] */
constexpr uint8_t SLV1_ADD_SLAVE1_ADD_MSK = 0xFE;                        /* 位[7:1] 掩码 */
constexpr uint8_t SLV1_ADD_R_1_POS = 0;                                  /* 位0 使能传感器 1 的读操作 */
constexpr uint8_t SLV1_ADD_R_1_DISABLE = 0x0u << SLV1_ADD_R_1_POS;       /* 从器件 1 读操作禁止（默认） */
constexpr uint8_t SLV1_ADD_R_1_ENABLE = 0x1u << SLV1_ADD_R_1_POS;        /* 从器件 1 读操作使能 */

constexpr uint8_t SLV1_SUBADD = 0x19;                                    /* 第二个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV1_SUBADD_SLAVE1_REG_POS = 0;                        /* 位[7:0] */
constexpr uint8_t SLV1_SUBADD_SLAVE1_REG_MSK = 0xFF;                     /* 位[7:0] 掩码 */

constexpr uint8_t SLV1_CONFIG = 0x1A;                                    /* 第二个外部传感器（R/W, 复位 0x10） */
constexpr uint8_t SLV1_CONFIG_BATCH_EXT_SENS_1_EN_POS = 3;               /* 位3 使能第二个从器件的 FIFO 数据批处理 */
constexpr uint8_t SLV1_CONFIG_BATCH_EXT_SENS_1_EN = 0x1u << SLV1_CONFIG_BATCH_EXT_SENS_1_EN_POS;
constexpr uint8_t SLV1_CONFIG_SLAVE1_NUMOP_POS = 0;                      /* 位[2:0] */
constexpr uint8_t SLV1_CONFIG_SLAVE1_NUMOP_MSK = 0x07;                   /* 位[2:0] 掩码 */

constexpr uint8_t SLV2_ADD = 0x1B;                                       /* 第三个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV2_ADD_SLAVE2_ADD_POS = 1;                           /* 位[7:1] */
constexpr uint8_t SLV2_ADD_SLAVE2_ADD_MSK = 0xFE;                        /* 位[7:1] 掩码 */
constexpr uint8_t SLV2_ADD_R_2_POS = 0;                                  /* 位0 使能传感器 2 的读操作 */
constexpr uint8_t SLV2_ADD_R_2_DISABLE = 0x0u << SLV2_ADD_R_2_POS;       /* 从器件 2 读操作禁止（默认） */
constexpr uint8_t SLV2_ADD_R_2_ENABLE = 0x1u << SLV2_ADD_R_2_POS;        /* 从器件 2 读操作使能 */

constexpr uint8_t SLV2_SUBADD = 0x1C;                                    /* 第三个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV2_SUBADD_SLAVE2_REG_POS = 0;                        /* 位[7:0] */
constexpr uint8_t SLV2_SUBADD_SLAVE2_REG_MSK = 0xFF;                     /* 位[7:0] 掩码 */

constexpr uint8_t SLV2_CONFIG = 0x1D;                                    /* 第三个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV2_CONFIG_BATCH_EXT_SENS_2_EN_POS = 3;               /* 位3 使能第三个从器件的 FIFO 数据批处理 */
constexpr uint8_t SLV2_CONFIG_BATCH_EXT_SENS_2_EN = 0x1u << SLV2_CONFIG_BATCH_EXT_SENS_2_EN_POS;
constexpr uint8_t SLV2_CONFIG_SLAVE2_NUMOP_POS = 0;                      /* 位[2:0] */
constexpr uint8_t SLV2_CONFIG_SLAVE2_NUMOP_MSK = 0x07;                   /* 位[2:0] 掩码 */

constexpr uint8_t SLV3_ADD = 0x1E;                                       /* 第四个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV3_ADD_SLAVE3_ADD_POS = 1;                           /* 位[7:1] */
constexpr uint8_t SLV3_ADD_SLAVE3_ADD_MSK = 0xFE;                        /* 位[7:1] 掩码 */
constexpr uint8_t SLV3_ADD_R_3_POS = 0;                                  /* 位0 使能传感器 3 的读操作 */
constexpr uint8_t SLV3_ADD_R_3_DISABLE = 0x0u << SLV3_ADD_R_3_POS;       /* 从器件 3 读操作禁止（默认） */
constexpr uint8_t SLV3_ADD_R_3_ENABLE = 0x1u << SLV3_ADD_R_3_POS;        /* 从器件 3 读操作使能 */

constexpr uint8_t SLV3_SUBADD = 0x1F;                                    /* 第四个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV3_SUBADD_SLAVE3_REG_POS = 0;                        /* 位[7:0] */
constexpr uint8_t SLV3_SUBADD_SLAVE3_REG_MSK = 0xFF;                     /* 位[7:0] 掩码 */

constexpr uint8_t SLV3_CONFIG = 0x20;                                    /* 第四个外部传感器（R/W, 复位 0x00） */
constexpr uint8_t SLV3_CONFIG_BATCH_EXT_SENS_3_EN_POS = 3;               /* 位3 使能第四个从器件的 FIFO 数据批处理 */
constexpr uint8_t SLV3_CONFIG_BATCH_EXT_SENS_3_EN = 0x1u << SLV3_CONFIG_BATCH_EXT_SENS_3_EN_POS;
constexpr uint8_t SLV3_CONFIG_SLAVE3_NUMOP_POS = 0;                      /* 位[2:0] */
constexpr uint8_t SLV3_CONFIG_SLAVE3_NUMOP_MSK = 0x07;                   /* 位[2:0] 掩码 */

constexpr uint8_t DATAWRITE_SLV0 = 0x21;                                 /* 要写入从器件寄存器的数据（R/W, 复位 0x00） */
constexpr uint8_t DATAWRITE_SLV0_SLAVE0_DATAW_POS = 0;                   /* 位[7:0] */
constexpr uint8_t DATAWRITE_SLV0_SLAVE0_DATAW_MSK = 0xFF;                /* 位[7:0] 掩码 */

constexpr uint8_t STATUS_MASTER = 0x22;                                  /* 传感器集线器源寄存器（R, 输出） */
constexpr uint8_t STATUS_MASTER_WR_ONCE_DONE_POS = 7;                    /* 位7 当 MASTER_CONFIG (14h) 中的 WRITE_ONCE 位配置为 */
constexpr uint8_t STATUS_MASTER_WR_ONCE_DONE = 0x1u << STATUS_MASTER_WR_ONCE_DONE_POS;
constexpr uint8_t STATUS_MASTER_SLAVE3_NACK_POS = 6;                     /* 位6 若从器件 3 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_SLAVE3_NACK = 0x1u << STATUS_MASTER_SLAVE3_NACK_POS;
constexpr uint8_t STATUS_MASTER_SLAVE2_NACK_POS = 5;                     /* 位5 若从器件 2 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_SLAVE2_NACK = 0x1u << STATUS_MASTER_SLAVE2_NACK_POS;
constexpr uint8_t STATUS_MASTER_SLAVE1_NACK_POS = 4;                     /* 位4 若从器件 1 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_SLAVE1_NACK = 0x1u << STATUS_MASTER_SLAVE1_NACK_POS;
constexpr uint8_t STATUS_MASTER_SLAVE0_NACK_POS = 3;                     /* 位3 若从器件 0 通信中出现未应答，则该位被置为 1 */
constexpr uint8_t STATUS_MASTER_SLAVE0_NACK = 0x1u << STATUS_MASTER_SLAVE0_NACK_POS;
constexpr uint8_t STATUS_MASTER_SENS_HUB_ENDOP_POS = 0;                  /* 位0 传感器集线器通信状态 */
constexpr uint8_t STATUS_MASTER_SENS_HUB_ENDOP = 0x1u << STATUS_MASTER_SENS_HUB_ENDOP_POS;


/* ==================== 7. SPI2/OIS 侧寄存器（只在辅助接口下可见；本项目只用主接口） ==================== */
constexpr uint8_t SPI2_WHO_AM_I = 0x0F;                                  /* SPI2 侧 WHO_AM_I 寄存器（R, 复位 0x70） */

constexpr uint8_t SPI2_STATUS_REG_OIS = 0x1E;                            /* OIS 状态寄存器（SPI2 侧）（R, 输出） */
constexpr uint8_t SPI2_STATUS_REG_OIS_GYRO_SETTLING_POS = 2;             /* 位2 */
constexpr uint8_t SPI2_STATUS_REG_OIS_GYRO_SETTLING = 0x1u << SPI2_STATUS_REG_OIS_GYRO_SETTLING_POS;
constexpr uint8_t SPI2_STATUS_REG_OIS_GDA_POS = 1;                       /* 位1 */
constexpr uint8_t SPI2_STATUS_REG_OIS_GDA = 0x1u << SPI2_STATUS_REG_OIS_GDA_POS;
constexpr uint8_t SPI2_STATUS_REG_OIS_XLDA_POS = 0;                      /* 位0 */
constexpr uint8_t SPI2_STATUS_REG_OIS_XLDA = 0x1u << SPI2_STATUS_REG_OIS_XLDA_POS;

constexpr uint8_t SPI2_OUT_TEMP_L = 0x20;                                /* SPI2 温度输出低字节（R, 输出） */

constexpr uint8_t SPI2_OUT_TEMP_H = 0x21;                                /* SPI2 温度输出高字节（R, 输出） */

constexpr uint8_t SPI2_OUTX_L_G_OIS = 0x22;                              /* SPI2 陀螺仪 X 轴 OIS 输出低字节（R, 输出） */

constexpr uint8_t SPI2_OUTX_H_G_OIS = 0x23;                              /* SPI2 陀螺仪 X 轴 OIS 输出高字节（R, 输出） */

constexpr uint8_t SPI2_OUTY_L_G_OIS = 0x24;                              /* SPI2 陀螺仪 Y 轴 OIS 输出低字节（R, 输出） */

constexpr uint8_t SPI2_OUTY_H_G_OIS = 0x25;                              /* SPI2 陀螺仪 Y 轴 OIS 输出高字节（R, 输出） */

constexpr uint8_t SPI2_OUTZ_L_G_OIS = 0x26;                              /* SPI2 陀螺仪 Z 轴 OIS 输出低字节（R, 输出） */

constexpr uint8_t SPI2_OUTZ_H_G_OIS = 0x27;                              /* SPI2 陀螺仪 Z 轴 OIS 输出高字节（R, 输出） */

constexpr uint8_t SPI2_OUTX_L_A_OIS = 0x28;                              /* SPI2 加速度计 X 轴 OIS 输出低字节（R, 输出） */

constexpr uint8_t SPI2_OUTX_H_A_OIS = 0x29;                              /* SPI2 加速度计 X 轴 OIS 输出高字节（R, 输出） */

constexpr uint8_t SPI2_OUTY_L_A_OIS = 0x2A;                              /* SPI2 加速度计 Y 轴 OIS 输出低字节（R, 输出） */

constexpr uint8_t SPI2_OUTY_H_A_OIS = 0x2B;                              /* SPI2 加速度计 Y 轴 OIS 输出高字节（R, 输出） */

constexpr uint8_t SPI2_OUTZ_L_A_OIS = 0x2C;                              /* SPI2 加速度计 Z 轴 OIS 输出低字节（R, 输出） */

constexpr uint8_t SPI2_OUTZ_H_A_OIS = 0x2D;                              /* SPI2 加速度计 Z 轴 OIS 输出高字节（R, 输出） */

constexpr uint8_t SPI2_HANDSHAKE_CTRL = 0x6E;                            /* UI / SPI2 共享寄存器的控制寄存器（R/W, 复位 0x00） */
constexpr uint8_t SPI2_HANDSHAKE_CTRL_SPI2_SHARED_REQ_POS = 1;           /* 位1 辅助 SPI (SPI2) 接口主器件使用该位请求访问UI_SPI2_SHARE */
constexpr uint8_t SPI2_HANDSHAKE_CTRL_SPI2_SHARED_REQ = 0x1u << SPI2_HANDSHAKE_CTRL_SPI2_SHARED_REQ_POS;
constexpr uint8_t SPI2_HANDSHAKE_CTRL_SPI2_SHARED_ACK_POS = 0;           /* 位0 辅助 SPI (SPI2) 接口侧 */
constexpr uint8_t SPI2_HANDSHAKE_CTRL_SPI2_SHARED_ACK = 0x1u << SPI2_HANDSHAKE_CTRL_SPI2_SHARED_ACK_POS;

constexpr uint8_t SPI2_INT_OIS = 0x6F;                                   /* OIS 中断配置寄存器和自检设置（R/W SPI2 / R 主控, 复位 0x00） */
constexpr uint8_t SPI2_INT_OIS_INT2_DRDY_OIS_POS = 7;                    /* 位7 在 INT2 引脚上使能 OIS 链 DRDY */
constexpr uint8_t SPI2_INT_OIS_INT2_DRDY_OIS = 0x1u << SPI2_INT_OIS_INT2_DRDY_OIS_POS;
constexpr uint8_t SPI2_INT_OIS_DRDY_MASK_OIS_POS = 6;                    /* 位6 使能/屏蔽 OIS 数据可用 */
constexpr uint8_t SPI2_INT_OIS_DRDY_MASK_OIS = 0x1u << SPI2_INT_OIS_DRDY_MASK_OIS_POS;
constexpr uint8_t SPI2_INT_OIS_ST_OIS_CLAMPDIS_POS = 4;                  /* 位4 在自检期间禁用 OIS 链钳位 */
constexpr uint8_t SPI2_INT_OIS_ST_OIS_CLAMPDIS = 0x1u << SPI2_INT_OIS_ST_OIS_CLAMPDIS_POS;
constexpr uint8_t SPI2_INT_OIS_ST_G_OIS_POS = 2;                         /* 位[3:2] */
constexpr uint8_t SPI2_INT_OIS_ST_G_OIS_MSK = 0x0C;                      /* 位[3:2] 掩码 */
constexpr uint8_t SPI2_INT_OIS_ST_XL_OIS_POS = 0;                        /* 位[1:0] */
constexpr uint8_t SPI2_INT_OIS_ST_XL_OIS_MSK = 0x03;                     /* 位[1:0] 掩码 */

constexpr uint8_t SPI2_CTRL1_OIS = 0x70;                                 /* OIS 配置寄存器（R/W SPI2 / R 主控, 复位 0x00） */
constexpr uint8_t SPI2_CTRL1_OIS_SIM_OIS_POS = 5;                        /* 位5 SPI2 3 线或 4 线接口 */
constexpr uint8_t SPI2_CTRL1_OIS_SIM_OIS = 0x1u << SPI2_CTRL1_OIS_SIM_OIS_POS;
constexpr uint8_t SPI2_CTRL1_OIS_OIS_XL_EN_POS = 2;                      /* 位2 使能加速度计 OIS 链 */
constexpr uint8_t SPI2_CTRL1_OIS_OIS_XL_EN = 0x1u << SPI2_CTRL1_OIS_OIS_XL_EN_POS;
constexpr uint8_t SPI2_CTRL1_OIS_OIS_G_EN_POS = 1;                       /* 位1 使能陀螺仪 OIS 链 */
constexpr uint8_t SPI2_CTRL1_OIS_OIS_G_EN = 0x1u << SPI2_CTRL1_OIS_OIS_G_EN_POS;
constexpr uint8_t SPI2_CTRL1_OIS_SPI2_READ_EN_POS = 0;                   /* 位0 在辅助 SPI 完全控制模式下，使能辅助 SPI 以读取寄存器 SPI2_OUT */
constexpr uint8_t SPI2_CTRL1_OIS_SPI2_READ_EN = 0x1u << SPI2_CTRL1_OIS_SPI2_READ_EN_POS;

constexpr uint8_t SPI2_CTRL2_OIS = 0x71;                                 /* OIS 配置寄存器（R/W SPI2 / R 主控, 复位 0x00） */
constexpr uint8_t SPI2_CTRL2_OIS_LPF1_G_OIS_BW_POS = 3;                  /* 位[4:3] */
constexpr uint8_t SPI2_CTRL2_OIS_LPF1_G_OIS_BW_MSK = 0x18;               /* 位[4:3] 掩码 */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_POS = 0;                       /* 位[2:0] */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_MSK = 0x07;                    /* 位[2:0] 掩码 */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_125dps = 0x0u << SPI2_CTRL2_OIS_FS_G_OIS_POS; /* ±125 dps */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_250dps = 0x1u << SPI2_CTRL2_OIS_FS_G_OIS_POS; /* ±250 dps */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_500dps = 0x2u << SPI2_CTRL2_OIS_FS_G_OIS_POS; /* ±500 dps */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_1000dps = 0x3u << SPI2_CTRL2_OIS_FS_G_OIS_POS; /* ±1000 dps */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_2000dps = 0x4u << SPI2_CTRL2_OIS_FS_G_OIS_POS; /* ±2000 dps */
constexpr uint8_t SPI2_CTRL2_OIS_FS_G_OIS_4000dps = 0x5u << SPI2_CTRL2_OIS_FS_G_OIS_POS; /* ±4000 dps */

constexpr uint8_t SPI2_CTRL3_OIS = 0x72;                                 /* OIS 配置寄存器（R/W SPI2 / R 主控, 复位 0x00） */
constexpr uint8_t SPI2_CTRL3_OIS_LPF_XL_OIS_BW_POS = 3;                  /* 位[5:3] */
constexpr uint8_t SPI2_CTRL3_OIS_LPF_XL_OIS_BW_MSK = 0x38;               /* 位[5:3] 掩码 */
constexpr uint8_t SPI2_CTRL3_OIS_FS_XL_OIS_POS = 0;                      /* 位[1:0] */
constexpr uint8_t SPI2_CTRL3_OIS_FS_XL_OIS_MSK = 0x03;                   /* 位[1:0] 掩码 */
constexpr uint8_t SPI2_CTRL3_OIS_FS_XL_OIS_2g = 0x0u << SPI2_CTRL3_OIS_FS_XL_OIS_POS; /* ±2 g */
constexpr uint8_t SPI2_CTRL3_OIS_FS_XL_OIS_4g = 0x1u << SPI2_CTRL3_OIS_FS_XL_OIS_POS; /* ±4 g */
constexpr uint8_t SPI2_CTRL3_OIS_FS_XL_OIS_8g = 0x2u << SPI2_CTRL3_OIS_FS_XL_OIS_POS; /* ±8 g */
constexpr uint8_t SPI2_CTRL3_OIS_FS_XL_OIS_16g = 0x3u << SPI2_CTRL3_OIS_FS_XL_OIS_POS; /* ±16 g */


/* ==================== 8. 满量程与换算 ==================== */
/* UI 输出换算示例，陀螺灵敏度取自 DS13510 Rev 4 表 2：
 *   accel_g = static_cast<float>(raw) * Range::XL_8g / 32768.0f;
 *   gyro_dps = static_cast<float>(raw) * Sensitivity::G_2000dps;
 *   gyro_rad_s = gyro_dps * 3.14159265f / 180.0f;
 *   temp_c = static_cast<float>(raw) / TEMP_LSB_PER_C + TEMP_ZERO_C;   */
namespace Range {
/* 加速度计满量程（g）：CTRL8_FS_XL_* / UI_CTRL3_OIS_FS_XL_OIS_* */
constexpr float XL_2g = 2.0f;
constexpr float XL_4g = 4.0f;
constexpr float XL_8g = 8.0f;
constexpr float XL_16g = 16.0f;
/* 陀螺仪 UI 链满量程（dps）：CTRL6_FS_G_* */
constexpr float G_125dps = 125.0f;
constexpr float G_250dps = 250.0f;
constexpr float G_500dps = 500.0f;
constexpr float G_1000dps = 1000.0f;
constexpr float G_2000dps = 2000.0f;
constexpr float G_4000dps = 4000.0f;
/* 陀螺仪 EIS/OIS 链满量程（dps）：CTRL_EIS_FS_G_EIS_* / UI_CTRL2_OIS_FS_G_OIS_* */
constexpr float G_EIS_125dps = 125.0f;
constexpr float G_EIS_250dps = 250.0f;
constexpr float G_EIS_500dps = 500.0f;
constexpr float G_EIS_1000dps = 1000.0f;
constexpr float G_EIS_2000dps = 2000.0f;
constexpr float G_OIS_4000dps = 4000.0f;
}  /* namespace Range */

namespace Sensitivity {
/* UI 陀螺仪灵敏度，单位 °/s/LSB（官方 mdps/LSB 数值乘 0.001）。 */
constexpr float G_125dps = 0.004375f;
constexpr float G_250dps = 0.008750f;
constexpr float G_500dps = 0.017500f;
constexpr float G_1000dps = 0.035000f;
constexpr float G_2000dps = 0.070000f;
constexpr float G_4000dps = 0.140000f;
}  /* namespace Sensitivity */

constexpr float TEMP_LSB_PER_C = 256.0f;   /* 温度：256 LSB/°C */
constexpr float TEMP_ZERO_C = 25.0f;       /* 温度：0 LSB 对应 25 °C */

/* ==================== 9. 突发读与用法示例 ==================== */
constexpr uint8_t OUT_BURST_START = OUT_TEMP_L;          /* 20h：温度 2 + 陀螺 6 + 加速度 6 */
constexpr uint8_t OUT_BURST_LEN = 14u;
constexpr uint8_t OUT_IMU_START = OUTX_L_G;              /* 22h：陀螺 XYZ + 加速度 XYZ */
constexpr uint8_t OUT_IMU_LEN = 12u;
constexpr uint8_t OUT_AREA_START = OUT_TEMP_L;           /* 20h~3Bh 整块输出区 */
constexpr uint8_t OUT_AREA_LEN = 28u;
constexpr uint8_t FIFO_WORD_START = FIFO_DATA_OUT_TAG;   /* 78h：TAG(1) + 数据(6) */
constexpr uint8_t FIFO_WORD_LEN = 7u;

/*
 * 初始化（主页面）：
 *   write(CTRL3, CTRL3_BDU | CTRL3_IF_INC);                     // 0x44
 *   write(CTRL8, CTRL8_FS_XL_8g);                               // 先定满量程
 *   write(CTRL6, CTRL6_FS_G_2000dps);
 *   write(CTRL1, CTRL1_OP_MODE_XL_HIGH_PERFORMANCE | CTRL1_ODR_XL_480Hz);   // 最后落 ODR
 *   write(CTRL2, CTRL2_OP_MODE_G_HIGH_PERFORMANCE | CTRL2_ODR_G_480Hz);
 * 取数（一次事务读 14 字节）：
 *   read(OUT_BURST_START, buf, OUT_BURST_LEN);
 *   accel_g  = int16(buf[8] | buf[9] << 8) * Range::XL_8g / 32768.0f;
 *   gyro_dps = int16(buf[2] | buf[3] << 8) * Sensitivity::G_2000dps;
 *   temp_c   = int16(buf[0] | buf[1] << 8) / TEMP_LSB_PER_C + TEMP_ZERO_C;
 * FIFO 批处理：
 *   write(FIFO_CTRL3, FIFO_CTRL3_BDR_XL_480Hz | FIFO_CTRL3_BDR_GY_480Hz);
 *   write(FIFO_CTRL4, FIFO_CTRL4_FIFO_MODE_CONTINUOUS);
 *   未读字数 = status1（DIFF_FIFO[7:0]）+ status2 的 DIFF_FIFO_8
 *   read(FIFO_WORD_START, w, FIFO_WORD_LEN);
 *   switch (w[0] & FIFO_DATA_OUT_TAG_TAG_SENSOR_MSK) { ... FIFO_DATA_OUT_TAG_GYRO ... }
 * 写 FSM/MLC 配置（进嵌入功能空间 + 选页）：
 *   write(FUNC_CFG_ACCESS, SPACE_EMBEDDED);
 *   write(PAGE_SEL, PAGE_SEL_PAGE_1);
 *   write(PAGE_RW, PAGE_RW_PAGE_WRITE);
 *   write(PAGE_ADDRESS, addr); write(PAGE_VALUE, value);        // 连续地址可连续写
 *   write(PAGE_RW, 0x00); write(FUNC_CFG_ACCESS, SPACE_MAIN);
 * 读改写：
 *   v = read(CTRL1); v = (v & ~CTRL1_ODR_XL_MSK) | CTRL1_ODR_XL_120Hz; write(CTRL1, v);
 */

}  /* namespace LSM6DSV16X */

#endif /* LSM6DSV16X_REG_H */

