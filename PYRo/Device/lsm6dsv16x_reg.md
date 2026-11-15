# LSM6DSV16X 寄存器对照表

> 依据 ST 数据手册 **DS13510 Rev 4**（LSM6DSV16X）整理，与 `lsm6dsv16x_reg.h` 一一对应。
> 列含义：**寄存器** 宏名后缀 / **地址** 十六进制 / **类型** R/W/R / **默认** 上电默认值（二进制）/
> **位域** 各位对应字段 / **功能与用法** 置什么位、启用什么功能。

## 0. 四个地址空间（最先要搞清楚的事）

同一地址在不同空间含义完全不同，访问前必须先切空间：

| 空间 | 怎么进入 | 代表寄存器 |
|---|---|---|
| ① 主页面 | 默认（`EMB_FUNC_REG_ACCESS=0`、`SHUB_REG_ACCESS=0`） | `CTRL1`(10h)、`FIFO_CTRL1`(07h)、`STATUS_REG`(1Eh)、`OUTX_L_A`(28h) |
| ② 嵌入功能 | `FUNC_CFG_ACCESS(01h)` 置 `EMB_FUNC_REG_ACCESS=1`（写 0x80） | `PAGE_SEL`(02h)、`EMB_FUNC_EN_A`(04h)、`FSM_ENABLE`(46h)、`MLC1_SRC`(70h) |
| ③ 高级功能页 | 在②基础上用 `PAGE_SEL(02h)` 选页 0/1/2 | `FSM_PROGRAMS`(7Ch，页1)、`MLC_EXT_SENSITIVITY_L`(E8h，页1) |
| ④ 传感器集线器 | `FUNC_CFG_ACCESS` 置 `SHUB_REG_ACCESS=1`（写 0x40） | `SENSOR_HUB_1`(02h)、`MASTER_CONFIG`(14h) |

⚠️ ②里的 `FSM_OUTS1`(4Ch)、`MLC1_SRC`(70h) 与①同名地址无关——①中 4Ch 是保留、70h 是 `UI_CTRL1_OIS`。

```c
// 进入嵌入功能空间，选高级功能页 1，写完再退出
LSM6DSV16X_WriteReg(LSM6DSV16X_FUNC_CFG_ACCESS, 0x80); // EMB_FUNC_REG_ACCESS = 1
LSM6DSV16X_WriteReg(LSM6DSV16X_PAGE_SEL, 0x01);       // 选 page 1
LSM6DSV16X_WriteReg(LSM6DSV16X_PAGE_RW, 0x40);        // PAGE_WRITE = 1
LSM6DSV16X_WriteReg(LSM6DSV16X_PAGE_ADDRESS, addr);   // 页内地址
LSM6DSV16X_WriteReg(LSM6DSV16X_PAGE_VALUE, value);    // 数据（连续地址可重复本步）
LSM6DSV16X_WriteReg(LSM6DSV16X_PAGE_RW, 0x00);        // 结束写
LSM6DSV16X_WriteReg(LSM6DSV16X_FUNC_CFG_ACCESS, 0x00); // 回到主页面
```

## 1. 主页面寄存器

| 寄存器 | 地址 | 类型 | 默认 | 位域 | 说明 |
|---|---|---|---|---|---|
| `FUNC_CFG_ACCESS` | 01h | R/W | 00000000 | bit7 EMB_FUNC_REG_ACCESS<br>bit6 SHUB_REG_ACCESS<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 FSM_WR_CTRL_EN<br>bit2 SW_POR<br>bit1 SPI2_RESET<br>bit0 OIS_CTRL_FROM_UI | 使能嵌入功能寄存器 |
| `PIN_CTRL` | 02h | R/W | 00100011 | bit7 OIS_PU_DIS<br>bit6 SDO_PU_EN<br>bit5 IBHR_POR_EN<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 固定1(2)<br>bit0 固定1(2) | SDO, OCS_Aux, SDO_Aux pins pull-up register (R/W). This register is no |
| `IF_CFG` | 03h | R/W | 00000000 | bit7 SDA_PU_EN<br>bit6 SHUB_PU_EN<br>bit5 ASF_CTRL<br>bit4 H_LACTIVE<br>bit3 PP_OD<br>bit2 SIM<br>bit1 固定0(1)<br>bit0 I2C_I3C_DISABLE | Interface configuration register (R/W). This register is not reset dur |
| `ODR_TRIG_CFG` | 06h | R/W | 00000000 | bit[7:0] ODR_TRIG_NODR | ODR 触发模式配置寄存器 |
| `FIFO_CTRL1` | 07h | R/W | 00000000 | bit[7:0] WTM | FIFO 控制寄存器 1 |
| `FIFO_CTRL2` | 08h | R/W | 00000000 | bit7 STOP_ON_WTM<br>bit6 FIFO_COMPR_RT_EN<br>bit5 固定0(1)<br>bit4 ODR_CHG_EN<br>bit3 固定0(1)<br>bit[2:1] UNCOMPR_RATE<br>bit0 XL_DUALC_BATCH_FROM_FSM | FIFO 控制寄存器 2 |
| `FIFO_CTRL3` | 09h | R/W | 00000000 | bit[7:4] BDR_GY<br>bit[3:0] BDR_XL | FIFO 控制寄存器 3 |
| `FIFO_CTRL4` | 0Ah | R/W | 00000000 | bit[7:6] DEC_TS_BATCH<br>bit[5:4] ODR_T_BATCH<br>bit3 G_EIS_FIFO_EN<br>bit[2:0] FIFO_MODE | FIFO 控制寄存器 4 |
| `COUNTER_BDR_REG1` | 0Bh | R/W | 00000000 | bit7 固定0(1)<br>bit[6:5] TRIG_COUNTER_BDR<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit[1:0] CNT_BDR_TH | 计数器批处理数据速率寄存器 1 |
| `COUNTER_BDR_REG2` | 0Ch | R/W | 00000000 | bit[7:0] CNT_BDR_TH | 计数器批处理数据速率寄存器 2 |
| `INT1_CTRL` | 0Dh | R/W | 00000000 | bit7 固定0(1)<br>bit6 INT1_CNT_BDR<br>bit5 INT1_FIFO_FULL<br>bit4 INT1_FIFO_OVR<br>bit3 INT1_FIFO_TH<br>bit2 固定0(1)<br>bit1 INT1_DRDY_G<br>bit0 INT1_DRDY_XL | INT1 引脚控制寄存器 |
| `INT2_CTRL` | 0Eh | R/W | 00000000 | bit7 INT2_EMB_FUNC_ENDOP<br>bit6 INT2_CNT_BDR<br>bit5 INT2_FIFO_FULL<br>bit4 INT2_FIFO_OVR<br>bit3 INT2_FIFO_TH<br>bit2 INT2_DRDY_G_EIS<br>bit1 INT2_DRDY_G<br>bit0 INT2_DRDY_XL | INT2 引脚控制寄存器 |
| `WHO_AM_I` | 0Fh | R | 01110000 | bit7 固定0<br>bit6 固定1<br>bit5 固定1<br>bit4 固定1<br>bit3 固定0<br>bit2 固定0<br>bit1 固定0<br>bit0 固定0 | WHO_AM_I 寄存器（R）。该寄存器为只读寄存器。其值固定为 70h。 |
| `CTRL1` | 10h | R/W | 00000000 | bit7 固定0(1)<br>bit[6:4] OP_MODE_XL<br>bit[3:0] ODR_XL | 加速度计控制寄存器 1 |
| `CTRL2` | 11h | R/W | 00000000 | bit7 固定0(1)<br>bit[6:4] OP_MODE_G<br>bit[3:0] ODR_G | 陀螺仪控制寄存器 2 |
| `CTRL3` | 12h | R/W | 01000100 | bit7 BOOT<br>bit6 BDU<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 IF_INC<br>bit1 固定0(1)<br>bit0 SW_RESET | 控制寄存器 3 |
| `CTRL4` | 13h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 INT2_ON_INT<br>bit3 DRDY_MASK<br>bit2 INT2_DRDY_TEMP<br>bit1 DRDY_PULSED<br>bit0 INT2_IN_LH | 控制寄存器 4 |
| `CTRL5` | 14h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit[2:1] BUS_ACT_SEL<br>bit0 INT_EN_I3C | 控制寄存器 5 |
| `CTRL6` | 15h | R/W | 00000000 | bit7 固定0(1)<br>bit[6:4] LPF1_G_BW<br>bit[3:0] FS_G | 控制寄存器 6 |
| `CTRL7` | 16h | R/W | 00000000 | bit7 AH_QVAR_EN<br>bit6 INT2_DRDY_AH_QVAR<br>bit[5:4] AH_QVAR_C_ZIN<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 LPF1_G_EN | 控制寄存器 7 |
| `CTRL8` | 17h | R/W | 00000000 | bit[7:5] HP_LPF2_XL_BW<br>bit4 固定0(1)<br>bit3 XL_DUALC_EN<br>bit2 固定0(1)<br>bit[1:0] FS_XL | 控制寄存器 8 |
| `CTRL9` | 18h | R/W | 00000000 | bit7 固定0(1)<br>bit6 HP_REF_MODE_XL<br>bit5 XL_FASTSETTL_MODE<br>bit4 HP_SLOPE_XL_EN<br>bit3 LPF2_XL_EN<br>bit2 固定0(1)<br>bit1 USR_OFF_W<br>bit0 USR_OFF_ON_OUT | 控制寄存器 9 |
| `CTRL10` | 19h | R/W | 00000000 | bit7 固定0(1)<br>bit6 EMB_FUNC_DEBUG<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit[3:2] ST_G<br>bit[1:0] ST_XL | 控制寄存器 10 |
| `CTRL_STATUS` | 1Ah | R | output | bit7 固定0<br>bit6 固定0<br>bit5 固定0<br>bit4 固定0<br>bit3 固定0<br>bit2 FSM_WR_CTRL_STATUS<br>bit1 固定0<br>bit0 固定0 |  |
| `FIFO_STATUS1` | 1Bh | R | output | bit[7:0] DIFF_FIFO | FIFO 状态寄存器 1 |
| `FIFO_STATUS2` | 1Ch | R | output | bit7 FIFO_WTM_IA<br>bit6 FIFO_OVR_IA<br>bit5 FIFO_FULL_IA<br>bit4 COUNTER_BDR_IA<br>bit3 FIFO_OVR_LATCHED<br>bit2 固定0<br>bit1 固定0<br>bit0 DIFF_FIFO | FIFO 状态寄存器 2 |
| `ALL_INT_SRC` | 1Dh | R | output | bit7 EMB_FUNC_IA<br>bit6 SHUB_IA<br>bit5 SLEEP_CHANGE_IA<br>bit4 D6D_IA<br>bit3 固定0<br>bit2 TAP_IA<br>bit1 WU_IA<br>bit0 FF_IA | 所有中断的源寄存器 |
| `STATUS_REG` | 1Eh | R | output | bit7 TIMESTAMP_ENDCOUNT<br>bit6 固定0<br>bit5 OIS_DRDY<br>bit4 GDA_EIS<br>bit3 AH_QVARDA<br>bit2 TDA<br>bit1 GDA<br>bit0 XLDA | STATUS_REG 寄存器由主接口 SPI/I²C 和 MIPI I3C® 读取 (R)。 |
| `OUT_TEMP_L` | 20h | R | output | bit[7:0] TEMP | 温度数据输出寄存器 (R)。L 和 H 寄存器共同以二进制补码表示一个 16 位字。 |
| `OUT_TEMP_H` | 21h | R | output | bit[7:2] TEMP1<br>bit[1:0] TEMP |  |
| `OUTX_L_G` | 22h | R | output | bit[7:0] D | Angular rate sensor pitch axis (X) angular rate output register (R). T |
| `OUTX_H_G` | 23h | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `OUTY_L_G` | 24h | R | output | bit[7:0] D | Angular rate sensor roll axis (Y) angular rate output register (R). Th |
| `OUTY_H_G` | 25h | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `OUTZ_L_G` | 26h | R | output | bit[7:0] D | Angular rate sensor yaw axis (Z) angular rate output register (R). The |
| `OUTZ_H_G` | 27h | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `OUTX_L_A` | 28h | R | output | bit[7:0] D | Linear acceleration sensor X-axis output register (R). The value is ex |
| `OUTX_H_A` | 29h | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `OUTY_L_A` | 2Ah | R | output | bit[7:0] D | Linear acceleration sensor Y-axis output register (R). The value is ex |
| `OUTY_H_A` | 2Bh | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `OUTZ_L_A` | 2Ch | R | output | bit[7:0] D | Linear acceleration sensor Z-axis output register (R). The value is ex |
| `OUTZ_H_A` | 2Dh | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `UI_OUTX_L_G_OIS_EIS` | 2Eh | R | output | bit[7:0] D | Angular rate sensor pitch axis (X) angular rate output register (R). T |
| `UI_OUTX_H_G_OIS_EIS` | 2Fh | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `UI_OUTY_L_G_OIS_EIS` | 30h | R | output | bit[7:0] D | Angular rate sensor roll axis (Y) angular rate output register (R). Th |
| `UI_OUTY_H_G_OIS_EIS` | 31h | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `UI_OUTZ_L_G_OIS_EIS` | 32h | R | output | bit[7:0] D | Angular rate sensor yaw axis (Z) angular rate output register (R). The |
| `UI_OUTZ_H_G_OIS_EIS` | 33h | R | output | bit[7:2] D1<br>bit[1:0] D |  |
| `AH_QVAR_OUT_L` | 3Ah | R | output | bit[7:0] AH_QVAR | Analog hub and Qvar data output register (R). L and H registers togeth |
| `AH_QVAR_OUT_H` | 3Bh | R | output | bit[7:2] AH_QVAR_1<br>bit[1:0] AH_QVAR |  |
| `UI_STATUS_REG_OIS` | 44h | R | output | bit7 固定0<br>bit6 固定0<br>bit5 固定0<br>bit4 固定0<br>bit3 固定0<br>bit2 GYRO_SETTLING<br>bit1 GDA_OIS<br>bit0 XLDA_OIS |  |
| `WAKE_UP_SRC` | 45h | R | output | bit7 固定0<br>bit6 SLEEP_CHANGE_IA<br>bit5 FF_IA<br>bit4 SLEEP_STATE<br>bit3 WU_IA<br>bit2 X_WU<br>bit1 Y_WU<br>bit0 Z_WU | 唤醒中断源寄存器 |
| `TAP_SRC` | 46h | R | output | bit7 固定0<br>bit6 TAP_IA<br>bit5 SINGLE_TAP<br>bit4 DOUBLE_TAP<br>bit3 TAP_SIGN<br>bit2 X_TAP<br>bit1 Y_TAP<br>bit0 Z_TAP | 敲击源寄存器 |
| `D6D_SRC` | 47h | R | output | bit7 固定0<br>bit6 D6D_IA<br>bit5 ZH<br>bit4 ZL<br>bit3 YH<br>bit2 YL<br>bit1 XH<br>bit0 XL | 纵向、横向、面朝上和面朝下源寄存器 |
| `STATUS_MASTER_MAINPAGE` | 48h | R | output | bit7 WR_ONCE_DONE<br>bit6 SLAVE3_NACK<br>bit5 SLAVE2_NACK<br>bit4 SLAVE1_NACK<br>bit3 SLAVE0_NACK<br>bit2 固定0<br>bit1 固定0<br>bit0 SENS_HUB_ENDOP | 传感器集线器源寄存器 |
| `EMB_FUNC_STATUS_MAINPAGE` | 49h | R | output | bit7 IS_FSM_LC<br>bit6 固定0<br>bit5 IS_SIGMOT<br>bit4 IS_TILT<br>bit3 IS_STEP_DET<br>bit2 固定0<br>bit1 固定0<br>bit0 固定0 | 嵌入功能状态寄存器 |
| `FSM_STATUS_MAINPAGE` | 4Ah | R | output | bit[7:0] IS_FSM | 有限状态机状态寄存器 |
| `MLC_STATUS_MAINPAGE` | 4Bh | R | output | bit7 固定0<br>bit6 固定0<br>bit5 固定0<br>bit4 固定0<br>bit[3:0] IS_MLC | 机器学习内核状态寄存器 |
| `INTERNAL_FREQ_FINE` | 4Fh | R | output | bit[7:0] FREQ_FINE | 内部频率寄存器 |
| `FUNCTIONS_ENABLE` | 50h | RW | 00000000 | bit7 INTERRUPTS_ENABLE<br>bit6 TIMESTAMP_EN<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 DIS_RST_LIR_ALL_INT<br>bit2 固定0(1)<br>bit[1:0] INACT_EN | 使能中断功能寄存器 |
| `DEN` | 51h | R/W | 00001110 | bit7 固定0(1)<br>bit6 LVL1_EN<br>bit5 LVL2_EN<br>bit4 DEN_XL_EN<br>bit3 DEN_X<br>bit2 DEN_Y<br>bit1 DEN_Z<br>bit0 DEN_XL_G | DEN 配置寄存器 |
| `INACTIVITY_DUR` | 54h | R/W | 00000100 | bit7 SLEEP_STATUS_ON_INT<br>bit[6:4] WU_INACT_THS_W<br>bit[3:2] XL_INACT_ODR<br>bit[1:0] INACT_DUR | 活动/静止配置寄存器 |
| `INACTIVITY_THS` | 55h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit[5:0] INACT_THS | 活动/静止阈值设置寄存器 |
| `TAP_CFG0` | 56h | R/W | 00000000 | bit7 固定0(1)<br>bit6 LOW_PASS_ON_6D<br>bit5 HW_FUNC_MASK_XL_SETTL<br>bit4 SLOPE_FDS<br>bit3 TAP_X_EN<br>bit2 TAP_Y_EN<br>bit1 TAP_Z_EN<br>bit0 LIR | 单击配置寄存器 0 |
| `TAP_CFG1` | 57h | R/W | 00000000 | bit[7:5] TAP_PRIORITY<br>bit[4:0] TAP_THS_X | 单击配置寄存器 1 |
| `TAP_CFG2` | 58h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit[4:0] TAP_THS_Y | 单击配置寄存器 2 |
| `TAP_THS_6D` | 59h | R/W | 00000000 | bit7 D4D_EN<br>bit[6:5] SIXD_THS<br>bit[4:0] TAP_THS_Z | 纵向/横向位置和单击功能阈值寄存器 |
| `TAP_DUR` | 5Ah | R/W | 00000000 | bit[7:4] DUR<br>bit[3:2] QUIET<br>bit[1:0] SHOCK | 单击识别功能设置寄存器 |
| `WAKE_UP_THS` | 5Bh | R/W | 00000000 | bit7 SINGLE_DOUBLE_TAP<br>bit6 USR_OFF_ON_WU<br>bit[5:0] WK_THS |  |
| `WAKE_UP_DUR` | 5Ch | R/W | 00000000 | bit7 FF_DUR<br>bit[6:5] WAKE_DUR<br>bit4 固定0(1)<br>bit[3:0] SLEEP_DUR | 自由落体、唤醒和睡眠模式功能持续时间设置寄存器 |
| `FREE_FALL` | 5Dh | R/W | 00000000 | bit[7:3] FF_DUR<br>bit[2:0] FF_THS | 自由落体功能持续时间设置寄存器 |
| `MD1_CFG` | 5Eh | R/W | 00000000 | bit7 INT1_SLEEP_CHANGE<br>bit6 INT1_SINGLE_TAP<br>bit5 INT1_WU<br>bit4 INT1_FF<br>bit3 INT1_DOUBLE_TAP<br>bit2 INT1_6D<br>bit1 INT1_EMB_FUNC<br>bit0 INT1_SHUB | Functions routing to INT1 pin register (R/W). Each bit in this registe |
| `MD2_CFG` | 5Fh | R/W | 00000000 | bit7 INT2_SLEEP_CHANGE<br>bit6 INT2_SINGLE_TAP<br>bit5 INT2_WU<br>bit4 INT2_FF<br>bit3 INT2_DOUBLE_TAP<br>bit2 INT2_6D<br>bit1 INT2_EMB_FUNC<br>bit0 INT2_TIMESTAMP | Functions routing to INT2 pin register (R/W). Each bit in this registe |
| `HAODR_CFG` | 62h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit[1:0] HAODR_SEL | HAODR 数据速率配置寄存器 |
| `EMB_FUNC_CFG` | 63h | R/W | 00000000 | bit7 XL_DUALC_BATCH_FROM_IF<br>bit6 固定0(1)<br>bit5 EMB_FUNC_IRQ_MASK_G_SETTL<br>bit4 EMB_FUNC_IRQ_MASK_XL_SETTL<br>bit3 EMB_FUNC_DISABLE<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 固定0(1) | 嵌入功能配置寄存器 |
| `UI_HANDSHAKE_CTRL` | 64h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 UI_SHARED_ACK<br>bit0 UI_SHARED_REQ | UI / SPI2 共享寄存器的控制寄存器（UI 侧） |
| `UI_SPI2_SHARED_0` | 65h | R/W | 00000000 | bit[7:0] D | UI / SPI2 共享寄存器 0 |
| `UI_SPI2_SHARED_1` | 66h | R/W | 00000000 | bit[7:0] D | UI / SPI2 共享寄存器 1 |
| `UI_SPI2_SHARED_2` | 67h | R/W | 00000000 | bit[7:0] D | UI / SPI2 共享寄存器 2 |
| `UI_SPI2_SHARED_3` | 68h | R/W | 00000000 | bit[7:0] D | UI / SPI2 共享寄存器 3 |
| `UI_SPI2_SHARED_4` | 69h | R/W | 00000000 | bit[7:0] D | UI / SPI2 共享寄存器 4 |
| `UI_SPI2_SHARED_5` | 6Ah | R/W | 00000000 | bit[7:0] D | UI / SPI2 共享寄存器 5 |
| `CTRL_EIS` | 6Bh | R/W | 00000000 | bit[7:6] ODR_G_EIS<br>bit5 固定0(1)<br>bit4 LPF_G_EIS_BW<br>bit3 G_EIS_ON_G_OIS_OUT_REG<br>bit[2:0] FS_G_EIS | 陀螺仪 EIS 通道控制寄存器 |
| `UI_INT_OIS` | 6Fh | R(SPI2full-controlmode)R/W(primaryIFfull-controlmode) | 00000000 | bit7 INT2_DRDY_OIS<br>bit6 DRDY_MASK_OIS<br>bit5 固定0(1)<br>bit4 ST_OIS_CLAMPDIS<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 固定0(1) | OIS 中断配置寄存器 |
| `UI_CTRL1_OIS` | 70h | R(SPI2full-controlmode)R/W(primaryIFfull-controlmode) | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 SIM_OIS<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 OIS_XL_EN<br>bit1 OIS_G_EN<br>bit0 SPI2_READ_EN | OIS 配置寄存器 |
| `UI_CTRL2_OIS` | 71h | R(SPI2full-controlmode)R/W(primaryIFfull-controlmode) | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit[4:3] LPF1_G_OIS_BW<br>bit[2:0] FS_G_OIS | OIS 配置寄存器 |
| `UI_CTRL3_OIS` | 72h | R(SPI2full-controlmode)R/W(primaryIFfull-controlmode) | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit[5:3] LPF_XL_OIS_BW<br>bit2 固定0(1)<br>bit[1:0] FS_XL_OIS | OIS 配置寄存器 |
| `X_OFS_USR` | 73h | R/W | 00000000 | bit[7:0] X_OFS_USR | Accelerometer X-axis user offset correction (R/W). The offset value se |
| `Y_OFS_USR` | 74h | R/W | 00000000 | bit[7:0] Y_OFS_USR | Accelerometer Y-axis user offset correction (R/W). The offset value se |
| `Z_OFS_USR` | 75h | R/W | 00000000 | bit[7:0] Z_OFS_USR | Accelerometer Z-axis user offset correction (R/W). The offset value se |
| `FIFO_DATA_OUT_TAG` | 78h | R | output | bit[7:3] TAG_SENSOR<br>bit[2:1] TAG_CNT<br>bit0  | FIFO 标签寄存器 |

### 主页面寄存器 · 位域明细与用法

| 寄存器 | 地址 | 位 | 字段 | 功能与用法 |
|---|---|---|---|---|
| `FUNC_CFG_ACCESS` | 01h | 7 | `EMB_FUNC_REG_ACCESS` | 使能对嵌入功能配置寄存器的访问。（1）默认值：0 |
| `FUNC_CFG_ACCESS` | 01h | 6 | `SHUB_REG_ACCESS` | 使能对传感器集线器（I²C 主器件）配置寄存器的访问。（2）默认值：0 |
| `FUNC_CFG_ACCESS` | 01h | 3 | `FSM_WR_CTRL_EN` | 使能 FSM 对 CTRL 寄存器的控制（FSM 可以自主更改器件的某些配置）。默认值：0（0：禁用；1：使能） |
| `FUNC_CFG_ACCESS` | 01h | 2 | `SW_POR` | 器件全局复位。默认值：0 |
| `FUNC_CFG_ACCESS` | 01h | 1 | `SPI2_RESET` | 从主接口复位 SPI2 的控制寄存器。该位必须置为 1，然后再置为 0（该位不会自动清零）。默认值：0 |
| `FUNC_CFG_ACCESS` | 01h | 0 | `OIS_CTRL_FROM_UI` | 使能从主接口对 OIS 配置的完全控制。默认值：0（0：从主接口对 OIS 链的完全控制禁用；1：从主接口对 OIS 链的完全控制使能） |
| `PIN_CTRL` | 02h | 7 | `OIS_PU_DIS` | 禁用 OCS_Aux 和 SDO_Aux 引脚上的上拉（适用于模式 1 和模式 2）。有关模式 3 中上拉电阻配置的更多详细信息，请参见表 23。默认值：0（0：OCS_Aux 和 SDO_Aux 引脚带上拉；1：OCS_Aux 和 SDO_Aux 引脚上拉断开） |
| `PIN_CTRL` | 02h | 6 | `SDO_PU_EN` | 使能 SDO 引脚上的上拉。详细信息请参见表 23。默认值：0（0：SDO 引脚上拉断开；1：SDO 引脚带上拉） |
| `PIN_CTRL` | 02h | 5 | `IBHR_POR_EN` | 选择器件在 "reset whole chip" I3C 模式之后执行的操作。默认值：1（0：配置复位（SW 复位 + 动态地址复位）；（1：全局复位（POR 复位）） |
| `IF_CFG` | 03h | 7 | `SDA_PU_EN` | 使能 SDA 引脚上的上拉。默认值：0（0：SDA 引脚上拉断开；1：SDA 引脚带上拉） |
| `IF_CFG` | 03h | 6 | `SHUB_PU_EN` | 使能主器件 I²C 上拉。默认值：0（0：辅助 I²C 线上的内部上拉禁用；1：辅助 I²C 线上的内部上拉使能） |
| `IF_CFG` | 03h | 5 | `ASF_CTRL` | 使能抗尖峰滤波器。默认值：0（0：抗尖峰滤波器由协议管理，并在广播地址之后关闭；1：SCL 和 SDA 线上的抗尖峰滤波器始终使能） |
| `IF_CFG` | 03h | 4 | `H_LACTIVE` | 中断激活电平。默认值：0（0：中断输出引脚高电平有效；1：中断输出引脚低电平有效） |
| `IF_CFG` | 03h | 3 | `PP_OD` | INT1 和 INT2 引脚上的推挽/开漏选择。默认值：0（0：推挽模式；1：开漏模式） |
| `IF_CFG` | 03h | 2 | `SIM` | SPI 串行接口模式选择。默认值：0（0：4 线接口；1：3 线接口） |
| `IF_CFG` | 03h | 0 | `I2C_I3C_DISABLE` |  |
| `ODR_TRIG_CFG` | 06h | [7:0] | `ODR_TRIG_NODR` |  |
| `FIFO_CTRL1` | 07h | [7:0] | `WTM` |  |
| `FIFO_CTRL2` | 08h | 7 | `STOP_ON_WTM` | 在阈值电平时停止传感链 FIFO 的值存储。默认值：0（0：FIFO 深度不受限制；1：FIFO 深度限制为阈值电平，在 FIFO_CTRL1 (07h) 中定义） |
| `FIFO_CTRL2` | 08h | 6 | `FIFO_COMPR_RT_EN` | 使能/禁用压缩算法运行。默认值：0（0：FIFO 压缩算法禁用；1：FIFO 压缩算法使能） |
| `FIFO_CTRL2` | 08h | 4 | `ODR_CHG_EN` | 使能 ODR CHANGE 虚拟传感器在 FIFO 中批处理。默认值：0（0：ODR CHANGE 虚拟传感器未在 FIFO 中批处理；1：ODR CHANGE 虚拟传感器在 FIFO 中批处理）注意：有关工作/功耗模式配置、设置、开启/关闭时间和动态更改的详细信息，请参见产品应用笔记。 |
| `FIFO_CTRL2` | 08h | [2:1] | `UNCOMPR_RATE` |  |
| `FIFO_CTRL2` | 08h | 0 | `XL_DUALC_BATCH_FROM_FSM` |  |
| `FIFO_CTRL3` | 09h | [7:4] | `BDR_GY` |  |
| `FIFO_CTRL3` | 09h | [3:0] | `BDR_XL` |  |
| `FIFO_CTRL4` | 0Ah | [7:6] | `DEC_TS_BATCH` |  |
| `FIFO_CTRL4` | 0Ah | [5:4] | `ODR_T_BATCH` |  |
| `FIFO_CTRL4` | 0Ah | 3 | `G_EIS_FIFO_EN` | 使能增强型 EIS 陀螺仪输出值的 FIFO 批处理。默认值：0（0：禁用；1：使能） |
| `FIFO_CTRL4` | 0Ah | [2:0] | `FIFO_MODE` |  |
| `COUNTER_BDR_REG1` | 0Bh | [6:5] | `TRIG_COUNTER_BDR` |  |
| `COUNTER_BDR_REG1` | 0Bh | [1:0] | `CNT_BDR_TH` |  |
| `COUNTER_BDR_REG2` | 0Ch | [7:0] | `CNT_BDR_TH` |  |
| `INT1_CTRL` | 0Dh | 6 | `INT1_CNT_BDR` | 使能 INT1 引脚上的 COUNTER_BDR_IA 中断。默认值：0 |
| `INT1_CTRL` | 0Dh | 5 | `INT1_FIFO_FULL` | 使能 INT1 引脚上的 FIFO 满标志中断。使用 MIPI I3C® 时，它还可用于触发 IBI。默认值：0 |
| `INT1_CTRL` | 0Dh | 4 | `INT1_FIFO_OVR` | 使能 INT1 引脚上的 FIFO 溢出中断。当使用 MIPI I3C® 接口时，它还可用于触发 IBI。默认值：0 |
| `INT1_CTRL` | 0Dh | 3 | `INT1_FIFO_TH` | 使能 INT1 引脚上的 FIFO 阈值中断。当使用 MIPI I3C® 接口时，它还可用于触发 IBI。默认值：0 |
| `INT1_CTRL` | 0Dh | 1 | `INT1_DRDY_G` | 使能 INT1 引脚上的陀螺仪数据准备就绪中断。当使用 MIPI I3C® 接口时，它还可用于触发 IBI。默认值：0 |
| `INT1_CTRL` | 0Dh | 0 | `INT1_DRDY_XL` | 使能 INT1 引脚上的加速度计数据准备就绪中断。当使用 MIPI I3C® 接口时，它还可用于触发 IBI。默认值：0 |
| `INT2_CTRL` | 0Eh | 7 | `INT2_EMB_FUNC_ENDOP` | 使能将嵌入功能操作结束信号路由到 INT2 引脚。默认值：0 |
| `INT2_CTRL` | 0Eh | 6 | `INT2_CNT_BDR` | 使能 INT2 上的 COUNTER_BDR_IA 中断。默认值：0 |
| `INT2_CTRL` | 0Eh | 5 | `INT2_FIFO_FULL` | 使能 INT2 引脚上的 FIFO 满标志中断。默认值：0 |
| `INT2_CTRL` | 0Eh | 4 | `INT2_FIFO_OVR` | 使能 INT2 引脚上的 FIFO 溢出中断。默认值：0 |
| `INT2_CTRL` | 0Eh | 3 | `INT2_FIFO_TH` | 使能 INT2 引脚上的 FIFO 阈值中断。默认值：0 |
| `INT2_CTRL` | 0Eh | 2 | `INT2_DRDY_G_EIS` | 使能 INT2 引脚上的陀螺仪 EIS 数据准备就绪中断。默认值：0 |
| `INT2_CTRL` | 0Eh | 1 | `INT2_DRDY_G` | INT2 引脚上的陀螺仪数据准备就绪中断。默认值：0 |
| `INT2_CTRL` | 0Eh | 0 | `INT2_DRDY_XL` | INT2 引脚上的加速度计数据准备就绪中断。默认值：0 |
| `CTRL1` | 10h | [6:4] | `OP_MODE_XL` |  |
| `CTRL1` | 10h | [3:0] | `ODR_XL` |  |
| `CTRL2` | 11h | [6:4] | `OP_MODE_G` |  |
| `CTRL2` | 11h | [3:0] | `ODR_G` |  |
| `CTRL3` | 12h | 7 | `BOOT` | 重启存储器内容。该位自动清零。默认值：0（0：正常模式；1：重启存储器内容） |
| `CTRL3` | 12h | 6 | `BDU` | 块数据更新。默认值：1（0：连续更新；1：在读取 LSB 和 MSB 之前，输出寄存器不会更新） |
| `CTRL3` | 12h | 2 | `IF_INC` | 在使用串行接口（I²C、MIPI I3C 或 SPI）进行多字节访问期间，寄存器地址自动递增。默认值：1（0：禁用；1：使能） |
| `CTRL3` | 12h | 0 | `SW_RESET` | 软件复位，将所有控制寄存器复位为其默认值。该位自动清零。默认值：0（0：正常模式；1：复位器件） |
| `CTRL4` | 13h | 4 | `INT2_ON_INT` |  |
| `CTRL4` | 13h | 3 | `DRDY_MASK` | 使能/屏蔽数据准备就绪信号。默认值：0（0：禁用；1：屏蔽 DRDY 信号（加速度计和陀螺仪），直到滤波器稳定结束（加速度计和陀螺仪独立屏蔽））注意：有关工作模式/功耗模式配置、设置、开启/关闭时间以及动态更改的详细信息，请参阅产品应用笔记。 |
| `CTRL4` | 13h | 2 | `INT2_DRDY_TEMP` | 使能 INT2 引脚上的温度传感器数据准备就绪中断。当使用 MIPI I3C® 接口且 CTRL4_C (13h) 中的 INT2_ON_INT1 = 1 时，它还可用于触发 IBI。默认值：0（0：禁用；1：使能） |
| `CTRL4` | 13h | 1 | `DRDY_PULSED` | 使能脉冲式数据准备就绪模式。默认值：0（0：数据准备就绪锁存模式（仅在读取关联输出寄存器的高字节后返回 0）；1：数据准备就绪脉冲式模式（数据准备就绪脉冲持续时间为 65 μs）） |
| `CTRL4` | 13h | 0 | `INT2_IN_LH` | 置为 1 可更改 DEN 或嵌入功能的 INT2 引脚输入触发的极性。默认值：0（0：DEN 和嵌入功能引脚的触发为低电平有效；1：DEN 和嵌入功能引脚的触发为高电平有效） |
| `CTRL5` | 14h | [2:1] | `BUS_ACT_SEL` |  |
| `CTRL5` | 14h | 0 | `INT_EN_I3C` | 当使能 I3C 时使能 INT 引脚。默认值：0（0：禁用；1：使能） |
| `CTRL6` | 15h | [6:4] | `LPF1_G_BW` |  |
| `CTRL6` | 15h | [3:0] | `FS_G` |  |
| `CTRL7` | 16h | 7 | `AH_QVAR_EN` | 使能模拟集线器和 Qvar 链。当该位置为 1 时，模拟集线器和 Qvar 缓冲器连接到 SDx/AH1/Qvar1 和 SCx/AH2/Qvar2 引脚。在将该位置为 1 之前，必须将加速度计和陀螺仪传感器配置为掉电模式。默认值：0（0：禁用；1：使能） |
| `CTRL7` | 16h | 6 | `INT2_DRDY_AH_QVAR` | INT2 引脚上的模拟集线器和 Qvar 数据准备就绪中断。默认值：0（0：禁用；1：使能） |
| `CTRL7` | 16h | [5:4] | `AH_QVAR_C_ZIN` |  |
| `CTRL7` | 16h | 0 | `LPF1_G_EN` | 使能陀螺仪数字 LPF1 滤波器。如果 OIS 链被禁用，可通过 CTRL6 (15h) 中的 LPF1_G_BW_[2:0] 选择带宽 |
| `CTRL8` | 17h | [7:5] | `HP_LPF2_XL_BW` |  |
| `CTRL8` | 17h | 3 | `XL_DUALC_EN` |  |
| `CTRL8` | 17h | [1:0] | `FS_XL` |  |
| `CTRL9` | 18h | 6 | `HP_REF_MODE_XL` | 使能加速度计高通滤波器参考模式（对高通路径有效 -HP_SLOPE_XL_EN 位必须为 1）。默认值：0（0：禁用，1：使能）(1) |
| `CTRL9` | 18h | 5 | `XL_FASTSETTL_MODE` | 使能加速度计 LPF2 与 HPF 快速稳定模式。写入该位后，滤波器设置第一个采样。仅在器件退出掉电模式期间有效。默认值：0（0：禁用，1：使能） |
| `CTRL9` | 18h | 4 | `HP_SLOPE_XL_EN` | 加速度计斜率滤波器/高通滤波器选择。参见图 32。默认值：0（0：选择低通滤波器路径；1：选择高通滤波器路径） |
| `CTRL9` | 18h | 3 | `LPF2_XL_EN` | 加速度计高分辨率选择。参见图 32。默认值：0（0：选择第一级数字滤波输出；1：选择 LPF2 第二级滤波输出） |
| `CTRL9` | 18h | 1 | `USR_OFF_W` | 寄存器 X_OFS_USR (73h)、Y_OFS_USR (74h)、Z_OFS_USR(75h) 的 XL 用户偏移位权重。默认值：0（0：2-10 g/LSB；1: 2-6 g/LSB) |
| `CTRL9` | 18h | 0 | `USR_OFF_ON_OUT` | 使能加速度计用户偏移校正模块；该模块对低通路径有效。参见图 32。默认值：0（0：加速度计用户偏移校正模块旁路；1：使能加速度计用户偏移校正模块） |
| `CTRL10` | 19h | 6 | `EMB_FUNC_DEBUG` | 使能嵌入功能的调试模式。（0：禁用；1：使能） |
| `CTRL10` | 19h | [3:2] | `ST_G` |  |
| `CTRL10` | 19h | [1:0] | `ST_XL` |  |
| `CTRL_STATUS` | 1Ah | 2 | `FSM_WR_CTRL_STATUS` | 该标志指示器件配置寄存器的当前控制者。当 FUNC_CFG_ACCESS (01h) 寄存器中的 FSM_WR_CTRL_EN 位的值改变时，该标志必须用作应答标志。默认值：0（0：所有寄存器和配置均可从标准接口写入；1：某些寄存器和配置处于 FSM 控制之下，从标准接口看为只读模式）。 |
| `FIFO_STATUS1` | 1Bh | [7:0] | `DIFF_FIFO` |  |
| `FIFO_STATUS2` | 1Ch | 7 | `FIFO_WTM_IA` | FIFO 水印状态。默认值：0（0：FIFO 填充量低于 WTM；1：FIFO 填充量等于或大于 WTM）水印通过 FIFO_CTRL2 (08h) 和 FIFO_CTRL1 (07h) 中的 WTM[7:0] 位设置。 |
| `FIFO_STATUS2` | 1Ch | 6 | `FIFO_OVR_IA` | FIFO 溢出状态。默认值：0（0：FIFO 未完全填满；1：FIFO 完全填满） |
| `FIFO_STATUS2` | 1Ch | 5 | `FIFO_FULL_IA` | 智能 FIFO 满状态。默认值：0（0：FIFO 未满；1：FIFO 将在下一个 ODR 时填满） |
| `FIFO_STATUS2` | 1Ch | 4 | `COUNTER_BDR_IA` | 计数器 BDR 达到 COUNTER_BDR_REG1 (0Bh) 和 COUNTER_BDR_REG2 (0Ch) 中设置的 CNT_BDR_TH_[10:0] 阈值。默认值：0读取这些寄存器时，该位复位。 |
| `FIFO_STATUS2` | 1Ch | 3 | `FIFO_OVR_LATCHED` | 锁存的 FIFO 溢出状态。默认值：0读取该寄存器时，该位复位。 |
| `FIFO_STATUS2` | 1Ch | 0 | `DIFF_FIFO` |  |
| `ALL_INT_SRC` | 1Dh | 7 | `EMB_FUNC_IA` | 嵌入功能中断状态。默认值：0（0：未检测到嵌入功能事件；1：检测到嵌入功能事件） |
| `ALL_INT_SRC` | 1Dh | 6 | `SHUB_IA` | 传感器集线器（I²C 主器件）中断状态。默认值：0（0：未产生传感器集线器中断；1：产生传感器集线器中断） |
| `ALL_INT_SRC` | 1Dh | 5 | `SLEEP_CHANGE_IA` | 检测活动/静止状态中的变化事件。默认值：0（0：未检测到状态变化；1：检测到状态变化） |
| `ALL_INT_SRC` | 1Dh | 4 | `D6D_IA` | 当竖向、横向、面朝上、面朝下位置发生改变时中断有效。默认值：0（0：未检测到位置变化；1：检测到位置变化） |
| `ALL_INT_SRC` | 1Dh | 2 | `TAP_IA` | 单击或双击事件检测状态，取决于 SINGLE_DOUBLE_TAP_bit 的值（参见WAKE_UP_THS (5Bh) 寄存器）。默认值：0（0：未检测到敲击事件；1：检测到敲击事件） |
| `ALL_INT_SRC` | 1Dh | 1 | `WU_IA` | 唤醒事件状态。默认值：0（0：未检测到事件，1：检测到事件） |
| `ALL_INT_SRC` | 1Dh | 0 | `FF_IA` | 自由落体事件状态。默认值：0（0：未检测到事件，1：检测到事件） |
| `STATUS_REG` | 1Eh | 7 | `TIMESTAMP_ENDCOUNT` | 在 5.6 ms 内提示时间戳溢出 |
| `STATUS_REG` | 1Eh | 5 | `OIS_DRDY` | 加速度计 OIS 或陀螺仪 OIS 有新的输出数据可用。默认值：0（0：OIS 链上无可用数据组（加速度计或陀螺仪）；1：OIS 链上有新的数据组（加速度计或陀螺仪）可用） |
| `STATUS_REG` | 1Eh | 4 | `GDA_EIS` | 增强型 EIS 陀螺仪有新数据可用。默认值：0（0：陀螺仪输出端无可用数据组；1：陀螺仪输出端有新的数据组可用） |
| `STATUS_REG` | 1Eh | 3 | `AH_QVARDA` | 模拟集线器或 Qvar 有新数据可用。默认值：0（0：模拟集线器或 Qvar 数据输出端无可用数据组；1：模拟集线器或 Qvar 数据输出端有新的数据组可用） |
| `STATUS_REG` | 1Eh | 2 | `TDA` | 温度有新数据可用。默认值：0（0：温度传感器输出端无可用数据组；1：温度传感器输出端有新的数据组可用） |
| `STATUS_REG` | 1Eh | 1 | `GDA` | 陀螺仪有新数据可用。默认值：0（0：陀螺仪输出端无可用数据组；1：陀螺仪输出端有新的数据组可用） |
| `STATUS_REG` | 1Eh | 0 | `XLDA` | 加速度计有新数据可用。默认值：0（0：加速度计输出端无可用数据组；1：加速度计输出端有新的数据组可用） |
| `OUT_TEMP_L` | 20h | [7:0] | `TEMP` |  |
| `OUT_TEMP_H` | 21h | [7:2] | `TEMP1` |  |
| `OUT_TEMP_H` | 21h | [1:0] | `TEMP` |  |
| `AH_QVAR_OUT_L` | 3Ah | [7:0] | `AH_QVAR` |  |
| `AH_QVAR_OUT_H` | 3Bh | [7:2] | `AH_QVAR_1` |  |
| `AH_QVAR_OUT_H` | 3Bh | [1:0] | `AH_QVAR` |  |
| `UI_STATUS_REG_OIS` | 44h | 2 | `GYRO_SETTLING` | 当陀螺仪输出处于建立阶段时为高电平 |
| `UI_STATUS_REG_OIS` | 44h | 1 | `GDA_OIS` | 陀螺仪 OIS 数据可用（读取输出数据的高位部分之一时复位）。默认值：0（0：陀螺仪 OIS 输出无可用数据集；1：陀螺仪输出有新的数据集可用） |
| `UI_STATUS_REG_OIS` | 44h | 0 | `XLDA_OIS` | 加速度计 OIS 数据可用（读取输出数据的高位部分之一时复位）。默认值：0（0：陀螺仪 OIS 输出无可用数据集；1：陀螺仪输出有新的数据集可用） |
| `WAKE_UP_SRC` | 45h | 6 | `SLEEP_CHANGE_IA` | 检测活动/静止状态的变化事件。默认值：0（0：未检测到状态变化；1：检测到状态变化） |
| `WAKE_UP_SRC` | 45h | 5 | `FF_IA` | 自由落体事件检测状态。默认值：0（0：未检测到自由落体事件；1：检测到自由落体事件） |
| `WAKE_UP_SRC` | 45h | 4 | `SLEEP_STATE` | 睡眠状态位。默认值：0（0：活动状态；1：静止状态） |
| `WAKE_UP_SRC` | 45h | 3 | `WU_IA` | 唤醒事件检测状态。默认值：0（0：未检测到唤醒事件；1：检测到唤醒事件。） |
| `WAKE_UP_SRC` | 45h | 2 | `X_WU` | X 轴上的唤醒事件检测状态。默认值：0（0：未检测到 X 轴上的唤醒事件；1：检测到 X 轴上的唤醒事件） |
| `WAKE_UP_SRC` | 45h | 1 | `Y_WU` | Y 轴上的唤醒事件检测状态。默认值：0（0：未检测到 Y 轴上的唤醒事件；1：检测到 Y 轴上的唤醒事件） |
| `WAKE_UP_SRC` | 45h | 0 | `Z_WU` | Z 轴上的唤醒事件检测状态。默认值：0（0：未检测到 Z 轴上的唤醒事件；1：检测到 Z 轴上的唤醒事件） |
| `TAP_SRC` | 46h | 6 | `TAP_IA` | 敲击事件检测状态。默认值：0（0：未检测到敲击事件；1：检测到敲击事件） |
| `TAP_SRC` | 46h | 5 | `SINGLE_TAP` | 单击事件状态。默认值：0（0：未检测到单击事件；1：检测到单击事件） |
| `TAP_SRC` | 46h | 4 | `DOUBLE_TAP` | 双击事件检测状态。默认值：0（0：未检测到双击事件；1：检测到双击事件。） |
| `TAP_SRC` | 46h | 3 | `TAP_SIGN` | 敲击事件检测到的加速度符号。默认值：0（0：敲击事件检测到正加速度符号；1：敲击事件检测到负加速度符号） |
| `TAP_SRC` | 46h | 2 | `X_TAP` | X 轴上的敲击事件检测状态。默认值：0（0：未检测到 X 轴上的敲击事件；1：检测到 X 轴上的敲击事件） |
| `TAP_SRC` | 46h | 1 | `Y_TAP` | Y 轴上的敲击事件检测状态。默认值：0（0：未检测到 Y 轴上的敲击事件；1：检测到 Y 轴上的敲击事件） |
| `TAP_SRC` | 46h | 0 | `Z_TAP` | Z 轴上的敲击事件检测状态。默认值：0（0：未检测到 Z 轴上的敲击事件；1：检测到 Z 轴上的敲击事件） |
| `D6D_SRC` | 47h | 6 | `D6D_IA` | 纵向、横向、面朝上、面朝下位置变化的中断有效。默认值：0（0：未检测到位置变化；1：检测到位置变化） |
| `D6D_SRC` | 47h | 5 | `ZH` | Z 轴高事件（高于阈值）。默认值：0（0：未检测到事件；1：检测到事件（高于阈值）） |
| `D6D_SRC` | 47h | 4 | `ZL` | Z 轴低事件（低于阈值）。默认值：0（0：未检测到事件；1：检测到事件（低于阈值）） |
| `D6D_SRC` | 47h | 3 | `YH` | Y 轴高事件（高于阈值）。默认值：0（0：未检测到事件；1：检测到事件（高于阈值）） |
| `D6D_SRC` | 47h | 2 | `YL` | Y 轴低事件（低于阈值）。默认值：0（0：未检测到事件；1：检测到事件（低于阈值）） |
| `D6D_SRC` | 47h | 1 | `XH` | X 轴高事件（超过阈值）。默认值：0（0：未检测到事件；1：检测到事件（超过阈值）） |
| `D6D_SRC` | 47h | 0 | `XL` | X 轴低事件（低于阈值）。默认值：0（0：未检测到事件；1：检测到事件（低于阈值）） |
| `STATUS_MASTER_MAINPAGE` | 48h | 7 | `WR_ONCE_DONE` | 当 MASTER_CONFIG (14h) 中的 WRITE_ONCE 位配置为 1 时，若对从器件 0 的写操作已执行并完成，则该位被置为 1。默认值：0 |
| `STATUS_MASTER_MAINPAGE` | 48h | 6 | `SLAVE3_NACK` | 若从器件 3 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER_MAINPAGE` | 48h | 5 | `SLAVE2_NACK` | 若从器件 2 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER_MAINPAGE` | 48h | 4 | `SLAVE1_NACK` | 若从器件 1 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER_MAINPAGE` | 48h | 3 | `SLAVE0_NACK` | 若从器件 0 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER_MAINPAGE` | 48h | 0 | `SENS_HUB_ENDOP` | 传感器集线器通信状态。默认值：0（0：传感器集线器通信未结束；1：传感器集线器通信已结束） |
| `EMB_FUNC_STATUS_MAINPAGE` | 49h | 7 | `IS_FSM_LC` | FSM 长计数器超时中断事件的中断状态位。（1：检测到中断；0：无中断） |
| `EMB_FUNC_STATUS_MAINPAGE` | 49h | 5 | `IS_SIGMOT` | 大幅运动检测的中断状态位（1：检测到中断；0：无中断） |
| `EMB_FUNC_STATUS_MAINPAGE` | 49h | 4 | `IS_TILT` | 倾斜检测的中断状态位（1：检测到中断；0：无中断） |
| `EMB_FUNC_STATUS_MAINPAGE` | 49h | 3 | `IS_STEP_DET` | 步伐检测的中断状态位（1：检测到中断；0：无中断） |
| `FSM_STATUS_MAINPAGE` | 4Ah | [7:0] | `IS_FSM` |  |
| `MLC_STATUS_MAINPAGE` | 4Bh | [3:0] | `IS_MLC` |  |
| `INTERNAL_FREQ_FINE` | 4Fh | [7:0] | `FREQ_FINE` |  |
| `FUNCTIONS_ENABLE` | 50h | 7 | `INTERRUPTS_ENABLE` | 使能基本中断（6D/4D、自由落体、唤醒、单击、活动/静止）。默认值：0（0：中断禁用；1：中断使能） |
| `FUNCTIONS_ENABLE` | 50h | 6 | `TIMESTAMP_EN` | 使能时间戳计数器。该计数器可在 TIMESTAMP0 (40h)、TIMESTAMP1 (41h)、TIMESTAMP2 (42h) 和 TIMESTAMP3 (43h) 中读取。默认值：0（0：禁用；1：使能） |
| `FUNCTIONS_ENABLE` | 50h | 3 | `DIS_RST_LIR_ALL_INT` | 当该位被置为 1 时，读取 ALL_INT_SRC (1Dh) 寄存器不会复位锁存的中断信号。这在读取相应状态寄存器之前不希望复位某些状态标志时很有用。默认值：0（0：禁用；1：使能） |
| `FUNCTIONS_ENABLE` | 50h | [1:0] | `INACT_EN` |  |
| `DEN` | 51h | 6 | `LVL1_EN` | 使能 DEN 数据电平感应触发。参见表 151。 |
| `DEN` | 51h | 5 | `LVL2_EN` | 使能 DEN 电平感应锁存。参见表 151。 |
| `DEN` | 51h | 4 | `DEN_XL_EN` | 将 DEN 功能扩展到加速度计传感器。默认值：0（0：禁用；1：使能） |
| `DEN` | 51h | 3 | `DEN_X` | DEN 值存储在 X 轴 LSB 中。默认值：1（0：DEN 未存储在 X 轴 LSB 中；1：DEN 存储在 X 轴 LSB 中） |
| `DEN` | 51h | 2 | `DEN_Y` | DEN 值存储在 Y 轴 LSB 中。默认值：1（0：DEN 未存储在 Y 轴 LSB 中；1：DEN 存储在 Y 轴 LSB 中） |
| `DEN` | 51h | 1 | `DEN_Z` | DEN 值存储在 Z 轴 LSB 中。默认值：1（0：DEN 未存储在 Z 轴 LSB 中；1：DEN 存储在 Z 轴 LSB 中） |
| `DEN` | 51h | 0 | `DEN_XL_G` | DEN 标记传感器选择。默认值：0（0：DEN 引脚信息标记在由 DEN_X、DEN_Y、DEN_Z 位选择的陀螺仪轴中；1：DEN 引脚信息标记在由 DEN_X、DEN_Y、DEN_Z 位选择的加速度计轴中） |
| `INACTIVITY_DUR` | 54h | 7 | `SLEEP_STATUS_ON_INT` | 活动/静止中断模式配置。若 INT1_SLEEP_CHANGE 或 INT2_SLEEP_CHANGE 位使能，则在 INT 引脚上驱动睡眠状态或睡眠变化。默认值：0（0：在 INT 引脚上通知睡眠变化；1：在 INT 引脚上报告睡眠状态） |
| `INACTIVITY_DUR` | 54h | [6:4] | `WU_INACT_THS_W` |  |
| `INACTIVITY_DUR` | 54h | [3:2] | `XL_INACT_ODR` |  |
| `INACTIVITY_DUR` | 54h | [1:0] | `INACT_DUR` |  |
| `INACTIVITY_THS` | 55h | [5:0] | `INACT_THS` |  |
| `TAP_CFG0` | 56h | 6 | `LOW_PASS_ON_6D` | 6D 功能上的 LPF2 滤波器选择。参见图 32。默认值：0（0：将 ODR/2 低通滤波数据发送到 6D 中断功能；1：将 LPF2 输出数据发送到 6D 中断功能） |
| `TAP_CFG0` | 56h | 5 | `HW_FUNC_MASK_XL_SETTL` | 使能当加速度计数据处于建立过程时屏蔽基本中断功能（6D/4D、自由落体、唤醒、单击、活动/静止）的执行触发。默认值：0（0：禁用；1：使能）注意：有关工作/功耗模式配置、设置、开启/关闭时间和动态更改的详细信息，请参见产品应用笔记。 |
| `TAP_CFG0` | 56h | 4 | `SLOPE_FDS` | 唤醒和活动/静止功能上的 HPF 或斜率滤波器选择。参见图 32。默认值：0（0：应用斜率滤波器；1：应用 HPF） |
| `TAP_CFG0` | 56h | 3 | `TAP_X_EN` | 使能单击识别中的 X 方向。默认值：0（0：X 方向禁用；1：X 方向使能） |
| `TAP_CFG0` | 56h | 2 | `TAP_Y_EN` | 使能单击识别中的 Y 方向。默认值：0（0：Y 方向禁用；1：Y 方向使能） |
| `TAP_CFG0` | 56h | 1 | `TAP_Z_EN` | 使能单击识别中的 Z 方向。默认值：0（0：Z 方向禁用；1：Z 方向使能） |
| `TAP_CFG0` | 56h | 0 | `LIR` | 锁存中断。默认值：0（0：中断请求未锁存；1：中断请求已锁存） |
| `TAP_CFG1` | 57h | [7:5] | `TAP_PRIORITY` |  |
| `TAP_CFG1` | 57h | [4:0] | `TAP_THS_X` |  |
| `TAP_CFG2` | 58h | [4:0] | `TAP_THS_Y` |  |
| `TAP_THS_6D` | 59h | 7 | `D4D_EN` | 使能 4D 方向检测。Z 轴位置检测被禁用。默认值：0（0：禁用；1：使能） |
| `TAP_THS_6D` | 59h | [6:5] | `SIXD_THS` |  |
| `TAP_THS_6D` | 59h | [4:0] | `TAP_THS_Z` |  |
| `TAP_DUR` | 5Ah | [7:4] | `DUR` |  |
| `TAP_DUR` | 5Ah | [3:2] | `QUIET` |  |
| `TAP_DUR` | 5Ah | [1:0] | `SHOCK` |  |
| `WAKE_UP_THS` | 5Bh | 7 | `SINGLE_DOUBLE_TAP` | 使能单击/双击事件。默认值：0（0：仅使能单击事件；1：使能单击和双击事件） |
| `WAKE_UP_THS` | 5Bh | 6 | `USR_OFF_ON_WU` | 将经过用户偏移校正的低通滤波数据（而非高通滤波数据）送至唤醒和活动/静止功能。参见图 32。默认值：0 |
| `WAKE_UP_THS` | 5Bh | [5:0] | `WK_THS` |  |
| `WAKE_UP_DUR` | 5Ch | 7 | `FF_DUR` |  |
| `WAKE_UP_DUR` | 5Ch | [6:5] | `WAKE_DUR` |  |
| `WAKE_UP_DUR` | 5Ch | [3:0] | `SLEEP_DUR` |  |
| `FREE_FALL` | 5Dh | [7:3] | `FF_DUR` |  |
| `FREE_FALL` | 5Dh | [2:0] | `FF_THS` |  |
| `MD1_CFG` | 5Eh | 7 | `INT1_SLEEP_CHANGE` | 将活动/静止识别事件路由至 INT1。默认值：0（0：禁止将活动/静止事件路由至 INT1；1：使能将活动/静止事件路由至 INT1） |
| `MD1_CFG` | 5Eh | 6 | `INT1_SINGLE_TAP` | 将单击识别事件路由至 INT1。默认值：0（0：禁止将单击事件路由至 INT1；1：使能将单击事件路由至 INT1） |
| `MD1_CFG` | 5Eh | 5 | `INT1_WU` | 将唤醒事件路由至 INT1。默认值：0（0：禁止将唤醒事件路由至 INT1；1：使能将唤醒事件路由至 INT1） |
| `MD1_CFG` | 5Eh | 4 | `INT1_FF` | 将自由落体事件路由至 INT1。默认值：0（0：禁止将自由落体事件路由至 INT1；1：使能将自由落体事件路由至 INT1） |
| `MD1_CFG` | 5Eh | 3 | `INT1_DOUBLE_TAP` | 将单击事件路由至 INT1。默认值：0（0：禁止将双击事件路由至 INT1；1：使能将双击事件路由至 INT1） |
| `MD1_CFG` | 5Eh | 2 | `INT1_6D` | 将 6D 事件路由至 INT1。默认值：0（0：禁止将 6D 事件路由至 INT1；1：使能将 6D 事件路由至 INT1） |
| `MD1_CFG` | 5Eh | 1 | `INT1_EMB_FUNC` | 将嵌入功能事件路由至 INT1。默认值：0（0：禁止将嵌入功能事件路由至 INT1；1：使能将嵌入功能事件路由至 INT1） |
| `MD1_CFG` | 5Eh | 0 | `INT1_SHUB` | 将传感器集线器通信结束事件路由至 INT1。默认值：0（0：禁止将传感器集线器通信结束事件路由至 INT1；1：使能将传感器集线器通信结束事件路由至 INT1） |
| `MD2_CFG` | 5Fh | 7 | `INT2_SLEEP_CHANGE` | 将活动/静止识别事件路由至 INT2。默认值：0（0：禁止将活动/静止事件路由至 INT2；1：使能将活动/静止事件路由至 INT2） |
| `MD2_CFG` | 5Fh | 6 | `INT2_SINGLE_TAP` | 将单击识别路由至 INT2。默认值：0（0：禁止将单击事件路由至 INT2；1：使能将单击事件路由至 INT2） |
| `MD2_CFG` | 5Fh | 5 | `INT2_WU` | 将唤醒事件路由至 INT2。默认值：0（0：禁止将唤醒事件路由至 INT2；1：使能将唤醒事件路由至 INT2） |
| `MD2_CFG` | 5Fh | 4 | `INT2_FF` | 将自由落体事件路由至 INT2。默认值：0（0：禁止将自由落体事件路由至 INT2；1：使能将自由落体事件路由至 INT2） |
| `MD2_CFG` | 5Fh | 3 | `INT2_DOUBLE_TAP` | 将单击事件路由至 INT2。默认值：0（0：禁止将双击事件路由至 INT2；1：使能将双击事件路由至 INT2） |
| `MD2_CFG` | 5Fh | 2 | `INT2_6D` | 将 6D 事件路由至 INT2。默认值：0（0：禁止将 6D 事件路由至 INT2；1：使能将 6D 事件路由至 INT2） |
| `MD2_CFG` | 5Fh | 1 | `INT2_EMB_FUNC` | 将嵌入功能事件路由至 INT2。默认值：0（0：禁止将嵌入功能事件路由至 INT2；1：使能将嵌入功能事件路由至 INT2） |
| `MD2_CFG` | 5Fh | 0 | `INT2_TIMESTAMP` | 使能将 5.6 ms 内时间戳溢出的告警路由至 INT2 引脚。 |
| `HAODR_CFG` | 62h | [1:0] | `HAODR_SEL` |  |
| `EMB_FUNC_CFG` | 63h | 7 | `XL_DUALC_BATCH_FROM_IF` |  |
| `EMB_FUNC_CFG` | 63h | 5 | `EMB_FUNC_IRQ_MASK_G_SETTL` | 当陀螺仪数据稳定时，使能/屏蔽嵌入功能的执行触发。默认值：0（0：禁用；1：屏蔽嵌入功能的执行触发，直至陀螺仪滤波器稳定结束）注意：有关工作/功耗模式配置、设置、开启/关闭时间以及动态更改的详细信息，请参见产品应用笔记。 |
| `EMB_FUNC_CFG` | 63h | 4 | `EMB_FUNC_IRQ_MASK_XL_SETTL` | 当加速度计数据稳定时，使能/屏蔽嵌入功能的执行触发。默认值：0（0：禁用；1：屏蔽嵌入功能的执行触发，直至加速度计滤波器稳定结束）注意：有关工作/功耗模式配置、设置、开启/关闭时间以及动态更改的详细信息，请参见产品应用笔记。 |
| `EMB_FUNC_CFG` | 63h | 3 | `EMB_FUNC_DISABLE` | 禁止嵌入功能的执行。默认值：0（0：禁用；1：不再生成嵌入功能执行触发，且该位重新置为 0 时将强制执行所有初始化过程）。 |
| `UI_HANDSHAKE_CTRL` | 64h | 1 | `UI_SHARED_ACK` | 主接口侧。该位用于应答握手。如果辅助接口未在访问共享寄存器，则该位由器件置为 1，并且允许在主接口上对 UI_SPI2_SHARED_0 (65h) 至 UI_SPI2_SHARED_5 (6Ah) 寄存器进行 R/W 操作。 |
| `UI_HANDSHAKE_CTRL` | 64h | 0 | `UI_SHARED_REQ` | 该位由主接口主器件用于请求访问 UI_SPI2_SHARED_0 (65h) 至 UI_SPI2_SHARED_5 (6Ah) 寄存器。当 R/W 操作完成后，主器件必须复位该位。 |
| `CTRL_EIS` | 6Bh | [7:6] | `ODR_G_EIS` |  |
| `CTRL_EIS` | 6Bh | 4 | `LPF_G_EIS_BW` | 陀螺仪数字 LPF_EIS 滤波器带宽选择。参见表 199。 |
| `CTRL_EIS` | 6Bh | 3 | `G_EIS_ON_G_OIS_OUT_REG` | 使能将陀螺仪 EIS 输出从 UI 输出地址 (2Eh – 33h) 路由至 OIS。当该位置为 1 时，无法从主接口读取陀螺仪 OIS 数据。默认值：0（0：禁用；1：使能） |
| `CTRL_EIS` | 6Bh | [2:0] | `FS_G_EIS` |  |
| `UI_INT_OIS` | 6Fh | 7 | `INT2_DRDY_OIS` | 使能从 UI 接口在 INT2 引脚上输出 OIS 链 DRDY。该设置优先于所有其他 INT2 设置。 |
| `UI_INT_OIS` | 6Fh | 6 | `DRDY_MASK_OIS` | 使能 / 屏蔽 OIS 数据可用。默认值：0（0：禁用；1：屏蔽 OIS DRDY 信号（加速度计和陀螺仪）直至滤波器建立结束（加速度计和陀螺仪独立屏蔽）） |
| `UI_INT_OIS` | 6Fh | 4 | `ST_OIS_CLAMPDIS` | 禁用自检期间的 OIS 链钳位。默认值：0（0：自检期间所有 OIS 链输出 = 8000h；1：OIS 链自检输出） |
| `UI_CTRL1_OIS` | 70h | 5 | `SIM_OIS` | SPI2 3 线或 4 线接口。默认值：0（0：4 线 SPI2；1：3 线 SPI2） |
| `UI_CTRL1_OIS` | 70h | 2 | `OIS_XL_EN` | 使能加速度计 OIS 链。默认值：0（0：加速度计 OIS 链禁用；1：加速度计 OIS 链使能） |
| `UI_CTRL1_OIS` | 70h | 1 | `OIS_G_EN` | 使能陀螺仪 OIS 链。默认值：0（0：陀螺仪 OIS 链禁用；1：陀螺仪 OIS 链使能） |
| `UI_CTRL1_OIS` | 70h | 0 | `SPI2_READ_EN` | 在主接口完全控制模式下，使能辅助 SPI 用于读取 OIS 数据，寄存器 SPI2_OUTX_L_G_OIS (22h) 和 SPI2_OUTX_H_G_OIS (23h) 至第 11.9 节 SPI2_OUTZ_L_A_OIS (2Ch) 和 SPI2_OUTZ_H_A_OIS (2Dh)。默认值：0（0：禁用从辅助 SPI 读取 OIS 数据；1：使能从辅助 SPI 读取 OIS 数据） |
| `UI_CTRL2_OIS` | 71h | [4:3] | `LPF1_G_OIS_BW` |  |
| `UI_CTRL2_OIS` | 71h | [2:0] | `FS_G_OIS` |  |
| `UI_CTRL3_OIS` | 72h | [5:3] | `LPF_XL_OIS_BW` |  |
| `UI_CTRL3_OIS` | 72h | [1:0] | `FS_XL_OIS` |  |
| `X_OFS_USR` | 73h | [7:0] | `X_OFS_USR` |  |
| `Y_OFS_USR` | 74h | [7:0] | `Y_OFS_USR` |  |
| `Z_OFS_USR` | 75h | [7:0] | `Z_OFS_USR` |  |
| `FIFO_DATA_OUT_TAG` | 78h | [7:3] | `TAG_SENSOR` |  |
| `FIFO_DATA_OUT_TAG` | 78h | [2:1] | `TAG_CNT` |  |

## 2. 嵌入功能寄存器

| 寄存器 | 地址 | 类型 | 默认 | 位域 | 说明 |
|---|---|---|---|---|---|
| `PAGE_SEL` | 02h | R/W | 00000001 | bit[7:4] PAGE_SEL<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 固定1(2) |  |
| `EMB_FUNC_EN_A` | 04h | R/W | 00000000 | bit7 MLC_BEFORE_FSM_EN<br>bit6 固定0(1)<br>bit5 SIGN_MOTION_EN<br>bit4 TILT_EN<br>bit3 PEDO_EN<br>bit2 固定0(1)<br>bit1 SFLP_GAME_EN<br>bit0 固定0(1) | 使能嵌入功能寄存器 |
| `EMB_FUNC_EN_B` | 05h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 MLC_EN<br>bit3 FIFO_COMPR_EN<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 FSM_EN | 使能嵌入功能寄存器 |
| `EMB_FUNC_EXEC_STATUS` | 07h | R | output | bit7 固定0<br>bit6 固定0<br>bit5 固定0<br>bit4 固定0<br>bit3 固定0<br>bit2 固定0<br>bit1 EMB_FUNC_EXEC_OVR<br>bit0 EMB_FUNC_ENDOP | 嵌入功能执行状态寄存器 |
| `PAGE_ADDRESS` | 08h | R/W | 00000000 | bit[7:0] PAGE_ADDR | 页地址寄存器 |
| `PAGE_VALUE` | 09h | R/W | 00000000 | bit[7:0] PAGE_VALUE | 页值寄存器 |
| `EMB_FUNC_INT1` | 0Ah | R/W | 00000000 | bit7 INT1_FSM_LC<br>bit6 固定0(1)<br>bit5 INT1_SIG_MOT<br>bit4 INT1_TILT<br>bit3 INT1_STEP_DETECTOR<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 固定0(1) | INT1 引脚控制寄存器 |
| `FSM_INT1` | 0Bh | R/W | 00000000 | bit[7:0] INT1_FSM | INT1 引脚控制寄存器 |
| `MLC_INT1` | 0Dh | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit[3:0] INT1_MLC | INT1 引脚控制寄存器 |
| `EMB_FUNC_INT2` | 0Eh | R/W | 00000000 | bit7 INT2_FSM_LC<br>bit6 固定0(1)<br>bit5 INT2_SIG_MOT<br>bit4 INT2_TILT<br>bit3 INT2_STEP_DETECTOR<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 固定0(1) | INT2 引脚控制寄存器 |
| `FSM_INT2` | 0Fh | R/W | 00000000 | bit[7:0] INT2_FSM | INT2 引脚控制寄存器 |
| `MLC_INT2` | 11h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit[3:0] INT2_MLC | INT2 引脚控制寄存器 |
| `EMB_FUNC_STATUS` | 12h | R | output | bit7 IS_FSM_LC<br>bit6 固定0<br>bit5 IS_SIGMOT<br>bit4 IS_TILT<br>bit3 IS_STEP_DET<br>bit2 固定0<br>bit1 固定0<br>bit0 固定0 | 嵌入功能状态寄存器 |
| `FSM_STATUS` | 13h | R | output | bit[7:0] IS_FSM | 有限状态机状态寄存器 |
| `MLC_STATUS` | 15h | R | output | bit7 固定0<br>bit6 固定0<br>bit5 固定0<br>bit4 固定0<br>bit[3:0] IS_MLC | 机器学习内核状态寄存器 |
| `PAGE_RW` | 17h | R/W | 00000000 | bit7 EMB_FUNC_LIR<br>bit6 PAGE_WRITE<br>bit5 PAGE_READ<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 固定0(1) |  |
| `EMB_FUNC_FIFO_EN_A` | 44h | R/W | 00000000 | bit7 MLC_FIFO_EN<br>bit6 STEP_COUNTER_FIFO_EN<br>bit5 SFLP_GBIAS_FIFO_EN<br>bit4 SFLP_GRAVITY_FIFO_EN<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 SFLP_GAME_FIFO_EN<br>bit0 固定0(1) | 嵌入功能 FIFO 配置寄存器 A |
| `EMB_FUNC_FIFO_EN_B` | 45h | R/W | - | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 固定0(1)<br>bit1 MLC_FILTER_FEATURE_FIFO_EN<br>bit0 固定0(1) | 嵌入功能 FIFO 配置寄存器 B |
| `FSM_ENABLE` | 46h | R/W | 00000000 | bit7 FSM8_EN<br>bit6 FSM7_EN<br>bit5 FSM6_EN<br>bit4 FSM5_EN<br>bit3 FSM4_EN<br>bit2 FSM3_EN<br>bit1 FSM2_EN<br>bit0 FSM1_EN | FSM 使能寄存器 |
| `FSM_LONG_COUNTER_L` | 48h | R/W | 00000000 | bit[7:0] FSM_LC | FSM 长计数器状态寄存器 |
| `FSM_LONG_COUNTER_H` | 49h | R/W | 00000000 | bit[7:2] FSM_LC_1<br>bit[1:0] FSM_LC |  |
| `INT_ACK_MASK` | 4Bh | R/W | 00000000 | bit[7:0] IACK_MASK | 复位状态寄存器 |
| `FSM_OUTS1` | 4Ch | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM1 输出寄存器 |
| `FSM_OUTS2` | 4Dh | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM2 输出寄存器 |
| `FSM_OUTS3` | 4Eh | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM3 输出寄存器 |
| `FSM_OUTS4` | 4Fh | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM4 输出寄存器 |
| `FSM_OUTS5` | 50h | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM5 输出寄存器 |
| `FSM_OUTS6` | 51h | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM6 输出寄存器 |
| `FSM_OUTS7` | 52h | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM7 输出寄存器 |
| `FSM_OUTS8` | 53h | R | output | bit7 P_X<br>bit6 N_X<br>bit5 P_Y<br>bit4 N_Y<br>bit3 P_Z<br>bit2 N_Z<br>bit1 P_V<br>bit0 N_V | FSM8 输出寄存器 |
| `SFLP_ODR` | 5Eh | R/W | 01011011 | bit7 固定0(1)<br>bit6 固定1(2)<br>bit[5:3] SFLP_GAME_ODR<br>bit2 固定0(1)<br>bit1 固定1(2)<br>bit0 固定1(2) | 传感器融合低功耗输出数据速率配置寄存器 |
| `FSM_ODR` | 5Fh | R/W | 01001011 | bit7 固定0(1)<br>bit6 固定1(2)<br>bit[5:3] FSM_ODR<br>bit2 固定0(1)<br>bit1 固定1(2)<br>bit0 固定1(2) | 有限状态机输出数据速率配置寄存器 |
| `MLC_ODR` | 60h | R/W | 00010101 | bit7 固定0(1)<br>bit[6:4] MLC_ODR<br>bit3 固定0(1)<br>bit2 固定1(2)<br>bit1 固定0(1)<br>bit0 固定1(2) | 机器学习内核输出数据速率配置寄存器 |
| `STEP_COUNTER_L` | 62h | R | output | bit[7:0] STEP | 步数计数器输出寄存器 |
| `STEP_COUNTER_H` | 63h | R | output | bit[7:2] STEP_1<br>bit[1:0] STEP |  |
| `EMB_FUNC_SRC` | 64h | R/W | output | bit7 PEDO_RST_STEP<br>bit6 固定0(1)<br>bit5 STEP_DETECTED<br>bit4 STEP_COUNT_DELTA_IA<br>bit3 STEP_OVERFLOW<br>bit2 STEPCOUNTER_BIT_SET<br>bit1 固定0(1)<br>bit0 固定0(1) | 嵌入功能源寄存器 |
| `EMB_FUNC_INIT_A` | 66h | R/W | 00000000 | bit7 MLC_BEFORE_FSM_INIT<br>bit6 固定0(1)<br>bit5 SIG_MOT_INIT<br>bit4 TILT_INIT<br>bit3 STEP_DET_INIT<br>bit2 固定0(1)<br>bit1 SFLP_GAME_INIT<br>bit0 固定0(1) | 嵌入功能初始化寄存器 |
| `EMB_FUNC_INIT_B` | 67h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 MLC_INIT<br>bit3 FIFO_COMPR_INIT<br>bit2 固定0(1)<br>bit1 固定0(1)<br>bit0 FSM_INIT | 嵌入功能初始化寄存器 |
| `MLC1_SRC` | 70h | R | output | bit[7:0] MLC1_SRC | 机器学习内核源寄存器 |
| `MLC2_SRC` | 71h | R | output | bit[7:0] MLC2_SRC | 机器学习内核源寄存器 |
| `MLC3_SRC` | 72h | R | output | bit[7:0] MLC3_SRC | 机器学习内核源寄存器 |
| `MLC4_SRC` | 73h | R | output | bit[7:0] MLC4_SRC | 机器学习内核源寄存器 |

### 嵌入功能寄存器 · 位域明细与用法

| 寄存器 | 地址 | 位 | 字段 | 功能与用法 |
|---|---|---|---|---|
| `PAGE_SEL` | 02h | [7:4] | `PAGE_SEL` |  |
| `EMB_FUNC_EN_A` | 04h | 7 | `MLC_BEFORE_FSM_EN` | 使能机器学习内核功能。当通过将该位置为 1 使能机器学习内核时，MLC 算法在 FSM 程序之前执行。默认值：0（0：禁用机器学习内核功能；1：使能机器学习内核功能并在 FSM 程序之前执行） |
| `EMB_FUNC_EN_A` | 04h | 5 | `SIGN_MOTION_EN` | 使能大幅运动检测功能。默认值：0（0：禁用大幅运动检测功能；1：使能大幅运动检测功能） |
| `EMB_FUNC_EN_A` | 04h | 4 | `TILT_EN` | 使能倾斜计算。默认值：0（0：倾斜算法禁用；1：倾斜算法使能） |
| `EMB_FUNC_EN_A` | 04h | 3 | `PEDO_EN` | 使能计步器算法。默认值：0（0：计步器算法禁用；1：计步器算法使能） |
| `EMB_FUNC_EN_A` | 04h | 1 | `SFLP_GAME_EN` | 使能用于 6 轴（加速度计 + 陀螺仪）游戏旋转向量的传感器融合低功耗算法。默认值：0（0：用于 6 轴加速度计 + 陀螺仪的传感器融合算法禁用；1：用于 6 轴加速度计 + 陀螺仪的传感器融合算法使能） |
| `EMB_FUNC_EN_B` | 05h | 4 | `MLC_EN` | 使能机器学习内核功能。将此位置为 1 使能机器学习内核时，MLC 算法在 FSM 程序执行之后执行。默认值：0（0：机器学习内核功能禁用；1：机器学习内核功能使能并在 FSM 程序之后执行） |
| `EMB_FUNC_EN_B` | 05h | 3 | `FIFO_COMPR_EN` | 使能 FIFO 压缩功能。默认值：0（0：FIFO 压缩功能禁用；1：FIFO 压缩功能使能） |
| `EMB_FUNC_EN_B` | 05h | 0 | `FSM_EN` | 使能有限状态机 (FSM) 功能。默认值：0（0：FSM 功能禁用；1：FSM 功能使能） |
| `EMB_FUNC_EXEC_STATUS` | 07h | 1 | `EMB_FUNC_EXEC_OVR` | 当嵌入功能程序的执行超过最大时间时，此位置为 1（新数据在算法结束之前生成）。默认值：0 |
| `EMB_FUNC_EXEC_STATUS` | 07h | 0 | `EMB_FUNC_ENDOP` | 此位置为 1 时，没有嵌入功能正在运行。默认值：0 |
| `PAGE_ADDRESS` | 08h | [7:0] | `PAGE_ADDR` |  |
| `PAGE_VALUE` | 09h | [7:0] | `PAGE_VALUE` |  |
| `EMB_FUNC_INT1` | 0Ah | 7 | `INT1_FSM_LC` | 将 FSM 长计数器超时中断事件路由到 INT1。默认值：0（0：到 INT1 的路由禁用；1：到 INT1 的路由使能） |
| `EMB_FUNC_INT1` | 0Ah | 5 | `INT1_SIG_MOT` | 将大幅运动事件路由到 INT1。默认值：0（0：到 INT1 的路由禁用；1：到 INT1 的路由使能） |
| `EMB_FUNC_INT1` | 0Ah | 4 | `INT1_TILT` | 将倾斜事件路由到 INT1。默认值：0（0：到 INT1 的路由禁用；1：到 INT1 的路由使能） |
| `EMB_FUNC_INT1` | 0Ah | 3 | `INT1_STEP_DETECTOR` |  |
| `FSM_INT1` | 0Bh | [7:0] | `INT1_FSM` |  |
| `MLC_INT1` | 0Dh | [3:0] | `INT1_MLC` |  |
| `EMB_FUNC_INT2` | 0Eh | 7 | `INT2_FSM_LC` | 将 FSM 长计数器超时中断事件路由到 INT2。默认值：0（0：到 INT2 的路由禁用；1：到 INT2 的路由使能） |
| `EMB_FUNC_INT2` | 0Eh | 5 | `INT2_SIG_MOT` | 将大幅运动事件路由到 INT2。默认值：0（0：到 INT2 的路由禁用；1：到 INT2 的路由使能） |
| `EMB_FUNC_INT2` | 0Eh | 4 | `INT2_TILT` | 将倾斜事件路由到 INT2。默认值：0（0：到 INT2 的路由禁用；1：到 INT2 的路由使能） |
| `EMB_FUNC_INT2` | 0Eh | 3 | `INT2_STEP_DETECTOR` |  |
| `FSM_INT2` | 0Fh | [7:0] | `INT2_FSM` |  |
| `MLC_INT2` | 11h | [3:0] | `INT2_MLC` |  |
| `EMB_FUNC_STATUS` | 12h | 7 | `IS_FSM_LC` | FSM 长计数器超时中断事件的中断状态位。（1：检测到中断；0：无中断） |
| `EMB_FUNC_STATUS` | 12h | 5 | `IS_SIGMOT` | 大幅运动检测的中断状态位（1：检测到中断；0：无中断） |
| `EMB_FUNC_STATUS` | 12h | 4 | `IS_TILT` | 倾斜检测的中断状态位（1：检测到中断；0：无中断） |
| `EMB_FUNC_STATUS` | 12h | 3 | `IS_STEP_DET` | 步伐检测的中断状态位（1：检测到中断；0：无中断） |
| `FSM_STATUS` | 13h | [7:0] | `IS_FSM` |  |
| `MLC_STATUS` | 15h | [3:0] | `IS_MLC` |  |
| `PAGE_RW` | 17h | 7 | `EMB_FUNC_LIR` | 嵌入功能的锁存中断模式。默认值：0（0：嵌入功能中断请求未锁存；1：嵌入功能中断请求已锁存） |
| `PAGE_RW` | 17h | 6 | `PAGE_WRITE` | 使能对所选高级功能专用页的写入。(1) 默认值：0（1：使能；0：禁用） |
| `PAGE_RW` | 17h | 5 | `PAGE_READ` | 使能对所选高级功能专用页的读取。(1) 默认值：0（1：使能；0：禁用） |
| `EMB_FUNC_FIFO_EN_A` | 44h | 7 | `MLC_FIFO_EN` | 使能将机器学习内核结果批处理到 FIFO 缓冲器中。默认值：0（0：禁用；1：使能） |
| `EMB_FUNC_FIFO_EN_A` | 44h | 6 | `STEP_COUNTER_FIFO_EN` | 使能将步数计数器值批处理到 FIFO 缓冲器中。默认值：0（0：禁用；1：使能） |
| `EMB_FUNC_FIFO_EN_A` | 44h | 5 | `SFLP_GBIAS_FIFO_EN` | 使能将 SFLP 算法计算得到的陀螺仪偏差值批处理到 FIFO缓冲器中。默认值：0（0：禁用；1：使能） |
| `EMB_FUNC_FIFO_EN_A` | 44h | 4 | `SFLP_GRAVITY_FIFO_EN` | 使能将 SFLP 算法计算得到的重力值批处理到 FIFO 缓冲器中。默认值：0（0：禁用；1：使能） |
| `EMB_FUNC_FIFO_EN_A` | 44h | 1 | `SFLP_GAME_FIFO_EN` | 使能将 SFLP算法计算得到的游戏旋转向量（四元数）值批处理到 FIFO 缓冲器中。默认值：0（0：禁用；1：使能） |
| `EMB_FUNC_FIFO_EN_B` | 45h | 1 | `MLC_FILTER_FEATURE_FIFO_EN` | 使能将机器学习内核滤波器和特征批处理到 FIFO 缓冲器中。默认值：0（0：禁用；1：使能） |
| `FSM_ENABLE` | 46h | 7 | `FSM8_EN` | 使能 FSM8。默认值：0（0：FSM8 禁用；1：FSM8 使能） |
| `FSM_ENABLE` | 46h | 6 | `FSM7_EN` | 使能 FSM7。默认值：0（0：FSM7 禁用；1：FSM7 使能） |
| `FSM_ENABLE` | 46h | 5 | `FSM6_EN` | 使能 FSM6。默认值：0（0：FSM6 禁用；1：FSM6 使能） |
| `FSM_ENABLE` | 46h | 4 | `FSM5_EN` | 使能 FSM5。默认值：0（0：FSM5 禁用；1：FSM5 使能） |
| `FSM_ENABLE` | 46h | 3 | `FSM4_EN` | 使能 FSM4。默认值：0（0：FSM4 禁用；1：FSM4 使能） |
| `FSM_ENABLE` | 46h | 2 | `FSM3_EN` | 使能 FSM3。默认值：0（0：FSM3 禁用；1：FSM3 使能） |
| `FSM_ENABLE` | 46h | 1 | `FSM2_EN` | 使能 FSM2。默认值：0（0：FSM2 禁用；1：FSM2 使能） |
| `FSM_ENABLE` | 46h | 0 | `FSM1_EN` | 使能 FSM1。默认值：0（0：FSM1 禁用；1：FSM1 使能） |
| `FSM_LONG_COUNTER_L` | 48h | [7:0] | `FSM_LC` |  |
| `FSM_LONG_COUNTER_H` | 49h | [7:2] | `FSM_LC_1` |  |
| `FSM_LONG_COUNTER_H` | 49h | [1:0] | `FSM_LC` |  |
| `INT_ACK_MASK` | 4Bh | [7:0] | `IACK_MASK` |  |
| `FSM_OUTS1` | 4Ch | 7 | `P_X` | FSM1 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS1` | 4Ch | 6 | `N_X` | FSM1 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS1` | 4Ch | 5 | `P_Y` | FSM1 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS1` | 4Ch | 4 | `N_Y` | FSM1 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS1` | 4Ch | 3 | `P_Z` | FSM1 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS1` | 4Ch | 2 | `N_Z` | FSM1 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS1` | 4Ch | 1 | `P_V` | FSM1 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS1` | 4Ch | 0 | `N_V` | FSM1 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 7 | `P_X` | FSM2 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 6 | `N_X` | FSM2 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 5 | `P_Y` | FSM2 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 4 | `N_Y` | FSM2 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 3 | `P_Z` | FSM2 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 2 | `N_Z` | FSM2 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 1 | `P_V` | FSM2 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS2` | 4Dh | 0 | `N_V` | FSM2 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 7 | `P_X` | FSM3 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 6 | `N_X` | FSM3 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 5 | `P_Y` | FSM3 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 4 | `N_Y` | FSM3 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 3 | `P_Z` | FSM3 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 2 | `N_Z` | FSM3 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 1 | `P_V` | FSM3 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS3` | 4Eh | 0 | `N_V` | FSM3 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 7 | `P_X` | FSM4 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 6 | `N_X` | FSM4 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 5 | `P_Y` | FSM4 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 4 | `N_Y` | FSM4 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 3 | `P_Z` | FSM4 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 2 | `N_Z` | FSM4 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 1 | `P_V` | FSM4 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS4` | 4Fh | 0 | `N_V` | FSM4 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 7 | `P_X` | FSM5 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 6 | `N_X` | FSM5 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 5 | `P_Y` | FSM5 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 4 | `N_Y` | FSM5 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 3 | `P_Z` | FSM5 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 2 | `N_Z` | FSM5 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 1 | `P_V` | FSM5 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS5` | 50h | 0 | `N_V` | FSM5 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 7 | `P_X` | FSM6 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 6 | `N_X` | FSM6 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 5 | `P_Y` | FSM6 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 4 | `N_Y` | FSM6 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 3 | `P_Z` | FSM6 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 2 | `N_Z` | FSM6 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 1 | `P_V` | FSM6 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS6` | 51h | 0 | `N_V` | FSM6 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 7 | `P_X` | FSM7 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 6 | `N_X` | FSM7 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 5 | `P_Y` | FSM7 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 4 | `N_Y` | FSM7 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 3 | `P_Z` | FSM7 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 2 | `N_Z` | FSM7 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 1 | `P_V` | FSM7 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS7` | 52h | 0 | `N_V` | FSM7 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 7 | `P_X` | FSM8 输出：在 X 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 6 | `N_X` | FSM8 输出：在 X 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 5 | `P_Y` | FSM8 输出：在 Y 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 4 | `N_Y` | FSM8 输出：在 Y 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 3 | `P_Z` | FSM8 输出：在 Z 轴上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 2 | `N_Z` | FSM8 输出：在 Z 轴上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 1 | `P_V` | FSM8 输出：在向量上检测到正向事件。（0：未检测到事件；1：检测到事件） |
| `FSM_OUTS8` | 53h | 0 | `N_V` | FSM8 输出：在向量上检测到负向事件。（0：未检测到事件；1：检测到事件） |
| `SFLP_ODR` | 5Eh | [5:3] | `SFLP_GAME_ODR` |  |
| `FSM_ODR` | 5Fh | [5:3] | `FSM_ODR` |  |
| `MLC_ODR` | 60h | [6:4] | `MLC_ODR` |  |
| `STEP_COUNTER_L` | 62h | [7:0] | `STEP` |  |
| `STEP_COUNTER_H` | 63h | [7:2] | `STEP_1` |  |
| `STEP_COUNTER_H` | 63h | [1:0] | `STEP` |  |
| `EMB_FUNC_SRC` | 64h | 7 | `PEDO_RST_STEP` | 复位计步器步数计数器。读/写位。（0：禁用；1：使能） |
| `EMB_FUNC_SRC` | 64h | 5 | `STEP_DETECTED` | 步伐检测器事件检测状态。只读位。（0：未检测到步伐检测事件；1：检测到步伐检测事件） |
| `EMB_FUNC_SRC` | 64h | 4 | `STEP_COUNT_DELTA_IA` | 计步器在 delta 时间内的步伐识别状态。只读位。（0：在 delta 时间内未识别到步伐；1：在 delta 时间内至少识别到一个步伐） |
| `EMB_FUNC_SRC` | 64h | 3 | `STEP_OVERFLOW` | 步数计数器溢出状态。只读位。（0：步数计数器值 < 2^16；1：步数计数器值达到 2^16） |
| `EMB_FUNC_SRC` | 64h | 2 | `STEPCOUNTER_BIT_SET` | 当步数增加时，该位等于 1。如果在 PEDO_SC_DELTAT_L (D0h) 和 PEDO_SC_DELTAT_H (D1h) 嵌入高级功能（第 1 页）寄存器中编程了定时器周期，则该位保持为 0。只读位。 |
| `EMB_FUNC_INIT_A` | 66h | 7 | `MLC_BEFORE_FSM_INIT` | 机器学习内核初始化请求（MLC 在 FSM 之前执行）。默认值：0 |
| `EMB_FUNC_INIT_A` | 66h | 5 | `SIG_MOT_INIT` | 大幅运动检测算法初始化请求。默认值：0 |
| `EMB_FUNC_INIT_A` | 66h | 4 | `TILT_INIT` | 倾斜算法初始化请求。默认值：0 |
| `EMB_FUNC_INIT_A` | 66h | 3 | `STEP_DET_INIT` | 计步器步数计数器/检测器算法初始化请求。默认值：0 |
| `EMB_FUNC_INIT_A` | 66h | 1 | `SFLP_GAME_INIT` | SFLP 游戏算法初始化请求。默认值：0 |
| `EMB_FUNC_INIT_B` | 67h | 4 | `MLC_INIT` | 机器学习内核初始化请求（MLC 在 FSM 之后执行）。默认值：0 |
| `EMB_FUNC_INIT_B` | 67h | 3 | `FIFO_COMPR_INIT` | FIFO 压缩特征初始化请求。默认值：0 |
| `EMB_FUNC_INIT_B` | 67h | 0 | `FSM_INIT` | FSM 初始化请求。默认值：0 |
| `MLC1_SRC` | 70h | [7:0] | `MLC1_SRC` |  |
| `MLC2_SRC` | 71h | [7:0] | `MLC2_SRC` |  |
| `MLC3_SRC` | 72h | [7:0] | `MLC3_SRC` |  |
| `MLC4_SRC` | 73h | [7:0] | `MLC4_SRC` |  |

## 3. 嵌入高级功能页寄存器

| 寄存器 | 地址 | 类型 | 默认 | 位域 | 说明 |
|---|---|---|---|---|---|
| `EXT_FORMAT` | 00h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit2 EXT_FORMAT_SEL<br>bit1 固定0(1)<br>bit0 固定0(1) |  |
| `EXT_3BYTE_SENSITIVITY_L` | 02h | R/W | 00000000 | bit[7:0] EXT_3BYTE_S | External sensor (3-byte output data) sensitivity value register for th |
| `EXT_3BYTE_SENSITIVITY_H` | 03h | R/W | 00001100 | bit[7:2] EXT_3BYTE_S_1<br>bit[1:0] EXT_3BYTE_S |  |
| `EXT_3BYTE_OFFSET_XL` | 06h | R/W | 00000000 | bit[7:0] EXT_3BYTE_OFF | External sensor (3-byte output data) offset value register for the fin |
| `EXT_3BYTE_OFFSET_L` | 07h | R/W | 01010100 | bit[7:2] EXT_3BYTE_OFF_1<br>bit[1:0] EXT_3BYTE_OFF |  |
| `EXT_3BYTE_OFFSET_H` | 08h | R/W | 00111111 | bit[7:4] EXT_3BYTE_OFF_2<br>bit[3:0] EXT_3BYTE_OFF_1 |  |
| `SFLP_GAME_GBIASX_L` | 6Eh | R/W | 00000000 | bit[7:0] GAME_GBIASX | SFLP 游戏算法 X 轴陀螺仪偏差寄存器 |
| `SFLP_GAME_GBIASX_H` | 6Fh | R/W | 00000000 | bit[7:2] GAME_GBIASX_1<br>bit[1:0] GAME_GBIASX |  |
| `SFLP_GAME_GBIASY_L` | 70h | R/W | 00000000 | bit[7:0] GAME_GBIASY | SFLP 游戏算法 Y 轴陀螺仪偏差寄存器 |
| `SFLP_GAME_GBIASY_H` | 71h | R/W | 00000000 | bit[7:2] GAME_GBIASY_1<br>bit[1:0] GAME_GBIASY |  |
| `SFLP_GAME_GBIASZ_L` | 72h | R/W | 00000000 | bit[7:0] GAME_GBIASZ | SFLP 游戏算法 Z 轴陀螺仪偏差寄存器 |
| `SFLP_GAME_GBIASZ_H` | 73h | R/W | 00000000 | bit[7:2] GAME_GBIASZ_1<br>bit[1:0] GAME_GBIASZ |  |
| `FSM_LC_TIMEOUT_L` | 7Ah | R/W | 00000000 | bit[7:0] FSM_LC_TIMEOUT | FSM 长计数器超时寄存器 |
| `FSM_LC_TIMEOUT_H` | 7Bh | R/W | 00000000 | bit[7:2] FSM_LC_TIMEOUT1<br>bit[1:0] FSM_LC_TIMEOUT |  |
| `FSM_PROGRAMS` | 7Ch | R/W | 00000000 | bit[7:0] FSM_N_PROG | FSM 程序数量寄存器 |
| `FSM_START_ADD_L` | 7Eh | R/W | 00000000 | bit[7:0] FSM_START | FSM 起始地址寄存器（R/W）。首个可用地址为 0x35C。 |
| `FSM_START_ADD_H` | 7Fh | R/W | 00000000 | bit[7:2] FSM_START1<br>bit[1:0] FSM_START |  |
| `PEDO_CMD_REG` | 83h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 CARRY_COUNT_EN<br>bit2 FP_REJECTION_EN<br>bit1 固定0(1)<br>bit0 固定0(1) | 计步器配置寄存器 |
| `PEDO_DEB_STEPS_CONF` | 84h | R/W | 00001010 | bit[7:0] DEB_STEP | 计步器去抖配置寄存器 |
| `FSM_EXT_SENSITIVITY_L` | BAh | R/W | 00100100 | bit[7:0] FSM_EXT_S | 用于有限状态机的外部传感器灵敏度值寄存器 |
| `FSM_EXT_SENSITIVITY_H` | BBh | R/W | 00010110 | bit[7:2] FSM_EXT_S_1<br>bit[1:0] FSM_EXT_S |  |
| `FSM_EXT_OFFX_L` | C0h | R/W | 00000000 | bit[7:0] FSM_EXT_OFFX |  |
| `FSM_EXT_OFFX_H` | C1h | R/W | 00000000 | bit[7:2] FSM_EXT_OFFX_1<br>bit[1:0] FSM_EXT_OFFX |  |
| `FSM_EXT_OFFY_L` | C2h | R/W | 00000000 | bit[7:0] FSM_EXT_OFFY |  |
| `FSM_EXT_OFFY_H` | C3h | R/W | 00000000 | bit[7:2] FSM_EXT_OFFY_1<br>bit[1:0] FSM_EXT_OFFY |  |
| `FSM_EXT_OFFZ_L` | C4h | R/W | 00000000 | bit[7:0] FSM_EXT_OFFZ | 外部传感器 Z 轴偏移寄存器 |
| `FSM_EXT_OFFZ_H` | C5h | R/W | 00000000 | bit[7:2] FSM_EXT_OFFZ_1<br>bit[1:0] FSM_EXT_OFFZ |  |
| `FSM_EXT_MATRIX_XX_L` | C6h | R/W | 00000000 | bit[7:0] FSM_EXT_MAT_XX | 外部传感器变换矩阵寄存器 |
| `FSM_EXT_MATRIX_XX_H` | C7h | R/W | 00111100 | bit[7:2] FSM_EXT_MAT_XX_1<br>bit[1:0] FSM_EXT_XX |  |
| `FSM_EXT_MATRIX_XY_L` | C8h | R/W | 00000000 | bit[7:0] FSM_EXT_MAT_XY | 外部传感器变换矩阵寄存器 |
| `FSM_EXT_MATRIX_XY_H` | C9h | R/W | 00000000 | bit[7:2] FSM_EXT_XY_1<br>bit[1:0] FSM_EXT_XY |  |
| `FSM_EXT_MATRIX_XZ_L` | CAh | R/W | 00000000 | bit[7:0] FSM_EXT_MAT_XZ | 外部传感器变换矩阵寄存器 |
| `FSM_EXT_MATRIX_XZ_H` | CBh | R/W | 00000000 | bit[7:2] FSM_EXT_MAT_XZ_1<br>bit[1:0] FSM_EXT_MAT_XZ |  |
| `FSM_EXT_MATRIX_YY_L` | CCh | R/W | 00000000 | bit[7:0] FSM_EXT_MAT_YY | 外部传感器变换矩阵寄存器 |
| `FSM_EXT_MATRIX_YY_H` | CDh | R/W | 00111100 | bit[7:2] FSM_EXT_MAT_YY_1<br>bit[1:0] FSM_EXT_MAT_YY |  |
| `FSM_EXT_MATRIX_YZ_L` | CEh | R/W | 00000000 | bit[7:0] FSM_EXT_MAT_YZ | 外部传感器变换矩阵寄存器 |
| `FSM_EXT_MATRIX_YZ_H` | CFh | R/W | 00000000 | bit[7:2] FSM_EXT_MAT_YZ_1<br>bit[1:0] FSM_EXT_MAT_YZ |  |
| `FSM_EXT_MATRIX_ZZ_L` | D0h | R/W | 00000000 | bit[7:0] FSM_EXT_MAT_ZZ | 外部传感器变换矩阵寄存器 |
| `PEDO_SC_DELTAT_L` | D0h | R/W | 00000000 | bit[7:0] PD_SC | 用于按增量时间进行步检测的时间周期寄存器 |
| `FSM_EXT_MATRIX_ZZ_H` | D1h | R/W | 00111100 | bit[7:2] FSM_EXT_MAT_ZZ_1<br>bit[1:0] FSM_EXT_MAT_ZZ |  |
| `PEDO_SC_DELTAT_H` | D1h | R/W | 00000000 | bit[7:2] PD_SC_1<br>bit[1:0] PD_SC |  |
| `EXT_CFG_A` | D4h | R/W | 00000101 | bit7 固定0(1)<br>bit[6:4] EXT_Y_AXIS<br>bit3 固定0(1)<br>bit[2:0] EXT_Z_AXIS | 外部传感器坐标（Z 轴和 Y 轴）旋转寄存器（r/w）。 |
| `EXT_CFG_B` | D5h | R/W | 00000010 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 固定0(1)<br>bit[2:0] EXT_X_AXIS | 外部传感器坐标（X 轴）旋转寄存器（r/w）。 |
| `MLC_EXT_SENSITIVITY_L` | E8h | R/W | 00000000 | bit[7:0] MLC_EXT_S | 机器学习内核的外部传感器灵敏度值寄存器 |
| `MLC_EXT_SENSITIVITY_H` | E9h | R/W | 00111100 | bit[7:2] MLC_EXT_S_1<br>bit[1:0] MLC_EXT_S |  |

### 嵌入高级功能页寄存器 · 位域明细与用法

| 寄存器 | 地址 | 位 | 字段 | 功能与用法 |
|---|---|---|---|---|
| `EXT_FORMAT` | 00h | 2 | `EXT_FORMAT_SEL` | 选择用于 FSM 和 MLC 处理的 AH / Qvar / 外部传感器数据格式。默认值：0（0：2 字节格式；1：3 字节格式） |
| `EXT_3BYTE_SENSITIVITY_L` | 02h | [7:0] | `EXT_3BYTE_S` |  |
| `EXT_3BYTE_SENSITIVITY_H` | 03h | [7:2] | `EXT_3BYTE_S_1` |  |
| `EXT_3BYTE_SENSITIVITY_H` | 03h | [1:0] | `EXT_3BYTE_S` |  |
| `EXT_3BYTE_OFFSET_XL` | 06h | [7:0] | `EXT_3BYTE_OFF` |  |
| `EXT_3BYTE_OFFSET_L` | 07h | [7:2] | `EXT_3BYTE_OFF_1` |  |
| `EXT_3BYTE_OFFSET_L` | 07h | [1:0] | `EXT_3BYTE_OFF` |  |
| `EXT_3BYTE_OFFSET_H` | 08h | [7:4] | `EXT_3BYTE_OFF_2` |  |
| `EXT_3BYTE_OFFSET_H` | 08h | [3:0] | `EXT_3BYTE_OFF_1` |  |
| `SFLP_GAME_GBIASX_L` | 6Eh | [7:0] | `GAME_GBIASX` |  |
| `SFLP_GAME_GBIASX_H` | 6Fh | [7:2] | `GAME_GBIASX_1` |  |
| `SFLP_GAME_GBIASX_H` | 6Fh | [1:0] | `GAME_GBIASX` |  |
| `SFLP_GAME_GBIASY_L` | 70h | [7:0] | `GAME_GBIASY` |  |
| `SFLP_GAME_GBIASY_H` | 71h | [7:2] | `GAME_GBIASY_1` |  |
| `SFLP_GAME_GBIASY_H` | 71h | [1:0] | `GAME_GBIASY` |  |
| `SFLP_GAME_GBIASZ_L` | 72h | [7:0] | `GAME_GBIASZ` |  |
| `SFLP_GAME_GBIASZ_H` | 73h | [7:2] | `GAME_GBIASZ_1` |  |
| `SFLP_GAME_GBIASZ_H` | 73h | [1:0] | `GAME_GBIASZ` |  |
| `FSM_LC_TIMEOUT_L` | 7Ah | [7:0] | `FSM_LC_TIMEOUT` |  |
| `FSM_LC_TIMEOUT_H` | 7Bh | [7:2] | `FSM_LC_TIMEOUT1` |  |
| `FSM_LC_TIMEOUT_H` | 7Bh | [1:0] | `FSM_LC_TIMEOUT` |  |
| `FSM_PROGRAMS` | 7Ch | [7:0] | `FSM_N_PROG` |  |
| `FSM_START_ADD_L` | 7Eh | [7:0] | `FSM_START` |  |
| `FSM_START_ADD_H` | 7Fh | [7:2] | `FSM_START1` |  |
| `FSM_START_ADD_H` | 7Fh | [1:0] | `FSM_START` |  |
| `PEDO_CMD_REG` | 83h | 3 | `CARRY_COUNT_EN` | 当用户希望仅在计数溢出事件产生中断时置为 1。 |
| `PEDO_CMD_REG` | 83h | 2 | `FP_REJECTION_EN` | 使能假阳性抑制功能 |
| `PEDO_DEB_STEPS_CONF` | 84h | [7:0] | `DEB_STEP` |  |
| `FSM_EXT_SENSITIVITY_L` | BAh | [7:0] | `FSM_EXT_S` |  |
| `FSM_EXT_SENSITIVITY_H` | BBh | [7:2] | `FSM_EXT_S_1` |  |
| `FSM_EXT_SENSITIVITY_H` | BBh | [1:0] | `FSM_EXT_S` |  |
| `FSM_EXT_OFFX_L` | C0h | [7:0] | `FSM_EXT_OFFX` |  |
| `FSM_EXT_OFFX_H` | C1h | [7:2] | `FSM_EXT_OFFX_1` |  |
| `FSM_EXT_OFFX_H` | C1h | [1:0] | `FSM_EXT_OFFX` |  |
| `FSM_EXT_OFFY_L` | C2h | [7:0] | `FSM_EXT_OFFY` |  |
| `FSM_EXT_OFFY_H` | C3h | [7:2] | `FSM_EXT_OFFY_1` |  |
| `FSM_EXT_OFFY_H` | C3h | [1:0] | `FSM_EXT_OFFY` |  |
| `FSM_EXT_OFFZ_L` | C4h | [7:0] | `FSM_EXT_OFFZ` |  |
| `FSM_EXT_OFFZ_H` | C5h | [7:2] | `FSM_EXT_OFFZ_1` |  |
| `FSM_EXT_OFFZ_H` | C5h | [1:0] | `FSM_EXT_OFFZ` |  |
| `FSM_EXT_MATRIX_XX_L` | C6h | [7:0] | `FSM_EXT_MAT_XX` |  |
| `FSM_EXT_MATRIX_XX_H` | C7h | [7:2] | `FSM_EXT_MAT_XX_1` |  |
| `FSM_EXT_MATRIX_XX_H` | C7h | [1:0] | `FSM_EXT_XX` |  |
| `FSM_EXT_MATRIX_XY_L` | C8h | [7:0] | `FSM_EXT_MAT_XY` |  |
| `FSM_EXT_MATRIX_XY_H` | C9h | [7:2] | `FSM_EXT_XY_1` |  |
| `FSM_EXT_MATRIX_XY_H` | C9h | [1:0] | `FSM_EXT_XY` |  |
| `FSM_EXT_MATRIX_XZ_L` | CAh | [7:0] | `FSM_EXT_MAT_XZ` |  |
| `FSM_EXT_MATRIX_XZ_H` | CBh | [7:2] | `FSM_EXT_MAT_XZ_1` |  |
| `FSM_EXT_MATRIX_XZ_H` | CBh | [1:0] | `FSM_EXT_MAT_XZ` |  |
| `FSM_EXT_MATRIX_YY_L` | CCh | [7:0] | `FSM_EXT_MAT_YY` |  |
| `FSM_EXT_MATRIX_YY_H` | CDh | [7:2] | `FSM_EXT_MAT_YY_1` |  |
| `FSM_EXT_MATRIX_YY_H` | CDh | [1:0] | `FSM_EXT_MAT_YY` |  |
| `FSM_EXT_MATRIX_YZ_L` | CEh | [7:0] | `FSM_EXT_MAT_YZ` |  |
| `FSM_EXT_MATRIX_YZ_H` | CFh | [7:2] | `FSM_EXT_MAT_YZ_1` |  |
| `FSM_EXT_MATRIX_YZ_H` | CFh | [1:0] | `FSM_EXT_MAT_YZ` |  |
| `FSM_EXT_MATRIX_ZZ_L` | D0h | [7:0] | `FSM_EXT_MAT_ZZ` |  |
| `PEDO_SC_DELTAT_L` | D0h | [7:0] | `PD_SC` |  |
| `FSM_EXT_MATRIX_ZZ_H` | D1h | [7:2] | `FSM_EXT_MAT_ZZ_1` |  |
| `FSM_EXT_MATRIX_ZZ_H` | D1h | [1:0] | `FSM_EXT_MAT_ZZ` |  |
| `PEDO_SC_DELTAT_H` | D1h | [7:2] | `PD_SC_1` |  |
| `PEDO_SC_DELTAT_H` | D1h | [1:0] | `PD_SC` |  |
| `EXT_CFG_A` | D4h | [6:4] | `EXT_Y_AXIS` |  |
| `EXT_CFG_A` | D4h | [2:0] | `EXT_Z_AXIS` |  |
| `EXT_CFG_B` | D5h | [2:0] | `EXT_X_AXIS` |  |
| `MLC_EXT_SENSITIVITY_L` | E8h | [7:0] | `MLC_EXT_S` |  |
| `MLC_EXT_SENSITIVITY_H` | E9h | [7:2] | `MLC_EXT_S_1` |  |
| `MLC_EXT_SENSITIVITY_H` | E9h | [1:0] | `MLC_EXT_S` |  |

## 4. 传感器集线器寄存器

| 寄存器 | 地址 | 类型 | 默认 | 位域 | 说明 |
|---|---|---|---|---|---|
| `SENSOR_HUB_1` | 02h | R | output | bit[7:0] SENSORHUB1 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_2` | 03h | R | output | bit[7:0] SENSORHUB2 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_3` | 04h | R | output | bit[7:0] SENSORHUB3 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_4` | 05h | R | output | bit[7:0] SENSORHUB4 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_5` | 06h | R | output | bit[7:0] SENSORHUB5 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_6` | 07h | R | output | bit[7:0] SENSORHUB6 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_7` | 08h | R | output | bit[7:0] SENSORHUB7 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_8` | 09h | R | output | bit[7:0] SENSORHUB8 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_9` | 0Ah | R | output | bit[7:0] SENSORHUB9 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_10` | 0Bh | R | output | bit[7:0] SENSORHUB10 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_11` | 0Ch | R | output | bit[7:0] SENSORHUB11 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_12` | 0Dh | R | output | bit[7:0] SENSORHUB12 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_13` | 0Eh | R | output | bit[7:0] SENSORHUB13 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_14` | 0Fh | R | output | bit[7:0] SENSORHUB14 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_15` | 10h | R | output | bit[7:0] SENSORHUB15 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_16` | 11h | R | output | bit[7:0] SENSORHUB16 | 传感器集线器输出寄存器 |
| `SENSOR_HUB_17` | 12h | R | output | bit[7:0] SENSORHUB17 | 传感器集线器输出寄存器 |
| `MASTER_CONFIG` | 14h | R/W | 00000000 | bit7 RST_MASTER_REGS<br>bit6 WRITE_ONCE<br>bit5 START_CONFIG<br>bit4 PASS_THROUGH_MODE<br>bit3 固定0(1)<br>bit2 MASTER_ON<br>bit[1:0] AUX_SENS_ON | 主器件配置寄存器 |
| `SLV0_ADD` | 15h | R/W | 00000000 | bit[7:1] SLAVE0_ADD<br>bit0 RW | 第一个外部传感器（传感器 0）的 I²C 从器件地址寄存器 |
| `SLV0_SUBADD` | 16h | R/W | 00000000 | bit[7:0] SLAVE0_REG | 第一个外部传感器（传感器 0）的寄存器地址 |
| `SLV0_CONFIG` | 17h | R/W | 10000000 | bit[7:5] SHUB_ODR<br>bit4 固定0(1)<br>bit3 BATCH_EXT_SENS_0_EN<br>bit[2:0] SLAVE0_NUMOP | 第一个外部传感器（传感器 0）配置和传感器集线器设置寄存器 |
| `SLV1_ADD` | 18h | R/W | 00000000 | bit[7:1] SLAVE1_ADD<br>bit0 R | 第二个外部传感器（传感器 1）的 I²C 从器件地址寄存器 |
| `SLV1_SUBADD` | 19h | R/W | 00000000 | bit[7:0] SLAVE1_REG | 第二个外部传感器（传感器 1）的寄存器地址寄存器 |
| `SLV1_CONFIG` | 1Ah | R/W | 00010000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定1(2)<br>bit3 BATCH_EXT_SENS_1_EN<br>bit[2:0] SLAVE1_NUMOP | 第二个外部传感器（传感器 2）配置寄存器 |
| `SLV2_ADD` | 1Bh | R/W | 00000000 | bit[7:1] SLAVE2_ADD<br>bit0 R | 第三个外部传感器（传感器 2）的 I²C 从器件地址寄存器 |
| `SLV2_SUBADD` | 1Ch | R/W | 00000000 | bit[7:0] SLAVE2_REG | 第三个外部传感器（传感器 2）的寄存器地址寄存器 |
| `SLV2_CONFIG` | 1Dh | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 BATCH_EXT_SENS_2_EN<br>bit[2:0] SLAVE2_NUMOP | 第三个外部传感器（传感器 2）配置寄存器 |
| `SLV3_ADD` | 1Eh | R/W | 00000000 | bit[7:1] SLAVE3_ADD<br>bit0 R | 第四个外部传感器（传感器 3）的 I²C 从器件地址寄存器 |
| `SLV3_SUBADD` | 1Fh | R/W | 00000000 | bit[7:0] SLAVE3_REG | 第四个外部传感器（传感器 3）的寄存器地址 |
| `SLV3_CONFIG` | 20h | R/W | 00000000 | bit7 固定0(1)<br>bit6 固定0(1)<br>bit5 固定0(1)<br>bit4 固定0(1)<br>bit3 BATCH_EXT_SENS_3_EN<br>bit[2:0] SLAVE3_NUMOP | 第四个外部传感器（传感器 3）配置寄存器 |
| `DATAWRITE_SLV0` | 21h | R/W | 00000000 | bit[7:0] SLAVE0_DATAW | 要写入从器件寄存器的数据 |
| `STATUS_MASTER` | 22h | R | output | bit7 WR_ONCE_DONE<br>bit6 SLAVE3_NACK<br>bit5 SLAVE2_NACK<br>bit4 SLAVE1_NACK<br>bit3 SLAVE0_NACK<br>bit2 固定0<br>bit1 固定0<br>bit0 SENS_HUB_ENDOP | 传感器集线器源寄存器 |

### 传感器集线器寄存器 · 位域明细与用法

| 寄存器 | 地址 | 位 | 字段 | 功能与用法 |
|---|---|---|---|---|
| `SENSOR_HUB_1` | 02h | [7:0] | `SENSORHUB1` |  |
| `SENSOR_HUB_2` | 03h | [7:0] | `SENSORHUB2` |  |
| `SENSOR_HUB_3` | 04h | [7:0] | `SENSORHUB3` |  |
| `SENSOR_HUB_4` | 05h | [7:0] | `SENSORHUB4` |  |
| `SENSOR_HUB_5` | 06h | [7:0] | `SENSORHUB5` |  |
| `SENSOR_HUB_6` | 07h | [7:0] | `SENSORHUB6` |  |
| `SENSOR_HUB_7` | 08h | [7:0] | `SENSORHUB7` |  |
| `SENSOR_HUB_8` | 09h | [7:0] | `SENSORHUB8` |  |
| `SENSOR_HUB_9` | 0Ah | [7:0] | `SENSORHUB9` |  |
| `SENSOR_HUB_10` | 0Bh | [7:0] | `SENSORHUB10` |  |
| `SENSOR_HUB_11` | 0Ch | [7:0] | `SENSORHUB11` |  |
| `SENSOR_HUB_12` | 0Dh | [7:0] | `SENSORHUB12` |  |
| `SENSOR_HUB_13` | 0Eh | [7:0] | `SENSORHUB13` |  |
| `SENSOR_HUB_14` | 0Fh | [7:0] | `SENSORHUB14` |  |
| `SENSOR_HUB_15` | 10h | [7:0] | `SENSORHUB15` |  |
| `SENSOR_HUB_16` | 11h | [7:0] | `SENSORHUB16` |  |
| `SENSOR_HUB_17` | 12h | [7:0] | `SENSORHUB17` |  |
| `MASTER_CONFIG` | 14h | 7 | `RST_MASTER_REGS` | 复位主器件逻辑和输出寄存器。必须先置为 1，再置为 0。默认值：0 |
| `MASTER_CONFIG` | 14h | 6 | `WRITE_ONCE` | 仅在第一个传感器集线器周期执行从器件 0 写操作。默认值：0（0：每个传感器集线器周期均执行写操作；1：仅在第一个传感器集线器周期执行写操作） |
| `MASTER_CONFIG` | 14h | 5 | `START_CONFIG` | 传感器集线器触发信号选择。默认值：0（0：传感器集线器触发信号为加速度计/陀螺仪数据就绪；1：传感器集线器触发信号为来自 INT2 引脚的外部信号） |
| `MASTER_CONFIG` | 14h | 4 | `PASS_THROUGH_MODE` | I²C 接口直通。默认值：0（0：直通禁用；1：直通使能，主 I²C 线路与传感器集线器线路短接） |
| `MASTER_CONFIG` | 14h | 2 | `MASTER_ON` | 使能传感器集线器 I²C 主器件。默认值：0（0：传感器集线器的 I²C 主器件禁用；1：传感器集线器的 I²C 主器件使能） |
| `MASTER_CONFIG` | 14h | [1:0] | `AUX_SENS_ON` |  |
| `SLV0_ADD` | 15h | [7:1] | `SLAVE0_ADD` |  |
| `SLV0_SUBADD` | 16h | [7:0] | `SLAVE0_REG` |  |
| `SLV0_CONFIG` | 17h | [7:5] | `SHUB_ODR` |  |
| `SLV0_CONFIG` | 17h | 3 | `BATCH_EXT_SENS_0_EN` | 使能第一个从器件的 FIFO 数据批处理。默认值：0 |
| `SLV0_CONFIG` | 17h | [2:0] | `SLAVE0_NUMOP` |  |
| `SLV1_ADD` | 18h | [7:1] | `SLAVE1_ADD` |  |
| `SLV1_SUBADD` | 19h | [7:0] | `SLAVE1_REG` |  |
| `SLV1_CONFIG` | 1Ah | 3 | `BATCH_EXT_SENS_1_EN` | 使能第二个从器件的 FIFO 数据批处理。默认值：0 |
| `SLV1_CONFIG` | 1Ah | [2:0] | `SLAVE1_NUMOP` |  |
| `SLV2_ADD` | 1Bh | [7:1] | `SLAVE2_ADD` |  |
| `SLV2_SUBADD` | 1Ch | [7:0] | `SLAVE2_REG` |  |
| `SLV2_CONFIG` | 1Dh | 3 | `BATCH_EXT_SENS_2_EN` | 使能第三个从器件的 FIFO 数据批处理。默认值：0 |
| `SLV2_CONFIG` | 1Dh | [2:0] | `SLAVE2_NUMOP` |  |
| `SLV3_ADD` | 1Eh | [7:1] | `SLAVE3_ADD` |  |
| `SLV3_SUBADD` | 1Fh | [7:0] | `SLAVE3_REG` |  |
| `SLV3_CONFIG` | 20h | 3 | `BATCH_EXT_SENS_3_EN` | 使能第四个从器件的 FIFO 数据批处理。默认值：0 |
| `SLV3_CONFIG` | 20h | [2:0] | `SLAVE3_NUMOP` |  |
| `DATAWRITE_SLV0` | 21h | [7:0] | `SLAVE0_DATAW` |  |
| `STATUS_MASTER` | 22h | 7 | `WR_ONCE_DONE` | 当 MASTER_CONFIG (14h) 中的 WRITE_ONCE 位配置为 1 时，若对从器件 0 的写操作已执行并完成，则该位被置为 1。默认值：0 |
| `STATUS_MASTER` | 22h | 6 | `SLAVE3_NACK` | 若从器件 3 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER` | 22h | 5 | `SLAVE2_NACK` | 若从器件 2 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER` | 22h | 4 | `SLAVE1_NACK` | 若从器件 1 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER` | 22h | 3 | `SLAVE0_NACK` | 若从器件 0 通信中出现未应答，则该位被置为 1。默认值：0 |
| `STATUS_MASTER` | 22h | 0 | `SENS_HUB_ENDOP` | 传感器集线器通信状态。默认值：0（0：传感器集线器通信未结束；1：传感器集线器通信已结束） |

## 5. 典型使用流程（置位什么 → 得到什么功能）

### 5.1 通信自检

```c
// 读 WHO_AM_I(0Fh)，必须等于 0x70
LSM6DSV16X_ReadReg(LSM6DSV16X_WHO_AM_I, &v);   // v == 0x70 说明 SPI 时序/CS/MSB/8 位帧都正确
```

### 5.2 基础配置（以 30 Hz、±2 g、±2000 dps 为例）

```c
LSM6DSV16X_WriteReg(LSM6DSV16X_CTRL3, 0x44);      // BDU=1（读 MSB 前不更新）+ IF_INC=1（多字节地址自增）
LSM6DSV16X_WriteReg(LSM6DSV16X_CTRL8, LSM6DSV16X_XL_FS_2G);          // FS_XL = ±2 g
LSM6DSV16X_WriteReg(LSM6DSV16X_CTRL6, LSM6DSV16X_GYRO_FS_2000DPS);   // FS_G  = ±2000 dps
LSM6DSV16X_WriteReg(LSM6DSV16X_CTRL1, LSM6DSV16X_XL_OP_MODE_HP | LSM6DSV16X_XL_ODR_30H);
LSM6DSV16X_WriteReg(LSM6DSV16X_CTRL2, LSM6DSV16X_GYRO_OP_MODE_HP | LSM6DSV16X_GYRO_ODR_30H);
// 读数据：22h~2D 一次突发 12 字节（陀螺仪 XYZ + 加速度计 XYZ）
LSM6DSV16X_ReadRegs(LSM6DSV16X_OUTX_L_G, buf, 12);
```

轮询数据就绪：`STATUS_REG(1Eh).XLDA`(bit0) / `.GDA`(bit1) 变为 1 表示有新数据。

### 5.3 中断输出（把数据就绪/事件引到 INT1）

```c
LSM6DSV16X_WriteReg(LSM6DSV16X_INT1_CTRL, 0x03);  // INT1_DRDY_XL(bit0) + INT1_DRDY_G(bit1)
```

### 5.4 FIFO 批处理

```c
LSM6DSV16X_WriteReg(LSM6DSV16X_FIFO_CTRL3, LSM6DSV16X_FIFO_BDR_XL_30H | LSM6DSV16X_FIFO_BDR_GY_30H);
LSM6DSV16X_WriteReg(LSM6DSV16X_FIFO_CTRL4, LSM6DSV16X_FIFO_MODE_CONTINUOUS);   // 满后覆盖最旧
// 每次读 1 个字：78h 起 7 字节 = 1 字节 tag + 6 字节数据
LSM6DSV16X_ReadRegs(LSM6DSV16X_FIFO_DATA_OUT_TAG, w, 7);
```

未读字数 = `FIFO_STATUS1(1Bh)` + `FIFO_STATUS2(1Ch).bit0`（9 位）。

### 5.5 启用 FSM / MLC（嵌入式功能通用步骤）

```c
// 1) 用 MEMS Studio 生成的配置写入对应空间（见第 0 节的分页流程）
// 2) 使能功能
LSM6DSV16X_WriteReg(LSM6DSV16X_FUNC_CFG_ACCESS, 0x80);            // 进嵌入功能空间
LSM6DSV16X_WriteReg(LSM6DSV16X_EMB_FUNC_EN_B,
                    (1 << LSM6DSV16X_EMB_FUNC_FSM_EN_SHIFTS));     // FSM_EN = 1
//   同时用 MLC 时再或上 MLC_EN；MLC 在 FSM 之前跑则置 EMB_FUNC_EN_A 的 MLC_BEFORE_FSM_EN
// 3) 初始化，并等 ENDOP 置 1
LSM6DSV16X_WriteReg(LSM6DSV16X_EMB_FUNC_INIT_B,
                    (1 << LSM6DSV16X_EMB_FUNC_FSM_INIT_SHIFTS));   // FSM_INIT = 1
do { LSM6DSV16X_ReadReg(LSM6DSV16X_EMB_FUNC_EXEC_STATUS, &v); }
while ((v & (1 << LSM6DSV16X_EMB_FUNC_EMB_FUNC_ENDOP_SHIFTS)) == 0);
LSM6DSV16X_WriteReg(LSM6DSV16X_FUNC_CFG_ACCESS, 0x00);            // 退出嵌入功能空间
// 4) 读结果
LSM6DSV16X_ReadReg(LSM6DSV16X_MLC1_SRC, &mlc);   // 决策树输出 = 你定义的类别码
LSM6DSV16X_ReadReg(LSM6DSV16X_FSM_OUTS1, &fsm);  // 状态机当前输出
```

> 宏名以 `lsm6dsv16x_reg.h` 实际定义为准（上例中 `EMB_FUNC_FSM_EN_SHIFTS`
> 由块名 EMB_FUNC + 字段 FSM_EN 组成，可用编辑器跳转确认）。

