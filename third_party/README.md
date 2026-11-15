# 第三方依赖

## CMSIS-DSP

本地来源为 `E:/Software/Develop/RM/Wheel_leg/PYRo-wl/third_party/CMSIS-DSP`。
来源仓库提交为 `356f5020e5b5e68b9719095a7d5fe39910aebfab`，导入内容按当前工作树文件做哈希固定。

本次导入 `Include`、`PrivateInclude`、`Source` 和入口 `CMakeLists.txt`，用于 Matrix、KF 与 VQF 的矩阵和四元数运算。
许可证全文取自当前 CubeMX CMSIS 包的 Apache-2.0 许可证，位于 `CMSIS-DSP/LICENSE.txt`。
来源、文件数、字节数、SHA-256 汇总方式见 [dependencies.json](dependencies.json)。

`third_party/CMakeLists.txt` 创建 `CMSISDSP` 目标，公共头文件由该目标向调用方传递。
CMSIS-Core 编译头文件来自 `CubeMX/Drivers/CMSIS/Include`。
依赖使用 Cortex-M4F 编译参数，启用聚合构建，关闭 float16 内核。

后续更新该快照时，重新核对矩阵、分解、三角求解与四元数接口，并更新依赖哈希与构建记录。
