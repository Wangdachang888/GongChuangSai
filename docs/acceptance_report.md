# GongChuangSai 自动验收报告

## 版本信息

- 分支：`refactor/architecture-v2`
- 远端基线：`79de1dd`
- 验收前代码提交：`4aa031f`
- 工作区状态：本次文档提交前为 clean；提交后应再次执行 `git status --short`。
- GCC：`D:\minGW\mingw64\bin\gcc.exe`
- Keil/Arm 工具链：`UV4`、`armcc`、`armclang` 均未找到。

## 自动测试

| 项目 | 命令 | 结果 | 日志位置 |
| --- | --- | --- | --- |
| 比赛路线、UART、圆环、比赛应用状态机 | `.\tests\race\run.ps1` | PASS | 命令标准输出；脚本不生成持久日志。 |
| ULog 正常、取消、超时、发送错误、格式错误等 | `.\tests\ulog\run.ps1` | PASS | 命令标准输出；脚本不生成持久日志。 |
| GCC 严格编译 | 两个脚本内含 `-std=c99 -Wall -Wextra -Werror` | PASS | 同上。 |
| Keil 固件编译与链接 | 无可用命令 | BUILD SKIP | 工具链缺失。 |

## 静态验收

- PASS：`main.c` 只保留初始化、TIM6 启动、主循环和实时回调桥接。
- PASS：TIM6 周期回调唯一，1 ms 电机更新、10 ms 底盘位置环保持不变。
- PASS：`HAL_UART_RxCpltCallback`、`HAL_UART_TxCpltCallback`、
  `HAL_UART_ErrorCallback` 各只有 `bsp_uart_router.c` 的有效定义。
- PASS：没有额外的 TIM OC/PWM 用户回调定义。
- PASS：视觉模块不包含 `mecanum_chassis.h`，且不调用 `Mecanum_SetPositionMode()`。
- PASS：Keil 工程 76 个文件引用全部存在，无重复引用；仅含注释的 `main_example.c` 已移出工程文件组。
- PASS：旧 `#SEQ`、二维码 `RaceVision_*` 和 V4 协议入口保留。
- PASS：圆环 session/step、重复帧、停稳、`STEP_DONE`、`ARRIVED`、`IDLE` 有主机测试。
- PASS：全零标定矩阵不会让圆环闭环写入实际运动目标。

## 功能回归范围

已保留并通过主机接口编译验证：麦克纳姆底盘、编码器/里程计、IMU、位置 PID、
路线状态机、二维码任务、夹取状态机、步进、舵机、日志和非比赛调试模式。主机
测试不能证明实际电机方向、负载、串口电平、相机噪声或机械干涉。

## 实机 SKIP 清单

- STM32F407 Keil 编译、链接、下载。
- 四轮方向、编码器计数、IMU 姿态和底盘位置精度。
- USART3 与 OpenMV 的真实电平和时序。
- 数字 1/2/3 圆环标定、微调方向、惯性和停稳时间。
- 夹爪、升降、旋转机构和故障停车的真实动作。

## 回退方式

保留 `origin/main` 的 `79de1dd` 基线，且所有改动位于
`refactor/architecture-v2`。验收前不要合并到 `main`，需要回退时使用 Git 历史
查看基线或切换到该分支的稳定提交，不执行强制重写历史。
