# GongChuangSai 重构验收报告

## 基本信息

- 分支：`refactor/architecture-v2`
- 重构前基线：`25a4b84`
- 最新功能提交：`37b1865`
- 当前验收状态：电脑端通过，等待人工实机验收。

## 文件与代码规模

统计范围为正式固件的 `Core/`、`Hardware/`、`IMU20948/` 下 `.c/.h` 文件：

| 指标 | 基线 | 重构后 | 说明 |
| --- | ---: | ---: | --- |
| 源文件数 | 73 | 75 | 增加独立 `app_ring_align.c/.h`，不是删除驱动。 |
| 源码行数 | 17469 | 17613 | 圆环控制从通信模块拆出，增加了边界检查和握手逻辑。 |
| 固件编译体积 | 未取得 | 未取得 | 当前环境没有 Keil/Arm 编译工具链，不能伪报结果。 |

历史工程、日志、查看器和压缩包已移入归档目录；这是路径整理，不改变正式固件
依赖。Keil 工程当前包含 76 个文件引用，引用路径检查通过且没有重复引用；仅含注释的
`main_example.c` 已移出工程文件组。

## 模块变更

| 变更 | 结果 |
| --- | --- |
| 新增 `App/app_ring_align.c/.h` | 承担标定换算、限幅、位置目标、停稳和圆环握手。 |
| 精简 `Hardware/vision.c/.h` | 保留 V4、旧 `#SEQ`、二维码、位置缓存和圆环通信状态；移除旧运动控制耦合。 |
| 保留 `App/app_race.c` | 继续作为比赛应用协调入口，不把业务堆回 `main.c`。 |
| 保留 `Core/Src/main.c` | 未修改启动、HAL 和实时调度入口。 |
| 清理主机测试桩 | 删除已废弃的 `Vision_SetRingChassis`、`Vision_SetRingCalibration` 定义。 |
| 资产整理 | 历史工程、工具、日志、图片和说明文档分目录保留。 |

## 公共接口检查

- 通过：新增圆环模块已加入 `MDK-ARM/GongChuangSai.uvprojx`。
- 通过：`AppRingAlign_*` 的声明、定义和比赛应用调用关系由 `tests/race` 编译验证。
- 通过：视觉 `Vision_*` 和二维码 `RaceVision_*` 兼容入口由 UART 和应用测试覆盖。
- 通过：没有发现正式源码中的重复 `HAL_UART_RxCpltCallback`。
- 通过：没有发现重复 `HAL_TIM_PeriodElapsedCallback`；TIM6 周期回调仍只在
  `Core/Src/main.c` 定义。
- 通过：没有发现额外的 HAL OC/PWM 回调定义。

## 功能保留清单

- 四轮麦克纳姆电机、编码器、里程计、IMU 和位置 PID。
- TIM6 1 ms 电机更新、10 ms 底盘位置环。
- 启停区选择、按键启动、路线移动、转向、二维码扫描、取消和故障停止。
- OpenMV V4 协议和旧 `#SEQ` 协议兼容。
- 数字 1、2、3 圆环识别接口、位置微调、停稳判断、`STEP_DONE`/`ARRIVED` 握手。
- 舵机、升降步进、夹取状态机和原有 D36/非比赛调试模式。
- ULog 日志路径和比赛运行模式。

## 自动化测试

| 测试 | 结果 |
| --- | --- |
| `tests/race/run.ps1`：路线、协议、圆环、比赛应用状态机 | PASS |
| `tests/ulog/run.ps1`：正常、提前完成、取消、超时、发送错误、格式错误、非有限值、手动和空指针场景 | PASS |
| GCC 编译选项：`-std=c99 -Wall -Wextra -Werror` | PASS |
| 圆环标定矩阵全零时禁止自动运动 | PASS（配置安全门） |
| Keil/Arm 固件编译和链接 | BUILD SKIP |

`BUILD SKIP` 的原因是当前环境未找到 `UV4`、`armcc` 或 `armclang`。主机 GCC 测试
不能替代 Keil 编译、下载和实机运行。

## 尚未完成的实机验收

- 用真实 STM32F407 工程完成 Keil 编译、链接和烧录。
- 使用实机 OpenMV、USART3 和圆环标定数据验证协议时序。
- 标定 `RACE_RING_J00_PX_PER_M` 到 `RACE_RING_J11_PX_PER_M` 后验证三种圆环的
  微调方向、到位阈值、停稳时间和超时行为。
- 验证夹取机构、升降步进、转盘和底盘在同一比赛流程中的机械干涉与急停行为。

## Git 状态

最终提交前应执行：

```powershell
git status --short
git log --oneline -5
```

本报告不执行合并、不修改 `main`，人工验收完成后再决定是否合并。

