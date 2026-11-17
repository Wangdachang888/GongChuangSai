# GongChuangSai 项目状态

## 1. 项目目标

本项目目标是为 STM32F407/C30D 主控的小车提供可维护的比赛固件，保留四轮麦克纳姆底盘、编码器里程计、ICM20948 姿态、OpenMV 视觉通信、二维码路线、数字圆环微调、夹取机构和 D36/步进调试能力。正式工程是 `MDK-ARM/GongChuangSai.uvprojx`，历史工程和实验包只用于追溯。

电脑端验收通过只表示协议、状态机和静态结构具备可测试性。Keil 编译、下载、OpenMV 通信、底盘运动和机械动作仍必须在目标设备上单独验收。

## 2. 当前 Git 与工具链

- 当前开发分支：`refactor/architecture-v2`
- 重构前稳定基线：`79de1dd`
- 当前工作区状态：以 `git status --short` 的实时结果为准。
- 目标工程：`MDK-ARM/GongChuangSai.uvprojx`
- 配置目录：`Config/`，包含 `app_mode.h` 和 `race_config.h`。
- 主机测试编译器：`D:\minGW\mingw64\bin\gcc.exe`
- 已发现主机测试：`tests/race/run.ps1`、`tests/ulog/run.ps1`
- 已定位 Keil/Arm 工具链：`D:\keil\UV4\UV4.exe`、`D:\keil\ARM\ARMCC\Bin\armcc.exe`、`D:\keil\ARM\ARMCC\Bin\armlink.exe`；注册表 MDK 产品版本为 `V5.24a`，实际 uVision 构建日志使用 MDK-ARM Plus `V5.37.0.0` 工具链组件。
- 不自动合并 `main`、不烧录；经授权的普通推送只允许当前开发分支，并按 `AGENTS.md` 的规则完成。

## 版本同步状态

- GitHub 远程：`git@github.com:Wangdachang888/GongChuangSai.git`。
- 授权自动推送分支：`refactor/architecture-v2`。
- 本次自动同步机制启用前最后确认：本地和远程开发分支均为 `4f26e96699d1ba9abd4419461ee924993a5a24c2`，状态为 `SYNC PASS`。
- 自动同步脚本首次真实成功提交并推送后确认：提交 `673abab4f2da835267a08b126a917e791cae741b` 的本地和远程 SHA 一致，状态为 `SYNC PASS`。
- 自动同步脚本输出修复提交 `a00371d5e52ff935dfbb86bae18da2566149049f` 已完成普通推送；其本地和远程 SHA 一致，状态为 `SYNC PASS`。
- 当前未推送提交和远程 SHA：必须通过 `git rev-parse HEAD`、`git ls-remote origin refs/heads/refactor/architecture-v2` 动态查询，不在本状态快照中固定当前提交自身的 SHA。
- 自动同步脚本：`tools/commit_and_push.ps1`；它只接受明确文件列表，不会默认暂存整个工作区。
- 当前已知同步异常：无；若推送失败，必须保留本地提交并记录 `SYNC FAIL`，不得强制推送。
- 最近完成阶段：长期维护规则、架构文档、硬件验收记录、工程引用清理、GitHub 开发分支首次同步和自动同步脚本成功验证。

## 3. 硬件与引脚事实

- 主控：STM32F407，底盘板为 C30D 资源布局。
- 四轮电机使用 TIM1/TIM9/TIM10/TIM11 PWM 和 TIM2/TIM3/TIM4/TIM5 编码器，具体方向、轮径、轮距和 PID 以 `Hardware/app_runtime.c` 为准。
- IMU 使用现有 ICM20948 驱动和应用层姿态解算；比赛运行要求 IMU 初始化和数据持续有效。
- USART3 的 PD8/PD9 用于 OpenMV/视觉协议；非比赛模式可以接入 D36 文本调试命令。
- PB14/PB15 是 TIM12 舵机通道；PC7 是 `SERVO3` 转盘舵机输出。
- 升降步进使用 TIM8_CH1 的 PC6 脉冲，PC8/PC9 是方向输出；TIM8 的周期和通道配置由 `GongChuangSai.ioc` 与 `tim.c` 共同约束。
- 真实限位、零位、驱动细分、负载、电源和机械干涉情况没有由当前电脑测试证明。

## 4. 当前软件架构

启动入口在 `Core/Src/main.c`：HAL、GPIO、定时器和 USART3 初始化后调用 `AppRuntime_Init()`，随后启动 TIM6 中断并在主循环调用 `AppRuntime_Poll()`。唯一的 TIM6 周期回调也在该文件中。

`Config/` 负责运行模式和比赛参数；`Control/Motor.c` 负责轮毂电机速度环，`Hardware/app_runtime.c` 负责模式选择、设备初始化和实时桥接；`Hardware/app_race.c` 负责比赛协调、按键、视觉、路线、圆环和夹取任务之间的互斥；`Control/mecanum_chassis.c` 是底盘运动学、里程计和位置 PID 的唯一执行层；`Hardware/vision.c` 负责协议、帧缓存和视觉结果；`Hardware/app_ring_align.c` 负责圆环标定换算和闭环移动；`Hardware/app_pick_fsm.c` 负责舵机、升降步进和夹取流程。

USART3 的 HAL 回调唯一入口是 `Hardware/bsp_uart_router.c`。比赛模式转给 `vision.c`，其他模式按配置转给 D36 命令模块。UART 中断不执行阻塞发送、底盘控制或机械动作。

## 5. 功能状态

| 功能 | 当前状态 | 证据或限制 |
| --- | --- | --- |
| HAL 启动与 TIM6 调度 | 已检查 | 主机静态检查；实机未运行 |
| 四轮电机、编码器、里程计 | 保留 | 参数未改；实机未验收 |
| IMU 初始化、偏置和航向反馈 | 保留 | 比赛状态机会检查有效性；实机零漂未验证 |
| 二维码路线与启停区 | 主机通过 | `tests/race`；OpenMV 实机未验收 |
| OpenMV V4、旧 `#SEQ`、二维码协议 | 主机通过 | `tests/race` 协议覆盖 |
| 数字 1/2/3 圆环接口 | 安全门已保留 | 当前四个圆环标定矩阵参数全为零，自动移动被禁止 |
| 圆环位置微调与握手 | 主机通过 | 真实相机、标定方向和停稳未验收 |
| 夹取状态机 | 代码保留 | 机械动作、舵机方向、升降步数未实机验收 |
| PC7 转盘舵机 | 代码保留 | 目标硬件动作未验收 |
| D36/步进调试模式 | 代码保留 | 需与比赛模式资源隔离，实机未验收 |
| ULog | 主机测试通过 | SD 卡路径、容量和持续写入未实机验收 |

## 6. 当前默认配置

- `APP_RUN_MODE` 默认是 `APP_MODE_RACE`。
- `RACE_PICK_ENABLE` 为启用状态。
- `RACE_PICK_DIRECT_KEY_TEST` 当前为 `1`，比赛固件默认仍走夹取直接按键测试入口；这不等于完整比赛任务链已经实机接通。
- `RACE_PICK_TURNTABLE_ENABLE` 为启用状态，PC7 作为转盘舵机使用。
- 圆环 `J00/J01/J10/J11` 当前全为 `0.0f`，因此未完成标定前不会发出圆环底盘移动目标。

## 7. 最近工作

近期重构将视觉协议与圆环运动控制拆开，新增 `Hardware/app_ring_align.c/.h`，保留 V4、旧 `#SEQ`、二维码和 OpenMV 兼容入口；同时保留底盘 PID、IMU、TIM6、步进、舵机、夹取和日志能力。历史实验资料已移入归档目录，主工程引用经过检查。

本轮长期维护工作新增根目录规则文件和长期状态文档，并已从 Keil 工程文件组移除仅含注释的 `main_example.c` 引用；历史文件本身仍保留。当前阶段将运行配置物理迁移到 `Config/`，不改变配置参数。

## 8. 当前任务与风险

当前任务是保持工程长期可交接、可回滚、可自动验收，并继续发现架构冲突。主要风险如下：

1. 当前环境无法执行 Keil/Arm 固件构建。
2. 圆环标定参数仍未测量，自动圆环流程安全禁止。
3. 机械动作均是开环步数/舵机角度，限位和实物误差没有软件闭环保证。
4. `app_runtime.c` 和 `app_race.c` 仍包含较多协调职责，后续拆分必须保持单一底盘控制权。
5. `Hardware/main_example.c` 是仅含注释的历史示例，已不再参与 Keil 构建；如果未来要删除历史文件，必须先确认归档可恢复。
6. 视觉和 D36 共用 USART3 资源，必须按编译模式保持唯一接收回调和唯一协议消费者。

## 9. 下一步优先级

1. 重新执行工程文件、回调、公开接口和阻塞路径静态检查。
2. 继续执行 Keil 命令行编译链接，并保存体积数据。
3. 测量并填写圆环像素到米制的标定矩阵，再进行低速实机验证。
4. 在断电检查和人工急停条件下逐项验证夹取、升降、转盘和底盘联动。
5. 完成 OpenMV、USART3、SD 卡日志和机械负载测试后再考虑合并稳定分支。

## 10. 测试与构建状态

- `tests/race/run.ps1`：`PASS`，覆盖路线、协议、圆环握手和比赛应用状态机。
- `tests/ulog/run.ps1`：`PASS`，覆盖正常、取消、超时、发送错误、格式错误、非有限值、手动和空指针场景。
- GCC `-std=c99 -Wall -Wextra -Werror`：随上述脚本通过。
- Keil/Arm 固件编译：`PASS`，ArmCC V5.06 update 5 (build 528)，`0 Error(s), 0 Warning(s)`。
- 真实烧录、OpenMV、底盘、圆环、夹取、舵机、步进和 SD 卡：`NOT_RUN`。

## 11. 给下一位 AI 的交接

先读取 `AGENTS.md`、本文件、`docs/ARCHITECTURE.md` 和 `docs/DECISIONS.md`，再执行 Git 状态检查。修改前不要改变已验证的硬件参数，不要在 UART 或 TIM6 中断里增加阻塞逻辑，不要创建第二个底盘 PID 或第二个 UART 回调。完成每个独立修改后运行相关测试、执行 `git diff --check`、更新长期文档并在当前开发分支提交。所有涉及真实机械和视觉的结论都必须注明尚未实机验证。

## 12. Config 迁移最终验收（2026-11-17）

- `tests/race/run.ps1`：`PASS`。覆盖路线、位姿变换、停留/速度、取消锁存、时间回绕、故障、严格协议、流恢复、UART 扫描门控、旧帧处理、复位命令、IRQ 保留、溢出/错误/设防失败、环形 ACK/会话、重复步拒绝、标定移动、稳定 `STEP_DONE`、`ARRIVED/IDLE` 和比赛状态机。
- `tests/ulog/run.ps1`：`PASS`。覆盖正常、取消、超时、发送错误、格式错误、非有限值、手动和空指针日志场景，以及流程完成、启动复位和取消路径。
- Keil 工程静态检查：`PASS`。`MDK-ARM/GongChuangSai.uvprojx` 包含 76 个文件引用，全部路径存在且无重复；`IncludePath` 包含 `../Config`；Keil 工程引用 `Config/app_mode.h` 和 `Config/race_config.h`，不再引用旧的 `Hardware/` 配置路径。
- Config 迁移阶段：`COMPLETE`。本次仅完成最终验证和验收记录，没有开始 Control、BSP、Services 或 App 的下一阶段迁移。
- Keil/Arm 固件编译：`PASS`，当前已定位并使用 UV4 与 ArmCC；本次 Config 验收未修改业务代码。
- 烧录、OpenMV 通信、底盘、圆环、夹取、舵机、步进和 SD 卡：`NOT_RUN`。

## 13. Keil 编译与自动测试验收（2026-11-17）

- 工程：`MDK-ARM/GongChuangSai.uvprojx`。
- 编译命令：`D:\keil\UV4\UV4.exe -b MDK-ARM\GongChuangSai.uvprojx -j0 -o <build-log>`。
- 编译结果：`PASS`；使用 ArmCC V5.06 update 5 (build 528)，生成 AXF 和 HEX，`0 Error(s), 0 Warning(s)`。
- 程序尺寸：`Code=54004`、`RO-data=8664`、`RW-data=356`、`ZI-data=9540`。
- `tests/race/run.ps1`：`PASS`。
- `tests/ulog/run.ps1`：`PASS`。
- 本轮没有修改业务代码、PID、硬件参数、引脚映射或控制算法；没有执行烧录和硬件在线调试。

## 14. Control 阶段迁移进度

- `Motor.c/h` 已使用 `git mv` 从 `Hardware/` 迁移到 `Control/`。
- Keil 工程已改为引用 `Control/Motor.c/h`，并将 `../Control` 加入 IncludePath；主机测试脚本同步加入 `-IControl`。
- 本单元保留全部函数接口、PID 实现、轮毂电机参数和 TIM6 调度，不修改控制算法。
- `mecanum_chassis.c/h` 已使用 `git mv` 从 `Hardware/` 迁移到 `Control/`；`nav.c/h` 尚未迁移。
