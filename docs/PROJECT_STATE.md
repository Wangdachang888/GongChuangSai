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
- 四轮电机使用 TIM1/TIM9/TIM10/TIM11 PWM 和 TIM2/TIM3/TIM4/TIM5 编码器，具体方向、轮径、轮距和 PID 以 `App/app_runtime.c` 为准。
- IMU 使用现有 ICM20948 驱动和应用层姿态解算；比赛运行要求 IMU 初始化和数据持续有效。
- USART3 的 PD8/PD9 用于 OpenMV/视觉协议；非比赛模式可以接入 D36 文本调试命令。
- PB14/PB15 是 TIM12 舵机通道；PC7 是 `SERVO3` 转盘舵机输出。
- 升降步进使用 TIM8_CH1 的 PC6 脉冲，PC8/PC9 是方向输出；TIM8 的周期和通道配置由 `GongChuangSai.ioc` 与 `tim.c` 共同约束。
- 真实限位、零位、驱动细分、负载、电源和机械干涉情况没有由当前电脑测试证明。

## 4. 当前软件架构

启动入口在 `Core/Src/main.c`：HAL、GPIO、定时器和 USART3 初始化后调用 `AppRuntime_Init()`，随后启动 TIM6 中断并在主循环调用 `AppRuntime_Poll()`。唯一的 TIM6 周期回调也在该文件中。

`Config/` 负责运行模式和比赛参数；`Control/Motor.c` 负责轮毂电机速度环，`Control/mecanum_chassis.c` 负责底盘运动学、里程计和位置 PID，`Control/nav.c` 负责导航辅助；`App/app_runtime.c` 负责模式选择、设备初始化和实时桥接；`App/app_race.c` 负责比赛协调、按键、视觉、路线、圆环和夹取任务之间的互斥；`Hardware/vision.c` 负责协议、帧缓存和视觉结果；`App/app_ring_align.c` 负责圆环标定换算和闭环移动；`App/app_pick_fsm.c` 负责舵机、升降步进和夹取流程。

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

近期重构将视觉协议与圆环运动控制拆开，新增 `App/app_ring_align.c/.h`，保留 V4、旧 `#SEQ`、二维码和 OpenMV 兼容入口；同时保留底盘 PID、IMU、TIM6、步进、舵机、夹取和日志能力。历史实验资料已移入归档目录，主工程引用经过检查。

本轮长期维护工作新增根目录规则文件和长期状态文档，并已从 Keil 工程文件组移除仅含注释的 `main_example.c` 引用；历史文件本身仍保留。当前阶段将运行配置物理迁移到 `Config/`，不改变配置参数。

## 8. 当前任务与风险

当前任务是保持工程长期可交接、可回滚、可自动验收，并继续发现架构冲突。主要风险如下：

1. Keil/Arm 固件构建已可在当前环境执行，但构建通过不等于烧录和实机验收通过。
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
- `mecanum_chassis.c/h` 已使用 `git mv` 从 `Hardware/` 迁移到 `Control/`。
- `nav.c/h` 已使用 `git mv` 从 `Hardware/` 迁移到 `Control/`；Control 阶段的三个指定模块物理迁移完成。
- Control 阶段：`COMPLETE`。本阶段仅迁移 `Motor.c/h`、`mecanum_chassis.c/h`、`nav.c/h`，未开始 BSP 阶段。

## 15. Control 反向依赖解除（2026-11-17）

- `Control/nav.c` 不再包含 `app_imu.h`，`Nav_Update()` 改为接收 `NavImuSnapshot_t`；`Control/mecanum_chassis.c` 通过 `MecanumChassis_SetImuSnapshot()` 使用应用层注入的姿态快照。
- `Control/mecanum_chassis.c/h` 不再包含 `bsp_oled.h`，原有编码器和电机速度调试显示移至 `App/display_nav.c/h`。
- `App/app_runtime.c` 在 IMU 初始化和轮询后更新快照；航向零偏重置由 `App/app_square.c`、`App/display_nav.c` 发起，保留原有复位行为。
- `tests/race/run.ps1`：`PASS`；`tests/ulog/run.ps1`：`PASS`；`git diff --check`：`PASS`。
- 本批使用 `D:\keil\UV4\UV4.exe -b MDK-ARM\GongChuangSai.uvprojx -j0` 完整编译：`PASS`；ArmCC V5.06 update 5 (build 528)，`0 Error(s), 0 Warning(s)`，生成 AXF/HEX。未执行烧录和实机验收。

## 16. Hardware 职责整理与依赖优化（2026-11-18）

- `Hardware/bsp_ulog.c` 改为接收 `ULogAppSnapshot`，由 `App/app_runtime.c` 组装比赛、方形任务和 IMU 快照；日志公共轮询接口、43 通道含义和停止语义保持不变。
- `Hardware/vision.c/.h` 移除对 `App/race_fsm.h` 的依赖；二维码任务码校验保留在视觉协议模块内部，`RACE_TASK_CODE_SIZE` 兼容别名保留，OpenMV/V4、旧 `#SEQ` 和圆环协议未改变。
- `Hardware/bsp_d36_cmd.c` 改为通过 `D36CmdStepperOps` 回调表访问步进业务；文本命令、回复格式、USART3 和阻塞发送行为保持不变。步进配置宏移至 `Config/stepper_config.h`，消除 `Hardware/app_stepper_drive.c` 对 App 头文件的配置依赖。
- `Hardware/bsp_uart_router.c` 未修改，仍是正式工程唯一 HAL UART 回调入口；`app_imu.c/.h` 的自包含驱动依赖保留。
- `MDK-ARM/GongChuangSai.uvprojx` 已加入 `Config/stepper_config.h`，IncludePath 继续包含 `../Config`。
- `tests/race/run.ps1`：`PASS`；`tests/ulog/run.ps1`：`PASS`，包含比赛/非比赛模式、禁用日志和流程场景。
- Keil UV4/ArmCC 完整构建：`PASS`，ArmCC V5.06 update 5，`0 Error(s), 0 Warning(s)`；程序尺寸 `Code=54276`、`RO-data=8660`、`RW-data=356`、`ZI-data=9564`。
- 工程路径、重复引用、Hardware→App 依赖和 UART 回调唯一性静态检查：`PASS`；未改变 STM32F407、USART、TIM6/TIM8、GPIO、PWM、编码器、控制参数或协议文本。
- 实机烧录、OpenMV、D36、IMU、底盘、机械动作和 SD 卡持续写入：`NOT_RUN`。Vision 内部保留一份纯任务码校验逻辑，后续可在不引入层间依赖的前提下统一协议校验实现。

## 17. 网页调试台迁移衔接（2026-11-18）

- 新增 `docs/AI_HANDOFF.md`，记录网页调试台的文件清单、离线构建方式、轨迹/波形/回放能力、当前 ULog43 通道和迁移核对步骤。
- 网页源码位于 `viewer/`；通过 `node viewer/build.cjs` 生成根目录的 `kart_log_view_mecanum_v3.html` 和 `kart_log_view_mecanum_v4.html`。
- 网页按四轮编码器重建 `vx`、`vy` 和角速度，四轮轨迹由车体位姿和轮心偏移计算；这些结果是编码器运动学估计，不等于对地真实速度。
- 当前网页和主机测试不改变固件硬件参数，也不能替代 Keil 构建、烧录、OpenMV、底盘或机械实机验收；相关实机项目继续为 `NOT_RUN`。

## 18. OpenMV 边走边扫二维码（2026-11-18）

- `App/race_fsm.c/.h` 将二维码启动标志附着到搜索路段的 `MOVE` 动作；底盘仍执行原有里程计目标和到位保持，扫描不再占用独立停车状态。
- `App/app_race.c` 在动作进入时只调用一次 `RaceVision_BeginScan()`，主循环持续调用 `RaceVision_Poll()`；二维码成功后由现有 `Race_SubmitQr()` 校验、保存并只生成一次任务计划。
- `Hardware/vision.c/.h` 实现 `RaceVision_ScanFailed()`，将 ACK 超时、OpenMV `TIMEOUT` 和协议错误传递给 App；App 进入明确扫码/链路故障并走安全停止路径。UART 中断仍只接收和排队字节。
- OpenMV `vision_openmv_stm32_v4_redownload/OpenMV/main.py` 的 `QR` 任务在主循环连续调用 `find_qrcodes()`，连续确认三帧后发送 `QR`、`DONE,QR` 和 `IDLE`；本批未修改 OpenMV 工程。
- `tests/race/run.ps1`、`tests/ulog/run.ps1`：`PASS`；Keil UV4/ArmCC 完整构建：`PASS`，0 Error、0 Warning。二维码板搜索段的起点、终点、行驶方向、摄像头安装角和覆盖速度仍为 `NEED_CONFIRMATION`；OpenMV 通信、底盘运动和机械流程实机验证为 `NOT_RUN`。

## 19. BT10 BLE 日志链路（2026-11-19）

- 固件 USART2 继续使用 PD5/PD6、115200 8N1；发送改为中断发送队列，USART3 视觉链路保持不变。
- 日志结束时追加 `#BTLOG,END`，Windows 接收端收到后执行 flush/fsync/close；运行中不允许通过主机命令修改参数。
- 新增 `tools/bt10_ble_receiver.py` 和使用说明，连接 BT10 BLE GATT FFE0/FFE1/FFE2，保存 CSV，可选广播到本机 WebSocket `127.0.0.1:8765`。
- `viewer/` 增加可选实时 WebSocket 入口，继续使用现有 ULog43 解析和 vx/vy、位移、轨迹曲线。
- `tests/ulog/run.ps1`：`PASS`；`tests/race/run.ps1`：`PASS`；网页 Node 测试：`PASS`；Python 语法/帮助检查：`PASS`；真实 BT10 BLE、Windows GATT 和网页 WebSocket 联调：`NOT_RUN`。
- 未修改电机参数、PID 数值、USART3、TIM6 调度和机械控制；未执行烧录与实机验收。

### 能力与兼容性边界

- 当前仍输出原有 ULog43 数值行，因此已有 43 列 CSV 的 `vx`、`vy`、目标速度、航向、x/y、累计位移、轮速、电压和网页分析能力继续兼容；旧 ULog26 只能通过已有转换流程导入，补零字段不能当作实测值。
- BT10 负责无线传输，网页直接连接时把数据实时显示并写入浏览器 IndexedDB 日志仓库；Windows Python 接收器还可以把同一数据持续写入电脑硬盘 CSV。浏览器仓库属于当前浏览器/电脑，不能替代车辆端介质。
- TF 卡是车辆端离线介质，车辆断开电脑或蓝牙仍可保留文件；BT10 本身不提供车辆端持久化，BLE 断链、电脑休眠、浏览器关闭或接收端异常都可能造成未保存数据。接收器会在断链时尽量 flush 已收到内容，但不能恢复断链期间的数据。
- `#BTLOG,END` 是 BT10 接收端的结束标记，用于关闭电脑文件和结束网页会话；它不是旧 TF 卡文件系统的关闭命令。固件停止 USART2 日志后，TF 卡记录器若仍存在，仍需按其原有协议单独确认文件已关闭。
- 旧的 BT10/WHEELTEC APP 控制帧、USART3 视觉协议和 ULog43 日志格式是三套不同协议。当前实现兼容的是 ULog43 数据内容和 USART2 115200 8N1 传输路径，不承诺把旧 APP 控制命令自动解释为日志行；`@GET/@SET` 仅是当前电脑接收器的调试命令，运行中禁止 `@SET`。
- 当前真实 BT10 长时间传输、BLE 断链重连、整车结束时序和比赛环境可靠性仍为 `NOT_RUN`，需要实车人工验收。

## 20. 双启停区二维码扫描流程（2026-11-19）

- `Race_StartQrRoute()` 编排 140 mm 初始平移、场地绝对 X=1000/1400 mm 到局部里程计位移的换算、100 mm/s 扫描速度、首次成功跳过反扫，以及首次漏扫后的 180°反向扫描动作。
- `RaceAction.scan_leg` 区分正向和反向扫描；二维码接收继续在主循环异步完成。二维码校验并保存后由 `RaceVision_AckQr()` 发送 `ACK,QR,<session>`，重复二维码不会重复生成任务计划。
- OpenMV QR 任务取消固定识别超时，保留同一任务会话并限频重发待确认二维码；只有收到匹配确认后才发送 `DONE,QR` 并进入 `IDLE`。
- `RACE_QR_FIELD_FRAME_CONFIRMED` 和 `RACE_QR_ROTATION_SAFE` 默认关闭。未完成场地原点、航向和旋转外廓实测前，App 拒绝启动绝对坐标路线，FSM 仍可用于主机测试但首次漏扫不会自动旋转。
- `tests/race/run.ps1`、`tests/ulog/run.ps1`：`PASS`；UV4/ArmCC 完整构建：`PASS`，0 Error、0 Warning；OpenMV Python 语法检查：`PASS`；OpenMV、场地路点、旋转空间、摄像头和整车动作：`NOT_RUN`/`NEED_CONFIRMATION`。
