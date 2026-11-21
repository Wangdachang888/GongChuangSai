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
| PID Burst 接收与网页分析 | 主机测试通过 | 独立 PID CSV、四轮指标和有边界候选已实现；固件高频缓存、BT10 吞吐和实车上传仍未验证 |
| 本地 AI 辅助调参 | 主机测试通过 | 基于指标的可解释候选排序、前馈偏置诊断和置信度已实现；不是云端模型，不自动写入车辆 |
| DeepSeek 本地代理 | 主机测试通过 | `tools/deepseek_pid_proxy.py` 只绑定本机、从环境变量读取密钥；API 调用和实车参数写入仍需人工验证 |

## 6. 当前默认配置

- `APP_RUN_MODE` 默认是 `APP_MODE_RACE`。
- `RACE_PICK_ENABLE` 为启用状态。
- `RACE_PICK_DIRECT_KEY_TEST` 当前为 `0`，正式比赛短按进入二维码路线；机械夹取测试仅通过测试构建配置显式开启。这不等于完整比赛任务链已经实机接通。
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
- PID Burst 网页解析/指标/候选、BT10 接收器语法与临时保存测试：`PASS`；STM32 高频缓存发送接口、230400 BT10 吞吐和停车后分批上传：`NOT_RUN`。
- 本地 AI 辅助候选排序、前馈低速回归诊断和安全边界测试：`PASS`；真实参数试验、回滚和实车闭环效果：`NOT_RUN`。
- DeepSeek 代理语法、无密钥拒绝和网页调用接口检查：`PASS`；真实 API 调用：`NOT_RUN`。
- 网页代理地址、`/health` 连接检查和按钮门控：`PASS`；浏览器自动启动本地 Python 进程：不支持，仍需手动启动代理。
- 本地一键启动脚本语法检查：`PASS`；实际双击启动和浏览器关联：`NOT_RUN`。
- 启动脚本读取用户级密钥、Python 命令选择和健康重试：`PASS`；实际 DeepSeek 网络调用：`NOT_RUN`。
- 启动脚本现改为运行时隐藏输入临时密钥，不写用户环境变量或配置文件；成功/失败会显示明确提示，并检查 Python、端口和 `/health`。重复启动时会复用已通过健康检查的 8787 代理。双击脚本的失败路径会等待用户确认；实际双击和真实 API 调用仍为 `NOT_RUN`。

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

## 20. 双启停区二维码扫描流程（2026-11-20）

- `Race_StartQrRoute()` 使用赛前锁定的人工启停区选择，保持两区车头 `Yaw=0°`，先沿 Y 正方向平移 140 mm，再将绝对场地 X 坐标转换为相对里程计位移。区 1 按 1400→1000→1400 mm 扫描并回到公共点，区 2 按 1000→1400 mm 扫描；两条路线最终汇合到 `(1400,290,0°)`。
- 扫描动作是普通 MOVE 的 `scan_start` 标记，速度限制为 100 mm/s；二维码接收继续在主循环异步完成。二维码校验并保存后由 `RaceVision_AckQr()` 发送 `ACK,QR,<session>`，当前扫描段仍完整执行，不会因提前成功跳过返程。
- OpenMV QR 任务取消固定识别超时，保留同一任务会话并限频重发待确认二维码；只有收到匹配确认后才发送 `DONE,QR` 并进入 `IDLE`。
- 不再按扫码次数推断启停区，不再执行扫码失败后的 180°旋转或反向补扫；二维码无效或扫描段结束仍未确认时进入故障并安全停止。`RACE_QR_FIELD_FRAME_CONFIRMED` 默认关闭，未完成场地原点、航向和路点实测前，App 拒绝启动绝对坐标路线。

## 22. 内环 PID 高频日志采样（2026-11-20）

- `Hardware/bsp_ulog.c/.h` 增加独立 PID Burst 环形缓冲接口，快照包含 1 ms 电机内环的四轮目标速度、实际速度和控制输出。
- TIM6 中断路径只复制固定大小标量到 256 帧 RAM 环，不执行格式化、浮点计算或 UART 发送；主循环通过现有 USART2 非阻塞队列逐帧发送 `#BTLOG,PID`，结束时发送 `#BTLOG,PID_END`。
- `ULOG_PID_BURST_ENABLE` 默认关闭，比赛构建不会自动占用 RAM、串口带宽或改变控制行为；仅在明确的诊断构建/测试流程中调用 `ULog_PidBurstStart/Stop`。
- 环形缓冲满时丢弃新样本并递增 `g_ulog_pid_burst_overrun`，不会阻塞 1 ms 电机更新。256 帧约覆盖 256 ms，不能表示无限长过程；长时间测试需分段或停车后分批上传。
- 主机编译、ULog 测试、Keil 完整构建和 `git diff --check` 已执行；BT10 实际吞吐、启停命令接入、停车后完整上传和实车内环效果仍为 `NOT_RUN`。
- `tests/race/run.ps1`、`tests/ulog/run.ps1`：`PASS`；UV4/ArmCC 完整构建：`PASS`，0 Error、0 Warning；OpenMV Python 语法检查：`PASS`；OpenMV、场地路点、旋转空间、摄像头和整车动作：`NOT_RUN`/`NEED_CONFIRMATION`。

## 21. 二维码后的原料区导航与视觉对准（2026-11-20）

- `App/app_race.c/.h` 增加了受配置门控的非阻塞流程：二维码路线完成并在公共汇合点停稳后，依次前往 `(2.100,0.290,0°)`、原地逆时针转 `90°`、前往 `(2.100,1.100,90°)`，再进入相机准备、JOB40 圆盘确认/微调和 JOB41 固定抓取位对齐；本阶段结束只报告 `APP_MATERIAL_ALIGN_COMPLETE`，不会启动 `PickFsm`。
- `Hardware/vision.c/.h` 增加 `AREA_OK,cx,cy,r` 和 `ARRIVED,MATERIAL,0,dx,dy` 的现有协议解析、结果/新鲜度访问接口；UART 中断仍只接收和排队，业务动作由主循环执行。
- `Config/race_config.h` 增加相机步数、俯仰、圆盘/抓取位标定矩阵和修正上限配置。`RACE_MATERIAL_ALIGN_ENABLE`、`RACE_CAMERA_CALIBRATED`、`RACE_GRIP_TARGET_CALIBRATED` 默认关闭/未标定；缺少真实标定时流程会安全故障，不会猜测步数或像素换算。
- `tests/race/run.ps1`：`PASS`（新增 AREA/ARRIVED 协议事件覆盖）；`tests/ulog/run.ps1`：`PASS`；OpenMV `py_compile`：`PASS`；Keil UV4/ArmCC 完整构建：`PASS`，0 Error、0 Warning；`git diff --check`：`PASS`。
- 相机安装高度/俯仰映射、圆盘和固定抓取位矩阵、夹爪标定、旋转包络、实际路线和整车动作：`NEED_CONFIRMATION`/`NOT_RUN`。

## 22. 比赛任务函数式架构规范同步（2026-11-20）

- 新增 `docs/RACE_ARCHITECTURE_RULES.md`，作为未来比赛业务函数式组织的唯一详细规范源，参考 GongXun2025 的任务函数顺序，但不复制其阻塞式执行方式。
- 更新 `AGENTS.md`、`docs/ARCHITECTURE.md` 和 `docs/DECISIONS.md`：后续比赛开发必须阅读该规范；`AppRace`、`RaceFsm`、`RaceTaskPlan`、`PickFsm`、Vision 和 Control 的职责边界以及单一底盘控制权保持明确。
- 本次仅同步文档和协作规则，未修改任何 `.c`、`.h`、CubeMX、Keil、驱动、标定、配置数值或运行行为；不代表当前比赛源码已经完成目标函数式整理。
- 文档链接和 `git diff --check`：待本轮执行；主机测试、Keil 编译和实机验证：`NOT_RUN`（本轮为文档-only）。


## 23. PID 20 秒记录与 200 Hz 实时链路（2026-11-20）

本节替代第 22 节的 256 帧试验实现。现已实现独立的 2000 帧定点 RAM 记录区，
以 100 Hz 保存 20 秒、200 Hz 最新快照实时发送、停车后上传；固件和网页协议已联通主机测试。
计时从显式开始采集起，包含等待运动时间；满 2000 帧自动冻结，提前停止保存已有数据。
20 秒缓存不是 20 秒 1 kHz 原始数据；1 ms 是 TIM6 调用周期，现有 Motor 速度 PID
实际每 5 ms 更新。未更改 Motor.c、PID 数值、前馈、底盘 10 ms 环或 USART3 资源。

- USART2 改为 230400 8N1，与用户确定的 BT10 配置匹配；USART2 IRQ 为优先级 5，
  低于 TIM6 的 4。旧 115200 串口接收端必须同步配置，普通 ULog43 字段和 20 Hz 不变。
- PID 功能编译启用但运行时默认未采集，开机不会自动采集或驱动车辆；
  ULOG_PID_BURST_ENABLE=0 可回收 56 KB 缓存。显式 @PID START 仅在停车且预热后接受；
  @PID STOP 只停止采集，不控制停车，不写 PID。
- ISR 内固定周期量化并复制标量，100 Hz 保存到 56000 字节记录区；200 Hz 实时包每帧36字节。
  主循环处理有界命令、CRC、组帧和发送。TX 队列预留 1024 字节给整车日志，
  饱和时丢实时帧并计数，停车上传同一记录在队列拥塞时重试。
- 四轮命令为零、实测小于 10 pulse/s 且不在 RUNNING 状态持续 500 ms 才上传；
  整车 END 等待 PID 上传结束后发送，保持 PID_BEGIN→数据→PID_STATS→PID_END→END 顺序。
- 定点精度 0.1，范围 -3276.8～3276.7；裁剪/非有限数计数可见，记录总输出不能代表实测电压。
- 网页增加开始/停止/下载/协议自测、四轮速度与输出曲线、实收/应收帧数和缺口显示，
  收到停车数据自动使用现有分析与 AI 候选流程。自测需断开实车，不代表无线硬件通过。
- Python 接收器同时支持旧文本和新二进制，按碎片重组、校验 CRC，转为兼容 PID CSV 和
  WebSocket 文本；后续独立 PID 文件自动编号，保留 PID_STATS。仓库 PID 分析入口复用已有解析。
- PASS：ULog 测试新增 20 秒保留、提前/整车结束顺序、停车门控、200 Hz 实时、
  生产异步队列饱和重试；C→Python 1/7/181字节分片、2000帧、CRC恢复、持久关闭；
  网页 20秒协议自测、C→JS字段/符号/时间核对、CRC恢复及已有网页回归；
  tests/race、Keil 完整构建、JavaScript/Python语法及 git diff --check。
- Keil 最终完整构建：0 Error、0 Warning；Code=56984、RO-data=8976、RW-data=400、ZI-data=70120 字节。
- NOT_RUN：BT10持续200Hz吞吐、网页真实GATT按键与串口双向联调、TIM6最坏耗时/抖动、
  高负载视觉与机构并行、实车曲线和参数效果。没有执行烧录。
- 浏览器 file:// 自动预览被工具策略拒绝；DOM/Canvas 主机测试不等于浏览器视觉验收。
- 记录为 RAM 暂存，掉电丢失；当前没有 BLE 应用层确认/自动补传机制。
  不承诺断链无丢帧或硬件完全无时序影响，必须检查实收帧数与 PID_STATS。
- 使用步骤与二进制布局详见 tools/BT10_BLE_RECEIVER.md。


## 24. 比赛流程与日志审计（2026-11-21）

本次核对正式 APP_MODE_RACE 构建、AppRace/任务计划、AppRuntime、TIM6、USART2 路由和
ULog43/PID Burst/网页解析调用链。结论是日志传输与解析主机测试通过，但当前比赛业务
没有接成两批物料到返回启停区的完整执行链，不能宣称日志已通过整场比赛验收。

已修复：ULog43 ch10～13 原来使用 %.0f，导致低速纵/横向实际值和目标值丢失小数，
例如 0.1234 m/s 被写成 0。现在使用 %.4f；ch37 右后轮目标由整数改为 %.1f，
与 FL/FR/RL 保持相同精度。字段位置、43列格式、频率、波特率、控制参数和资源均不变。
现有主机测试增加正负低速及四轮目标/实际值的 CSV 反解析校验；先复现 FAIL，再修复为 PASS。

现有功能边界与审计时问题（故障PID补传已在第25节修复，其余问题仍待处理）：

- 正式配置 RACE_QR_FIELD_FRAME_CONFIRMED=0，按键启动会报配置故障；这是未完成场地标定的
  安全门，未擅自打开。RACE_MATERIAL_ALIGN_ENABLE=0，相机及抓取位标定也未就绪。
- 开启场地确认后的当前执行路径是二维码路线到公共点，随后在当前任务为 PICK 时直接
  启动一次 PickFsm，再推进一次计划索引；没有原料确认成功后再夹取的完整业务门控，
  更没有两批粗加工、取回、暂存、码垛和返回执行。RaceTaskPlan 只是生成动作清单。
- AppRace_LogFinished 当前用 RACE_DONE 加机构不忙作为局部结束条件，未检查任务清单全部
  完成或车体进入300×300 mm启停区；会关闭普通日志，不代表整场比赛完成。
- 可选原料导航代码的 material_start_after_qr 判断放在 Race_Tick 之前，previous_state
  刚从 s_race.state 复制，两者此时不可能分别为 RUNNING/DONE，因此入口未正常接通。
  即使移动判断，还需同时解决公共点直接夹取、局部 DONE 停录、动作阶段可观测性和控制权。
  本次未擅自打开开关或重写比赛主线。
- PID 停车门控使用实际轮速<10 pulse/s；取消/故障导致 AppRace_ControlTick 返回0后，
  TIM6 不再调用 Motor_Update，Motor_Stop 不清除 actual_speed。若最后轮速>=阈值，
  即使实车随后停稳，软件轮速也可能冻结，导致 PID 待上传和最终 END 一直等待。
  此项在第25节通过终止后零目标/零输出确认修复；不伪造轮速，不把输出归零当作物理停稳证据。
- ULog43 记录全局路线状态/动作索引和计划索引，不含 PickFsm/原料视觉对准的完整子状态、
  批次/颜色/工位/物料确认事件；网页不能凭当前字段完整识别用户描述的所有比赛阶段。
- 普通日志开机2秒后20Hz发送，结束/取消/故障后锁止到复位；前2秒故障可能没有任何日志。
  PID 为显式开启的20秒100Hz RAM缓存和目标200Hz实时流；有待补传时 END 延后。
  BLE 断链无应用层自动补传，RAM掉电不保留；TF外设文件关闭仍须其独立协议。

验证结果：

- PASS：tests/ulog/run.ps1（含低速精度、四轮CSV、20秒缓存、异步队列、停车门控、END顺序、
  C→Python分片/CRC及流程测试）；tests/race/run.ps1；node viewer/test.cjs；
  node --check viewer/viewer.js；python -m py_compile tools/bt10_ble_receiver.py。
- 网页测试初次与 ULog 测试并行运行时读取了正在重写的 tmp/pid_wire_test.bin，出现缺表头
  FAIL；ULog 测试结束后顺序重跑 PASS。后续共享该夹具的测试须顺序执行。
- PASS：Keil UV4/ArmCC 构建，0 Error、0 Warning；Code=56984、RO-data=8976、RW-data=400、
  ZI-data=70120；git diff --check。
- NOT_RUN：真实BT10持续吞吐、浏览器GATT、TIM6最坏耗时、取消/故障后真实停车补传、
  OpenMV/机构并发、整场比赛与TF落盘。未执行烧录，不声称对比赛完全无时序影响。


## 25. 故障/取消后的 PID 补传解锁（2026-11-21）

- 使用最小日志侧补丁：AppRuntime 在 RACE_FAULT/RACE_ABORTED 时只读检查四轮
  target_cmd、target_speed 和 output 均严格为0，填写内部快照 terminal_outputs_stopped。
- 仅当普通日志已结束并请求最终停止时，PID上传允许使用该确认替代冻结的 actual_speed。
  继续复用原有连续500ms等待、非阻塞发送和 PID_END→END 顺序；确认丢失会重新计时。
- 正常运行/普通停车与开始采集的低轮速门控保持不变。该确认不能启动新采集、恢复控制、
  改写反馈或证明物理静止；车辆可能仍有惯性滑行，补传只发送已冻结数据。
- 未修改 AppRace/RaceFsm/PickFsm、Motor、主程序中断调度、PID、前馈、引脚和USART资源。
  ULog43、二进制包和网页协议均不变，无需修改网页或额外用户操作。
- 回归先复现 FAIL，修复后 PASS：故障/取消后轮速冻结300 pulse/s仍上传10帧并结束；
  未结束/未确认时拒绝上传，499ms内不上传，确认撤销后重新等待，结束后拒绝重启。
  tests/ulog（含20秒缓存、生产异步队列、C→Python、开关编译）、tests/race、
  node viewer/test.cjs、git diff --check 均PASS。
- Keil UV4/ArmCC：PASS，0 Error、0 Warning；Code=57200、RO-data=8976、RW-data=400、
  ZI-data=70120。未新增正式源码/头文件/目录，也未烧录。
- NOT_RUN：实车故障/取消后BT10补传、真实PWM归零与惯性滑行、TIM6最坏耗时和整车验收。
  第24节其他比赛流程问题仍保持未完成状态。


## 26. 解耦持续实时 PID 与 20 秒缓存（2026-11-21）

此节取代第23/24节及旧使用说明中“满20秒自动停止PID采集”的行为描述。

- 保持普通 ULog43 20 Hz 自动记录；显式 @PID START 成功后，PID 实时发送目标
  200 Hz，直到 @PID STOP 或整体停录，不再受缓存容量限制。START 的停车/预热门控不变。
- 同时用原有56,000字节RAM以100 Hz循环保留最近2000帧；写满仅覆盖最早帧。
  STOP/整体停录冻结缓存，既有停车500ms门控及故障/取消后零目标/零输出兜底保持不变，
  然后按时间顺序上传；PID_END之后才允许最终END。停止采集不控制车辆停车。
- PID_ARMED 增加 cache=rolling，PID_STATS 增加 overwritten 正常覆盖计数，
  live_drop/CRC缺口独立表示发送/接收完整性。36字节包、量化精度与原有解析保持兼容。
- 网页实时图保留最近4000帧并显示累计数；IndexedDB升级v2，保留旧仓库，
  新会话约1秒/64KiB文本分段写入且事务失败保留待写数据。移除无限增长的bt10.raw副本。
  完整实收200Hz PID和停车100Hz缓存分开下载，均支持原PID分析；会话导出保留全部混合原文。
- 记录仍受浏览器配额和电脑资源约束；异步关闭可能丢失未提交尾段，BLE断链不自动补回
  200Hz历史。浏览器窗口4000帧、MCU最近20秒缓存、电脑完整实收会话是三个不同边界。
- 修改范围限于既有 Hardware/bsp_ulog.c/.h、viewer源文件及生成HTML、现有测试和文档；
  无新增/删除/移动正式源码和目录；未改比赛流程、控制器、PID/前馈、资源或1ms/10ms调度。
- PASS：ULog/race主机回归、异步队列饱和、C→Python分片/CRC与连续360.03秒72006帧
  （含u16序号回绕）、最后2000帧342040～362030ms的顺序和完整性；网页30秒模拟、
  C→JS同一连续夹具、4000帧显示窗口与72006帧导出、旧仓库升级、6000帧分段保存、
  事务失败重试不重复、首次打开失败后重试、排队END选择新会话、分段删除；JS/Python语法。
- 网页测试最初仍断言旧模拟终点20000ms而实际为30000ms，出现FAIL；更新为30秒实时/
  最近20秒缓存预期后重跑PASS。共享夹具仍按ULog完成后再执行网页测试。
- PASS：Keil UV4/ArmCC，0 Error/0 Warning，Code=57240、RO-data=9004、RW-data=404、
  ZI-data=70124；静态RAM只增加8字节；git diff --check。
- NOT_RUN：烧录、真实浏览器/GATT/磁盘配额与长期后台保存、BT10持续吞吐、TIM6最坏耗时、
  视觉/机构并发与整车验收；第24节尚未实现的比赛业务不因本次解耦变为完成。


## 27. 蓝牙在线PID读写与真实参数基准（2026-11-21）

- 用户明确授权在线改PID。新增GET/SET到现有bsp_ulog解析链，应用运行层通过注册处理器
  独立拥有参数和停车检查。FL/FR/RL/RR速度环、X/Y/TH位置环均可读取并单通道原子改Kp/Ki/Kd；
  默认参数不变。Motor新增只读配置接口返回实际编译周期、前馈/死区/斜坡/低通/kick。
- SET要求READY或业务已完成DONE、速度模式、四轮零目标/零输出、有限实测速率<10pulse/s
  连续500ms；拒绝运行/故障/取消、Tune模式、位置控制、越界/非有限值、旧版本、采集/待上传。
  修改只重置对应PID历史，版本递增；不改目标、轮序、前馈、限幅、姿态、硬件或比赛状态。
- 普通END仍锁止日志。显式参数GET/SET可在END之后应答，不重启日志或比赛；网页控制回复
  不进入数据仓库。主循环解析预算32字节/一次最多一条命令，RX中断只排队，TX沿用非阻塞队列。
  已执行SET的应答受拥堵会保留重试，不重复应用；浏览器超时也不自动重发。
- 网页连接后自动读七组实际值，显示周期、前馈和限幅等；写入执行GET→带版本SET→GET确认，
  读回一致后才显示成功。支持上组参数填入回滚、本机保存值重新载入（仍需点击写入确认）。
- 每次采集输出七组PID_USED真实快照；实时/停车CSV与Python独立文件均保留。候选按选中轮子
  指标和本轮快照计算，避免默认手填值或四轮均值冒充该轮参数；快照缺失/混用或运动样本不足
  时不提供直接填入。位置环本次为手动读写，不能使用速度环候选。DeepSeek代理保留真实参数上下文，
  仅复核排序，不写车辆；未调用付费外部API验证。
- RAM写入不持久化至Flash；复位恢复源码默认。下一轮按原比赛流程人工复位、网页读取、载入
  本机保存值、写入读回确认、再开始采集/原按键启动。不能声称自动重跑或跨断电保留车端参数。
- PASS：tests/ulog（直接编译实际AppRuntime参数段和Motor编译常量，覆盖七组GET/SET、
  499/500ms、运行/故障/残余输出/NaN拒绝、版本重放/范围/分片/长报文、采集忙、END后请求、
  异步满队列SET只执行一次）；race原有回归；C→Python/JS参数协议与导出快照；
  网页七组读取、20字节发送串行、SET后GET、版本/数值不匹配、超时/迟到包/断链与本机回滚。
- 修复测试中发现的GET请求/READ回复名称匹配错误；初次端到端测试卡住（FAIL），修复后PASS。
  新测试IRQ桩初次与现有flow定义冲突、异步测试格式告警导致FAIL，改桩声明/语句后完整重跑PASS。
- PASS：Keil UV4/ArmCC 0错误0警告，Code=65028、RO-data=9404、RW-data=428、ZI-data=70572；
  JS/Python语法与git diff --check。未新增/移动/删除正式源码或目录；未烧录。
- 工作区原有Config/race_config.h和MDK-ARM/GongChuangSai.uvprojx改动及pid_burst_demo.csv保留，
  不纳入本次提交；Keil是在当前工作区配置下编译，不能冒称这两项也已同步远程。
- NOT_RUN：真实BT10/GATT读写、实车连续停车门控与修改后响应、控制最坏耗时、重启后人工
  重应用与整车验收；原有未接通的比赛任务链仍未由本次参数通道实现。

## 28. PID辨识/预测与DeepSeek证据分析v2（2026-11-21）

- 根据用户实车日志/DeepSeek评分普遍不足40的反馈，确认旧算法固定比例产生候选、对所有候选
  重复扣旧日志指标分数，既没有辨识电机，也没有验证候选响应。该方法被替换，不能将旧分数
  与新指标指数当作同一标尺，不能声称算法更新会让车辆实际性能自动改善。
- 仅更改既有viewer源文件/生成v3/v4网页、DeepSeek代理/启动器、相关测试和长期文档；
  无新增/删除/移动正式源码或目录。固件、默认PID/前馈、采样/传输、比赛流程和资源完全未修改。
- 现在分开显示本轮实测表现、模型测试误差、候选预测；修复减速滞后冒充超调、短暂入带冒充
  稳定、缓慢目标漂移冒充平台的指标错误。真实时间质量单独保留；5/6ms量化可重采样。
- 实际总输出→实测速率辨识带延迟ARX和输出误差模型，Huber/变量投影、60/20/20顺序划分；
  测试段不参与拟合/选模，尺度来自训练段。缺输出/时间、恒值激励、乱序/缺帧、独立预测失败
  会停止推荐。还以实测反馈审计实际控制器，并要求原PID完整闭环重放足够吻合。
- SIMC PI种子+局部搜索，每个候选独立模拟现有并联PID、条件积分、前馈、kick、同向限制和
  slew；目标已过斜坡、有效模型已含低通，不重复叠加。限制单次±20%和固件范围，有限扰动
  场景都不退化且中位目标函数改善≥2%才允许填表，仍须原在线停车写入/读回确认。
- 新增所选轮子的文件目标/实测/输出图、可用时的预测叠加、正反向/零目标及停滞诊断。
  长图保留时间桶极值；独立模型最多最后12000帧，并缓存辨识结果；不混用其他轮子的指标。
- DeepSeek v2接收真实参数、控制器审计、分段、最多160个时序包络、模型验证和候选预测，
  不再主观重打分。AI可提出最多4组参数，代理和网页校验边界并本地仿真复核；模型失效只诊断。
  迟到/越界/非有限提案拒绝，输出转义；不自动写车。健康接口advisor_version=2，启动器只会
  重启经过命令行确认的本项目旧Python监听进程，避免新网页连接旧代理。
- 已联网读取GEKKO、SysIdentPy、python-control、simple-pid、arduino-pid-autotuner、PidTuner
  和SIMC作者说明；参考方法由本项目独立实现，未复制第三方源码或引入新运行依赖。
  参考链接、目标函数、阈值、模型适用限制详见tools/BT10_BLE_RECEIVER.md的v2章节。
- 用户提供真实PID CSV只读分析：4133帧/20.906秒，平均197.65Hz、5/6ms，无时间戳跳帧。
  七组真实参数齐全；FR/RL饱和13.43%/15.12%，存在大输出低轮速；控制器输出审计基本吻合，
  但四轮简化电机模型均未通过独立验证，不能承诺该日志已能自动给出可靠最优参数。
  文件未加入Git；原有比赛配置、工程排版和pid_burst_demo.csv的内容哈希保持不变。
- PASS：ULog/race回归、独立已知延迟/噪声模型恢复、测试段变化拒绝、数据不足/缺输出/恒值/
  坏时间拒绝、候选逐项预测与步幅/改善检查、文件曲线及原有日志仓库/参数通信、AI模拟提供者/
  证据传递/提案/迟到/转义、真实4133帧日志回归、JS/Python/PowerShell语法、git diff --check。
- 初次网页测试仍要求旧4帧样本生成3组候选（FAIL），更新为数据不足不得推荐后PASS；
  AI模拟提供者测试的中文断言受Windows PowerShell管道编码影响FAIL，改为ASCII协议断言后
  ULog完整重跑PASS；真实JSON报告读取初次使用系统GBK失败，明确UTF-8后报告核对成功。
- Keil NOT_RUN（本次未改固件）；真实新参数效果、BT10/GATT、控制时序、浏览器视觉验收、
  旧代理进程迁移与付费DeepSeek网络请求仍NOT_RUN。没有烧录或驱动车辆。

## 29. 四轮PID统一编辑和一键写入（2026-11-21）

- 用户要求避免逐轮切换输入。既有PID页增加统一Kp/Ki/Kd填入四轮、四行分别编辑、
  载入四轮最新读回/本机保存/各轮上次写入前值，以及一键写入并全部读回确认。
- 复用既有单轮GET/SET协议；校验全部输入、同版本四轮停车预检，再逐轮SET/GET，
  最后同一最终版本复读四轮。操作中禁用参数编辑和重复操作，不修改位置环和前馈配置。
  没有新增固件原子批量接口，失败可能部分生效；中止后标明逐轮确认/未确认/未写入，
  清空可能过期的读回表，必须重新读取，不自动重发或回滚。已确认值按轮保存到本机。
- 场地适配边界写入界面和说明：当前地面/轮胎/载重/电池下逐轮辨识与独立验证，
  改参数后还须低速启停、正反向、横移多次实测。单轮候选不得当四轮已验证参数；
  编码器不能独立验证打滑后的真实地面位置。用户现有4133帧日志模型仍未通过，不虚报最优PID。
- PASS：网页构建、JS语法、ULog/race回归、网页交互/仓库/AI原有回归和用户真实CSV回归；
  新增四轮统一/分别编辑、全量输入/停车预检、顺序写入/最终版本复核、本机载入、
  第三轮拒绝/实际SET后丢ACK/最终版本变化中止测试；git diff --check。
- 本次仅既有网页源文件/生成v3/v4、相关测试和文档变化，无正式文件新增/删除/移动。
  原有race_config.h、uvprojx与pid_burst_demo.csv哈希不变且不提交。无固件修改，Keil NOT_RUN。
- 浏览器视觉验收SKIP：浏览器工具拒绝file://，只允许HTTP/HTTPS，未绕过限制。
  真实BT10四轮写入、部分失败后人工核对、新参数在场地的效果仍NOT_RUN；未烧录或驱动车辆。

## 30. 独立底盘速度/PID采集构建（2026-11-21）

- 用户授权新增 `App/app_pid_test.c/.h`、`PID_Test` Target。`APP_PID_TEST_ENABLE` 默认0；
  `APP_RUN_MODE=4` 与测试启用必须成对，和比赛/普通模式及 MOTOR_TUNE 冲突时编译拒绝。
  AppRuntime 独占接入，复用现有 PE0 消抖、体坐标速度入口、IMU、TIM6 和 USART2 日志；
  不启动比赛、Square、Tune、D36、视觉或机械任务，不引入新控制器或协议。
- 默认 0.03 m/s 纵向，初始零速1s，三组正2s/零1s/反2s/零1s，末尾零1s；名义20s、25s总超时。
  横向与0.06/0.09配置可选；独立极低速0.006/0.012配置正反各4s，名义32s、40s总超时，
  需普通档实机验证后再启用，所有档位均 NEED_HARDWARE_TUNING。
- 上电IDLE，IMU有效且预热/偏置就绪、真实停车500ms、网页采集实际设防后，只能物理短按释放开始。
  运行中按下取消；结束、超时、失效或采集中断均调用既有停止路径并锁存至复位。
  中间零速不重置PID。ISR仅保留250ms主循环紧急切断，业务推进在Poll；TIM6 1ms/10ms、
  速度PID内部5ms、实际PID/前馈/斜坡/方向/轮径/引脚/硬件资源未改。
- 测试状态单独输出到现有日志快照，不伪造比赛状态；运行时含零速段禁止停车补传/写PID。
  停止后不清零actual_speed，继续测速，零目标/输出且实际轮速<10pulse/s后沿用500ms上传；
  无旧反馈兜底。DONE可按既有条件写RAM参数，取消/故障只读；真实停车条件不满足则等待，不冒充停稳。
- ULog43列、PID包/CRC、PID_USED、200Hz实时/100Hz最近20秒缓存及网页均未改。
  使用方法、状态/原因、人工试验元数据记录见 `tools/BT10_BLE_RECEIVER.md` 的 PID_Test 章节。
- PASS：tests/ulog（实际测试源、实际Runtime参数/快照/安全桥接代码与按键消抖，覆盖六种配置、
  逐阶段方向/幅值/时间、三组一次、回绕、取消/超时/故障、未设防拒绝、上传/END和参数门控）；
  tests/race；既有异步队列、跨语言协议、连续72006帧/最近20秒回归；最终diff检查。
- PASS：UV4/ArmCC双Target完整Rebuild，均0 Error/0 Warning。PID_Test尺寸Code=46592、
  RO=1832、RW=392、ZI=67768；比赛Code=65028、RO=9404、RW=428、ZI=70572，与原体积一致。
  uVision会合并Target文件列表，已在比赛Target显式IncludeInBuild=0；比赛编译日志、
  链接输入及Map均无测试对象/函数/状态，原有芯片/编译选项/工程源引用不变。
- 测试首次错误使用0.01m/s验证0.006档限速拒绝（FAIL），改为确实更低的0.001阈值后完整重跑PASS。
  初次Map检查发现比赛编译空测试对象（FAIL），显式排除后完整Rebuild/Map检查PASS。
- 工作区原有race_config.h场地确认修改、uvprojx原App组排版及pid_burst_demo.csv均保留，
  不纳入本批提交；uvprojx仅提交Target/显式排除增量。编译基于当前工作区，不声称原有场地确认已同步。
- NOT_RUN：烧录、真实车轮动作/停稳/按键/漂移、BT10持续吞吐与采集、现场低速/极低速适用性、
  TIM6最坏耗时和整车验收。没有实测数据时不推荐自动调参；本批不自动进入下一项工作。
