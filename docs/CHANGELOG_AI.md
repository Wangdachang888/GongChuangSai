# AI 维护变更日志

本文件记录 AI 参与的工程维护变更。提交记录只描述已经落地的内容；实机验证状态以 `docs/PROJECT_STATE.md` 和 `docs/HARDWARE_VERIFICATION.md` 为准。

## 已有重构记录

- `25a4b84`：建立架构基线、模块职责和重构前后对照文档。
- `9a78477`：将视觉通信与数字圆环控制解耦，保留旧二维码和 `#SEQ` 兼容。
- `37b1865`：让主机测试自动发现 GCC，并支持显式配置编译器。
- `4aa031f`：整理历史资产，清理死代码和测试桩。
- `791c252`：补充架构、接口和验收文档。
- `a59690b`：修正文档尾部格式，保持验收报告可读。

## 长期维护系统

- `a3a4e30`：新增根目录 `AGENTS.md`，规定上下文恢复、最小修改、测试状态、硬件参数保护和本地提交规则。
- 当前维护单元：新增 `docs/PROJECT_STATE.md` 和本日志，记录真实工程状态、风险、测试限制和下一位 AI 的交接信息。
- 当前维护单元：新增 `docs/ARCHITECTURE.md` 和 `docs/DECISIONS.md`，固定目录职责、实时链路、状态机、UART/底盘控制权和长期设计决策。
- 当前维护单元：新增 `docs/HARDWARE_VERIFICATION.md` 并更新 `README.md`，集中记录实机验收边界和长期维护入口。
- `54e31cd`：从 `MDK-ARM/GongChuangSai.uvprojx` 移除仅含注释的 `Hardware/main_example.c` 无效引用，保留历史文件但不再参与正式构建。
- 当前维护单元：同步旧验收文档中的 Keil 引用统计和工程清理说明，使其与当前 76 个引用一致。
- `673abab`：首次确认 GitHub 远程仓库和开发分支权限，将本地 `refactor/architecture-v2` 推送并验证两端 SHA 一致；新增 `tools/commit_and_push.ps1`，并更新自动提交/推送规则与版本同步状态。
- `a00371d`：修正自动同步脚本对 Git 正常推送输出的显示，并记录首次脚本驱动的 `SYNC PASS`。
- 当前维护单元：验证自动同步脚本的成功、敏感文件拒绝和无变化 `NOOP` 场景；`tests/race`、`tests/ulog` 和脚本语法检查均通过。
- 当前维护单元：将 `app_mode.h` 和 `race_config.h` 物理迁移到 `Config/`，同步 Keil 工程路径、IncludePath、主机测试 IncludePath 和相关文档；控制算法、引脚、PID 与配置参数未修改。

## Config 迁移最终验收（2026-11-17）

提交：本次文档验证提交（不在提交内容中自引用当前 SHA）
范围：`tests/race/run.ps1`、`tests/ulog/run.ps1`、`MDK-ARM/GongChuangSai.uvprojx`、`docs/PROJECT_STATE.md`
原因：完成 Config 迁移阶段的最终电脑端验收，确认不进入下一阶段迁移。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；Keil 工程 76 个文件引用、路径存在性、重复引用和 `../Config` IncludePath 静态检查 PASS；Keil/Arm BUILD SKIP（工具链不存在）。
实机：NOT_RUN；Config 阶段标记为 COMPLETE。

## Keil 编译与自动测试验收（2026-11-17）

提交：本次编译验收文档提交（不在提交内容中自引用当前 SHA）
范围：`docs/PROJECT_STATE.md`、`docs/CHANGELOG_AI.md`
原因：记录已定位的 Keil/Arm 工具链、命令行编译结果和主机测试结果。
验证：`D:\keil\UV4\UV4.exe -b MDK-ARM\GongChuangSai.uvprojx -j0 -o <build-log>` PASS；ArmCC V5.06 update 5 (build 528)，0 Error、0 Warning；`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS。
实机：NOT_RUN；未执行烧录和硬件在线调试，未修改业务代码。

## Control 阶段：Motor 模块迁移（2026-11-17）

提交：本次 Motor 迁移提交（不在提交内容中自引用当前 SHA）
范围：`Hardware/Motor.c`、`Hardware/Motor.h` → `Control/Motor.c`、`Control/Motor.h`；Keil IncludePath；race/ulog 测试 IncludePath；架构文档
原因：按 Control 层物理迁移计划整理轮毂电机速度环模块。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；迁移路径、旧路径清理、Keil 文件引用和 `../Control` IncludePath 静态检查 PASS；`git diff --check` PASS。
实机：NOT_RUN；未修改函数接口、PID、硬件参数或控制算法。

## Control 阶段：Mecanum chassis 模块迁移（2026-11-17）

提交：本次 Mecanum chassis 迁移提交（不在提交内容中自引用当前 SHA）
范围：`Hardware/mecanum_chassis.c`、`Hardware/mecanum_chassis.h` → `Control/mecanum_chassis.c`、`Control/mecanum_chassis.h`；Keil 工程路径；架构和 IMU/圆环文档路径
原因：按 Control 层物理迁移计划整理麦轮运动学、里程计和位置 PID 执行模块。
验证：迁移路径、旧路径清理、Keil 文件引用和主机 IncludePath 静态检查 PASS；待本单元完成后运行 `tests/race/run.ps1`、`tests/ulog/run.ps1` 并记录结果。
实机：NOT_RUN；不修改运动学、位置 PID、轮径、轮距或函数接口。

## Control 阶段：Navigation 模块迁移（2026-11-17）

提交：本次 Navigation 迁移提交（不在提交内容中自引用当前 SHA）
范围：`Hardware/nav.c`、`Hardware/nav.h` → `Control/nav.c`、`Control/nav.h`；Keil Control 分组；IMU 和项目文档路径
原因：完成 Control 层指定的导航辅助模块物理迁移。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；Keil 工程 76 个文件引用、Control 分组 6 个文件、路径存在性和重复引用静态检查 PASS；Keil 命令行编译 PASS，ArmCC V5.06 update 5 (build 528)，0 Error、0 Warning。
实机：NOT_RUN；不修改导航算法、阈值、硬件参数或函数接口。

## Control 阶段完成（2026-11-17）

本阶段指定的 `Motor.c/h`、`mecanum_chassis.c/h` 和 `nav.c/h` 均已迁移到 `Control/`，Keil 工程已拆出独立 `Control` 分组，主机测试依赖已加入 `-IControl`。本阶段完成后停止，未开始第三阶段 BSP 迁移。

## Control 反向依赖解除（2026-11-17）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`Control/nav.c`、`Control/nav.h`、`Control/mecanum_chassis.c`、`Control/mecanum_chassis.h`、`Hardware/app_runtime.c`、`Hardware/app_square.c`、`Hardware/display_nav.c`、`Hardware/display_nav.h`
原因：解除 Control 对 `app_imu.h` 和 `bsp_oled.h` 的反向依赖，保持姿态、复位和调试显示行为。
验证：Control 无上述两个头文件或对应 IMU 调用；`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；`git diff --check` PASS；Keil 完整编译 PASS（ArmCC V5.06 update 5，0 Error、0 Warning）。
实机：NOT_RUN；未修改 PID、运动学、IMU 算法、TIM6 周期、UART 或安全门。

## 记录格式

后续每个独立单元使用以下字段追加：

```text
提交：<commit id>
范围：<文件或模块>
原因：<问题或目标>
验证：<命令及 PASS/FAIL/SKIP/NOT_RUN>
实机：<NOT_RUN / 已完成的具体硬件验证>
```

## PID Burst 独立分析（2026-11-20）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`tools/bt10_ble_receiver.py`、`tools/BT10_BLE_RECEIVER.md`、`viewer/index.html`、`viewer/viewer.js`、`viewer/viewer.css`、`viewer/test.cjs` 及生成的 v3/v4 页面
原因：为停车后上传的高频电机内环数据提供独立接收、保存和分析路径，避免把高频样本混入 20 Hz ULog43 任务日志。
内容：接收 `#BTLOG,PID_BEGIN/PID/PID_END`，保存同名 `.pid.csv`；网页 PID 页面支持四轮目标/实际/输出字段，计算 IAE、ISE、峰值超调、稳定时间、尾段稳态误差和输出饱和占比，并生成有边界的人工候选参数。候选不会自动下发车辆。
验证：`node --check viewer/viewer.js`、`node viewer/build.cjs`、`node viewer/test.cjs`、`python -m py_compile tools/bt10_ble_receiver.py`、接收器 `--help`、`git diff --check`：`PASS`。
固件/实机：STM32 高频环形缓存和停车后 BT10 分批上传尚未接入正式固件；230400 吞吐、BLE 连接稳定性、缓存容量和实车验证：`NOT_RUN`。

## 本地 AI 辅助调参（2026-11-20）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`viewer/index.html`、`viewer/viewer.js`、`viewer/viewer.css`、`viewer/test.cjs`
方法依据：参考 WPILib SysId 的分段试验思想、Arduino PID AutoTune 的继电反馈风险边界、IMC-PI/FOPDT 初值思想，以及 SciPy/Optuna 类离线无梯度搜索的安全约束；本项目未复制第三方代码。
内容：增加低速模式和“保留前馈”选项；对 Burst 低速低误差样本做前馈斜率/偏置诊断；根据超调、稳定时间、饱和和稳态误差给候选排序、评分、置信度和解释。候选变化受边界限制，AI 只做本地可解释推荐，不联网、不写 Flash、不在比赛态下发参数。
验证：网页语法、构建、Burst 解析/指标、候选边界和 AI 排序断言：`PASS`。
限制：当前 Burst 只有总输出时仍无法完全分离前馈、PID、kick 和限幅；真正自动辨识需要记录各项控制分量，并在停车/台架模式完成低速正反向、横移、旋转和负载复测。

## DeepSeek 本地复核代理（2026-11-20）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`tools/deepseek_pid_proxy.py`、`viewer/index.html`、`viewer/viewer.js`、生成的 v3/v4 页面
内容：增加仅绑定 `127.0.0.1` 的本地代理。网页先运行本地确定性算法，再把结构化指标和候选交给 DeepSeek 做排序与解释；密钥只从 `DEEPSEEK_API_KEY` 环境变量读取，不写入源码、网页、日志或 Git。代理强制返回人工确认标志，不能写入车辆。
使用：`$env:DEEPSEEK_API_KEY='新的密钥'; python tools/deepseek_pid_proxy.py`，然后在网页 PID 页面点击“调用 DeepSeek 复核”。
安全边界：浏览器无法连接代理、密钥缺失、返回格式错误或模型不可用时，保留离线建议；真实 API 调用、费用、网络稳定性和实车参数验证：`NOT_RUN`。

补充：网页 PID 页面现在提供代理地址、连接检查和“调用 DeepSeek 复核”按钮。连接检查调用本机 `/health`，只有确认代理和密钥已配置后才启用复核按钮；浏览器不会启动后台 Python 进程。

补充：新增 `tools/start_deepseek_pid.ps1`，检查环境变量后后台启动代理并打开 `kart_log_view_mecanum_v4.html`，用于简化本机启动流程。
补充：启动脚本现在同时读取当前进程和 Windows 用户级 `DEEPSEEK_API_KEY`，自动选择 `python`/`py`，并等待健康接口最多 3 秒，减少双击启动失败。

## App 阶段：比赛业务模块物理迁移（2026-11-18）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`Hardware/` 下 12 对业务源文件迁移到 `App/`；`MDK-ARM/GongChuangSai.uvprojx`；`tests/race/run.ps1`；`tests/ulog/run.ps1`
原因：建立独立 App 物理目录，保持公开接口、比赛状态机、协议、控制参数和实时周期不变。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；Keil 工程 77 个文件引用、路径存在性和重复引用检查 PASS；UV4/ArmCC 完整构建 PASS（0 Error、0 Warning）；`git diff --check` PASS。
实机：NOT_RUN；`app_stepper_drive.c`、`bsp_d36_cmd.c`、`bsp_ulog.c`、`vision.h` 对 App 的既有反向依赖保留为后续解耦风险。

## Hardware 阶段：职责整理与依赖优化（2026-11-18）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`Hardware/bsp_ulog.c/.h`、`Hardware/vision.c/.h`、`Hardware/bsp_d36_cmd.c/.h`、`Hardware/app_stepper_drive.c`、`Config/stepper_config.h`、`App/app_runtime.c`、`App/app_stepper_openloop.h`、测试和 Keil 工程引用
原因：解除明确的 Hardware→App 反向依赖，保留硬件资源、协议、日志通道和实时周期。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；Keil UV4/ArmCC 完整构建 PASS（0 Error、0 Warning）；工程路径/重复引用、Hardware→App 依赖和 UART 回调唯一性检查 PASS；`git diff --check` PASS。
实机：NOT_RUN；未修改 IMU 底层驱动、Motor、TIM6/TIM8、UART 资源、GPIO/PWM/编码器映射或控制参数。

## 网页调试台迁移衔接文档（2026-11-18）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`docs/AI_HANDOFF.md`、`docs/PROJECT_STATE.md`
原因：为迁移到另一个工程提供可直接接续的网页文件清单、构建方式、交互能力、ULog43 通道定义、运动学假设、限制和验证边界。
验证：`node viewer/build.cjs` PASS；临时使用 `field_assets/logs/log0_kart.csv` 运行 `node viewer/test.cjs` PASS；`git diff --check` PASS。
实机：NOT_RUN；本单元只新增迁移文档，没有修改固件或网页业务代码。

## OpenMV 边走边扫二维码（2026-11-18）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`App/app_race.c`、`App/race_fsm.c/.h`、`Hardware/vision.c/.h`、相关 race 主机测试
原因：让二维码搜索任务在路线 `MOVE` 期间异步运行，保持里程计导航连续，并补充扫描失败向 App 的安全故障传递。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；UV4/ArmCC 完整构建 PASS（0 Error、0 Warning）；Keil 工程引用、USART3 回调唯一性、USART2/TIM6 未改和 `git diff --check` 检查通过。
实机：NOT_RUN；二维码搜索段几何和摄像头安装参数 NEED_CONFIRMATION，OpenMV 通信与底盘运动仍需实机验收。

## BT10 BLE 日志链路（2026-11-19）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`Hardware/bsp_ulog.c/.h`、`Hardware/bsp_uart_router.c`、`Core/Src/stm32f4xx_it.c`、`tools/bt10_ble_receiver.py`、`tools/BT10_BLE_RECEIVER.md`、`viewer/`、日志测试
原因：用 BT10 BLE GATT 替代 USART2 日志发送链路，Windows 保存 CSV，运行中只读并支持网页实时查看。
验证：`tests/ulog/run.ps1` PASS；`tests/race/run.ps1` PASS；`node --check viewer/viewer.js` PASS；`node viewer/test.cjs` PASS（使用 `field_assets/logs/log0_kart.csv`）；Python `py_compile` 和 `--help` PASS；`git diff --check` PASS。
实机：BT10 BLE 连接、GATT 特征、WebSocket 实时联调、烧录和整车流程验收 `NOT_RUN`；Keil 完整编译待执行。

### 兼容性结论补充

- BT10 继续发送 ULog43 数值行，既有网页解析、vx/vy、位移、轨迹、波形和 43 列 CSV 导入保持兼容；旧 ULog26 仍需转换，转换补零列不代表真实测量。
- 与 TF 卡相比，BT10 是无线实时通道，文件保存发生在网页 IndexedDB 或 Windows 接收器的电脑硬盘，车辆脱离电脑/接收端后没有车载持久副本；BLE 断链期间的数据无法补回。
- `#BTLOG,END` 只表示 BT10 接收端和网页会话结束，不等同于 TF 卡文件系统 close。旧 BT10 APP 控制协议、USART3 视觉协议、ULog43 日志格式彼此独立；当前不会把旧控制帧自动转换成日志行。
- 以上为代码和主机测试结论，真实 BT10 长时间传输、断链恢复及整车结束时序仍为 `NOT_RUN`。

## 双启停区二维码扫描运动流程（2026-11-20）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`App/app_race.c`、`App/race_fsm.c/.h`、`Config/race_config.h`、`Hardware/vision.c/.h`、OpenMV `main.py`、race 测试
原因：废弃旧版按扫码次数判区和 180°反扫，改为赛前人工选区、两条固定路线和公共汇合点；保留现有位置环、异步 QR 接收和 V4 交付确认。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；UV4/ArmCC PASS（0 Error、0 Warning）；OpenMV `py_compile` PASS；`git diff --check` PASS。
安全边界：场地绝对坐标转换、路点、摄像头安装、OpenMV 通信和整车动作尚未实机验证；`RACE_QR_FIELD_FRAME_CONFIRMED` 默认关闭，相关项目为 `NEED_CONFIRMATION`/`NOT_RUN`。

## 二维码后的原料区导航与视觉对准（2026-11-20）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`App/app_race.c/.h`、`Config/race_config.h`、`Hardware/vision.c/.h`、`tests/race/test_uart.c`、长期状态文档
原因：在既有二维码路线完成后增加公共汇合点到原料区的非阻塞导航、相机准备安全门、JOB40 圆盘事件/微调和 JOB41 固定抓取位事件/微调，阶段终点不启动抓取。
验证：`tests/race/run.ps1` PASS；`tests/ulog/run.ps1` PASS；OpenMV `py_compile` PASS；UV4/ArmCC 完整构建 PASS（0 Error、0 Warning）；`git diff --check` PASS。
安全边界：自动流程默认关闭；相机开环步数、俯仰舵机映射、圆盘/抓取位矩阵和 `GRIP_TARGET_CALIBRATED` 仍需实机确认，未完成时自动流程拒绝执行；实机验证 NOT_RUN。

## 比赛任务函数式架构规范同步（2026-11-20）

提交：本轮待提交，提交后不在长期文档中自引用当前 SHA
范围：`docs/RACE_ARCHITECTURE_RULES.md`、`AGENTS.md`、`docs/ARCHITECTURE.md`、`docs/DECISIONS.md`、`docs/PROJECT_STATE.md`
原因：将用户指定的 GongXun2025 任务函数式业务组织风格纳入长期规则，保留当前 STM32 非阻塞 Poll、TIM6 实时控制和既有底层接口。
验证：本轮为文档-only；链接检查和 `git diff --check` 执行后记录结果；主机测试、Keil 编译和实机验证 `NOT_RUN`。
边界：规范是未来开发约束，不代表当前 AppRace、RaceFsm、PickFsm 或视觉流程已经完成目标重构；未修改任何固件源码、工程配置或硬件参数。
## 2026-11-20

- 修复 `tools/start_deepseek_pid.ps1` 在未配置环境变量时静默退出的问题。
- 启动时使用隐藏输入临时 DeepSeek key，key 只通过当前进程环境变量传给代理，不写用户环境变量、网页或仓库。
- 增加 Python、端口占用、代理立即退出和 `/health` 检查提示；失败时等待用户按 Enter，避免双击后窗口瞬间关闭。
- 重复启动时若 8787 已是健康的本地代理，则直接复用并打开网页。
# 2026-11-20

- 新增可选的 1 kHz 电机内环 PID Burst 采样：TIM6 只写固定大小环形缓冲，主循环复用 ULog USART2 非阻塞发送队列输出四轮目标/实际/输出数据。
- Burst 默认关闭，满载仅丢样本并记录 `g_ulog_pid_burst_overrun`，不会阻塞比赛控制；输出协议为 `#BTLOG,PID_BEGIN`、`#BTLOG,PID`、`#BTLOG,PID_END`。


## 2026-11-20：20 秒 PID 诊断链路

- 将试验性 256 帧发送环替换为 100 Hz×2000 帧独立定点记录区，新增 200 Hz、36字节 CRC
  二进制实时通道和停车500ms后的记录上传。TIM6 1ms 调度、Motor 5ms 内部PID窗口和底盘10ms环不变。
- USART2 匹配 BT10 230400 8N1，日志IRQ优先级5；主循环非阻塞发送、保留普通ULog空间，
  队列饱和只降实时上传，不清空记录。采集默认未启动、显式停车开始，不控制车辆或写参数。
- 现有UART路由接入USART2有界接收队列；BEGIN/END与整车END排序，显式停止仅停止采集。
- 网页增加四轮目标/实测/总输出曲线、启停、下载、20秒协议自测、统计和仓库PID入口；
  Python增量组包、CRC恢复、兼容CSV与WebSocket、独立PID文件编号与统计保存。
- PASS：ULog和生产异步队列饱和、跨语言协议与分片、网页回归、race回归、Keil构建；
  NOT_RUN：实车通信、最坏ISR耗时、持续200Hz蓝牙、机构视觉并发与参数效果。未烧录。


## 比赛流程和日志正确性检查（2026-11-21）

- 修改既有 Hardware/bsp_ulog.c：修复低速车体速度被整数格式化为0；ch10～13保留4位小数，
  ch37右后轮目标统一保留1位小数，保持ULog43列数/字段位置及原有资源和调度。
- tests/ulog/test_ulog.c 增加正负低速、四轮目标/实际速度的真实CSV反解析回归，先复现失败后修复通过。
- docs/PROJECT_STATE.md 第24节记录比赛默认标定门、局部DONE停录、原料流程入口不通、公共点直接夹取、
  故障后实际轮速冻结可能阻止PID补传，以及子任务可观测性不足。未扩写或启用未标定比赛任务。
- PASS：ULog、race、网页回归、JS/Python语法、Keil0错误0警告、git diff --check。
  网页读取共享二进制夹具的并行竞争曾FAIL，顺序重跑PASS；测试不得并行读写同一夹具。
- NOT_RUN：烧录、真实GATT/TF、TIM6耗时、故障停车补传及全比赛验收；硬件待办同步到验收记录。


## 最小修复：故障/取消后PID补传（2026-11-21）

- App/app_runtime.c 只在终止故障/取消状态确认四轮目标、滤波目标和输出全部为0；
  Hardware/bsp_ulog.h/.c 传递内部确认，并仅在最终停录请求后允许替代冻结轮速完成补传。
- 复用500ms连续等待；确认撤销后计时复位；正常采集与比赛控制流程不变，无控制恢复或伪造零速。
- tests/ulog/test_ulog.c、run.ps1 增加冻结300 pulse/s、FAULT/ABORTED、门控撤销、提前上传拒绝和END回归。
  先复现FAIL，再修复PASS；ULog、race、网页回归、Keil0错误0警告、git diff --check均PASS。
- docs/PROJECT_STATE.md、HARDWARE_VERIFICATION.md 同步修复与实机NOT_RUN边界；未烧录。
