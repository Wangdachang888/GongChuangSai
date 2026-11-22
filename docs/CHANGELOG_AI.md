# AI 维护变更日志

## 2026-11-22：修复 PID_TEST 回复在日志拥堵时丢失

- `PID_TEST` 的 `READY`、`NOT_READY`、`STARTED`、`STOPPED` 和 `STATUS` 回复改为可靠排队，UART 队列暂满时保留并重试，不再静默丢包。
- 网页对 RESET/STOP/STATUS 回复增加 2 秒超时提示；超时会明确指出没有收到车辆应答，不再显示无限等待。
- 验证：`node viewer/build.cjs`、`node viewer/test.cjs`、`tests/ulog/run.ps1 -Compiler D:\minGW\mingw64\bin\gcc.exe`、`git diff --check` PASS。未烧录，BT10 实车验证 NOT_RUN。

## 2026-11-22：修复 PID 高频会话 ARM 握手被整车日志拥堵阻塞

- PID 高频会话期间暂停同一 USART2 上低优先级的 43 列整车日志，保留 200 Hz 二进制 PID 实时帧，避免 230400 波特率下队列持续拥堵。
- `PID_ARMED` 改为主循环可重试的握手帧；有效的 `@PID START` 不再因瞬时队列不足被静默丢弃。停止 PID 会话时清除待发送握手状态。
- 未修改比赛状态机、TIM6 电机节拍、底盘目标或默认 PID。未烧录，BT10 实车连续测试仍需人工验收。
- 验证：`tests/ulog/run.ps1 -Compiler D:\minGW\mingw64\bin\gcc.exe` PASS；`git diff --check` PASS。

## 2026-11-22：取消 PID_Test 停车自动上传并解除连续测试阻塞

- 正式固件默认关闭 `ULOG_PID_PARKED_UPLOAD_ENABLE`。PID_Test 运行期间仍发送 200 Hz BT10 实时帧，网页持续归档；动作终止后直接发送 `#BTLOG,END`，不再等待最多 2000 条停车缓存。
- 保留 `ULOG_PID_PARKED_UPLOAD_ENABLE=1` 的主机兼容构建，仅用于旧协议解码和缓存回归，不参与正式固件行为。
- READY 状态仍要求日志和 PID 实时采集已设防，避免无采集时误启动；终态不再把已停止采集报告为运行期采集故障，允许直接 `TEST_RESET`。
- 验证：`tests/ulog/run.ps1`、`tests/race/run.ps1`、`node viewer/build.cjs`、`node viewer/test.cjs`、`git diff --check` PASS。未烧录，BT10 连续多轮与实车验收 NOT_RUN。

## 2026-11-22：增加 BT10 在线速度环控制配置写入

- `Motor_t` 保留默认宏值作为初始化，并增加运行时速度环周期窗口、积分限幅、输出限幅、死区补偿和前馈增益；TIM6 1 ms 电机更新与底盘 10 ms 调度不变。
- 新增兼容命令 `@PID CONFIG <id> <revision> <axis> <period_ms> <integral_max> <output_max> <deadzone> <ff_gain>`。只允许 FL/FR/RL/RR 速度环，周期限制 1…20 ms 且为整数，输出/积分/死区/前馈均做范围校验；停车、零输出、低速连续 500 ms 和版本校验沿用原门控，参数只写 RAM。
- 网页 PID 页增加单轮控制配置和四轮统一配置的一键写入，并对每轮读回、版本和最终值复核；旧 `@PID SET/GET` 保持兼容，位置环仍只写 Kp/Ki/Kd。
- 主机验证：`tests/ulog/run.ps1`、`tests/race/run.ps1`、`node viewer/build.cjs`、`node viewer/test.cjs`、`git diff --check` PASS；UV4 命令返回 0，PID_Test 输出已更新。BT10 实机写入、参数效果和比赛机械验收 NOT_RUN。

## 2026-11-22：增加伯德图专用独立测试动作

- PID_Test 新增可选 `APP_PID_TEST_FREQUENCY=1`：固定正向工作点叠加四个与 1024 点/5 ms
  频谱窗对齐的多正弦分量；4 s 预热、40.96 s 激励、2 s 停车，避免跨零和大行程。
- 默认比赛和原三档/creep动作不变；频域宏与creep互斥，反向要求独立构建和独立CSV。
- 文档说明宏设置、采集顺序、目标来源确认和不能拼接方向。固件编译、烧录、BT10传输和
  实车运行均未执行。

## 2026-11-21：伯德图与非参数频域PID初选

- 现有viewer.js/index.html加入参考互谱比频响、两半相干性/一致性、单方向小扰动门控与伯德图。
  不把闭环Y/R当电机、不用一次FFT比值或现有一阶拟合代替实测频率证据。
- 离散PI相位整形及有界P/I搜索保留Kd，带内相位余量/敏感度与预设压力检查后才可人工填入。
  频域初选独立于时域模型，原AI提案和固件停车写入/读回保持既有门控；不自动开车或写参。
- 新增已知离散系统幅相/延迟恢复、正向初选、噪声/工况变化/频带不足/数据缺口与UI来源确认回归。
  测试初次暴露密集确定性多正弦的Hann泄漏，以及带外单频噪声不能测试带内相干性的问题；
  夹具改为间隔激励频点上核对解析频响、独立宽带噪声，修正后重跑，不放宽实车门控。
- 同步生成v3/v4网页；实车11512帧只读复核，四轮均未形成可靠频带，不声称整定成功。
  PASS：node viewer/build.cjs、node viewer/test.cjs完整主机回归、JS语法、git diff --check。
  无固件/默认PID/硬件配置改动；实车频域初选效果、浏览器视觉与真实通信验收NOT_RUN。

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


## 持续PID实时流与滚动缓存解耦（2026-11-21）

- Hardware/bsp_ulog.c/.h：移除缓存满自动停止，复用2000帧环形缓存，显式停止后按时间
  顺序上传最近20秒，新增正常覆盖统计；普通ULog43/停车门控/终止补传与比赛控制保持不变。
- viewer/viewer.js、index.html及根目录v3/v4：持续采集按钮、累计实时帧数、30秒协议自测、
  完整实时与停车缓存分别导出；仓库v2兼容旧记录、事务分段保存及失败重试，移除全量raw副本。
- tests/ulog、viewer/test.cjs：360秒72006帧与序号回绕、最后20秒、旧仓库升级/分段/重试/
  新会话END导出与删除回归；同步更新BT10使用说明和项目状态/硬件验收边界。
- PASS：ULog/race、C→Python/JS、网页回归、JS/Python语法、Keil0错误0警告、git diff --check。
  网页初次使用旧20秒断言FAIL，更新30秒预期后PASS。静态RAM增加8字节；未烧录。
- NOT_RUN：BT10真实持续吞吐、浏览器实测、TIM6最坏耗时、整车视觉机构并发与长期保存配额。


## 在线PID参数与每轮真实基准（2026-11-21）

- 现有AppRuntime、Motor只读接口、bsp_ulog接通七通道实际参数GET/带版本SET；应用层停车
  门控、单通道原子修改与历史复位；采集/待上传拒绝写，END后显式应答不重启比赛或日志。
- 网页七组读取、单通道编辑、候选填入、SET后GET核对、上组回滚/本机保存值重应用；
  20字节GATT发送串行，断线/超时不自动重复写入。位置环支持手动写，禁止套用速度环候选。
- 采集添加真实PID_USED快照，网页/Python实时和停车导出保留；所选轮子候选和DeepSeek复核
  使用真实记录基准、周期、前馈上下文，不再把手填值/四轮平均当作该轮实际参数。
- PASS：ULog/race、实际运行层代码及常量主机测试、异步SET回复保留、跨语言协议、网页端到端
  读写/不匹配/超时/回滚、JS/Python语法、Keil0错误0警告、git diff --check。
  初次GET/READ匹配错误与测试桩/告警FAIL已修复并重跑PASS。
- 默认PID、前馈、资源、比赛流程不变，不写Flash、不烧录；实车GATT/时序/参数效果均NOT_RUN。
  未纳入工作区原有比赛配置/工程排版与未跟踪CSV；详见PROJECT_STATE第27节。

## PID辨识和DeepSeek证据分析v2（2026-11-21）

- 替换固定比例/旧日志重复扣分：带延迟ARX和输出误差辨识、60/20/20独立验证、训练尺度归一化、
  实测反馈控制器审计、实际PID/前馈/kick/限幅闭环重放、SIMC种子与局部搜索、有限扰动检查。
- 修复减速超调/短暂稳定/慢目标漂移误判，明确时间质量；新增所选轮子的文件曲线/预测叠加和
  正反向/零目标/停滞证据，候选各算自身结果。模型失败保留原参数，不制造“高分最优PID”。
- AI代理v2传递真实配置、分段、时序包络和验证结果；允许有界提案，经本地仿真才可填入，
  不再由AI重打分。升级代理健康版本及已验证旧进程处理，保留既有密钥管理和停车写入流程。
- 只读核对用户4133帧实车日志，FR/RL饱和13.43%/15.12%，控制器重放基本吻合，四轮简化
  模型独立验证均失败；不声称已得到可靠最优参数。原始CSV未入库，未改固件和比赛流程。
- 联网查阅GEKKO/SysIdentPy/python-control/simple-pid/继电整定/PidTuner/SIMC公开资料，
  参考来源及完整方法、限制、操作步骤更新至BT10说明；独立实现，无新第三方依赖。
- PASS：ULog/race、已知延迟/噪声模型、数据/模型拒绝、逐候选预测、真实文件、AI模拟提供者
  及跨语言证据、提案范围/迟到/转义、网页/仓库/参数通信、JS/Python/PowerShell语法与diff检查。
  旧网页候选数量断言及PowerShell中文管道断言初次FAIL，修正后PASS；诊断JSON读取编码已修正。
- NOT_RUN：新参数实车效果、浏览器/GATT、代理进程迁移、付费DeepSeek请求；本次无固件修改，
  Keil NOT_RUN，未烧录。用户原有三个未提交文件保持不变。

## 四轮PID统一输入和一键写入（2026-11-21）

- 既有PID页支持一组参数填四轮或直接编辑四行，一键依次写入并逐轮/整组版本读回；
  载入四轮实际值、本机保存值、各轮上次写入前值都只填表，明确RAM生效及复位重应用。
- 复用固件协议与停车门控，全部输入和四轮预检通过后才发SET；失败停止后续写入，
  清除可能过期的读回值，报告部分/未知结果，无自动重试/回滚，不声称固件原子批量。
- 补充逐轮实测及换地面/载重后重验证要求，候选预测不能替代场地验收；原日志模型仍不合格。
- PASS：构建/JS语法、ULog/race、网页与真实CSV回归，四轮输入/预检/串行读写/保存、
  拒绝/ACK丢失/版本变化和最终复核异常测试、git diff --check。固件/比赛资源未改。
- Keil及真实BT10/新参数效果NOT_RUN；浏览器file://预览受工具协议限制SKIP，无绕过。
  用户原有三个未提交文件哈希不变，未新增/删除/移动正式文件，未烧录。

## 独立PID速度数据采集模块（2026-11-21）

- 按用户授权新增app_pid_test.c/.h及PID_Test Target；默认关闭且编译模式互斥，
  AppRuntime集中复用PE0、既有车体系速度控制、IMU健康和日志，不修改比赛执行器。
- 默认0.03m/s纵向20秒三组正/零/反/零；单轴单档一次，终态锁存至复位。
  可配置横向、0.06/0.09及独立极低速4秒平台，不改变PID/前馈/斜坡/测速/资源/调度。
- 采集设防与物理按键联锁；取消、总超时、IMU/主循环/反馈/采集中断停止。
  测试运行不会冒充停车；终止后继续真实测速，普通停车500ms门控结束/补传，无伪造比赛状态。
- 复用原PID参数和日志协议，状态/原因用现有快照；操作与人工元数据见BT10说明。
  增补既有ULog测试，无新增正式测试目录/脚本；架构、决策、项目状态、硬件验证记录同步更新。
- PASS：ULog/race回归、新序列/配置/回绕/故障/按键/日志/PID访问测试；UV4双Target Rebuild，
  0错误0警告；比赛编译日志/链接输入/Map隔离；引用/重复/硬件差异和git diff --check。
  早期极低速限速测试断言错误与uVision跨Target自动加入空对象分别FAIL，修复后重跑PASS。
- 原工作区race_config.h、工程原有排版及CSV保留且不混入提交；两个Stash不改。
  NOT_RUN：烧录、实车动作/漂移/蓝牙/停稳、TIM6最坏时序；不代替用户硬件验收或承诺模型通过。


## 2026-11-21：两米空间三档单方向测试动作

- 在原app_pid_test/app_mode中替换普通动作，新增方向与L/M/H可覆盖配置；
  默认0.015/0.030/0.045m/s，ABC三遍、54s激励/56s全程，名义位移1.62m、超时70s。
  原极低速诊断保留；未更改比赛、PID/前馈、硬件分配、周期与日志协议。
- 更新既有ULog测试及BT10使用说明、决策、项目状态、硬件待验收记录，无正式文件结构变化。
- PASS：ULog/race回归、非法档位/方向/时序/位移/安全及日志链路；Keil双Target完整Rebuild
  均0错误0警告，比赛Map无测试模块，git diff --check。初次总时长口算断言FAIL后纠正重跑。
- 原有race_config/uvprojx/CSV哈希不变且排除提交；不烧录、不驱动车辆。
  NOT_RUN：实车停车包络、低速/场地适用性、蓝牙持续吞吐、模型有效性、TIM6最坏耗时。


## 2026-11-21：PID建模与AI审查整改

- 仅修改既有网页算法/界面、AI代理/启动器、测试与文档；生成根目录v3/v4离线网页。
  修正先测速后输出仿真顺序，审计/仿真共用控制步骤；只用200Hz原样本整定，100Hz只诊断。
  绘图保持原时间戳；基准重放35%收紧为25%，模型独立25%/15%与原参数边界保持。
- 新增既有PID页的跨CSV冻结验证、分方向/速度测试误差与训练范围、异常帧和零反馈提示；
  外部失败拒绝候选，文件/验证切换使旧AI过期。有界P/D种子与坐标细化补充原网格/SIMC。
  DeepSeek v3限制基于未对齐/缺失重放/外部失败证据的提案；不自动写车或放宽分数。
- PASS：实际Motor.c GCC→JS 1600周期逐模块一致性、独立已知模型/外部验证/100Hz拒绝、
  两份用户CSV只读回归、完整网页/蓝牙/仓库/四轮写参、ULog/race回归、语法与diff检查。
  初次测试C桩换行转义编译FAIL，修复后完整重跑PASS。未改正式固件、默认PID/前馈或硬件。
- 原race_config.h、uvprojx、pid_burst_demo.csv哈希不变且不提交；未新增正式文件/目录。
  NOT_RUN：实车、真实代理升级/付费DeepSeek请求、跨物理试验效果、最坏中断时序；Keil未重跑。
  多轮统计不确定性、位置环自动建模和新参数效果连续学习尚需数据，未标为已完成。
# 2026-11-22

- `feat(pid-test): add BT10 controlled multi-action sessions`：新增 BT10 主循环测试命令、请求 ID 应答、五种预定动作、终态自动关闭高频流和无需 MCU 复位的安全 rearm；比赛模式和既有日志/PID协议保持隔离。
- PID 页面新增动作选择、准备/开始/停止/状态按钮，并在 `PID_ARMED` 后启动动作、`PID_END` 后提示下一轮。
- 验证：`tests/ulog/run.ps1` PASS；`tests/race/run.ps1` PASS；`node viewer/build.cjs`、`node viewer/test.cjs`、`git diff --check` PASS。Keil、BT10 实机和机械验收 NOT_RUN。
# 2026-11-22

- PID 网页增加在线四轮死区测量入口，复用安全的低速正反向诊断动作；上传完成后生成正/反向阈值估计并可逐轮填入现有死区补偿字段，未自动写车。
- PID 停车缓存上传增加帧数、百分比、速率、已用时间、ETA 和完成状态可视化；保持单缓存一致性，下一轮仍在 `PID_END` 后准备。
