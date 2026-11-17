# GongChuangSai

## 主工程

当前唯一正式 STM32 工程是根目录下的 `MDK-ARM/GongChuangSai.uvprojx`。源码位于
`Core/`、`Drivers/`、`IMU20948/` 和 `Hardware/`。历史工程副本保存在本机
`archive/historical/`，不参与构建；旧 ZIP 包保存在 `archive/packages/`。

OpenMV 的正式部署源代码是 `vision_openmv_stm32_v4_redownload/OpenMV/` 下的
`main.py`、`config.py` 和 `utils.py`。

## 目录职责

- `Core/`：CubeMX 生成的启动、外设和中断桥接。
- `Hardware/`：底盘、轮毂电机、IMU、视觉通信、圆环闭环、夹取状态机和设备驱动。
- `IMU20948/`：ICM20948 驱动和姿态解算。
- `MDK-ARM/`：Keil 工程文件。
- `tests/race/`、`tests/ulog/`：主机端协议、状态机和日志测试。
- `docs/`：架构、硬件和使用文档。
- `tools/`：日志分析和历史查看器工具。
- `field_assets/`：比赛场地素材和历史轨迹数据。

## 实时调度

`Core/Src/main.c` 只负责 HAL 初始化、定时器启动和主循环。`Hardware/app_runtime.c`
负责模式初始化与调度。TIM6 每 1 ms 更新四个轮毂电机，每 10 ms 调用一次麦轮
底盘位置环；视觉闭环沿用同一条底盘位置控制链，不创建第二套 PID。

## 运行模式

`Hardware/app_mode.h` 中的 `APP_RUN_MODE` 支持 `APP_MODE_LEGACY`、`APP_MODE_SERVO`、
`APP_MODE_STEPPER` 和 `APP_MODE_RACE`。比赛固件使用 `APP_MODE_RACE`；该模式保留
二维码路线、OpenMV V4 协议、旧 `#SEQ` 兼容、夹取状态机和数字 1/2/3 圆环闭环。
圆环自动运动要求 `Hardware/race_config.h` 中的标定矩阵有效，当前全零配置会安全
禁止自动移动。

## UART 资源

USART3 的 HAL UART 接收、发送完成和错误回调唯一归属
`Hardware/bsp_uart_router.c`。比赛模式由它转给 `Hardware/vision.c`；其他模式才
转给 D36 文本命令。视觉协议解析与底盘运动已分开：`vision.c` 只管理通信和视觉
数据，`app_ring_align.c` 管理像素到米制位移、限幅、底盘目标和 STEP_DONE 握手。

## 构建和测试

目标固件需要 Keil MDK/Arm 编译工具链；当前环境没有 `UV4`、`armcc` 或 `armclang`，
所以电脑端测试不能替代固件编译。主机测试会自动从 PATH 查找 `gcc`，也可以显式
传入工具链路径：

```powershell
.\tests\race\run.ps1
.\tests\ulog\run.ps1
.\tests\race\run.ps1 -Compiler C:\path\to\gcc.exe
```

缺少 GCC 时脚本明确报错。测试生成的 `.exe` 被 `.gitignore` 忽略。

## 验收文档

- `AGENTS.md`：新会话恢复、修改、测试、硬件保护和本地提交规则。
- `docs/PROJECT_STATE.md`：当前分支、默认配置、功能状态、风险和交接信息。
- `docs/ARCHITECTURE.md`：真实目录职责、初始化顺序、实时链路和状态机。
- `docs/DECISIONS.md`：长期架构与硬件保护决策。
- `docs/CHANGELOG_AI.md`：AI 维护提交和验证记录。
- `docs/HARDWARE_VERIFICATION.md`：目标硬件验收矩阵；未实测项目保持 `NOT_RUN`。
- `docs/architecture_before.md`：重构前调用链和耦合问题。
- `docs/architecture_after.md`：重构后目录、调用图和状态机。
- `docs/module_ownership.md`：硬件资源唯一管理者。
- `docs/uart_protocol.md`：USART3、V4 和旧二维码协议。
- `docs/ring_align_flow.md`：数字圆环闭环时序与安全条件。
- `docs/refactor_changes.md`：分批变更和保留项。
- `docs/acceptance_report.md`：自动验收、BUILD SKIP 和实机待验收项。

## 回退

重构只在 `refactor/architecture-v2` 分支进行，不自动合并稳定分支。回退到本次
重构前的稳定基线：

```powershell
git switch refactor/architecture-v2
git switch --detach 79de1dd
```

历史工程副本的完整内容仍保存在本机 `archive/historical/`；Git 中的原始版本可由
基线提交恢复。实机电机、视觉和比赛场地验收必须在目标硬件上另行完成。

## 长期维护

后续会话先读取 `AGENTS.md`、`docs/PROJECT_STATE.md`、`docs/ARCHITECTURE.md` 和
`docs/DECISIONS.md`，再检查 Git 状态和相关源码。每个独立修改都要运行相关测试、执行
`git diff --check`、更新长期文档并提交当前开发分支；不会自动合并、推送或烧录。
