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

## 记录格式

后续每个独立单元使用以下字段追加：

```text
提交：<commit id>
范围：<文件或模块>
原因：<问题或目标>
验证：<命令及 PASS/FAIL/SKIP/NOT_RUN>
实机：<NOT_RUN / 已完成的具体硬件验证>
```
