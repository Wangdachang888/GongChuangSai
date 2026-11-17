# 工程资产归档清单

## 正式工程

当前唯一参与构建的 STM32 工程为根目录下的 `Core/`、`Drivers/`、`IMU20948/`、
`Hardware/` 和 `MDK-ARM/GongChuangSai.uvprojx`。OpenMV 正式部署源代码为
`vision_openmv_stm32_v4_redownload/OpenMV/` 下的 `main.py`、`config.py` 和
`utils.py`。

## 历史工程副本

以下目录均为历史调试或精简版本，不加入当前 Keil 工程，也不作为源码依赖。它们
会移到 `archive/historical/`，保留完整内容以便回溯：

| 原目录 | 主要内容 | 处理 |
| --- | --- | --- |
| `当前抓取控制代码_整理版/` | 旧视觉协议拆分版和夹取状态机 | 归档保留 |
| `夹取底盘精简版/` | 旧底盘、IMU、步进和夹取组合工程 | 归档保留 |
| `整理版_底盘调试夹取/` | 旧 D36 命令、底盘和夹取调试工程 | 归档保留 |

核对结果显示，旧目录包含未迁移的 `race_vision*`、`app_pick_control*` 和旧 D36
调试实现，但它们与当前 `vision.*`、`app_pick_fsm.*`、`app_pick_motion.*` 和
`bsp_d36_cmd.*` 接口不兼容，因此不直接覆盖当前实现。

## 工具与数据

- `tools/log_analysis/`：历史日志分析 Python 脚本。
- `tools/viewer/`：旧版 HTML 查看器和渲染检查脚本；正式可用的查看器仍在
  `viewer/index.html`。
- `field_assets/logs/`：历史 CSV 轨迹日志。
- `docs/hardware/`：D36 接线图、OLED 预览和原有使用说明。
- `archive/packages/`：旧工程 ZIP 包和查看器工程包，作为可恢复输入保存。

归档目录只改变文件路径，不改变文件内容。当前构建脚本、Keil 工程和 OpenMV
运行源代码均不引用归档目录。
