# GongChuangSai 麦轮底盘 —— ICM-20948 组合导航 使用说明

> 本文件同时放在工程根目录（`README_IMU.md`）和模块目录（`IMU20948_Module\integration\INTEGRATION.md`）。
> 最后更新：2026-10-19

---

## 1. 整体结构

```
ICM20948 ─200Hz─► app_imu ──► yaw（陀螺 + 磁力计）
                                  │
轮速编码器 ─10ms─► 正运动学 ─► vx,vy,ω ─┤
                                  ▼
                          nav.c：航向融合 · 零速修正 · 打滑检测
                                  ▼
                     世界系 x,y,yaw ──► 底盘位置环
                                  ▼
                     display_nav.c：OLED 4 页可视化
```

| 层 | 文件 | 职责 |
|---|---|---|
| 驱动 | `IMU20948\src\icm20948.c` | 寄存器、量程、磁力计、零偏标定 |
| 姿态 | `IMU20948\src\imu_attitude.c` | 互补 / 卡尔曼 / Mahony |
| 移植 | `IMU20948\port\imu_port_stm32_hal.c` | PB10/PB11 软件 I2C（不依赖 I2C 外设） |
| 应用 | `Hardware\app_imu.c` | 200Hz 采样、姿态更新、掉线降级 |
| 导航 | `Hardware\nav.c` | 航向融合、ZUPT、打滑检测、位姿积分 |
| 显示 | `Hardware\display_nav.c` | OLED 4 页调参界面 |

---

## 2. 工程里改了哪些地方

| 文件 | 改动 |
|---|---|
| `Core\Src\main.c` | 打开 `OLED_Init()`；加 `DisplayNav_Init()`、`app_imu_init()`；主循环加 `app_imu_poll()` + `DisplayNav_Poll()` |
| `Control\mecanum_chassis.c` | `Mecanum_UpdateState()` 改为调用 `nav.c`；加 `MecanumChassis_GetNav()`；`Init`/`ResetPosition` 里初始化/复位导航 |
| `Control\mecanum_chassis.h` | 引入 `nav.h`，声明 `MecanumChassis_GetNav()` |
| `MDK-ARM\GongChuangSai.uvprojx` | 新增 `IMU20948` 组（3 个 .c）；`Hardware` 组加 3 个 .c；Include 加 `../IMU20948/inc` |

新增文件：`Hardware\app_imu.c/h`、`nav.c/h`、`display_nav.c/h`、`IMU20948\`（8 个文件）。

**Keil 工程已经配好，直接 Build 即可。**

---

## 3. 编译说明

- 编译器是 **AC5（`uAC6=0`，默认 C90）**。代码已按 C90 编写，
  `nav.c` / `imu_attitude.c` 里对 `fmodf/cosf/sinf` 做了显式声明，
  **不需要**额外勾 C99。
- 已知唯一警告：`mecanum_chassis.c(555): warning #177-D: function "Disp_SpeedVal" was declared but never referenced`
  —— 它是你原代码里只被注释掉的 `Chassis_Display_MotorSpeed()` 使用的函数，无害。
- 之前报过 `identifier "I2C_HandleTypeDef" is undefined`，已修：`app_imu.c` 按
  `HAL_I2C_MODULE_ENABLED` 条件编译，未启用 I2C 外设时传 `NULL`。

---

## 4. OLED 页面（中文）

**PE0 短按翻页；长按 0.8 秒 → 位姿 + IMU 航向一起归零。**

四页分别是：**姿态 / 位置 / 速度 / 状态**。

### P1　姿态（看车头朝哪、转得多快）

```
姿态          1/4
航向  +090.0
角速度 +0.12
读数 0012345
```

| 行 | 含义 | 单位 | 正常 |
|---|---|---|---|
| 航向 | IMU 解算出的车头朝向，逆时针为正 | 度 | 手转 90° 应变化 90° |
| 角速度 | 轮式里程计算出的车体旋转速度 | rad/s | 静止为 0 |
| 读数 | IMU 累计成功采样帧数 | 帧 | 每秒约 +200 |

### P2　位置（看车走到哪了）

```
位置          2/4
X    +1.234
Y    +0.567
航向  +090.0
```

| 行 | 含义 | 单位 |
|---|---|---|
| X | 世界系前后坐标，向前为正 | 米 |
| Y | 世界系左右坐标，向左为正 | 米 |
| 航向 | 当前车头朝向 | 度 |

上电时 X/Y 归零；长按 PE0 也会把 X/Y 和航向一起清零。

### P3　速度（看目标速度、看有没有打滑）

```
速度          3/4
前进  +0.35
转向  +0.12
空转  00000
```

| 行 | 含义 | 单位 |
|---|---|---|
| 前进 | 速度环的目标前进速度 | m/s |
| 转向 | 速度环的目标角速度 | rad/s |
| 空转 | 打滑累计次数（轮子空转时 +1） | 次 |

### P4　状态（看电压、看 IMU 有没有出错）

```
状态          4/4
电压  12.6/12.5
错误  0000
请按键
```

| 行 | 含义 | 正常 |
|---|---|---|
| 电压 | 驱动器侧 / 主板侧电压（V） | 12.6 左右；没接显示 `--.-` |
| 错误 | IMU 总线错误累计次数 | 恒 0 |
| 请按键 | 提示：短按 PE0 翻下一页 | — |

### 字库说明

汉字来自 `oledfont.h` 的 `cfont16` 表。原表 100 字，另补充了 13 字：
`姿 航 角 度 置 压 错 误 俯 仰 偏 米 秒`，现在共 113 字。
新增汉字用 Windows 黑体渲染成 16×16 点阵，格式与原有条目一致（32 字节/字）。

## 5. 首次上电检查顺序

| 步骤 | 看什么 | 正常 |
|---|---|---|
| 1 | OLED 是否亮 | 有画面 |
| 2 | P4 页 | `F` 每秒 +200、`ERR` 恒 0、`OK`=1 |
| 3 | P1 页，手转小车逆时针 90° | `A` ≈ **+90** |
| 4 | P1 页，推车前进 1m | `X` 变化约 1.0；停下 1~2 拍后 `S`=1 |
| 5 | P2 页，跑位置模式 | `XR/XA` 收敛 |

**第 3 步若显示 -90**：`Hardware\app_imu.h` 里
```c
#define APP_IMU_YAW_SIGN   (1.0f)   →   (-1.0f)
```

---

## 6. 调参

默认值在 `nav.c` 的 `Nav_Init()`：

```c
zupt_v = 0.015;  zupt_w = 0.05;
slip_thr = 0.35; slip_v_scale = 0.25;
wheel_weight = 0.10;
```

| 现象 | 调整 |
|---|---|
| 停车后位置还慢慢变 | `zupt_v` 调大（0.03） |
| 原地慢转误判打滑 | `slip_thr` 调大（0.6） |
| 真打滑时位置冲出去 | `slip_v_scale` 调小（0.15） |
| 长时间跑航向偏 | `wheel_weight` 调大（0.2） |
| 室内磁干扰大 | `app_imu.c` 里 `cfg.enable_mag = false` |

也可以运行期调用：

```c
Nav_t *nav = MecanumChassis_GetNav();
Nav_SetParams(nav, 0.015f, 0.05f, 0.35f, 0.10f);
```

---

## 7. 注意事项

1. **PB10/PB11 不需要在 CubeMX 里配置**：软件 I2C 在 `app_imu_init()` 里把它们配成开漏输出 + 上拉。
   PB12（INT）也只在 `APP_IMU_USE_DRDY_INT = 1` 时才自动配置成 EXTI。
2. **PE0 被改成上拉输入**（CubeMX 里原为推挽输出）。板子说明 PE0 是用户按键；
   若实际没按键，翻页不响应，导航照常。要关掉：`display_nav.h` 里 `DISPLAY_NAV_USE_KEY` 改 0。
3. **`app_imu_init()` 阻塞约 1 秒**（复位 + 零偏标定），期间车必须静止；
   已放在 `HAL_TIM_Base_Start_IT(&htim6)` 之前。
4. **`app_imu_poll()` 不要放进 TIM6 中断**：I2C 阻塞会打乱 1ms 控制节拍。
5. **重新用 CubeMX 生成代码是安全的**：所有改动都在 USER CODE 区或 `Hardware\` 下。
   但生成后 `MX_GPIO_Init()` 会把 PE0 改回输出，`DisplayNav_Init()` 会在其后重新配成输入，无需手动处理。
6. **如果要开 PB12 中断**：`app_imu.h` 里 `APP_IMU_USE_DRDY_INT` 改 1；
   若同时在 CubeMX 里给 PB12 配了 EXTI，则必须把 `APP_IMU_EXTI_MANAGED` 改 0（否则中断入口重复定义）。

---

## 8. 备份

原始文件已移动到 `_backup_20261018\`：

- `Core_Src_main.c.bak_*`
- `Hardware_mecanum_chassis.c.bak_*`
- `Hardware_mecanum_chassis.h.bak_*`
- `Hardware_app_imu.c.bak_*`
- `MDK-ARM_GongChuangSai.uvprojx.bak_*`

回退方式：把对应文件拷回原路径即可。

---

## 9. 回归测试

PC 端测试放在 `IMU20948_Module\test\`，`run_tests.ps1` 一键运行：

| 套件 | 检查项 |
|---|---|
| 驱动 + 姿态算法 | 98 |
| app_imu 应用层 | 21 |
| 组合导航 nav | 25 |
| **合计** | **144 全过** |

导航测试覆盖：直线 1m 位移、转 90° 后前进 1m 的世界系坐标、打滑判定、
ZUPT 冻结位姿、IMU 掉线退化为轮式航向、位姿重置。
