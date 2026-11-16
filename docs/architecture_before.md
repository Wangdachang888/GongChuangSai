# GongChuangSai 重构前架构基线

基线提交：`79de1dd`  
重构分支：`refactor/architecture-v2`  
记录时间：2026-11-17

## 当前入口与启动顺序

`Core/Src/main.c` 只包含 HAL、时钟、GPIO、定时器、USART3 初始化，然后调用
`AppRuntime_Init()`，启动 TIM6 中断，并在主循环中持续调用 `AppRuntime_Poll()`。

```text
main
 ├─ HAL_Init / SystemClock_Config
 ├─ MX_GPIO_Init / MX_TIM*_Init / MX_USART3_UART_Init
 ├─ AppRuntime_Init
 │   ├─ 根据 APP_RUN_MODE 初始化步进、舵机、OLED、D36、显示、IMU
 │   ├─ 初始化四个 Motor_t 和 MecanumChassis_t
 │   ├─ APP_MODE_RACE: ULog_Init -> AppRace_Init
 │   └─ 停止底盘输出
 ├─ HAL_TIM_Base_Start_IT(TIM6)
 └─ while(1): AppRuntime_Poll
```

`AppRuntime_Poll()` 在比赛模式中依次执行 IMU 轮询、`AppRace_Poll()`、电池轮询和
日志轮询。TIM6 每 1 ms 进入 `AppRuntime_MotorTick()` 更新四个轮毂电机；每 10 次
中断进入 `AppRuntime_ChassisTick()`，调用底盘位置环。`AppRace_ControlTick()` 是
比赛模式的故障门控，IMU 失联或主循环停滞时禁止继续输出。

## 主要模块职责（重构前）

| 模块 | 当前职责 | 公开接口重点 |
| --- | --- | --- |
| `app_runtime.c` | 模式分支、硬件实例、PID 初值、主循环调度、TIM6 桥接 | `AppRuntime_Init/Poll/ControlTick/MotorTick/ChassisTick` |
| `app_race.c` | 比赛协调、按键、视觉、底盘动作切换、夹取状态机和日志状态 | `AppRace_Init/Poll/ControlTick/StartRingAlign` |
| `race_fsm.c` | 启停区、二维码路线、移动/转向/扫描动作状态 | `Race_StartQrRoute/Race_Tick/Race_Abort` |
| `race_task_plan.c` | 根据二维码颜色和位置生成物料任务队列 | `RaceTaskPlan_Build/Start/Current/Advance/Done` |
| `app_pick_fsm.c` | 夹爪、夹持旋转、升降、转盘的非阻塞顺序和超时 | `PickFsm_Start/StartReverse/Poll/Abort` |
| `app_pick_motion.c` | 将动作姿态和高度转换为舵机角度、步数和速度 | `PickMotion_Set*/StartLift` |
| `bsp_stepper.c`、`app_stepper_*` | D36 步进底层脉冲和上层运动接口 | `AppStepper_*`、`Stepper_*` |
| `bsp_servo.c` | 舵机 PWM、速度/加速度轨迹 | `Servo_*` |
| `vision.c` | USART3 收发、V4/旧二维码协议、位置缓存、圆环算法和底盘写入 | `Vision_*`、`RaceVision_*` |
| `bsp_uart_router.c` | 唯一 HAL UART 回调入口 | `HAL_UART_*Callback` |
| `mecanum_chassis.c` | 编码器里程计、麦轮运动学、位置 PID 和输出 | `MecanumChassis_*`、`Mecanum_SetPositionMode` |
| `Motor.c` | 单轮编码器速度环和 PWM 输出 | `Motor_Init/Update/Stop` |
| `nav.c` | 里程计/姿态导航辅助 | `Nav_*` |

## 当前依赖与控制权

```text
main.c
  -> app_runtime.c
       -> app_race.c
            -> race_fsm.c / race_task_plan.c
            -> app_pick_fsm.c -> app_pick_motion.c -> bsp_servo.c / app_stepper_openloop.c
            -> vision.c -> Mecanum_SetPositionMode (圆环路径)
       -> mecanum_chassis.c -> Motor.c / nav.c / app_imu.c
       -> bsp_uart_router.c -> vision.c (比赛模式)
```

比赛模式正常路径的底盘目标由 `app_race.c` 写入；圆环路径目前绕过协调层，
由 `vision.c` 直接持有 `MecanumChassis_t *` 并调用 `Mecanum_SetPositionMode()`。
这是当前最主要的业务层与设备控制层耦合点。`MecanumChassis_Loop()` 只在 TIM6
的 10 ms 节拍中执行，因此任何新的圆环模块都必须沿用这条实时路径，不能新增
第二套位置 PID。

USART3 的 HAL 接收、发送完成和错误回调只有 `Hardware/bsp_uart_router.c` 一个
入口。比赛模式由路由器转给 `vision.c`；非比赛模式才转给 D36 文本命令模块。

## 比赛状态与夹取状态

`app_race.c` 负责将 `RaceFsm` 的路径状态、视觉二维码结果和 `PickFsm` 的完成
状态拼接起来。`RaceTaskPlan` 能生成两批物料、存放、叠放和回起点等动作，但当前
默认 `RACE_PICK_DIRECT_KEY_TEST=1`，按键直接启动夹取测试；因此不能把任务队列中
定义的全部动作描述为已在主流程中执行。

夹取状态机是非阻塞的：每次 `PickFsm_Poll()` 只推进一个状态，底层舵机和步进由
各自的 `Poll`/TIM8 机制继续运行，并使用舵机、步进超时和方向稳定等待保护。该
模块与底盘控制相互独立，但由 `app_race.c` 统一调度。

## 功能分类

### 当前比赛模式实际执行

- 四轮麦克纳姆编码器速度环、里程计、IMU 和底盘位置 PID。
- USART3 的 OpenMV V4 帧协议、旧 `#SEQ` 二维码兼容和扫码状态。
- 启停区选择、预热/按键启动、路线动作、故障停止。
- `RACE_PICK_DIRECT_KEY_TEST=1` 下的按键夹取动作。
- 数字圆环闭环接口已存在，但只有有效标定矩阵才允许自动运动；当前配置矩阵为
  全零，因此默认禁止自动闭环移动。
- ULog 运行日志。

### 已保留但当前没有完整接入主比赛链路

- `RaceTaskPlan` 中的两批物料、存放、叠放和回起点完整任务描述。
- 非直接按键模式下的完整夹取任务衔接。
- D36 文本步进调试命令（比赛模式不占用 USART3）。

### 仅供测试/兼容模式

- `APP_MODE_SERVO` 舵机按键模式。
- `APP_MODE_STEPPER`/`APP_MODE_LEGACY` 的方形、调参、D36 和显示调试路径。
- `tests/race`、`tests/ulog` 的主机模拟测试。

## 本次重构的边界

保留 `Core/`、`Drivers/`、`IMU20948/`、`MDK-ARM/`、底盘运动学和所有已验证机械
参数。重构只整理职责、接口和仓库资产，不改变电机方向、PID、轮径、轮距、编码器、
步进脉冲或夹取机械参数。圆环算法将从 `vision.c` 移到独立的应用协调模块；视觉
模块只保留通信、协议校验、事件和位置数据缓存。

架构参考为 `D:\浏览器下载文件\GongXun2025-main.zip` 中的 `控制代码` 目录，
仅借鉴其按 `CarBody`、`Hardware`、`Control` 分层的组织方式，不迁移其旧驱动或
替换当前 HAL/实时控制实现。
