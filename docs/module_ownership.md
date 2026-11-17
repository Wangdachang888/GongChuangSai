# 模块与资源所有权

本表描述正式工程的唯一管理者和跨模块调用边界。历史目录不属于运行时依赖。

| 资源或职责 | 唯一管理者 | 允许的调用方 | 禁止事项 |
| --- | --- | --- | --- |
| TIM6 实时调度 | `Core/Src/main.c` 的 `HAL_TIM_PeriodElapsedCallback` | `AppRuntime_ControlTick`、`AppRuntime_MotorTick`、`AppRuntime_ChassisTick` | 其他模块定义第二个 TIM6 回调。 |
| 四轮电机速度环与 PWM | `Control/Motor.c` | `app_runtime.c` 的 1 ms 调度 | 应用层直接改 PWM 或编码器计数。 |
| 底盘运动学、里程计和位置 PID | `Control/mecanum_chassis.c` | `app_runtime.c` 的 10 ms 调度 | 新增第二套位置 PID。 |
| 导航辅助与位姿反馈 | `Control/nav.c` | `Control/mecanum_chassis.c` | 修改导航阈值或姿态融合链而不重新验收。 |
| 底盘位置目标 | 比赛模式由 `App/app_race.c` 和 `App/app_ring_align.c` 通过协调状态互斥写入 | 比赛状态机、圆环闭环 | `vision.c` 直接写目标；两个应用同时运行。 |
| USART3 HAL 回调 | `Hardware/bsp_uart_router.c` | 路由到 `vision.c` 或非比赛 D36 模块 | 在视觉、D36 或测试业务中重复定义 HAL 回调。 |
| OpenMV 消息与协议状态 | `Hardware/vision.c` | `app_race.c`、`app_ring_align.c` | UART ISR 中执行底盘、矩阵或机械动作。 |
| 圆环微调状态 | `App/app_ring_align.c` | `App/app_race.c` | 用旧坐标重复移动，或在移动中宣告到位。 |
| 比赛路线状态 | `App/race_fsm.c` | `App/app_race.c` | 任务规划模块直接写底盘。 |
| 任务顺序规划 | `App/race_task_plan.c` | `App/app_race.c` | 重复消费同一个任务完成事件。 |
| 机械抓取状态 | `App/app_pick_fsm.c` | `App/app_race.c` 调度 | 在 `main.c` 或 UART 回调中控制舵机、步进。 |
| 机械动作参数 | `App/app_pick_motion.c`、`Config/race_config.h` | `App/app_pick_fsm.c` | 修改已验证机械参数而不重新验收。 |
| 步进脉冲 | `Hardware/bsp_stepper.c`、`app_stepper_openloop.c` | 夹取应用或非比赛调试模式 | 业务层直接操作 TIM8 CCR。 |
| 舵机轨迹 | `Hardware/bsp_servo.c` | 夹取状态机、按键测试模式 | 使用阻塞延时替代 `Servo_Poll`。 |
| IMU 数据与姿态 | `Hardware/app_imu.c`、`IMU20948/` | 运行时轮询、底盘反馈 | 视觉模块直接读取或修改 IMU 状态。 |
| 日志 | `Hardware/bsp_ulog.c` | `AppRuntime_Poll`、比赛应用 | 在实时中断中格式化和写文件。 |

比赛模式下，`bsp_uart_router.c` 将 USART3 唯一转给 `vision.c`；D36 文本命令只在
非比赛模式启用。圆环通信和圆环运动分离，但通过 `AppRingAlign` 的 active 状态保证
底盘控制权不重叠。
