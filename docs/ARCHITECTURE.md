# GongChuangSai 软件架构

## 1. 目录与职责

| 目录或模块 | 职责 | 不应承担的职责 |
| --- | --- | --- |
| `Config/` | 运行模式和比赛参数配置头文件 | 控制算法、硬件驱动和运行时状态 |
| `Core/Src/main.c` | HAL、外设初始化、TIM6 启动、主循环和唯一 TIM6 周期回调 | 比赛业务、视觉解析、机械动作 |
| `App/app_runtime.c` | 运行模式初始化、主循环调度、TIM6 到电机/底盘的桥接 | 新增第二套控制器 |
| `Control/Motor.c` | 单轮 PWM、编码器采样和速度闭环 | 四轮运动学和比赛流程 |
| `Control/mecanum_chassis.c` | 麦轮运动学、里程计、位置 PID、速度/位置目标执行 | UART 协议和机械夹取 |
| `Control/nav.c` | 航向融合、零速修正、打滑检测和位姿辅助 | 直接写底盘目标或处理 UART |
| `App/app_race.c` | 比赛协调、按键、路线、视觉、圆环、夹取互斥 | 直接解析 UART 字节、实现第二套底盘 PID |
| `App/race_fsm.c` | 启停区、路线动作、扫描、到位、取消、超时和故障状态 | 直接操作硬件 UART 或舵机 |
| `App/race_task_plan.c` | 根据二维码颜色/位置生成和推进任务计划 | 直接写底盘目标 |
| `Hardware/vision.c` | USART3 字节流、V4/旧协议、二维码和圆环握手数据 | 直接调用 `Mecanum_SetPositionMode` |
| `App/app_ring_align.c` | 圆环标定换算、限幅、停稳判定、底盘目标和完成握手 | 解析原始 UART 帧 |
| `App/app_pick_fsm.c` | 夹爪 PB14、机构旋转 PB15、升降步进和 PC7 转盘的非阻塞流程 | 控制底盘 |
| `Hardware/bsp_uart_router.c` | 唯一 HAL UART 回调入口，按编译模式路由 | 阻塞发送或执行业务动作 |
| `Hardware/bsp_stepper_openloop.c` 等 | D36/步进脉冲、方向、使能和运动曲线 | 解释比赛任务 |
| `IMU20948/`、`Hardware/app_imu.c` | ICM20948 驱动、姿态解算、偏置和有效性 | 改写底盘目标 |
| `MDK-ARM/`、`GongChuangSai.ioc` | 正式工程和 CubeMX 资源描述 | 作为运行时业务代码 |

## 2. 初始化顺序

```text
HAL_Init
  -> SystemClock_Config
  -> MX_GPIO_Init
  -> MX_TIM1/2/3/4/5/6/9/10/11/12/8_Init
  -> MX_USART3_UART_Init
  -> AppRuntime_Init
       -> 运行模式设备初始化
       -> 四轮 Motor_Init
       -> MecanumChassis_Init
       -> IMU、视觉、夹取和日志初始化
  -> HAL_TIM_Base_Start_IT(TIM6)
  -> while(1): AppRuntime_Poll
```

比赛模式中，`AppRace_Init` 初始化比赛 FSM、输入、UI、夹取 FSM、视觉上下文和圆环控制；圆环标定矩阵无效时，圆环移动由安全门拒绝。

## 3. 实时调度

TIM6 的计数周期是 1 ms。`Core/Src/main.c` 的唯一周期回调先执行四路 `Motor_Update(..., 0.001f)`，每累计 10 次调用一次 `AppRuntime_ChassisTick()`，从而保持 10 ms 麦轮位置环。`AppRuntime_ControlTick()` 在比赛模式下检查 IMU/主循环停滞状态，异常时阻止继续输出并触发故障处理。

主循环中的 `AppRuntime_Poll()` 运行非中断任务：IMU 轮询、视觉协议轮询、比赛 FSM、夹取 FSM、舵机/步进轮询、日志和显示。所有任务必须保持非阻塞；任何串口长报文都应排队发送。

## 4. 比赛状态机

`app_race.c` 是协调层，主要顺序为：

1. 更新 IMU 帧时间和当前底盘反馈。
2. 处理视觉帧并提交二维码任务计划。
3. 若圆环闭环正在运行，则把底盘控制权交给 `app_ring_align.c`，其他动作暂停。
4. 处理按键启动、长按选区、短按开始和运行中取消。
5. 调用 `Race_Tick` 推进移动、转向、扫码、等待和故障状态。
6. 进入夹取任务时只启动 `PickFsm`，夹取完成后推进任务计划。
7. 在状态切换时由比赛协调层设置一次位置目标，底盘位置环在 TIM6 的 10 ms 链路中执行。

```mermaid
stateDiagram-v2
    [*] --> READY
    READY --> RUNNING: 按键/路线开始
    RUNNING --> MOVE: 移动动作
    MOVE --> TURN: 到位
    TURN --> SCAN: 到位
    SCAN --> RUNNING: 收到二维码/任务计划
    RUNNING --> PICK: 当前动作是夹取
    PICK --> RUNNING: PickFsm 完成
    RUNNING --> DONE: 计划完成
    READY --> FAULT: IMU/视觉/配置错误
    RUNNING --> ABORTED: 按键取消
    RUNNING --> FAULT: 超时/停滞
```

`RACE_PICK_DIRECT_KEY_TEST` 当前默认开启，因此按键可以直接启动夹取测试。该模式是当前代码事实，不能描述为完整比赛任务链已完成。

## 5. 夹取状态机

`app_pick_fsm.c` 通过 `app_pick_motion.c` 发出设备目标，使用忙状态和超时推进，不用 `HAL_Delay`。正向流程覆盖归位、升降接近、抓取、夹紧、上升、放置、夹爪打开、旋转归位和转盘多次动作；反向流程覆盖从车上取料到地面、预夹紧、旋转、放下和升降归位。具体步数、角度、速度和触发剩余步数集中在 `Config/race_config.h` 并由 `PickConfig` 注入。

升降零位是人工建立的开环软件坐标，当前代码没有声称存在自动限位寻找。PB14/PB15/PC7 的舵机动作由统一舵机驱动器排队，PC6/PC8/PC9 的步进由升降轴驱动器控制；转盘步进电机旧控制路径不应与 PC7 舵机路径同时启用。

## 6. OpenMV 与 USART3

`bsp_uart_router.c` 是唯一 HAL UART 回调入口。比赛模式下：

```text
USART3 RX IRQ
  -> HAL_UART_RxCpltCallback
  -> Vision_RxCpltCallback
  -> Vision_RxByte / 环形接收缓冲
  -> Vision_Poll
  -> RaceVision / AppRingAlign / AppRace
```

视觉 TX 使用非阻塞队列和发送完成回调。V4 报文、旧 `#SEQ` 行、二维码结果和圆环 ACK/位置/`STEP_DONE`/`ARRIVED`/`IDLE` 保持兼容。视觉解析不直接写底盘；圆环控制接收结构化数据后计算位移并调用唯一的底盘位置接口。

非比赛模式可以由同一个路由入口把 USART3 交给 D36 文本命令模块。禁止同时启用两个协议消费者，也禁止增加第二个 `HAL_UART_RxCpltCallback`。

## 7. 底盘控制权

正式底盘执行路径只有 `mecanum_chassis.c`。比赛协调、圆环闭环和历史路线模块都只能在自己的动作边界设置目标，不能并行写入。`AppRace` 在圆环活动时设置互斥标志并暂停普通路线；故障、取消和超时统一调用停止路径。TIM6 只消费当前目标并执行电机/底盘控制，不在中断内决定比赛动作。

历史 `app_square.c`、调试显示和 D36 模式仍可在非比赛模式使用，但不能和比赛模式同时成为有效目标写入者。`main_example.c` 仅是注释示例，已从正式 Keil 源文件列表移除后不再参与构建。

## 8. 验证边界

`tests/race` 和 `tests/ulog` 用主机桩验证协议、状态机、握手、错误和日志；它们不能验证 STM32 中断时序、舵机负载、步进失步、机械惯性、摄像头安装和 SD 卡介质。真实验证结果必须写入 `docs/HARDWARE_VERIFICATION.md`。
