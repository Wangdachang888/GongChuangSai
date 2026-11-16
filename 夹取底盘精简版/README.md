# 夹取与底盘精简工程

这个工程只保留两条运行链：四轮麦克纳姆底盘闭环运动，以及夹取机构的单次动作状态机。视觉、二维码路线、日志、方形轨迹、电池显示、D36 文本命令和实验性舵机按键程序均未加入 Keil 工程。

## 工程入口

打开 `MDK-ARM/GongChuangSai_PickChassis.uvprojx`，执行 Rebuild All。当前环境没有 Keil 编译器，因此这里只完成了工程引用检查和 GCC 语法检查；烧录前仍需在 Keil 中实际编译。

## 运行方式

- 上电后保持夹取机构的升降机构在人工建立的零位；程序不会假装执行限位找零。
- `PE0` 按键短按一次：执行一次“地面取料到车上”。
- `PE0` 按键长按后松开：执行一次“车上取料到地面”。
- `PB14` 是夹爪，`PB15` 是整套夹持机构旋转，`PC7` 是托盘舵机。
- `PC7` 的正向任务按第几次取料依次使用 `90°`、回到 `0°`、相对转 `270°`，第三次完成后回到 `0°`。
- 第二路 D36 步进轴保持禁用，不会向 MOTORB 输出脉冲；升降只使用 `STEPPER_AXIS_1`，STEP 为 `PC6`。

## 代码分层

- `Hardware/app_pick_control.c`：按键消抖、短按/长按识别和状态机入口。
- `Hardware/app_pick_fsm.c`：夹取、转运、释放、复位的状态转换和超时处理。
- `Hardware/app_pick_motion.c`：把状态转换映射为 PB14、PB15、PC7 舵机和升降步进动作。
- `Hardware/app_pick_config.h`：夹爪角度、机构旋转角度、升降脉冲、速度和超时参数。
- `Hardware/bsp_servo.c`：PB14/PB15 定时器舵机和 PC7 软件定时舵机。
- `Hardware/app_stepper_openloop.c`、`app_stepper_drive.c`、`bsp_stepper.c`：升降步进脉冲及加减速。
- `Hardware/mecanum_chassis.c`、`Motor.c`、`nav.c`、`app_imu.c`：底盘四轮闭环、里程计和 ICM20948 姿态。

## 主要引脚

| 功能 | 引脚 |
|---|---|
| 四轮 PWM | PE5/PE6、PB8/PB9、PE9/PE11/PE13/PE14 |
| 四轮编码器 | TIM2、TIM3、TIM4、TIM5 |
| 升降 STEP | PC6 / TIM8_CH1 |
| 升降 DIR | PC8 |
| 升降 EN | PA2 |
| PB14/PB15 舵机 | TIM12_CH1/CH2 |
| PC7 托盘舵机 | TIM8_CH2 软件定时 |
| 按键 | PE0，上拉输入，按下为低电平 |
| ICM20948 | PB10/PB11 软件 I2C |

`PC7` 已从第二路步进轴中移出，`STEPPER_AXIS_2` 保持禁用。不要把 PC7 重新配置成 TIM8_CH2 PWM，否则会和托盘舵机冲突。

## 参数修改

只在 `Hardware/app_pick_config.h` 修改夹取参数。升降脉冲方向以当前手动零位和实物方向为准，修改后必须重新编译并烧录。这里的零位是软件参考零位，工程没有接入限位开关。
