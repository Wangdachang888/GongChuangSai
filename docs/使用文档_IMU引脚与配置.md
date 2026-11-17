# ICM-20948 组合导航 —— 引脚与配置核对文档

> 用途：逐项核对本模块**用到了哪些引脚、配成什么模式、依据是什么**，
> 以及 CubeMX 里应该/不应该配置什么。所有数值均从工程当前代码中读取，
> 标注了对应文件与函数，可逐条对照。
>
> 工程：`D:\New Folder\GongChuangSai`（STM32F407VET6，HAL，AC5）
> 模块：`IMU20948\` + `Hardware\app_imu.c` / `nav.c` / `display_nav.c`

---

## 0. 一分钟结论

| 检查项 | 结论 |
|---|---|
| 我用了哪些引脚 | **PB10（SCL）、PB11（SDA）必用**；PB12（INT）默认不用；PE0（按键）用于翻页 |
| 这些引脚要在 CubeMX 里配吗 | **不需要**。PB10/PB11 在运行期被配成开漏输出；PB12/PE0 只在启用对应功能时才配置 |
| 和主板资源说明冲突吗 | **不冲突**（PB10/PB11/PB12 本来就是 ICM20948 专用；PD8/PD9 是 USART3，我们不碰） |
| 用硬件 I2C 吗 | **不用**。工程未启用 I2C 外设，走 GPIO 模拟（软件 I2C） |
| 会占用定时器吗 | **不会**。微秒延时不占定时器（软件 I2C 用 DWT 的 CYCCNT） |

---

## 1. 引脚使用总表

### 1.1 本模块直接使用的引脚

| 引脚 | 用途 | 模式 | 上拉/下拉 | 速度 | 初始电平 | 配置位置 |
|---|---|---|---|---|---|---|
| **PB10** | I2C SCL | `GPIO_MODE_OUTPUT_OD`（开漏输出） | `GPIO_PULLUP` | `GPIO_SPEED_FREQ_HIGH` | 高（释放） | `IMU20948\port\imu_port_stm32_hal.c` → `soft_i2c_gpio_init()` |
| **PB11** | I2C SDA | `GPIO_MODE_OUTPUT_OD` | `GPIO_PULLUP` | `GPIO_SPEED_FREQ_HIGH` | 高（释放） | 同上 |
| **PB12** | ICM20948 INT1（数据就绪） | `GPIO_MODE_INPUT` + EXTI | 默认 `NOPULL`（高有效）；`PULLUP`（低有效） | `GPIO_SPEED_FREQ_LOW` | — | `Hardware\app_imu.c` → `drdy_exti_init()`，**仅当 `APP_IMU_USE_DRDY_INT=1`** |
| **PE0** | 显示页翻页 / 长按归零 | `GPIO_MODE_INPUT` | `GPIO_PULLUP` | —（输入） | — | `App\display_nav.c` → `DisplayNav_Init()`，**仅当 `DISPLAY_NAV_USE_KEY=1`** |

> 关键点：**PB11 完全没有在 CubeMX 里出现，PB10 在 `.ioc` 里标了 `GPIO_Output`，
> 但两者最终都由 `soft_i2c_gpio_init()` 重新配置成开漏输出**，所以 CubeMX 里是否配置
> 不影响功能（详见第 3 节）。

### 1.2 本模块间接使用、但不归我们配置的引脚

| 引脚 | 用途 | 谁配置 | 我们是否重配 |
|---|---|---|---|
| PD11 | OLED DC | CubeMX（`MX_GPIO_Init`，推挽输出） | 否 |
| PD12 | OLED RST | 同上 | 否 |
| PD13 | OLED SDA | 同上 | 否 |
| PD14 | OLED SCL | 同上 | 否 |
| PD8 / PD9 | USART3（保留，可做调试串口） | CubeMX | 否 |

`display_nav.c` 只调用 `OLED_ShowChar()` / `OLED_Refresh_Gram()`，引脚由 `bsp_oled.c` 的宏操作，
我们不改这些引脚的配置。

---

## 2. 逐引脚配置详解（可对照代码）

### 2.1 PB10 / PB11 —— 软件 I2C

**为什么要软件 I2C**：工程没有启用 I2C 外设（`stm32f4xx_hal_conf.h` 里没有
`HAL_I2C_MODULE_ENABLED`），而且 PB10/PB11 在板子上就是普通 IO 接 ICM20948，
因此用 GPIO 模拟。

**配置代码**（`imu_port_stm32_hal.c`）：

```c
g.Mode      = GPIO_MODE_OUTPUT_OD;   /* 开漏：写 1 释放，写 0 拉低 */
g.Pull      = GPIO_PULLUP;           /* 内部上拉（约 40k） */
g.Speed     = GPIO_SPEED_FREQ_HIGH;  /* 高速翻转，保证 250kHz 下的边沿 */
g.Alternate = 0;                     /* 不用复用功能 */

g.Pin = GPIO_PIN_10;  HAL_GPIO_Init(GPIOB, &g);   /* SCL */
g.Pin = GPIO_PIN_11;  HAL_GPIO_Init(GPIOB, &g);   /* SDA */
```

**为什么 SDA 也用开漏输出而不是切换输入模式**：
开漏输出写 1 就是释放总线（由上拉决定电平），读 `IDR` 得到真实电平，
省去了 `MODER` 来回切换，时序更稳、代码更短。

**最终寄存器状态**（可在 Keil 寄存器窗口核对）：

| 寄存器 | 字段 | 值 | 含义 |
|---|---|---|---|
| `GPIOB->MODER` | MODER10 / MODER11 | `01` | 通用输出 |
| `GPIOB->OTYPER` | OT10 / OT11 | `1` | 开漏 |
| `GPIOB->PUPDR` | PUPD10 / PUPD11 | `01` | 上拉 |
| `GPIOB->OSPEEDR` | OSPEED10 / OSPEED11 | `10`/`11` | 高速 |

**电气检查**（上电后、未通信时）：

- 万用表量 PB10、PB11 对 GND 应为 **3.0~3.3V**（上拉释放）
- 若量到 0V：模块没供电、或引脚被别的外设占用、或 SDA 被从机拉死
- 若量到 1.6V 左右：上拉太弱（内部 40k + 长线容性负载），需要外加 4.7k 上拉

### 2.2 PB12 —— 数据就绪中断（默认关闭）

**当前状态**：`app_imu.h` 里 `APP_IMU_USE_DRDY_INT = 0`，**PB12 完全不被碰**，
保持复位后的高阻输入。此时 IMU 靠 5ms 定时轮询。

**如果打开**（`APP_IMU_USE_DRDY_INT = 1`），配置为：

```c
g.Pin   = GPIO_PIN_12;
g.Mode  = APP_IMU_INT_ACTIVE_LOW ? GPIO_MODE_IT_FALLING : GPIO_MODE_IT_RISING;
g.Pull  = APP_IMU_INT_ACTIVE_LOW ? GPIO_PULLUP : GPIO_NOPULL;
g.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &g);

HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
```

| `APP_IMU_INT_ACTIVE_LOW` | EXTI 边沿 | 上拉 | 适用模块 INT 输出 |
|---|---|---|---|
| `0`（默认） | 上升沿 | 无 | 推挽，空闲低、就绪拉高 |
| `1` | 下降沿 | 内部上拉 | 开漏，空闲高、就绪拉低（需外部上拉） |

**极性怎么定**：上电后保持静止，万用表量 PB12——
静止**低** → 用默认值；静止**高** → 改 `APP_IMU_INT_ACTIVE_LOW = 1`。

> 若同时在 CubeMX 里给 PB12 配了 EXTI，必须把 `APP_IMU_EXTI_MANAGED` 改为 0，
> 否则 `EXTI15_10_IRQHandler` 会和 `stm32f4xx_it.c` 里的重复定义。

### 2.3 PE0 —— 翻页按键

**背景**：CubeMX 里 PE0 是 `GPIO_Output`（推挽输出、输出低，见 `gpio.c` 第 69 行），
但主板资源说明里 PE0 是「用户按键」，所以 `DisplayNav_Init()` 在运行期把它改成输入：

```c
g.Pin  = GPIO_PIN_0;
g.Mode = GPIO_MODE_INPUT;
g.Pull = GPIO_PULLUP;      /* 按键另一端接 GND，按下读到 0 */
```

- 短按（<800ms）→ 翻页
- 长按（≥800ms）→ `Mecanum_ResetPosition()`，位姿 + IMU 航向一起归零
- 消抖 20ms

> 如果板子上 PE0 其实没接按键，翻页不会响应，页面停在 P1，**导航功能不受影响**。
> 想彻底关掉：`display_nav.h` 里 `DISPLAY_NAV_USE_KEY = 0`。

---

## 3. CubeMX 应该怎么配（重点核对）

### 3.1 必须满足的三条

| # | 要求 | 当前状态 | 怎么查 |
|---|---|---|---|
| 1 | **PB10/PB11 不能分配给任何外设** | ✅ PB11 未分配；PB10 是 `GPIO_Output`（无外设） | 打开 `.ioc`，Pinout 视图看这两脚不是绿色/黄色复用色 |
| 2 | **USART3 不能在 PB10/PB11** | ✅ 已改到 PD8/PD9 | `.ioc` 里搜 `USART3_TX` / `USART3_RX` |
| 3 | **不需要启用 I2C 外设** | ✅ 没有 I2C | `.ioc` 里搜不到 I2C；Keil 分组里没有 `stm32f4xx_hal_i2c.c` |

### 3.2 当前 `.ioc` 与代码的不同步（需要你确认）

我对比了 `.ioc`（2026-10-19 21:26）和 `gpio.c`（2026-10-19 23:05），发现代码比 `.ioc` 新：

| 引脚 | `.ioc` 里是什么 | `gpio.c` / `main.h` 里是什么 | 影响 |
|---|---|---|---|
| **PB10** | `GPIO_Output` | **没有配置** | 无害：运行期由我们的代码配成开漏 |
| **PB11** | 未分配 | 没有配置 | 无害：同上 |
| **PA2 / PA3** | `GPIO_Output` + Label `EN1`/`EN2` | `EN1_Pin` / `EN2_Pin`（推挽输出） | ✅ 已同步进 `.ioc`，可安全重新生成 |
| **PC8 / PC9** | `TIM8_CH3` / `TIM8_CH4`（PWM） | `DIR1_Pin` / `DIR2_Pin`（推挽输出） | ⚠️ 危险，见 3.2.1 |

**结论**：

- **对 IMU 模块没有任何影响**，现在编译烧录就能跑；
- 但如果你之后打开 CubeMX 点「Generate Code」，`gpio.c` 会被重写，
  **PC8/PC9（DIR1/DIR2）会被改成 TIM8 复用**（EN1/EN2 已改到 PA2/PA3 并写入 `.ioc`，不受影响），
  电机使能/方向会失效。**重新生成前请先在 CubeMX 里把 EN/DIR 补上并保存 `.ioc`。**

### 3.2.1 TIM8 的真实状态（与 IMU 无关，但决定 DIR 能不能用）

你的 TIM8 确实已经用于步进电机（D36）的 STEP 脉冲，这点和 IMU 模块**没有任何冲突**。
但我把 `.ioc` 和代码逐项对了一下，两者并不一致：

| 项目 | `.ioc`（CubeMX 里的设置） | 代码实际 |
|---|---|---|
| TIM8 模式 | PWM Generation **CH1–CH4** | `tim.c` 是 `HAL_TIM_OC_Init` + **OC CH1/CH2**（翻转模式发脉冲） |
| 输出引脚 | CH1/CH2 → PC6/PC7；CH3/CH4 → PC8/PC9 | `HAL_TIM_MspPostInit` **只配置 PC6/PC7** |
| 谁在用 | — | `bsp_stepper.c` 直接操作 `TIM8->CCR1/CCR2`、`CCMR1`、`TIM8_CC_IRQn` |

结论：**芯片上目前只有 PC6/PC7 是 TIM8 复用输出，PC8/PC9 是普通 GPIO（DIR1/DIR2）。**
DIR 还能用，只是因为生成出来的 `MspPostInit` 里没写 CH3/CH4 的复用配置。

**风险**：`.ioc` 里 CH3/CH4 的 PWM 仍然开着。下次点「Generate Code」，
`HAL_TIM_MspPostInit` 会把 **PC8/PC9 配成 AF3_TIM8**，DIR1/DIR2 立刻失效
（步进电机只会朝一个方向走，或者完全不动）。

**二选一修掉：**

| 方案 | 操作 | 结果 |
|---|---|---|
| **A（推荐）** | CubeMX → TIM8 → **Channel 3 / Channel 4 设为 Disable**；PC8/PC9 设为 `GPIO_Output`，Label `DIR1`/`DIR2` | 与现有代码一致，重新生成安全 |
| B | 保留 CH3/CH4 的 PWM，把 **DIR1/DIR2 挪到其它空闲脚** | 要改 `main.h` 标签和实际接线 |

> EN1/EN2 现已改到 **PA2/PA3** 并写入 `.ioc`（Label `EN1`/`EN2`），重新生成不会丢失。

### 3.3 建议的 CubeMX 配置（可选，仅为了图面清晰）

| 引脚 | 建议设置 |
|---|---|
| PB10 | `GPIO_Output`；Output level = **High**；Mode = **Output Open Drain**；Pull-up；Speed = High；User Label = `IMU_SCL` |
| PB11 | 同上，Label = `IMU_SDA` |
| PB12 | 暂不配；要用中断时配 `GPIO_EXTI12` + NVIC，并把 `APP_IMU_EXTI_MANAGED` 改 0 |

**不配也完全能跑**，因为 `soft_i2c_gpio_init()` 会覆盖。

---

## 4. 时序与采样配置

| 项目 | 值 | 位置 | 计算/说明 |
|---|---|---|---|
| 软件 I2C 半周期 | **2 µs** | `IMU_STM32_SOFT_HALF_US` | 约 250 kHz（SDA/SCL 各 2µs 高 + 2µs 低） |
| I2C 起始超时 | 50 ms | `IMU_STM32_I2C_TIMEOUT_MS` | 实际软件 I2C 每步都有界，此宏只在硬件模式生效 |
| IMU 采样周期 | **5 ms（200 Hz）** | `APP_IMU_PERIOD_MS` | 主循环轮询，内部自动分频 |
| 传感器 ODR | **225 Hz** | `APP_IMU_SMPLRT_DIV = 4` | 1125 / (1+4) = 225 Hz，略高于 200Hz 采样，保证每次都是新数据 |
| 原始值平均 | **2 点** | `cfg.avg_samples` | 延迟约 5ms；调大更平滑但延迟增加 |
| 底盘控制环 | 10 ms | `MecanumChassis_Loop(chassis, 0.01f)` | 原有不变 |
| 电机速度环 | 1 ms | `Motor_Update(..., 0.001f)` | 原有不变 |
| OLED 刷新 | **100 ms** | `DISPLAY_NAV_PERIOD_MS` | 全屏 1024 字节 bit-bang，约 1~2ms |
| 初始化阻塞 | **≈ 0.9 s** | `icm20948_init()` | 100ms 复位 + 400 帧 × 2ms 零偏标定 |

**总线负载估算**：每次读取 12B（加计+陀螺）+ 2B（温度）+ 9B（磁力计）= 23 字节，
200Hz × 23 × 9bit ≈ **41.4 kbit/s**，在 250kHz 总线上约占 17%，余量充足。

**时序自检**（有示波器/逻辑分析仪时）：

- SCL 频率 ≈ 250 kHz（周期约 4 µs）
- 每 5 ms 出现一簇 23 字节的传输
- 传输之间 SCL/SDA 均为高

---

## 5. ICM-20948 芯片侧配置（寄存器级）

以下值由 `icm20948_init()` / `sensors_config()` 写入，可在数据手册对应寄存器核对。

| 寄存器（bank） | 名称 | 写入值 | 含义 | 依据 |
|---|---|---|---|---|
| bank0 `0x06` | `PWR_MGMT_1` | `0x80` → `0x01` | 先复位，再选自动时钟源 | 复位后 100ms 等待 |
| bank2 `0x00` | `GYRO_SMPLRT_DIV` | `0x04` | 陀螺 ODR = 1125/(1+4) = 225Hz | |
| bank2 `0x01` | `GYRO_CONFIG_1` | **`0x35`** | DLPFCFG=6（5.7Hz）+ FS=±1000dps + DLPF_EN | bit[5:3]=110, bit[2:1]=10, bit0=1 |
| bank2 `0x11` | `ACCEL_SMPLRT_DIV_2` | `0x04` | 加计 ODR = 225Hz | |
| bank2 `0x14` | `ACCEL_CONFIG` | **`0x33`** | DLPFCFG=6（5.7Hz）+ FS=±4g + DLPF_EN | bit[5:3]=110, bit[2:1]=01, bit0=1 |
| bank2 `0x09` | `ODR_ALIGN_EN` | `0x01` | 陀螺/加计采样对齐 | 减小姿态相位误差 |
| bank0 `0x03` | `USER_CTRL` | `0x20` | 使能 I2C 主机（磁力计用） | 初始化末尾 |
| bank3 `0x03` | `I2C_SLV0_ADDR` | `0x8C` | 读 AK09916（0x0C \| 读位） | |
| bank3 `0x04` | `I2C_SLV0_REG` | `0x10` | 从 ST1 开始 | |
| bank3 `0x05` | `I2C_SLV0_CTRL` | `0x89` | 使能 + 9 字节 | ST1+6 字节磁场+TMPS+ST2 |
| AK09916 `0x32` | `CNTL3` | `0x01` | 软复位 | |
| AK09916 `0x31` | `CNTL2` | `0x08` | 连续测量 100Hz | |
| bank0 `0x0F` | `INT_PIN_CFG` | `0x10` | 任意读清中断（仅启用 INT 时写） | |
| bank0 `0x11` | `INT_ENABLE_1` | `0x01` / `0x00` | 原始数据就绪中断开/关 | |

**量程换算**（驱动内部）：

| 物理量 | 量程 | LSB 系数 | 换算 |
|---|---|---|---|
| 加速度 | ±4g | 8192 LSB/g | `accel_g = raw / 8192` |
| 角速度 | ±1000dps | 32.8 LSB/dps | `gyro_dps = raw / 32.8 - 零偏` |
| 温度 | — | 326.8 LSB/℃ | `T = raw / 326.8 + 25` |
| 磁力计 | ±4900 µT | 0.15 µT/LSB | `B = raw × 0.15` |

---

## 6. 与主板资源说明的逐脚核对

对照文件：`STM32F407VET6(C30D-V2.1_V2.2)主板资源分配说明.pdf`（2026.04，共 2 页）
对照对象：工程当前 `.ioc` + `gpio.c` + `main.h` + 本模块代码
核对日期：2026-10-20

判定说明：✅ 一致　⚠️ 偏离但可用/需补　❌ 与文档分配冲突

### 6.1 本模块相关（ICM20948）

| 引脚 | 文档 | 你的 CubeMX | 你的代码 | 判定 |
|---|---|---|---|---|
| PB10 | ICM20948 普通 IO | `GPIO_Output` | 软件 I2C SCL（改开漏输出） | ⚠️ 模式多余，但无害 |
| PB11 | ICM20948 普通 IO | **未配置** | 软件 I2C SDA（运行期配开漏） | ⚠️ 建议补进 CubeMX |
| PB12 | ICM20948 外部中断 | **未配置** | 默认不用；开启时配 EXTI | ✅ 可选 |

> 结论：**IMU 三根线与文档完全对得上**，只是 CubeMX 里没画全，不影响运行。

### 6.2 被你改作他用的引脚（重点）

| 引脚 | 文档分配 | 你现在的用途 | 判定 |
|---|---|---|---|
| **PC6 / PC7** | 航模/舵机接口 TIM8_CH1/CH2 | D36 步进 **STEP1/STEP2**（OC toggle 脉冲） | ❌ 偏离文档 |
| **PC8 / PC9** | 航模/舵机接口 TIM8_CH3/CH4 | D36 步进 **DIR1/DIR2**（普通 GPIO） | ❌ 偏离文档，且与 `.ioc` 里 CH3/CH4 冲突 |
| PB14 / PB15 | 航模/舵机接口 TIM12_CH1/CH2 | 保持 TIM12（未用） | ✅ 一致 |
| PA2 / PA3 | 预留 IO（H2 第 12/10 脚） | D36 步进 **EN1/EN2** | ✅ 已写入 `.ioc` |

**说明**：板子把 PC6–PC9 引到"航模/舵机"排针，你的车是麦轮、不接舵机，
把这 4 个脚挪去做步进是**物理可行**的，但要注意两点：

1. 与文档不一致，建议在文档/原理图上标注「PC6–PC9 已改作 D36 步进」；
2. `.ioc` 里 TIM8 的 **CH3/CH4 仍开着 PWM**，一旦重新生成，CubeMX 会把 PC8/PC9 配成
   TIM8 复用，DIR1/DIR2 立即失效（详见 3.2.1）。

### 6.3 其余引脚（未改动，全部一致）

| 功能 | 文档引脚 | 你的配置 | 判定 |
|---|---|---|---|
| 用户按键 | PE0 | `GPIO_Output`（代码运行期改上拉输入） | ⚠️ 模式应改输入，代码已兜底 |
| LED | PE8 | GPIO_Output | ✅ |
| 蜂鸣器 | PA8 | GPIO_Output | ✅ |
| OLED | PD11–PD14 | GPIO_Output | ✅ |
| SWD | PA13 / PA14 | Serial Wire | ✅ |
| 使能开关 | PD3 | GPIO_Output | ✅ |
| USART3 | PD8 / PD9 | USART3 异步 | ✅ |
| 电机 A PWM | PB8 / PB9 | TIM10_CH1 / TIM11_CH1 | ✅ |
| 电机 B PWM | PE5 / PE6 | TIM9_CH1/CH2 | ✅ |
| 电机 C PWM | PE9 / PE11 | TIM1_CH1/CH2 | ✅ |
| 电机 D PWM | PE13 / PE14 | TIM1_CH3/CH4 | ✅ |
| 编码器 A | PA15 / PB3 | TIM2_CH1/CH2 | ✅ |
| 编码器 B | PB4 / PB5 | TIM3_CH1/CH2 | ✅ |
| 编码器 C | PB6 / PB7 | TIM4_CH1/CH2 | ✅ |
| 编码器 D | PA0 / PA1 | TIM5_CH1/CH2 | ✅ |
| USART1 | PA9 / PA10 | 未配置 | ✅ 未使用 |
| USART2 | PD5 / PD6 | 未配置 | ✅ 未使用 |
| CAN | PD0 / PD1 | 未配置 | ✅ 未使用 |
| USB | PA11 / PA12 | 未配置 | ✅ 未使用 |
| ADC 电压 | PB0 | 未配置 | ⚠️ 未做电压检测 |
| ADC 电位器 | PB1 | 未配置 | ⚠️ 未做车型选择 |
| CCD/ELE | PC4 / PA4 / PA5 | 未配置 | ✅ 未使用 |

**结论**：除 PC6–PC9（航模口改步进）外，其余引脚与主板资源说明一致；
所有未使用的外设都保持未配置，没有抢用。

### 6.4 建议修正清单（按优先级）

| # | 改什么 | 在哪改 | 不改的后果 |
|---|---|---|---|
| 1 | TIM8 关闭 CH3/CH4；PC8/PC9 设 `GPIO_Output`（DIR1/DIR2） | CubeMX | 重新生成后 DIR 失效 |
| 2 | ~~PE1/PE2 设为 GPIO_Output~~ **已完成**：EN1/EN2 改到 PA2/PA3 并写进 `.ioc` | CubeMX | — |
| 3 | PE0 改为 `GPIO_Input` + Pull-up | CubeMX | 现靠代码兜底，重生成仍会变输出 |
| 4 | PB11 设为 `GPIO_Output`（开漏、初始高）；PB10 同样改开漏高 | CubeMX | 无功能影响，只是图面不全 |
| 5 | （可选）PB0/PB1 配 ADC | CubeMX | 没有电压检测与车型选择 |

> 第 1~3 项做完后请**重新编译并跑一遍**：`tim.c` 会按 PWM 模式重新生成，
> 但 `Stepper_Init()` 运行期会把 `CCMR1` 重设为 TOGGLE，脉冲输出逻辑不变，
> 仍需实测一次 STEP 波形。

---

## 7. 上电核对清单（照着打勾）

### 7.1 硬件层

- [ ] 万用表量 **PB10 对 GND**：3.0~3.3V
- [ ] 万用表量 **PB11 对 GND**：3.0~3.3V
- [ ] 模块 VCC = 3.3V（不要接 5V，ICM20948 是 1.8V 内核 + 3.3V IO，模块一般带 LDO，按模块标注来）
- [ ] 模块 GND 与主板共地
- [ ] （可选）示波器看 PB10 有 250kHz 突发

### 7.2 软件层（OLED P4 页）

- [ ] `OK1`：IMU 初始化成功
- [ ] `F` 每秒 +200（允许 ±10）
- [ ] `ERR` 恒为 0
- [ ] `DT` ≈ 0.005

### 7.3 姿态与导航（P1 页）

- [ ] 手拿小车**逆时针**转 90° → `A` ≈ **+90**（±5）
- [ ] 前后倾斜 → `P`（pitch）跟着变，符号符合"车头下压为负"
- [ ] 推车前进 1m → `X` ≈ 1.0（±0.1）
- [ ] 推车左移 1m → `Y` ≈ 1.0（±0.1）
- [ ] 静止 3 秒 → `S` = 1，`X/Y` 不再变化
- [ ] 原地快转 → `P` 短暂为 1（打滑），转完恢复 0

### 7.4 Keil 调试窗口

Watch 里加：

```
app_imu_dbg        ← 看 yaw_deg / frames / errors / dt / ok
s_nav              ← 看 x / y / yaw / stationary / slipping / slip_count
s_imu              ← 展开 cfg 可核对运行时的量程/采样率/DLPF
```

---

## 8. 配置错误的典型现象 → 对应原因

| 现象 | 最可能的原因 | 去哪检查 |
|---|---|---|
| `F` 一直是 0 | PB10/PB11 接反、模块没供电、无上拉 | 万用表量电压；对调 SCL/SDA 试 |
| `F` 涨但 `ERR` 也在涨 | 上拉不足 / 线太长 / 速率偏高 | 加 4.7k 上拉；`IMU_STM32_SOFT_HALF_US` 改 5 |
| `OK` = 0 | WHO_AM_I ≠ 0xEA | 地址错（AD0 接高应为 0x69）、或芯片型号不对 |
| `yaw` 方向反 | 模块安装方向或坐标约定 | `APP_IMU_YAW_SIGN` 改 `-1.0f` |
| `yaw` 漂移大 | 标定时车动了 / 磁干扰 / 零偏未更新 | 静止重新上电；`cfg.enable_mag=false` |
| 静止时 `X/Y` 还在变 | ZUPT 阈值偏小 | `nav.c` 里 `zupt_v` 0.015 → 0.03 |
| 正常行驶 `P` 频繁为 1 | 打滑阈值偏小 | `slip_thr` 0.35 → 0.6 |
| 真打滑时位置冲出去 | 打滑衰减不足 | `slip_v_scale` 0.25 → 0.15 |
| 开了 INT 后 `F` 暴涨到几千 | INT 极性/边沿配反 | 量 PB12 静止电平，改 `APP_IMU_INT_ACTIVE_LOW` |
| 编译报 `I2C_HandleTypeDef` 未定义 | 已修复；若重现说明 `HAL_I2C_MODULE_ENABLED` 被误开 | `app_imu.c` 顶部的条件编译 |
| 编译报 `fmodf/cosf` 未声明 | AC5 的 C90 模式 | 勾选 C99 Mode，或确认 `nav.c` / `imu_attitude.c` 顶部的 extern 声明还在 |
| 长按归零没反应 | PE0 没按键 / 被 CubeMX 改回输出 | `display_nav.c` 的 `DisplayNav_Init()` 是否执行；量 PE0 电平 |

---

## 9. 想改配置时改哪里（索引）

| 想改什么 | 改哪个文件 | 宏/字段 | 注意 |
|---|---|---|---|
| I2C 引脚 | `IMU20948\port\imu_port_stm32_hal.c` | `IMU_STM32_SCL_PIN` / `IMU_STM32_SDA_PIN` | 改引脚后不需要动 CubeMX |
| I2C 速率 | 同上 | `IMU_STM32_SOFT_HALF_US`（2→5 降速） | 越小越快，受上拉限制 |
| 用硬件 I2C | 同上 | `IMU_STM32_USE_SOFT_I2C` 改 0 | 需要在 CubeMX 启用 I2C 并把句柄传给 `app_imu.c` |
| 采样率 | `Hardware\app_imu.h` + `app_imu.c` | `APP_IMU_PERIOD_MS`（5ms）与 `APP_IMU_SMPLRT_DIV`（4） | **两者要匹配**：ODR 应略高于采样率 |
| 加速度量程 | `Hardware\app_imu.c` | `cfg.accel_fs` | 改为 8G 需确认冲击不饱和 |
| 陀螺量程 | 同上 | `cfg.gyro_fs` | 500dps 分辨率更高 |
| 磁力计开关 | 同上 | `cfg.enable_mag` | 室内建议 false |
| 零偏标定次数 | 同上 | `cfg.gyro_calib_samples` | 400×2ms ≈ 0.8s |
| 数据就绪中断 | `Hardware\app_imu.h` | `APP_IMU_USE_DRDY_INT` / `APP_IMU_INT_ACTIVE_LOW` / `APP_IMU_EXTI_MANAGED` | 见 2.2 |
| 导航阈值 | `Control\nav.c` → `Nav_Init()` | `zupt_v` / `zupt_w` / `slip_thr` / `slip_v_scale` / `wheel_weight` | 也可运行期 `Nav_SetParams()` |
| yaw 方向 | `Hardware\app_imu.h` | `APP_IMU_YAW_SIGN` | ±1 |
| 显示刷新周期 | `App\display_nav.h` | `DISPLAY_NAV_PERIOD_MS` | 太小会占用主循环 |
| 翻页按键 | 同上 | `DISPLAY_NAV_USE_KEY` / `DISPLAY_NAV_KEY_PIN` | 换键就改这两个 |
| 长按归零时间 | 同上 | `DISPLAY_NAV_LONG_MS` | 800ms |

---

## 10. 文件与职责（当前工程）

```
IMU20948\
├── inc\
│   ├── icm20948.h            驱动 API、配置结构体、错误码
│   ├── icm20948_regs.h       4 个 bank + AK09916 寄存器定义
│   ├── imu_attitude.h        互补 / 卡尔曼 / Mahony
│   ├── imu_port.h            移植层接口（I2C 读写 + 延时）
│   └── imu_port_stm32_hal.h  STM32 HAL 移植层接口
├── src\
│   ├── icm20948.c            复位、量程、磁力计、零偏、突发读
│   └── imu_attitude.c        姿态解算
└── port\
    └── imu_port_stm32_hal.c  PB10/PB11 软件 I2C + DWT 微秒延时

Hardware\
├── app_imu.c / .h            200Hz 采样、姿态、掉线降级、调试镜像
├── nav.c / .h                组合导航（航向融合/ZUPT/打滑检测）
└── display_nav.c / .h        OLED 4 页 + PE0 按键
```

**调用链**（`main.c`）：

```
HAL_Init → SystemClock_Config → MX_GPIO_Init → MX_TIM1..12 → MX_USART3
   → OLED_Init()                 ← 第 118 行
   → 电机/底盘初始化
   → DisplayNav_Init()           ← 第 195 行，配置 PE0
   → app_imu_init()              ← 第 196 行，配置 PB10/PB11，约 0.9s
   → HAL_TIM_Base_Start_IT(&htim6)  ← 第 201 行，最后才开 1ms 中断
   → while(1):
        app_imu_poll()           ← 第 215 行，200Hz
        DisplayNav_Poll(&chassis)
        到位判断
```

---

## 11. 审查建议（如果只抽查三项）

1. **PB11 没在 CubeMX 里配** —— 正常，代码运行期配成开漏；量电压 3.3V 即可确认。
2. **PB10 在 `.ioc` 里是 `GPIO_Output`，但 `gpio.c` 没生成** —— `.ioc` 与代码不同步，
   不影响本模块，但重新生成代码前要先补齐 EN1/EN2、DIR1/DIR2（见 3.2）。
3. **`APP_IMU_USE_DRDY_INT` 默认为 0** —— PB12 现在完全没被使用，白线接不接都不影响；
   要用中断再按 2.2 开。
