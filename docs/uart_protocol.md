# USART3 与 OpenMV 协议

## 所有权与执行上下文

USART3 的 HAL 回调唯一位于 `Hardware/bsp_uart_router.c`。比赛模式转发到
`Vision_RxCpltCallback`、`Vision_TxCpltCallback` 和 `Vision_ErrorCallback`；非比赛
模式才转发 D36 命令模块。回调只接收字节、推进环形缓冲和重新挂接接收，不执行底盘
运动、停车、矩阵计算或机械动作。

完整解析在主循环的 `Vision_Poll()` 中执行。发送使用队列和发送完成回调，避免在
UART 中断或运动控制周期内阻塞发送。

## V4 帧格式

```text
@sequence,length,body*XX\n
```

- `sequence` 是 16 位递增序号，接收端拒绝旧序号和重复序号；`READY,IDLE` 可重新
  建立连接状态。
- `length` 是 body 字节数。
- `body` 不允许包含 `*` 和 `@`。
- `XX` 是从 `sequence` 开始到 body 结束的 ASCII 字节 XOR 校验，使用两位十六进制。
- 帧长度、body 长度和可打印字符均有上限；异常帧计入错误并丢弃当前帧。

## 旧二维码兼容

仍支持：

```text
#SEQ,1,2,3,4,5,6,1,2,3,4,5,6\n
```

旧格式由 `RaceVisionDecoder`/`RaceVision_ParseSeq` 校验 12 个数字，并转换成现有
的四段二维码任务字符串。该兼容路径不与 V4 接收缓冲共享第二个 HAL 回调。

## 常用消息

| 方向 | body | 作用 |
| --- | --- | --- |
| STM32 -> OpenMV | `JOB,id` | 启动二维码、材料或区域视觉任务。 |
| STM32 -> OpenMV | `JOB,id,tol_x,tol_y` | 启动带像素容差的任务。 |
| STM32 -> OpenMV | `ALIGN,target,session` | 启动数字 1/2/3 圆环闭环。 |
| STM32 -> OpenMV | `STEP_DONE,step,moving` | STM32 已停稳并请求下一次观测。 |
| STM32 -> OpenMV | `ARRIVED,RING,target,dx,dy,session,count` | 连续稳定到位，结束闭环。 |
| STM32 -> OpenMV | `CANCEL` / `RESET` / `STATUS` | 取消、复位或查询状态。 |
| OpenMV -> STM32 | `READY,IDLE` | 视觉模块空闲可用。 |
| OpenMV -> STM32 | `ACK,JOB,id` | 确认普通任务。 |
| OpenMV -> STM32 | `ACK,ALIGN,target,session` | 确认圆环会话。 |
| OpenMV -> STM32 | `POS,kind,target,dx,dy,cx,cy` | 普通任务坐标。 |
| OpenMV -> STM32 | `POS,RING,target,dx,dy,cx,cy,session,step` | 圆环当前步坐标。 |
| OpenMV -> STM32 | `ARRIVED,RING,target,dx,dy` | OpenMV 侧到位报告。 |
| OpenMV -> STM32 | `IDLE` | 结束当前会话。 |
| OpenMV -> STM32 | `ERR,reason` / `TIMEOUT` / `LOST,reason` | 错误、超时或目标丢失。 |

## 安全规则

普通位置数据必须在范围内且与当前任务、session、step 匹配。圆环移动期间
`vision.c` 清除旧坐标；`app_ring_align.c` 只有收到新坐标、计算出有效小步并确认
底盘停稳后才发送 `STEP_DONE`。超时、错误、取消会进入错误或等待空闲状态，并由
应用层负责底盘安全停止。

