# BT10 Windows BLE 日志接收器

`bt10_ble_receiver.py` 使用 Windows BLE GATT 接收 STM32 USART2 的文本日志，
并原样保存为 CSV，现有 `viewer/kart_log_view_mecanum_v4.html` 可直接导入。

## 使用

```powershell
python -m pip install bleak websockets
python tools/bt10_ble_receiver.py BT10 -o bt10_logs\run01.csv --no-console --websocket
```

若固件同时发送独立 PID Burst（以 `#BTLOG,PID_BEGIN` 开始、`#BTLOG,PID,`
逐行发送、`#BTLOG,PID_END` 结束），可加 `--pid-burst`：

```powershell
python tools/bt10_ble_receiver.py BT10 -o bt10_logs\run01.csv --pid-burst --no-console --websocket
```

它会把 PID Burst 保存为同目录的 `run01.pid.csv`，主日志仍原样保存为
ULog43 CSV。`PID_BEGIN` 后的字段名应使用网页支持的带字段名格式，例如
`timestamp_ms,fl_target,fl_actual,fl_output,...`；也接受 `#BTLOG,PID,{...}`
JSON 行并按首个对象的键生成表头。PID 文件在 `PID_END` 或蓝牙断开时执行
flush/fsync/close；不启用 `--pid-burst` 时这些行仍会原样保存在主日志中。

`BT10` 可以替换为 Windows 蓝牙设置中显示的设备名或地址。程序使用 WHEELTEC
BT10 的 FFE0 服务、FFE1 通知特征和 FFE2 写入特征。收到
`#BTLOG,END`（也兼容带附加字段的 `#BTLOG,END,...`）后执行 flush、fsync 并关闭文件；蓝牙断开时也会保存已收到的
最后一段数据。

不带 `--no-console` 时，可以在控制台输入 `@GET,STATE`、`@GET,PARAMS` 和
`@SET,...` 等命令。接收到 `#BTLOG,STATE,RUNNING` 后，主机不会发送 `@SET`，
但仍允许查询状态和参数。最终权限检查仍必须由 STM32 执行。

`--websocket` 为网页实时曲线开启本机转发，默认监听
`ws://127.0.0.1:8765`。网页连接或关闭不会影响 CSV 保存；省略该选项时无需
安装 `websockets`，接收器仍可单独保存日志。


## 20 秒 PID 诊断（2026-11-20）

1. 重新编译并由使用者烧录固件；将 BT10 串口端设置为 **230400、8N1**。
   MCU USART2 同步为 230400；普通 ULog43 保持 20 Hz。旧 TF 记录器若接在
   同一串口，也需匹配波特率，但不应把二进制 PID 帧当作旧 CSV 文件。
2. 用 Chrome/Edge 打开根目录 kart_log_view_mecanum_v4.html，直接连接 BT10。
   需要双向接线：PD5→模块 RX，PD6←模块 TX，共地。
3. PID 页面先点击“网页协议自测（模拟）”，确认四轮曲线、2000 帧数据和分析；
   模拟自测需断开实车/电脑日志源，不能代替实物蓝牙验收。
4. 连接实车，开机至少等待 2 秒，停车时点击“开始 20 秒采集”。
   收到 PID_ARMED 才表示实际启动。此命令不驱动车辆、不改 PID；
   按原来的流程运行小车，计时从开始命令成功即刻开始，包含等待运动时间。
5. 满 2000 帧自动冻结，也可以提前点“停止采集”。
   **停止采集不会让车停车。** 按原流程停车且退出 RUNNING 状态，
   命令目标接近零、四轮实测小于 10 pulse/s 持续 500 ms 后分批上传。
6. 页面显示实收/应收帧数、实时序号缺口、CRC 错误、量化裁剪数；上传结束后
   自动进入现有 PID 指标/候选分析，可下载独立 CSV。日志仓库条目另有
   “PID 分析”入口。电脑接收器加 --pid-burst 也能保存独立文件，
   同一接收会话的后续 Burst 使用 _2、_3 后缀，避免覆盖上次记录。

运行时默认不采集；命令仅为 @PID START\n、@PID STOP\n。
无需安装 Python 即可用网页直连。Python 接收器可继续做接收、保存和 WebSocket 转发，
但网页开始/停止按钮只向直接连接的 BT10 发送命令。整车日志 END 仍会锁定停止，
下一轮整车采集需要按现有方式复位；不要在上传完成之前断电。

### 数据与实时边界

- TIM6 电机调用 1 ms、底盘环 10 ms 均保持不变；现有 Motor 内部测速/PID
  窗口实际是 5 ms（200 Hz），并非每 1 ms 都重新计算速度 PID。
- 实时上传目标 200 Hz，每帧 36 字节，约 7.2 KB/s；界面绘图限制为约 10 Hz，
  数据接收频率与显示器刷新率不同。主循环或 BLE 拥塞会降低实收率，缺口可见。
- RAM 独立保存 100 Hz×2000 帧，首末时间跨度 19.99 秒，内存 56,000 字节。
  每帧时间戳为 u32，12 个通道为有符号 i16，分辨率 0.1，
  范围 -3276.8～3276.7 pulse/s / 控制输出单位；超界和非有限值会统计裁剪，
  不能把裁剪后的曲线当作原始真实值。输出是前馈+PID+kick+限幅后的总输出，
  不是实测电压或电流。
- 中断内固定次数取值、量化和复制，不格式化、不发串口、不分配内存；
  主循环每轮最多发一帧，每帧 CRC16 校验。队列紧张优先丢实时帧，
  保留整车日志空间，停车记录上传则暂停重试同一索引。
- USART2 日志 IRQ 降为优先级 5，避免抢占优先级 4 的 TIM6；USART3 与电机参数未改。
- 内存不是掉电存储；没有 BLE 应用层 ACK/自动补传承诺，链路断开仍可能丢失电脑端数据。
  有效日志需核对 PID_STATS 的 count 与实收帧数。BT10 吞吐、TIM6 最坏耗时和
  实车同时运行视觉/机构时的表现均为 NOT_RUN。

### 二进制协议 v1

固定 36 字节，小端：A5 5A 01 kind seq:u16 timestamp_ms:u32 values:12*i16 crc:u16。
kind=1 实时、kind=2 停车记录；CRC 为 CCITT-FALSE（初值 FFFF，多项式 1021），
覆盖第 2～33 字节。12 个值按 FL、FR、RL、RR，每轮 target/actual/output。
实时 seq 从 1 开始，记录 seq 从 0 开始。普通 ASCII 行与二进制帧交错发送；
接收端需先按字节组包，再解码 ASCII，不能对整条链路直接做 TextDecoder。

Python 将实时帧转换成 #BTLOG,PID_LIVE,seq,...，停车帧转换成 #BTLOG,PID,...，
这样旧 ULog43 和现有 PID CSV 分析继续兼容。
BEGIN/END 与 PID_STATS 仍为 ASCII。原始文本 PID Burst 接收也继续保留。
