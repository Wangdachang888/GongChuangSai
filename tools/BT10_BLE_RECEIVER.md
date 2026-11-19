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
