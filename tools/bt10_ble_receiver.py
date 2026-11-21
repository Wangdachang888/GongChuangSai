#!/usr/bin/env python3
"""BT10 BLE log receiver and guarded parameter console.

The receiver stores the STM32 text stream verbatim.  CSV rows therefore remain
compatible with the existing offline viewer, while ``#BTLOG`` metadata is
retained for diagnostics.  The device is expected to expose the WHEELTEC
service FFE0, notify characteristic FFE1, and write characteristic FFE2.

Requires: ``pip install bleak``
"""

from __future__ import annotations

import argparse
import binascii
import struct
import asyncio
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import sys
from typing import Callable, Optional

try:
    import websockets
except ImportError:  # WebSocket forwarding is optional.
    websockets = None  # type: ignore[assignment]

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # Keep --help and parser tests usable without bleak.
    BleakClient = None  # type: ignore[assignment,misc]
    BleakScanner = None  # type: ignore[assignment,misc]


SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
NOTIFY_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"


class LogReceiver:
    """Line assembler and durable CSV writer, independent of BLE transport."""

    def __init__(self, output: Path, on_line: Optional[Callable[[str], None]] = None,
                 pid_output: Optional[Path] = None) -> None:
        self.output = output
        self.on_line = on_line
        self.pid_output = pid_output
        self._file = None
        self._pid_file = None
        self._pid_header_written = False
        self._pid_session = 0
        self._pid_header_fields: list[str] = []
        self._pid_used: dict[str, str] = {}
        self._pending = bytearray()
        self._wire_resync = False
        self.state = "UNKNOWN"
        self.lines = 0
        self.dropped = 0
        self.closed_by_end = False
        self.pid_lines = 0
        self.pid_closed_by_end = False

    def _open(self) -> None:
        if self._file is None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            # A selected output path represents one capture session.
            self._file = self.output.open("wb")

    def feed(self, data: bytes) -> None:
        """Demux bounded ASCII lines and 36-byte PID binary frames."""
        self._pending.extend(data)
        while self._pending:
            if self._wire_resync:
                candidates = [x for x in (self._pending.find(b"\xa5\x5a"),
                                         self._pending.find(b"#BTLOG,")) if x >= 0]
                if not candidates:
                    if len(self._pending) > 6:
                        del self._pending[:-6]
                    return
                del self._pending[:min(candidates)]
                self._wire_resync = False
            if self._pending[0] == 0xA5:
                if len(self._pending) < 2:
                    return
                if self._pending[1] != 0x5A:
                    del self._pending[0]
                    self.dropped += 1
                    continue
                if len(self._pending) < 36:
                    return
                f = bytes(self._pending[:36])
                crc = binascii.crc_hqx(f[2:34], 0xFFFF)
                if f[2] != 1 or f[3] not in (1, 2) or crc != int.from_bytes(f[34:36], "little"):
                    del self._pending[0]
                    self.dropped += 1
                    self._wire_resync = True
                    continue
                del self._pending[:36]
                seq, ms, *values = struct.unpack("<HI12h", f[4:34])
                row = ",".join([str(ms)] + [f"{v / 10:.1f}" for v in values])
                prefix = f"#BTLOG,PID_LIVE,{seq}," if f[3] == 1 else "#BTLOG,PID,"
                self.handle_line((prefix + row).encode("ascii"))
                continue
            newline = self._pending.find(b"\n")
            magic = self._pending.find(b"\xa5")
            if magic >= 0 and (newline < 0 or magic < newline):
                del self._pending[:magic]
                self.dropped += 1
                continue
            if newline < 0:
                if len(self._pending) > 4096:
                    self._pending.clear()
                    self.dropped += 1
                return
            raw = bytes(self._pending[:newline]).rstrip(b"\r")
            del self._pending[:newline + 1]
            self.handle_line(raw)

    def handle_line(self, raw: bytes) -> None:
        if not raw:
            return
        # END is a durable session boundary.  Do not reopen the file if a
        # delayed notification arrives after that boundary.
        if self.closed_by_end:
            return
        self._open()
        self._file.write(raw + b"\n")  # type: ignore[union-attr]
        self.lines += 1
        try:
            line = raw.decode("utf-8", errors="replace")
        except Exception:
            return
        if self.on_line is not None:
            self.on_line(line + "\n")
        self._handle_pid_line(line)
        if line.startswith("#BTLOG,STATE,"):
            self.state = line.split(",", 2)[2].strip().upper()
        elif line == "#BTLOG,END" or line.startswith("#BTLOG,END,"):
            self.close_durable()

    def _open_pid(self) -> None:
        if self._pid_file is not None or self.pid_output is None:
            return
        self.pid_output.parent.mkdir(parents=True, exist_ok=True)
        path = self.pid_output
        if self._pid_session > 1:
            path = path.with_name(f"{path.stem}_{self._pid_session}{path.suffix}")
        self._pid_file = path.open("w", encoding="utf-8", newline="")

    def _write_pid_header(self, fields: list[str]) -> None:
        if self._pid_header_written or self.pid_output is None:
            return
        self._open_pid()
        if self._pid_file is None:
            return
        # Use csv.writer so a future field containing a comma remains valid CSV.
        for snapshot in self._pid_used.values():
            self._pid_file.write(snapshot + "\n")
        csv.writer(self._pid_file, lineterminator="\n").writerow(fields)
        self._pid_header_written = True
        self._pid_header_fields = list(fields)

    def _handle_pid_line(self, line: str) -> None:
        """Decode the optional independent PID burst stream.

        Protocol is deliberately additive: normal ULog43 rows are untouched.
        ``#BTLOG,PID_BEGIN,<field names...>`` starts a CSV file,
        ``#BTLOG,PID,<CSV row>`` appends one sample, and
        ``#BTLOG,PID_END`` fsyncs and closes it.  A JSON object payload is also
        accepted and written as a named CSV row for forward compatibility.
        """
        if self.pid_output is None or not line.startswith("#BTLOG,PID"):
            return
        if line.startswith("#BTLOG,PID_ARMED"):
            self._pid_used.clear()
            return
        if line.startswith("#BTLOG,PID_USED,"):
            fields = line.split(",")
            if len(fields) == 22 and fields[4] in ("FL", "FR", "RL", "RR", "X", "Y", "TH"):
                self._pid_used[fields[4]] = line
            return
        if line == "#BTLOG,PID_END" or line.startswith("#BTLOG,PID_END,"):
            if self._pid_file is not None:
                self._pid_file.flush()
                os.fsync(self._pid_file.fileno())
                self._pid_file.close()
                self._pid_file = None
            self.pid_closed_by_end = True
            return
        if line.startswith("#BTLOG,PID_STATS,"):
            if self._pid_file is not None:
                self._pid_file.write(line + "\n")
            return
        if line.startswith("#BTLOG,PID_BEGIN"):
            if self._pid_file is not None:
                self._pid_file.flush()
                os.fsync(self._pid_file.fileno())
                self._pid_file.close()
                self._pid_file = None
            self._pid_session += 1
            self.pid_closed_by_end = False
            self._pid_header_written = False
            self._pid_header_fields = []
            fields = line.split(",")[2:]
            if fields:
                self._write_pid_header(fields)
            return
        if not line.startswith("#BTLOG,PID,") or self.pid_closed_by_end:
            return
        payload = line[len("#BTLOG,PID,"):]
        self._open_pid()
        if self._pid_file is None:
            return
        if payload.lstrip().startswith("{"):
            try:
                obj = json.loads(payload)
            except json.JSONDecodeError:
                obj = None
            if isinstance(obj, dict):
                if not self._pid_header_written:
                    keys = list(obj.keys())
                    self._write_pid_header(keys)
                fields = [obj.get(k, "") for k in (self._pid_header_fields or [])]
                csv.writer(self._pid_file, lineterminator="\n").writerow(fields)
            else:
                self._pid_file.write(payload + "\n")
        else:
            self._pid_file.write(payload + "\n")
        self.pid_lines += 1

    def close_durable(self) -> None:
        if self._pid_file is not None:
            self._pid_file.flush()
            os.fsync(self._pid_file.fileno())
            self._pid_file.close()
            self._pid_file = None
        if self._file is None:
            return
        self._file.flush()
        os.fsync(self._file.fileno())
        self._file.close()
        self._file = None
        self.closed_by_end = True

    def close(self) -> None:
        # Preserve a final partial line if the link disappears mid-frame.
        if self._pending and all(b < 128 for b in self._pending):
            self.handle_line(bytes(self._pending))
            self._pending.clear()
        self.close_durable()
        if self._pid_file is not None:
            self._pid_file.flush()
            os.fsync(self._pid_file.fileno())
            self._pid_file.close()
            self._pid_file = None


def default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("bt10_logs") / f"kart_{stamp}.csv"


class LocalWebSocket:
    """Optional newline broadcaster for the local viewer."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.clients = set()
        self.server = None

    async def _handler(self, websocket, *_args) -> None:
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)

    async def start(self) -> None:
        if websockets is None:
            raise RuntimeError("启用 WebSocket 转发需要安装: python -m pip install websockets")
        self.server = await websockets.serve(self._handler, self.host, self.port)
        print(f"WebSocket 转发已启动: ws://{self.host}:{self.port}")

    async def broadcast(self, text: str) -> None:
        if not self.clients:
            return
        clients = tuple(self.clients)
        results = await asyncio.gather(
            *(client.send(text) for client in clients),
            return_exceptions=True,
        )
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                self.clients.discard(client)

    async def stop(self) -> None:
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()
            self.server = None


async def find_device(address_or_name: str):
    if BleakScanner is None:
        raise RuntimeError("缺少 bleak，请先执行: python -m pip install bleak")
    # A MAC/address is accepted directly; otherwise scan by display name.
    devices = await BleakScanner.discover(timeout=8.0)
    needle = address_or_name.casefold()
    for device in devices:
        if device.address.casefold() == needle or (device.name or "").casefold() == needle:
            return device
    raise RuntimeError(f"未找到 BT10 设备: {address_or_name}")


async def run(args: argparse.Namespace) -> int:
    if BleakClient is None:
        print("缺少 bleak，请先执行: python -m pip install bleak", file=sys.stderr)
        return 2
    device = await find_device(args.device)
    websocket = LocalWebSocket(args.ws_host, args.ws_port) if args.websocket else None
    if websocket is not None:
        await websocket.start()
    on_line = (lambda line: asyncio.create_task(websocket.broadcast(line))) if websocket else None
    pid_output = args.pid_output
    if pid_output is None and args.pid_burst:
        pid_output = Path(args.output).with_suffix(".pid.csv")
    receiver = LogReceiver(Path(args.output), on_line=on_line, pid_output=pid_output)

    def on_notification(_characteristic, data: bytearray) -> None:
        receiver.feed(bytes(data))

    try:
        async with BleakClient(device) as client:
            await client.start_notify(NOTIFY_UUID, on_notification)
            print(f"已连接 {device.name or device.address}，日志保存到 {receiver.output}")
            if receiver.pid_output is not None:
                print(f"PID Burst 独立文件：{receiver.pid_output}")
            if args.command:
                await send_command(client, args.command, receiver)
            if args.no_console:
                while client.is_connected:
                    await asyncio.sleep(0.5)
            else:
                print("输入 @GET,STATE / @GET,PARAMS / @SET,...；RUNNING 状态拒绝 @SET。输入 quit 退出。")
                while client.is_connected:
                    try:
                        command = await asyncio.to_thread(input, "> ")
                    except (EOFError, KeyboardInterrupt):
                        break
                    command = command.strip()
                    if not command or command.casefold() in {"quit", "exit"}:
                        break
                    await send_command(client, command, receiver)
            await client.stop_notify(NOTIFY_UUID)
    finally:
        receiver.close()
        if websocket is not None:
            await websocket.stop()
    return 0


async def send_command(client, command: str, receiver: LogReceiver) -> None:
    if not command.endswith("\n"):
        command += "\n"
    if command.startswith("@SET,") and receiver.state == "RUNNING":
        print("拒绝发送：设备处于 RUNNING，当前策略只允许查看。", file=sys.stderr)
        return
    await client.write_gatt_char(WRITE_UUID, command.encode("ascii"), response=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="WHEELTEC BT10 BLE 日志接收器")
    parser.add_argument("device", help="BT10 的蓝牙名称或地址")
    parser.add_argument("-o", "--output", type=Path, default=default_output(), help="CSV 输出路径")
    parser.add_argument("--command", help="连接后发送一次命令")
    parser.add_argument("--no-console", action="store_true", help="不启动交互命令行，仅接收日志")
    parser.add_argument("--websocket", action="store_true", help="将每行日志广播到本机 WebSocket")
    parser.add_argument("--pid-burst", action="store_true",
                        help="保存 #BTLOG,PID_BEGIN/PID/PID_END 到独立 .pid.csv 文件")
    parser.add_argument("--pid-output", type=Path,
                        help="PID Burst 独立 CSV 路径（默认与 --output 同目录同名 .pid.csv）")
    parser.add_argument("--ws-host", default="127.0.0.1", help="WebSocket 监听地址")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket 监听端口")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"BT10 接收失败: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
