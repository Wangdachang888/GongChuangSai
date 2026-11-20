"""Local, opt-in DeepSeek advisor for PID candidate review.

The browser never receives the API key.  Set DEEPSEEK_API_KEY in the local
process environment before starting this proxy.  The endpoint only returns
structured advice; it does not send commands to the vehicle.
"""
from __future__ import annotations

import argparse
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen

API_URL = "https://api.deepseek.com/chat/completions"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def safe_payload(payload: object) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("payload must be an object")
    candidates = payload.get("candidates")
    if not isinstance(candidates, list) or not candidates or len(candidates) > 8:
        raise ValueError("candidates must contain 1..8 items")
    clean = []
    for item in candidates:
        if not isinstance(item, dict):
            continue
        try:
            kp, ki, kd = (float(item[k]) for k in ("kp", "ki", "kd"))
        except (KeyError, TypeError, ValueError):
            continue
        clean.append({
            "name": str(item.get("name", "candidate"))[:80],
            "kp": clamp(kp, 0.05, 10.0),
            "ki": clamp(ki, 0.0, 8.0),
            "kd": clamp(kd, 0.0, 3.0),
            "score": clamp(float(item.get("score", 0)), 0.0, 100.0),
            "reason": str(item.get("reason", ""))[:300],
        })
    if not clean:
        raise ValueError("no valid candidates")
    return {
        "summary": payload.get("summary", {}) if isinstance(payload.get("summary", {}), dict) else {},
        "candidates": clean,
        "constraints": {
            "max_delta_ratio": 0.20,
            "no_vehicle_write": True,
            "low_speed_mode": bool(payload.get("lowSpeed", True)),
            "feedforward_preserved": bool(payload.get("feedforward", True)),
        },
    }


def ask_deepseek(data: dict, api_key: str) -> dict:
    system = (
        "你是移动机器人PID复核助手。只根据输入的结构化指标和候选参数排序，"
        "不得创造未提供的测量事实，不得建议比赛中在线写入参数。输出严格JSON："
        "{ranked:[{name,score,reason}],warnings:[string],requires_manual_test:true}。"
        "score范围0到100；超调、输出饱和、数据不足时降低评分并给出警告。"
    )
    body = json.dumps({
        "model": "deepseek-chat",
        "temperature": 0.1,
        "response_format": {"type": "json_object"},
        "messages": [
            {"role": "system", "content": system},
            {"role": "user", "content": json.dumps(data, ensure_ascii=False)},
        ],
    }).encode("utf-8")
    request = Request(API_URL, data=body, headers={
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}",
    }, method="POST")
    with urlopen(request, timeout=30) as response:
        result = json.loads(response.read().decode("utf-8"))
    content = result["choices"][0]["message"]["content"]
    advice = json.loads(content)
    if not isinstance(advice, dict):
        raise ValueError("DeepSeek response is not an object")
    advice["requires_manual_test"] = True
    return advice


class Handler(BaseHTTPRequestHandler):
    server_version = "PIDProxy/1.0"

    def log_message(self, *_args) -> None:
        return

    def _reply(self, status: int, payload: dict) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._reply(204, {})

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/v1/pid-advice":
            self._reply(404, {"error": "not found"})
            return
        key = os.environ.get("DEEPSEEK_API_KEY", "").strip()
        if not key:
            self._reply(503, {"error": "DEEPSEEK_API_KEY is not set"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if size <= 0 or size > 256 * 1024:
                raise ValueError("payload size must be 1..256 KiB")
            data = safe_payload(json.loads(self.rfile.read(size).decode("utf-8")))
            self._reply(200, ask_deepseek(data, key))
        except Exception as exc:  # keep provider details out of browser logs
            self._reply(400, {"error": str(exc)[:240]})


def main() -> int:
    parser = argparse.ArgumentParser(description="本机 DeepSeek PID 复核代理")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    args = parser.parse_args()
    if args.host not in {"127.0.0.1", "localhost"}:
        parser.error("代理只允许绑定本机地址")
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"PID proxy listening on http://{args.host}:{args.port}/v1/pid-advice")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
