"""Local, opt-in DeepSeek advisor for PID candidate review.

The browser never receives the API key.  Set DEEPSEEK_API_KEY in the local
process environment before starting this proxy.  The endpoint only returns
structured advice; it does not send commands to the vehicle.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen

API_URL = "https://api.deepseek.com/chat/completions"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def bounded_evidence(value: object, depth: int = 0) -> object:
    """Keep numerical evidence finite/bounded; never pass an arbitrary raw log."""
    if depth > 7:
        return None
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        return value[:1200]
    if isinstance(value, list):
        return [bounded_evidence(x, depth + 1) for x in value[:180]]
    if isinstance(value, dict):
        return {str(k)[:80]: bounded_evidence(v, depth + 1) for k, v in list(value.items())[:50]}
    return None


def safe_payload(payload: object) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("payload must be an object")
    candidates = payload.get("candidates")
    if not isinstance(candidates, list) or len(candidates) > 8:
        raise ValueError("candidates must contain 0..8 items (zero means diagnosis only)")
    clean = []
    for item in candidates:
        if not isinstance(item, dict):
            continue
        try:
            kp, ki, kd = (float(item[k]) for k in ("kp", "ki", "kd"))
        except (KeyError, TypeError, ValueError):
            continue
        if not all(math.isfinite(x) and 0 <= x <= limit for x, limit in zip((kp, ki, kd), (10, 8, 3))):
            continue
        clean.append({
            "name": str(item.get("name", "candidate"))[:80],
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "score": bounded_evidence(item.get("score")),
            "reason": str(item.get("reason", ""))[:300],
            "prediction": bounded_evidence(item.get("prediction")),
            "eligible": item.get("eligible") is True,
        })
    axis = payload.get("axis")
    if axis not in {"FL", "FR", "RL", "RR"}:
        axis = None
    recorded = payload.get("recordedParameters")
    actual = None
    if isinstance(recorded, dict) and axis == recorded.get("axis"):
        keys = ("kp", "ki", "kd", "periodMs", "outputMax", "integralMax", "revision",
                "ffGain", "deadzone", "ffMinSpeed", "ramp", "slew", "lpf",
                "kickStep", "kickMax", "stuckRatio")
        try:
            numeric = {k: float(recorded[k]) for k in keys}
            if all(math.isfinite(x) for x in numeric.values()) and numeric["periodMs"] > 0 and numeric["outputMax"] > 0:
                actual = {"axis": axis, **numeric}
        except (KeyError, TypeError, ValueError):
            pass
    return {
        "axis": axis,
        "recordedParameters": actual,
        "parameterSource": "recorded_runtime" if actual else "unverified_offline",
        "summary": bounded_evidence(payload.get("summary", {})),
        "quality": bounded_evidence(payload.get("quality", {})),
        "model": bounded_evidence(payload.get("model", {})),
        "controllerAudit": bounded_evidence(payload.get("controllerAudit", {})),
        "segments": bounded_evidence(payload.get("segments", [])),
        "series": bounded_evidence(payload.get("series", [])[:160]) if isinstance(payload.get("series"), list) else [],
        "seriesStride": bounded_evidence(payload.get("seriesStride")),
        "baselinePrediction": bounded_evidence(payload.get("baselinePrediction")),
        "candidates": clean,
        "constraints": {
            "max_delta_ratio": 0.20,
            "no_vehicle_write": True,
            "low_speed_mode": bool(payload.get("lowSpeed", True)),
            "feedforward_preserved": bool(payload.get("feedforward", True)),
        },
    }


def validate_advice(advice: object, data: dict) -> dict:
    if not isinstance(advice, dict):
        raise ValueError("DeepSeek response is not an object")
    names = {c["name"] for c in data["candidates"]}
    ranked = []
    for item in advice.get("ranked", [])[:8] if isinstance(advice.get("ranked"), list) else []:
        if isinstance(item, dict) and item.get("name") in names and not any(x["name"] == item["name"] for x in ranked):
            ranked.append({"name": item["name"], "reason": str(item.get("reason", ""))[:1200]})
    actual = data.get("recordedParameters")
    proposed = []
    model = data.get("model") if isinstance(data.get("model"), dict) else {}
    audit = data.get("controllerAudit") if isinstance(data.get("controllerAudit"), dict) else {}
    # Proposals are allowed only with a locally validated controller/plant.
    can_propose = actual and model.get("valid") is True and isinstance(data.get("baselinePrediction"), dict)
    can_propose = can_propose and isinstance(audit.get("normalizedError"), (int, float)) and audit["normalizedError"] <= 0.08
    for item in advice.get("proposed", [])[:4] if can_propose and isinstance(advice.get("proposed"), list) else []:
        if not isinstance(item, dict):
            continue
        try:
            gains = {k: float(item[k]) for k in ("kp", "ki", "kd")}
        except (KeyError, ValueError, TypeError):
            continue
        bounds = {k: (max(0.0, actual[k] * .8), min(limit, max(actual[k] * 1.2,
                  actual["kp"] * actual["periodMs"] / 1000 * .1 if k == "kd" else .0001)))
                  for k, limit in zip(("kp", "ki", "kd"), (10, 8, 3))}
        if all(math.isfinite(v) and bounds[k][0] - 1e-8 <= v <= bounds[k][1] + 1e-8 for k, v in gains.items()):
            proposed.append({"name": str(item.get("name", "小步试验"))[:80], **gains,
                             "reason": str(item.get("reason", ""))[:1200]})
    warnings = advice.get("warnings", [])
    return {"diagnosis": str(advice.get("diagnosis", ""))[:3000], "ranked": ranked,
            "proposed": proposed, "next_experiment": str(advice.get("next_experiment", ""))[:1500],
            "warnings": [str(x)[:1200] for x in warnings[:12]] if isinstance(warnings, list) else [],
            "requires_manual_test": True, "score_source": "local_model_only"}


def ask_deepseek(data: dict, api_key: str) -> dict:
    system = (
        "你是移动机器人速度环系统辨识与PID诊断工程师。使用实测时序包络、分段、正反向指标、"
        "真实PID_USED及前馈/限幅/积分/kick/slew配置分析。series是抽样包络而非完整原始序列；"
        "不可据它发明精确频谱、相位裕度或稳定性结论。区分实测表现、独立模型验证和候选预测，"
        "不得把当前差的日志分数直接当新候选分数。不要重新打分，不要为了得到高分放宽标准。"
        "先检查时间质量、目标激励、饱和、低速停滞、正反向差异，再检查模型20%测试段、"
        "残差与原PID重放。停滞只能提示摩擦/负载/电源/机械待查，不断言原因。"
        "完整目标平台才可判断阶跃超调/稳定时间。该固件是并联PID，Ki单位每秒，"
        "D对误差微分；目标已经过斜坡，模型含测速LPF，不要套默认Arduino离散Ki/Kd。"
        "可以比较现有候选，或在真实基准±20%内提出最多4组参数，前馈等配置保持不变；"
        "基准Kd=0时新Kd最多Kp*periodMs/1000*0.1。model无效或没有baselinePrediction时"
        "只能诊断、提出人工补采方案，proposed必须为空。不能自行写车或建议比赛中改参。"
        "输出严格JSON：{diagnosis:string,ranked:[{name,reason}],proposed:[{name,kp,ki,kd,reason}],"
        "next_experiment:string,warnings:[string],requires_manual_test:true}。"
        "诊断应引用具体数值及其证据来源，并说明优先改PID还是应先补采/检查其他因素。"
    )
    body = json.dumps({
        "model": "deepseek-chat",
        "temperature": 0.1,
        "max_tokens": 3000,
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
    return validate_advice(advice, data)


class Handler(BaseHTTPRequestHandler):
    server_version = "PIDProxy/2.0"

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

    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/health":
            self._reply(404, {"error": "not found"})
            return
        self._reply(200, {"ok": True, "provider": "deepseek", "advisor_version": 2,
                         "key_configured": bool(os.environ.get("DEEPSEEK_API_KEY", "").strip())})

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
