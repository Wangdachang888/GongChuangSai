param([string]$Compiler = '')
$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($Compiler)) {
    $gcc = Get-Command gcc -ErrorAction SilentlyContinue
    if ($gcc) { $Compiler = $gcc.Source }
}
if ([string]::IsNullOrWhiteSpace($Compiler) -or
    -not (Test-Path -LiteralPath $Compiler -PathType Leaf)) {
    throw 'GCC not found. Pass -Compiler <path-to-gcc.exe> or add gcc to PATH.'
}
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    $common = @('-DULOG_TEST', '-std=c99', '-Wall', '-Wextra', '-Werror', '-Itests/ulog', '-Itests/race/stubs', '-IConfig', '-IControl', '-IApp', '-IHardware', '-include', 'tests/race/stubs/board.h')
    # Compile the exact parameter access block and actual Motor constants.
@'
from pathlib import Path
runtime=Path("App/app_runtime.c").read_text(encoding="utf-8")
block="/* PID_ACCESS_BEGIN:"+runtime.split("/* PID_ACCESS_BEGIN:",1)[1].split("/* PID_ACCESS_END */",1)[0]
motor=Path("Control/Motor.c").read_bytes().decode("gb18030",errors="replace")
constants=motor[motor.index("#ifndef MOTOR_FF_GAIN"):motor.index("static int32_t Encoder_ReadDelta")]
head='#include "board.h"\n#include "app_runtime.h"\n#include "app_mode.h"\n#include "app_race.h"\n#include "app_square.h"\n#include <math.h>\nextern Motor_t motor_left_front,motor_right_front,motor_left_back,motor_right_back;\nextern MecanumChassis_t chassis;\n'
Path("tmp/runtime_pid_access.c").write_text(head+constants+block+'\nvoid ULog_TestObserve(void){pid_edit_observe();}\n',encoding="utf-8")
'@ | python -
    if ($LASTEXITCODE) { throw 'Runtime PID fixture extraction failed' }
    foreach ($mode in @(0, 3)) {
        $binary = "tmp/test_ulog_$mode.exe"
        & $Compiler @common "-DAPP_RUN_MODE=$mode" -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c Hardware/bsp_ulog.c App/race_task_plan.c tmp/runtime_pid_access.c -lm -o $binary
        if ($LASTEXITCODE) { throw 'Logger compile failed' }
        foreach ($case in @('normal','early','abort','timeout','tx_error','format_error','nonfinite','manual','null','init_error','burst_short','burst_live','burst_deferred','burst_fault','burst_abort','burst_continuous','params')) {
            & (Join-Path $root $binary) $case
            if ($LASTEXITCODE) { throw "Logger test failed: $mode $case" }
        }
    }
    & $Compiler @common -DULOG_ASYNC_TEST -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c Hardware/bsp_ulog.c App/race_task_plan.c tmp/runtime_pid_access.c -lm -o tmp/test_ulog_async.exe
    if ($LASTEXITCODE) { throw 'Async logger compile failed' }
    & (Join-Path $root 'tmp/test_ulog_async.exe') async
    if ($LASTEXITCODE) { throw 'Async logger backpressure failed' }
    & (Join-Path $root 'tmp/test_ulog_async.exe') params_async
    if ($LASTEXITCODE) { throw 'Async PID parameter reply failed' }
    if (Get-Command python -ErrorAction SilentlyContinue) {
@'
from pathlib import Path
import csv
from tools.bt10_ble_receiver import LogReceiver
for name in ("pid_wire_test", "pid_async_wire"):
    wire = Path("tmp/" + name + ".bin").read_bytes()
    for chunk in (1, 7, 181):
        output = Path("tmp/" + name + "_decoded.csv")
        pid = output.with_suffix(".pid.csv")
        r = LogReceiver(output, pid_output=pid)
        for i in range(0, len(wire), chunk):
            r.feed(wire[i:i + chunk])
        assert r.closed_by_end and r.pid_closed_by_end and r.pid_lines == 2000
        rows = list(csv.reader(line for line in pid.open(encoding="utf-8")
                               if not line.startswith("#")))
        assert len(rows) == 2001
        assert int(rows[-1][0]) - int(rows[1][0]) == 19990
    start = wire.index(bytes([165, 90, 1, 2]))
    good = wire[start:start + 36]
    bad = bytearray(good)
    bad[15] ^= 1
    r = LogReceiver(Path("tmp/pid_crc_recovery.csv"))
    r.feed(bad + b"#BTLOG,STATE,READY\n" + good)
    assert r.dropped and r.state == "READY"
    r.close()
wire = Path("tmp/pid_continuous_wire.bin").read_bytes()
r = LogReceiver(Path("tmp/pid_continuous_decoded.csv"), pid_output=Path("tmp/pid_continuous_decoded.pid.csv"))
for i in range(0, len(wire), 181):
    r.feed(wire[i:i + 181])
assert r.closed_by_end and r.pid_lines == 2000
rows = list(csv.reader(line for line in Path("tmp/pid_continuous_decoded.pid.csv").open(encoding="utf-8") if not line.startswith("#")))
assert int(rows[1][0]) == 342040 and int(rows[-1][0]) == 362030
live = [line for line in Path("tmp/pid_continuous_decoded.csv").read_text(encoding="utf-8").splitlines() if line.startswith("#BTLOG,PID_LIVE,")]
assert len(live) == 72006
seqs = [int(line.split(",")[2]) for line in live]
assert all((b - a) % 65536 == 1 for a, b in zip(seqs, seqs[1:]))
r=LogReceiver(Path("tmp/pid_params_decoded.csv"),pid_output=Path("tmp/pid_params_decoded.pid.csv"))
wire=Path("tmp/pid_params_wire.bin").read_bytes()
for i in range(0,len(wire),7):
    r.feed(wire[i:i+7])
r.close()
pid_text=Path("tmp/pid_params_decoded.pid.csv").read_text(encoding="utf-8")
assert pid_text.count("#BTLOG,PID_USED,")==7 and "PID_USED,0,2,FL,READ,3," in pid_text
reply=Path("tmp/pid_params_async.bin").read_bytes()
assert reply.count(b"#BTLOG,PID_PARAM,1,1,FL,SET,")==1
from tools.deepseek_pid_proxy import safe_payload
actual_line=next(line for line in pid_text.splitlines() if line.startswith("#BTLOG,PID_USED,0,2,FL,"))
f=actual_line.split(",")
keys=["kp","ki","kd","integralMax","outputMax","periodMs","ffGain","deadzone","ffMinSpeed","ramp","slew","lpf","kickStep","kickMax","stuckRatio"]
actual={"axis":"FL","revision":int(f[3]),**dict(zip(keys,map(float,f[6:21])))}
payload={"axis":"FL","recordedParameters":actual,"candidates":[{"kp":2.7,"ki":1.5,"kd":0}]}
safe=safe_payload(payload)
assert safe["recordedParameters"]["kp"]==3 and safe["parameterSource"]=="recorded_runtime"
assert safe_payload({"candidates":payload["candidates"]})["parameterSource"]=="unverified_offline"
print("PASS Python: split/CRC; continuous 72006 live samples; sequence wrap; last-20s cache; actual seven-axis PID metadata retained")
'@ | python -
        if ($LASTEXITCODE) { throw 'PID binary receiver verification failed' }
    } else {
        Write-Output 'SKIP: Python PID receiver tests (Python missing)'
    }
    & $Compiler @common -DULOG_PID_BURST_ENABLE=0 -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c Hardware/bsp_ulog.c App/race_task_plan.c tmp/runtime_pid_access.c -lm -o tmp/test_ulog_no_burst.exe
    if ($LASTEXITCODE) { throw 'No-burst logger compile failed' }
    & (Join-Path $root 'tmp/test_ulog_no_burst.exe') normal
    if ($LASTEXITCODE) { throw 'No-burst logger failed' }
    & $Compiler @common -DULOG_ENABLE=0 tests/ulog/test_ulog.c Hardware/bsp_ulog.c -lm -o tmp/test_ulog_disabled.exe
    if ($LASTEXITCODE) { throw 'Disabled logger compile failed' }
    & (Join-Path $root 'tmp/test_ulog_disabled.exe')
    if ($LASTEXITCODE) { throw 'Disabled logger failed' }
    foreach ($direct in @(0,1)) {
        $extra = @('-include', 'tests/ulog/config.h')
        if ($direct) { $extra += '-DTEST_DIRECT=1' }
        $binary = "tmp/test_log_flow_$direct.exe"
        & $Compiler @common @extra tests/ulog/test_flow.c App/app_race.c App/race_ui.c App/race_fsm.c App/app_pick_fsm.c App/app_pick_motion.c App/race_task_plan.c -lm -o $binary
        if ($LASTEXITCODE) { throw 'Flow compile failed' }
        & (Join-Path $root $binary)
        if ($LASTEXITCODE) { throw 'Flow test failed' }
    }
} finally { Pop-Location }
