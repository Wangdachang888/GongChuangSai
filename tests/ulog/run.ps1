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
    foreach ($mode in @(0, 3)) {
        $binary = "tmp/test_ulog_$mode.exe"
        & $Compiler @common "-DAPP_RUN_MODE=$mode" -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c Hardware/bsp_ulog.c App/race_task_plan.c -lm -o $binary
        if ($LASTEXITCODE) { throw 'Logger compile failed' }
        foreach ($case in @('normal','early','abort','timeout','tx_error','format_error','nonfinite','manual','null','init_error','burst_short','burst_live','burst_deferred','burst_fault','burst_abort')) {
            & (Join-Path $root $binary) $case
            if ($LASTEXITCODE) { throw "Logger test failed: $mode $case" }
        }
    }
    & $Compiler @common -DULOG_ASYNC_TEST -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c Hardware/bsp_ulog.c App/race_task_plan.c -lm -o tmp/test_ulog_async.exe
    if ($LASTEXITCODE) { throw 'Async logger compile failed' }
    & (Join-Path $root 'tmp/test_ulog_async.exe') async
    if ($LASTEXITCODE) { throw 'Async logger backpressure failed' }
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
print("PASS Python: sync/async firmware -> 2000 samples, 1/7/181-byte splits, CRC recovery, durable close")
'@ | python -
        if ($LASTEXITCODE) { throw 'PID binary receiver verification failed' }
    } else {
        Write-Output 'SKIP: Python PID receiver tests (Python missing)'
    }
    & $Compiler @common -DULOG_PID_BURST_ENABLE=0 -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c Hardware/bsp_ulog.c App/race_task_plan.c -lm -o tmp/test_ulog_no_burst.exe
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
