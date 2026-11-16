param([string]$Compiler = 'D:/minGW/mingw64/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    $common = @('-DULOG_TEST', '-std=c99', '-Wall', '-Wextra', '-Werror', '-Itests/ulog', '-Itests/race/stubs', '-IHardware', '-include', 'tests/race/stubs/board.h')
    foreach ($mode in @(0, 3)) {
        $binary = "tmp/test_ulog_$mode.exe"
        & $Compiler @common "-DAPP_RUN_MODE=$mode" -Dsnprintf=ulog_test_snprintf tests/ulog/test_ulog.c Hardware/bsp_ulog.c Hardware/race_task_plan.c -lm -o $binary
        if ($LASTEXITCODE) { throw 'Logger compile failed' }
        foreach ($case in @('normal','early','abort','timeout','tx_error','format_error','nonfinite','manual','null','init_error')) {
            & (Join-Path $root $binary) $case
            if ($LASTEXITCODE) { throw "Logger test failed: $mode $case" }
        }
    }
    & $Compiler @common -DULOG_ENABLE=0 tests/ulog/test_ulog.c Hardware/bsp_ulog.c -lm -o tmp/test_ulog_disabled.exe
    if ($LASTEXITCODE) { throw 'Disabled logger compile failed' }
    & (Join-Path $root 'tmp/test_ulog_disabled.exe')
    if ($LASTEXITCODE) { throw 'Disabled logger failed' }
    foreach ($direct in @(0,1)) {
        $extra = @('-include', 'tests/ulog/config.h')
        if ($direct) { $extra += '-DTEST_DIRECT=1' }
        $binary = "tmp/test_log_flow_$direct.exe"
        & $Compiler @common @extra tests/ulog/test_flow.c Hardware/app_race.c Hardware/race_fsm.c Hardware/app_pick_fsm.c Hardware/app_pick_motion.c Hardware/race_task_plan.c -lm -o $binary
        if ($LASTEXITCODE) { throw 'Flow compile failed' }
        & (Join-Path $root $binary)
        if ($LASTEXITCODE) { throw 'Flow test failed' }
    }
} finally { Pop-Location }
