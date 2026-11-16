param([string]$Compiler = 'D:/minGW/mingw64/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$raceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $raceRoot
try {
    $common = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-Itests/race/stubs', '-IHardware')
    $cases = @(
        @{ Name = 'test_race'; Extra = @(); Sources = @('Hardware/race_fsm.c', 'Hardware/race_vision_protocol.c', 'Hardware/race_task_plan.c') },
        @{ Name = 'test_uart'; Extra = @(); Sources = @('Hardware/race_fsm.c', 'Hardware/race_vision.c', 'Hardware/race_vision_protocol.c', 'Hardware/bsp_uart_router.c') },
        @{ Name = 'test_app'; Extra = @('-include', 'tests/race/stubs/board.h', '-include', 'tests/race/stubs/config.h'); Sources = @('Hardware/race_fsm.c', 'Hardware/app_race.c', 'Hardware/app_pick_fsm.c', 'Hardware/app_pick_motion.c', 'Hardware/race_task_plan.c') }
    )
    foreach ($case in $cases) {
        $source = 'tests/race/' + $case.Name + '.c'
        $binary = 'tests/race/' + $case.Name + '.exe'
        $compileArgs = $common + $case.Extra + @($source) + $case.Sources + @('-lm', '-o', $binary)
        & $Compiler @compileArgs
        if ($LASTEXITCODE -ne 0) { throw ('Compile failed: ' + $case.Name) }
        & (Join-Path $raceRoot $binary)
        if ($LASTEXITCODE -ne 0) { throw ('Test failed: ' + $case.Name) }
    }
} finally {
    Pop-Location
}
