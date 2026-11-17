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
$raceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $raceRoot
try {
    $common = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-Itests/race/stubs', '-IHardware', '-include', 'tests/race/stubs/board.h')
    $cases = @(
        @{ Name = 'test_race'; Extra = @(); Sources = @('Hardware/race_fsm.c', 'Hardware/vision.c', 'Hardware/race_task_plan.c') },
        @{ Name = 'test_uart'; Extra = @(); Sources = @('Hardware/race_fsm.c', 'Hardware/vision.c', 'Hardware/bsp_uart_router.c') },
        @{ Name = 'test_ring'; Extra = @(); Sources = @('Hardware/race_fsm.c', 'Hardware/vision.c', 'Hardware/app_ring_align.c') },
        @{ Name = 'test_app'; Extra = @('-include', 'tests/race/stubs/board.h', '-include', 'tests/race/stubs/config.h'); Sources = @('Hardware/race_fsm.c', 'Hardware/app_race.c', 'Hardware/race_ui.c', 'Hardware/app_pick_fsm.c', 'Hardware/app_pick_motion.c', 'Hardware/race_task_plan.c') }
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
