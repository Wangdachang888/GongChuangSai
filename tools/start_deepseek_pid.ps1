param(
    [string]$ProjectRoot = (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)),
    [int]$Port = 8787
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $ProjectRoot

if ([string]::IsNullOrWhiteSpace($env:DEEPSEEK_API_KEY)) {
    Write-Host 'DEEPSEEK_API_KEY is not set.'
    Write-Host '$env:DEEPSEEK_API_KEY="your-new-key"'
    Write-Host 'For persistent user configuration:'
    Write-Host '[Environment]::SetEnvironmentVariable("DEEPSEEK_API_KEY","your-new-key","User")'
    exit 1
}

$proxy = Join-Path $ProjectRoot 'tools\deepseek_pid_proxy.py'
$viewer = Join-Path $ProjectRoot 'kart_log_view_mecanum_v4.html'
if (-not (Test-Path -LiteralPath $proxy) -or -not (Test-Path -LiteralPath $viewer)) {
    throw 'Project files are incomplete: proxy or viewer is missing.'
}

$args = @($proxy, '--host', '127.0.0.1', '--port', $Port)
$process = Start-Process -FilePath 'python' -ArgumentList $args -WorkingDirectory $ProjectRoot -WindowStyle Hidden -PassThru
Start-Sleep -Milliseconds 700
try {
    $health = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/health" -TimeoutSec 3
    if (-not $health.ok -or -not $health.key_configured) {
        throw 'Proxy started but key health check failed.'
    }
} catch {
    if (-not $process.HasExited) { Stop-Process -Id $process.Id -Force }
    throw "DeepSeek proxy failed to start: $($_.Exception.Message)"
}

Start-Process -FilePath $viewer
Write-Host "DeepSeek proxy started, PID=$($process.Id); viewer opened."
Write-Host 'In the PID page, click Connect DeepSeek, then Call DeepSeek review.'
