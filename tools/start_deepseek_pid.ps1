param(
    [string]$ProjectRoot = (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)),
    [int]$Port = 8787
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $ProjectRoot
trap {
    Write-Host "FAILED: $($_.Exception.Message)" -ForegroundColor Red
    Read-Host 'Press Enter to close' | Out-Null
    break
}

Write-Host 'Starting local DeepSeek PID proxy...'
Write-Host 'The key is used only by this process and is not saved.'
$secure = Read-Host 'Enter a NEW DeepSeek API key (input is hidden)' -AsSecureString
$ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secure)
try { $key = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr) }
finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr) }
if ([string]::IsNullOrWhiteSpace($key)) { throw 'No API key was entered.' }
$env:DEEPSEEK_API_KEY = $key

$proxy = Join-Path $ProjectRoot 'tools\deepseek_pid_proxy.py'
$viewer = Join-Path $ProjectRoot 'kart_log_view_mecanum_v4.html'
if (-not (Test-Path -LiteralPath $proxy) -or -not (Test-Path -LiteralPath $viewer)) {
    throw 'Project files are incomplete: proxy or viewer is missing.'
}

$args = @($proxy, '--host', '127.0.0.1', '--port', $Port)
$python = (Get-Command python -ErrorAction SilentlyContinue).Source
if ([string]::IsNullOrWhiteSpace($python)) { $python = (Get-Command py -ErrorAction SilentlyContinue).Source }
if ([string]::IsNullOrWhiteSpace($python)) { throw 'Python 3 was not found. Install Python and add it to PATH.' }
$existing = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
if ($existing) { throw "Port $Port is already in use. Close the old proxy or choose another port." }
$process = $null
$process = Start-Process -FilePath $python -ArgumentList $args -WorkingDirectory $ProjectRoot -WindowStyle Hidden -PassThru
Write-Host "Proxy process started (PID=$($process.Id)); checking health..."
try {
    $health = $null
    for ($i = 0; $i -lt 10 -and $null -eq $health; $i++) {
        Start-Sleep -Milliseconds 300
        try { $health = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/health" -TimeoutSec 2 } catch { }
        if ($process.HasExited) { throw "Proxy exited immediately with code $($process.ExitCode)." }
    }
    if ($null -eq $health -or -not $health.ok -or -not $health.key_configured) {
        throw "Health check failed at http://127.0.0.1:$Port/health"
    }
} catch {
    if (-not $process.HasExited) { Stop-Process -Id $process.Id -Force }
    throw "DeepSeek proxy failed to start: $($_.Exception.Message)"
}

Start-Process -FilePath $viewer
Write-Host ''
Write-Host 'SUCCESS: proxy is running.' -ForegroundColor Green
Write-Host 'The PID web page has been opened.'
Write-Host 'Click Connect DeepSeek, then Call DeepSeek review.'
Write-Host 'Keep this window open while using the AI review.'
