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
$runningHealth = $null
try { $runningHealth = Invoke-RestMethod -Uri "http://127.0.0.1:$Port/health" -TimeoutSec 2 } catch { }
if ($runningHealth -and $runningHealth.ok -and $runningHealth.key_configured) {
    if ($runningHealth.provider -eq 'deepseek' -and [int]$runningHealth.advisor_version -lt 2) {
        # Restart only this project's verified Python listener; never kill an
        # arbitrary process merely because it occupies the configured port.
        $pidListener = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction Stop | Select-Object -First 1
        $pidOldProcess = Get-CimInstance Win32_Process -Filter "ProcessId=$($pidListener.OwningProcess)"
        $pidExpectedScript = Join-Path $ProjectRoot 'tools\deepseek_pid_proxy.py'
        if (-not $pidOldProcess -or -not $pidOldProcess.CommandLine -or $pidOldProcess.Name -notmatch '^python(w|\d*(\.\d+)*)?\.exe$' -or
            $pidOldProcess.CommandLine.IndexOf($pidExpectedScript, [StringComparison]::OrdinalIgnoreCase) -lt 0) {
            throw "An older proxy uses port $Port, but its process cannot be verified as this project. Close it manually and rerun."
        }
        Stop-Process -Id $pidListener.OwningProcess -ErrorAction Stop
        Write-Host 'Stopped the verified old project proxy; starting advisor v2.'
        $runningHealth = $null
    }
}
if ($runningHealth -and $runningHealth.ok -and $runningHealth.key_configured -and [int]$runningHealth.advisor_version -ge 2) {
    Start-Process -FilePath (Join-Path $ProjectRoot 'kart_log_view_mecanum_v4.html')
    Write-Host "An existing healthy proxy is already running on port $Port." -ForegroundColor Green
    Write-Host 'The PID web page has been opened. Keep the existing proxy running.'
    Read-Host 'Press Enter to close this launcher' | Out-Null
    exit 0
}
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

$quotedProxy = '"' + $proxy.Replace('"', '\"') + '"'
$args = @($quotedProxy, '--host', '127.0.0.1', '--port', $Port)
$python = (Get-Command python -ErrorAction SilentlyContinue).Source
if ([string]::IsNullOrWhiteSpace($python)) { $python = (Get-Command py -ErrorAction SilentlyContinue).Source }
if ([string]::IsNullOrWhiteSpace($python)) { throw 'Python 3 was not found. Install Python and add it to PATH.' }
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
Write-Host "If the browser did not open, open this file manually: $viewer"
Read-Host 'Press Enter to close this launcher (the proxy will keep running)' | Out-Null
