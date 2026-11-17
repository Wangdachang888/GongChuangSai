[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$CommitMessage,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string[]]$Files,

    [string]$Remote = 'origin',

    [ValidateRange(1, 3)]
    [int]$MaxPushRetries = 2
)

$ErrorActionPreference = 'Stop'
$AllowedBranch = 'refactor/architecture-v2'
$ExpectedOrigin = '(?i)github\.com[/:]Wangdachang888/GongChuangSai(?:\.git)?$'
$BlockedPath = '(?i)(^|[\\/])(\.env|.*\.(pem|key|p12|pfx|secret|token|hex|axf|bin|elf|exe|log))$'
$SecretPattern = '(ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|-----BEGIN [^-]*PRIVATE KEY-----|(?i)aws_secret_access_key\s*=)'

function Invoke-Git {
    param([Parameter(Mandatory = $true)][string[]]$Arguments)

    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = @(& git @Arguments 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }
    if ($exitCode -ne 0) {
        $details = ($output | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine
        $details = $details.Trim()
        throw "git $($Arguments -join ' ') failed: $details"
    }
    return @($output | Where-Object { $_ -is [string] })
}

function Normalize-RepoPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    $normalized = $Path.Trim().Replace('\', '/')
    while ($normalized.StartsWith('./')) { $normalized = $normalized.Substring(2) }
    if ([IO.Path]::IsPathRooted($normalized) -or
        $normalized -eq '..' -or $normalized.StartsWith('../')) {
        throw "File path must stay inside the repository: $Path"
    }
    return $normalized
}

function Get-NameSet {
    param(
        [Parameter(Mandatory = $true)]
        [AllowEmptyCollection()]
        [string[]]$Names
    )

    $set = @{}
    foreach ($name in $Names) {
        if (-not [string]::IsNullOrWhiteSpace($name)) {
            $set[(Normalize-RepoPath $name)] = $true
        }
    }
    return $set
}

function Test-SetEqual {
    param(
        [Parameter(Mandatory = $true)][hashtable]$Expected,
        [Parameter(Mandatory = $true)][hashtable]$Actual
    )

    if ($Expected.Count -ne $Actual.Count) { return $false }
    foreach ($name in $Expected.Keys) {
        if (-not $Actual.ContainsKey($name)) { return $false }
    }
    return $true
}

function Get-RemoteSha {
    param(
        [Parameter(Mandatory = $true)][string]$RemoteName,
        [Parameter(Mandatory = $true)][string]$Ref
    )

    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $line = @(& git ls-remote $RemoteName $Ref 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }
    if ($exitCode -ne 0) {
        $details = ($line | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine
        $details = $details.Trim()
        throw "Unable to query remote SHA: $details"
    }
    $line = @($line | Where-Object { $_ -is [string] -and -not [string]::IsNullOrWhiteSpace($_) })
    if ($line.Count -eq 0 -or [string]::IsNullOrWhiteSpace($line[0])) { return '' }
    return (($line[0] -split '\s+')[0]).Trim()
}

try {
    $repoRoot = (Invoke-Git @('rev-parse', '--show-toplevel') | Select-Object -First 1).Trim()
    Set-Location -LiteralPath $repoRoot

    $branch = (Invoke-Git @('branch', '--show-current') | Select-Object -First 1).Trim()
    if ($branch -ne $AllowedBranch -or $branch -in @('main', 'master')) {
        throw "Automatic push is allowed only on $AllowedBranch; current branch is '$branch'."
    }

    $originUrl = (Invoke-Git @('remote', 'get-url', $Remote) | Select-Object -First 1).Trim()
    if ($originUrl -notmatch $ExpectedOrigin) {
        throw "Remote '$Remote' is not the authorized GongChuangSai GitHub repository: $originUrl"
    }

    $rawFiles = @()
    foreach ($fileItem in $Files) {
        $rawFiles += @($fileItem -split ',')
    }
    $normalizedFiles = @($rawFiles | ForEach-Object { Normalize-RepoPath $_ } | Select-Object -Unique)
    if ($normalizedFiles.Count -eq 0) { throw 'At least one explicit file is required.' }
    foreach ($path in $normalizedFiles) {
        if ($path -match $BlockedPath) {
            throw "Refusing sensitive or generated file: $path"
        }
        $fullPath = Join-Path $repoRoot ($path.Replace('/', '\'))
        if (Test-Path -LiteralPath $fullPath -PathType Leaf) {
            $item = Get-Item -LiteralPath $fullPath
            if ($item.Length -gt 10MB) { throw "Refusing file larger than 10 MB: $path" }
            if ($item.Extension -match '(?i)\.(c|h|md|ps1|txt|ioc|xml|json|yml|yaml)$') {
                $content = Get-Content -LiteralPath $fullPath -Raw
                if ($content -match $SecretPattern) {
                    throw "Refusing file containing a credential-like pattern: $path"
                }
            }
        }
    }

    $expected = Get-NameSet $normalizedFiles
    $worktreeNames = @()
    $worktreeNames += @(Invoke-Git @('diff', '--name-only'))
    $worktreeNames += @(Invoke-Git @('ls-files', '--others', '--exclude-standard'))
    $cachedNames = @(Invoke-Git @('diff', '--cached', '--name-only'))
    $worktreeSet = Get-NameSet $worktreeNames
    $cachedSet = Get-NameSet $cachedNames

    $unrelated = @($worktreeSet.Keys + $cachedSet.Keys |
        Sort-Object -Unique | Where-Object { -not $expected.ContainsKey($_) })
    if ($unrelated.Count -gt 0) {
        throw "Unrelated or unapproved working-tree/index changes exist: $($unrelated -join ', ')"
    }

    $addArguments = @('add', '--') + $normalizedFiles
    Invoke-Git $addArguments | Out-Null
    $cachedAfter = @(Invoke-Git @('diff', '--cached', '--name-only'))
    $cachedAfterSet = Get-NameSet $cachedAfter
    if ($cachedAfterSet.Count -eq 0) {
        Write-Output 'NOOP: no file changes; no commit or push created.'
        exit 0
    }
    if (-not (Test-SetEqual $expected $cachedAfterSet)) {
        throw "Staged paths do not exactly match the explicit file list: $($cachedAfter -join ', ')"
    }

    Invoke-Git @('diff', '--cached', '--check') | Out-Null
    Invoke-Git @('commit', '-m', $CommitMessage) | ForEach-Object { Write-Output $_ }
    $localSha = (Invoke-Git @('rev-parse', 'HEAD') | Select-Object -First 1).Trim()
    $remoteRef = "refs/heads/$branch"
    $remoteSha = Get-RemoteSha $Remote $remoteRef
    if ($remoteSha) {
        & git merge-base --is-ancestor $remoteSha $localSha 2>$null
        if ($LASTEXITCODE -ne 0) {
            throw "SYNC FAIL: remote $remoteSha is not an ancestor of local $localSha; refusing merge or force push."
        }
    }

    $pushed = $false
    for ($attempt = 1; $attempt -le $MaxPushRetries; $attempt++) {
        $previousPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            $pushOutput = @(& git push $Remote $branch 2>&1)
            $pushCode = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $previousPreference
        }
        $pushOutput | ForEach-Object { Write-Output $_ }
        if ($pushCode -eq 0) {
            $pushed = $true
            break
        }
        if ($attempt -lt $MaxPushRetries) {
            Write-Output "Push attempt $attempt failed; retrying without force push."
            Start-Sleep -Seconds 2
        }
    }
    if (-not $pushed) {
        throw "SYNC FAIL: ordinary push failed after $MaxPushRetries attempt(s); local commit $localSha was preserved."
    }

    $remoteSha = Get-RemoteSha $Remote $remoteRef
    if ($remoteSha -ne $localSha) {
        throw "SYNC FAIL: local SHA $localSha does not match remote SHA $remoteSha."
    }
    Write-Output "SYNC PASS: branch=$branch local=$localSha remote=$remoteSha"
    Write-Output "WORKTREE: $((Invoke-Git @('status', '--short')) -join ' ')"
}
catch {
    Write-Error ("SYNC FAIL: " + $_.Exception.Message)
    exit 1
}
