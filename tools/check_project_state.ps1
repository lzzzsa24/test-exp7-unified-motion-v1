$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$statePath = Join-Path $repoRoot "PROJECT_STATE.md"
$script:failureCount = 0

function Write-Result {
    param(
        [ValidateSet("OK", "WARN", "ERROR")]
        [string]$Level,
        [string]$Message
    )

    # Write-Host keeps status text visible without contaminating a helper
    # function's success-output return value.
    Write-Host ("[{0}] {1}" -f $Level, $Message)
    if ($Level -eq "ERROR") {
        $script:failureCount++
    }
}

function Get-StateField {
    param(
        [string]$Text,
        [string]$Name
    )

    $pattern = "(?m)^\s*" + [regex]::Escape($Name) + "\s*:\s*(\S+)\s*$"
    $match = [regex]::Match($Text, $pattern)
    if (-not $match.Success) {
        return $null
    }
    return $match.Groups[1].Value
}

function Resolve-Commit {
    param(
        [string]$Label,
        [string]$Commit
    )

    if ([string]::IsNullOrWhiteSpace($Commit)) {
        Write-Result "ERROR" ("Missing {0}." -f $Label)
        return $null
    }

    $resolved = (& git rev-parse --verify ("{0}^{{commit}}" -f $Commit) 2>$null)
    if ($LASTEXITCODE -ne 0) {
        Write-Result "ERROR" ("{0} does not resolve to a commit: {1}" -f $Label, $Commit)
        return $null
    }

    Write-Result "OK" ("{0} resolves to {1}." -f $Label, $resolved)
    return $resolved.Trim()
}

if (-not (Test-Path -LiteralPath $statePath -PathType Leaf)) {
    Write-Result "ERROR" ("Missing shared state file: {0}" -f $statePath)
    exit 1
}

Push-Location $repoRoot
try {
    & git rev-parse --is-inside-work-tree *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Result "ERROR" ("Not a Git worktree: {0}" -f $repoRoot)
        exit 1
    }

    $stateText = Get-Content -LiteralPath $statePath -Raw
    $fields = @{}
    $requiredFields = @(
        "state_schema_version",
        "state_updated_at",
        "integration_branch",
        "repository_head_at_update",
        "latest_code_commit",
        "flashed_source_commit",
        "flash_record_commit",
        "deployed_tag",
        "formal_bin_path",
        "formal_hex_path",
        "formal_bin_size_bytes",
        "flashed_bin_sha256",
        "flashed_hex_sha256"
    )

    foreach ($field in $requiredFields) {
        $value = Get-StateField -Text $stateText -Name $field
        if ([string]::IsNullOrWhiteSpace($value)) {
            Write-Result "ERROR" ("PROJECT_STATE.md is missing field {0}." -f $field)
        }
        else {
            $fields[$field] = $value
        }
    }

    if ($script:failureCount -gt 0) {
        exit 1
    }

    $branch = (& git branch --show-current).Trim()
    $head = (& git rev-parse HEAD).Trim()
    Write-Output "PROJECT STATE CHECK"
    Write-Output ("repository : {0}" -f $repoRoot)
    Write-Output ("live branch: {0}" -f $branch)
    Write-Output ("live HEAD  : {0}" -f $head)
    Write-Output ("state date : {0}" -f $fields["state_updated_at"])

    if ($branch -eq $fields["integration_branch"]) {
        Write-Result "OK" "Live branch matches the integration branch."
    }
    else {
        Write-Result "WARN" ("Live branch is {0}; integration branch is {1}. This is expected only in an intentional worker worktree." -f $branch, $fields["integration_branch"])
    }

    $anchor = Resolve-Commit "repository_head_at_update" $fields["repository_head_at_update"]
    $latest = Resolve-Commit "latest_code_commit" $fields["latest_code_commit"]
    $flashed = Resolve-Commit "flashed_source_commit" $fields["flashed_source_commit"]
    $flashRecord = Resolve-Commit "flash_record_commit" $fields["flash_record_commit"]
    $candidateText = Get-StateField -Text $stateText -Name "candidate_source_commit"
    $candidate = $null
    if (-not [string]::IsNullOrWhiteSpace($candidateText)) {
        $candidate = Resolve-Commit "candidate_source_commit" $candidateText
    }

    if ($null -ne $anchor) {
        & git merge-base --is-ancestor $anchor $head
        if ($LASTEXITCODE -eq 0) {
            Write-Result "OK" "The recorded repository anchor is an ancestor of live HEAD."
        }
        else {
            Write-Result "ERROR" "The recorded repository anchor is not an ancestor of live HEAD. Shared state is from another history."
        }
    }

    $tagCommit = (& git rev-list -n 1 $fields["deployed_tag"] 2>$null)
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($tagCommit)) {
        Write-Result "ERROR" ("Missing deployed tag: {0}" -f $fields["deployed_tag"])
    }
    elseif ($null -ne $flashed -and $tagCommit.Trim() -eq $flashed) {
        Write-Result "OK" ("Deployed tag points to flashed source commit {0}." -f $flashed)
    }
    else {
        Write-Result "ERROR" ("Deployed tag points to {0}, not flashed source {1}." -f $tagCommit.Trim(), $flashed)
    }

    foreach ($kind in @("bin", "hex")) {
        $pathField = "formal_{0}_path" -f $kind
        $hashField = "flashed_{0}_sha256" -f $kind
        $relativePath = $fields[$pathField] -replace "/", [IO.Path]::DirectorySeparatorChar
        $artifactPath = Join-Path $repoRoot $relativePath
        $expectedHash = $fields[$hashField].ToUpperInvariant()
        $candidateHash = Get-StateField -Text $stateText -Name ("candidate_{0}_sha256" -f $kind)
        if (-not [string]::IsNullOrWhiteSpace($candidateHash)) {
            $candidateHash = $candidateHash.ToUpperInvariant()
        }

        if ($expectedHash -notmatch "^[0-9A-F]{64}$") {
            Write-Result "ERROR" ("Invalid SHA-256 in {0}." -f $hashField)
        }
        elseif (-not [string]::IsNullOrWhiteSpace($candidateHash) -and
                $candidateHash -notmatch "^[0-9A-F]{64}$") {
            Write-Result "ERROR" ("Invalid SHA-256 in candidate_{0}_sha256." -f $kind)
        }
        elseif (-not (Test-Path -LiteralPath $artifactPath -PathType Leaf)) {
            Write-Result "WARN" ("Local {0} artifact is absent; rebuild from the recorded source commit: {1}" -f $kind.ToUpperInvariant(), $artifactPath)
        }
        else {
            $actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $artifactPath).Hash.ToUpperInvariant()
            if ($actualHash -eq $expectedHash) {
                Write-Result "OK" ("Local {0} matches the flashed SHA-256." -f $kind.ToUpperInvariant())
            }
            elseif (-not [string]::IsNullOrWhiteSpace($candidateHash) -and
                    $actualHash -eq $candidateHash) {
                Write-Result "OK" ("Local {0} matches the recorded unflashed candidate; the board still uses the separate flashed hash." -f $kind.ToUpperInvariant())
            }
            else {
                Write-Result "WARN" ("Local {0} differs from the flashed image. This can be valid after an unflashed rebuild. expected={1} actual={2}" -f $kind.ToUpperInvariant(), $expectedHash, $actualHash)
            }
        }
    }

    $binRelative = $fields["formal_bin_path"] -replace "/", [IO.Path]::DirectorySeparatorChar
    $binPath = Join-Path $repoRoot $binRelative
    if (Test-Path -LiteralPath $binPath -PathType Leaf) {
        $actualSize = (Get-Item -LiteralPath $binPath).Length
        $expectedSize = [int64]$fields["formal_bin_size_bytes"]
        $candidateSizeText = Get-StateField -Text $stateText -Name "candidate_bin_size_bytes"
        if ($actualSize -eq $expectedSize) {
            Write-Result "OK" ("Local BIN size matches the flashed snapshot: {0} bytes." -f $actualSize)
        }
        elseif (-not [string]::IsNullOrWhiteSpace($candidateSizeText) -and
                $actualSize -eq [int64]$candidateSizeText) {
            Write-Result "OK" ("Local BIN size matches the unflashed candidate: {0} bytes." -f $actualSize)
        }
        else {
            Write-Result "WARN" ("Local BIN size is {0} bytes; flashed snapshot records {1}." -f $actualSize, $expectedSize)
        }
    }

    $dirty = @(& git status --short)
    if ($dirty.Count -eq 0) {
        Write-Result "OK" "Working tree is clean."
    }
    else {
        Write-Result "WARN" ("Working tree has {0} changed path(s):" -f $dirty.Count)
        $dirty | ForEach-Object { Write-Output ("  {0}" -f $_) }
    }

    if ($script:failureCount -gt 0) {
        Write-Output ("RESULT: FAILED ({0} error(s))" -f $script:failureCount)
        exit 1
    }

    Write-Output "RESULT: OK"
    exit 0
}
finally {
    Pop-Location
}
