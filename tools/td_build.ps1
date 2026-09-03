<#
.SYNOPSIS
    Headless Anlogic TD build driver for this project.

.DESCRIPTION
    Runs the standard TD flow (scripts/DefaultFlow.tcl) against the run directories
    that TD's GUI created, without opening the GUI.

    Three things this script gets right that are easy to trip over:
      1. The .tcl path must use FORWARD slashes. Tcl treats backslashes as escape
         characters, so "D:\Download\TD\scripts\DefaultFlow.tcl" arrives as
         "D:DownloadTDscriptsDefaultFlow.tcl" and fails with "couldn't read file".
      2. DefaultFlow.tcl does `source ./settings.cfg`, i.e. it reads its config from
         the CURRENT WORKING DIRECTORY. So we must cd into the run directory; passing
         the script by absolute path from elsewhere does not work.
      3. DefaultFlow.tcl has no `exit`, so td_commands_prompt.exe never terminates on
         its own -- it falls back to an interactive prompt and blocks on stdin. We
         therefore run it through tools/td_flow_exit.tcl, which sources the flow and
         exits explicitly. Without this the syn stage hangs forever and the phy stage
         is silently never started. A watchdog timeout is layered on top so that even
         an unexpected hang kills the process instead of wedging the pipeline.

    It also avoids run.bat, whose trailing `pause` would block forever in a
    non-interactive shell (and which is in fact never reached, see point 3).

.PARAMETER Stage
    syn  : read_design -> opt_gate        (run dir syn_1)
    phy  : opt_place  -> bitgen           (run dir phy_1, imports ../syn_1 gate db)
    all  : syn then phy

.PARAMETER SkipPrjSync
    Do not refresh the .prj snapshots from the .al before building.
    By default we sync, because a stale .prj silently drops newly added source files
    and turns them into black boxes (HDL-8007).

.EXAMPLE
    powershell -NoProfile -File tools\td_build.ps1 -Stage all
    powershell -NoProfile -File tools\td_build.ps1 -Stage phy
#>
[CmdletBinding()]
param(
    [ValidateSet('syn', 'phy', 'all')]
    [string]$Stage = 'all',
    [string]$TdProjectDir = 'd:\Nizhenghang\Project\2026Anlu1\src\td_project',
    [string]$TdBin        = 'D:\Download\TD\bin\td_commands_prompt.exe',
    [string]$FlowScript   = 'D:/Download/TD/scripts/DefaultFlow.tcl',   # forward slashes on purpose
    [string]$LogDir       = 'd:\Nizhenghang\Project\2026Anlu1\_build_logs',
    [int]$TimeoutMinutes  = 20,
    [switch]$SkipPrjSync
)

$ErrorActionPreference = 'Continue'

if (-not (Test-Path -LiteralPath $TdBin)) { throw "TD executable not found: $TdBin" }

$repoRoot  = 'd:\Nizhenghang\Project\2026Anlu1'
$runsDir   = Join-Path $TdProjectDir ((Get-ChildItem -Path $TdProjectDir -Directory -Filter '*_Runs' | Select-Object -First 1).Name)
$prjName   = (Get-ChildItem -Path $TdProjectDir -Filter '*.al' -File | Select-Object -First 1).BaseName

# The wrapper is what makes td_commands_prompt.exe actually exit. Forward slashes
# because the path is consumed by Tcl, not by the Win32 loader.
$flowWrapper = (Join-Path $repoRoot 'tools\td_flow_exit.tcl').Replace('\', '/')
if (-not (Test-Path -LiteralPath $flowWrapper)) { throw "flow wrapper not found: $flowWrapper" }
$env:TD_FLOW_SCRIPT = $FlowScript

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'

# ---------------------------------------------------------------- pre-flight
if (-not $SkipPrjSync) {
    Write-Output "---- syncing .prj snapshots from .al ----"
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $repoRoot 'tools\sync_prj_from_al.ps1') -Apply
    if ($LASTEXITCODE -ne 0) { throw "sync_prj_from_al.ps1 failed" }
}

function Invoke-RunStage {
    param([string]$RunDirName, [string]$Tag, [string]$MustRefresh = '', [string[]]$ClearBefore = @())

    # Status goes out through this script-scoped variable, NOT the return value.
    # `$ok = Invoke-RunStage ...` does not do what it looks like it does: every
    # Write-Output below lands on the same success stream as the return value, so
    # the caller would collect the diagnostic strings together with the boolean
    # into an Object[], and `if ($ok)` on a multi-element array is TRUE no matter
    # what the last element says. That is not hypothetical -- it is how this
    # script came to print "BUILD OK (all)" over a syn stage that had died with
    # USR-8003 and left a stale gate.db for phy to consume. Callers must invoke
    # this bare and then read $script:StageOk.
    $script:StageOk = $true

    $runDir = Join-Path $runsDir $RunDirName
    if (-not (Test-Path -LiteralPath (Join-Path $runDir 'settings.cfg'))) {
        throw "settings.cfg not found in $runDir"
    }

    # A leftover .stop.f makes DefaultFlow.tcl halt immediately.
    $stopFile = Join-Path $runDir '.stop.f'
    if (Test-Path -LiteralPath $stopFile) {
        Write-Warning "removing stale .stop.f in $runDir"
        Remove-Item -LiteralPath $stopFile -Force
    }
    # Clear stale error markers so we do not misread a previous failure as this one's.
    Get-ChildItem -Path $runDir -Filter '.*.error.f' -Force -File -ErrorAction SilentlyContinue | Remove-Item -Force

    # Start from no prior output of this stage's own. TD's `export_db` intermittently
    # dies with USR-8003 "Cannot write file" when the target .db already exists from
    # an earlier run. It is not a plain overwrite prohibition -- the 10:15 run rewrote
    # _elaborate.db happily and then failed on _gate.db, while 10:38 and 10:41 both
    # failed on _elaborate.db itself. Whatever holds the handle, removing the target
    # first has been reliable: the 10:43 run created all three syn databases fresh
    # with no error. Clearing them also closes the worse hazard, which is not
    # intermittent at all: a stage that fails after an earlier one succeeded leaves a
    # valid-looking OLD artifact behind, the next stage consumes it, and the flow
    # reports a clean build of the old design.
    foreach ($stale in $ClearBefore) {
        $p = Join-Path $runDir $stale
        if (Test-Path -LiteralPath $p) {
            Remove-Item -LiteralPath $p -Force
            Write-Output "cleared        : $RunDirName\$stale"
        }
    }

    $log    = Join-Path $LogDir ("{0}_{1}.log"     -f $Tag, $stamp)
    $logErr = Join-Path $LogDir ("{0}_{1}.stderr"  -f $Tag, $stamp)
    Write-Output ""
    Write-Output "---- [$RunDirName] building (log: $log) ----"

    # Start-Process rather than a pipeline: it gives us WaitForExit with a deadline,
    # so a hung TD cannot wedge the driver the way an open-ended pipe did.
    $runStart = Get-Date
    $proc = Start-Process -FilePath $TdBin `
                          -ArgumentList $flowWrapper `
                          -WorkingDirectory $runDir `
                          -RedirectStandardOutput $log `
                          -RedirectStandardError $logErr `
                          -NoNewWindow -PassThru

    if (-not $proc.WaitForExit($TimeoutMinutes * 60 * 1000)) {
        Write-Warning "[$RunDirName] exceeded $TimeoutMinutes min watchdog - killing PID $($proc.Id)"
        try { $proc.Kill() } catch { }
        Write-Output "exit code      : TIMEOUT"
        $script:StageOk = $false
        return
    }
    # Start-Process -PassThru -NoNewWindow hands back an object whose ExitCode is
    # $null, and `$null -ne 0` is TRUE in PowerShell -- so an unqualified
    # `$rc -ne 0` fails every stage including the ones that worked. Refresh to try
    # for a real code, and treat "no code available" as not-a-failure: the error
    # markers, the ERROR line count and the stale-artifact check below are the
    # authority on whether the stage actually did its job.
    $rc = $null
    try { $proc.Refresh(); $rc = $proc.ExitCode } catch { }

    $errMarkers = @(Get-ChildItem -Path $runDir -Filter '.*.error.f' -Force -File -ErrorAction SilentlyContinue)
    $logErrors  = @(Select-String -LiteralPath $log -Pattern 'ERROR' -SimpleMatch -ErrorAction SilentlyContinue)
    $stderrText = if (Test-Path -LiteralPath $logErr) { (Get-Item -LiteralPath $logErr).Length } else { 0 }

    Write-Output "exit code      : $(if ($null -eq $rc) { '(unavailable)' } else { $rc })"
    Write-Output "error markers  : $($errMarkers.Count) $(if ($errMarkers.Count) { '(' + (($errMarkers | ForEach-Object { $_.Name }) -join ', ') + ')' })"
    Write-Output "ERROR lines    : $($logErrors.Count)"
    Write-Output "stderr bytes   : $stderrText"
    if ($logErrors.Count) {
        $logErrors | Select-Object -First 15 | ForEach-Object { Write-Output ("    L{0}: {1}" -f $_.LineNumber, $_.Line.Trim()) }
    }

    if (($null -ne $rc -and $rc -ne 0) -or $errMarkers.Count -gt 0 -or $logErrors.Count -gt 0) {
        $script:StageOk = $false
    }

    # Independent of TD's own opinion: the artifact this stage exists to produce
    # must have been rewritten by THIS run. TD can fail at the last step and still
    # leave a perfectly good-looking older file behind -- the 10:15 build died at
    # `export_db _gate.db` with USR-8003 (file locked), and phy then happily
    # placed and routed the previous build's gate.db. The bitstream came out with
    # a config body byte identical to the one it was meant to replace, and every
    # downstream number (route.qor, final_timing.rpt, #RAMs) looked fine because
    # it was a fine build -- of the old design.
    if ($MustRefresh) {
        $art = Join-Path $runDir $MustRefresh
        if (-not (Test-Path -LiteralPath $art)) {
            Write-Output "stale check    : MISSING $MustRefresh"
            $script:StageOk = $false
        } else {
            $artTime = (Get-Item -LiteralPath $art).LastWriteTime
            $fresh   = $artTime -ge $runStart
            Write-Output ("stale check    : {0}  written {1}  run started {2}  -> {3}" -f `
                $MustRefresh, $artTime.ToString('HH:mm:ss'), $runStart.ToString('HH:mm:ss'), `
                $(if ($fresh) { 'FRESH' } else { 'STALE, not rewritten by this run' }))
            if (-not $fresh) { $script:StageOk = $false }
        }
    }
}

$ok = $true

if ($Stage -eq 'syn' -or $Stage -eq 'all') {
    Invoke-RunStage -RunDirName 'syn_1' -Tag 'syn' `
                    -MustRefresh "$prjName`_gate.db" `
                    -ClearBefore @("$prjName`_elaborate.db", "$prjName`_rtl.db", "$prjName`_gate.db")
    $ok = $script:StageOk
    if ($ok) {
        $ts = Join-Path $runsDir "syn_1\$prjName`_gate.ts"
        if (Test-Path -LiteralPath $ts) { Write-Output "gate.ts        : $((Get-Content -LiteralPath $ts) -join ' | ')" }
        $qor = Join-Path $runsDir 'syn_1\gate.qor'
        if (Test-Path -LiteralPath $qor) {
            Select-String -LiteralPath $qor -Pattern 'Setup WNS|Setup TNS|violated S-EP|#slices|#RAMs|#DSPs|Over-quota|sd_card_clk|ext_mem_clk|video_clk' |
                Select-Object -First 20 | ForEach-Object { Write-Output ("    " + $_.Line.Trim()) }
        }
    } else {
        Write-Warning "synthesis stage FAILED - not continuing to phy"
    }
}

if ($ok -and ($Stage -eq 'phy' -or $Stage -eq 'all')) {
    Invoke-RunStage -RunDirName 'phy_1' -Tag 'phy' `
                    -MustRefresh "$prjName.bit" `
                    -ClearBefore @("$prjName.bit", "$prjName`_place.db", "$prjName`_pr.db", "$prjName`_eco_pr.db")
    $ok = $script:StageOk
    # Only summarise on success. These files are read back off disk, so after a
    # failure they still hold the PREVIOUS build's numbers and printing them makes
    # a dead run look like a clean one -- which is exactly what the 10:15 log did.
    if ($ok) {
        $ts = Join-Path $runsDir "phy_1\$prjName`_phy.ts"
        if (Test-Path -LiteralPath $ts) { Write-Output "phy.ts         : $((Get-Content -LiteralPath $ts) -join ' | ')" }
        $qor = Join-Path $runsDir 'phy_1\route.qor'
        if (Test-Path -LiteralPath $qor) {
            Write-Output "route.qor      :"
            Get-Content -LiteralPath $qor | ForEach-Object { Write-Output ("    " + $_.TrimEnd()) }
        }
        $bit = Join-Path $runsDir "phy_1\$prjName.bit"
        if (Test-Path -LiteralPath $bit) {
            $fi = Get-Item -LiteralPath $bit
            Write-Output ("bitstream      : {0}  {1} bytes  {2}" -f $fi.Name, $fi.Length, $fi.LastWriteTime)
        }
    } else {
        Write-Warning "phy stage FAILED - no bitstream was generated by this run"
    }
}

Write-Output ""
if ($ok) { Write-Output "==== BUILD OK ($Stage) ====" } else { Write-Output "==== BUILD FAILED ($Stage) ====" }
exit $(if ($ok) { 0 } else { 1 })
