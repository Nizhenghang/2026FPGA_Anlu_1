<#
.SYNOPSIS
    Sync the source file list of a TD project (.al) into the .prj snapshots that the
    headless flow actually reads.

.DESCRIPTION
    Why this is needed:
      TD's GUI "Run" generates <RunDir>/<ProjectName>.prj as a snapshot of the .al
      project, and scripts/DefaultFlow.tcl does `open_project <name>.prj`.
      If a source file is added in the GUI but no full flow has been re-run since,
      the .prj snapshot stays stale and the new module becomes a black box during
      elaborate, failing with:  HDL-8007 ERROR: <module> is a black box

    What this script does:
      Treats the .al as the single source of truth and inserts any <File> block that
      is missing from each run directory's .prj, rewriting the path from the .al
      directory's relative form into that run directory's relative form.

    Why plain text processing:
      .al / .prj contain bare '&' in attribute names such as "UsedInP&R", which is not
      well-formed XML. Using an XML parser would either fail or reformat the file and
      break TD. So everything here is line based.

.PARAMETER Apply
    Write the changes back. Without it the script only reports what it would insert.

.EXAMPLE
    powershell -NoProfile -File tools\sync_prj_from_al.ps1
    powershell -NoProfile -File tools\sync_prj_from_al.ps1 -Apply
#>
[CmdletBinding()]
param(
    [string]$TdProjectDir = 'd:\Nizhenghang\Project\2026Anlu1\src\td_project',
    [switch]$Apply
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------- helpers

function Read-Lines {
    param([string]$Path)
    # Explicit UTF8: PS 5.1 falls back to ANSI for files without a BOM.
    return @(Get-Content -LiteralPath $Path -Encoding UTF8)
}

function Write-LinesNoBom {
    param([string]$Path, [string[]]$Lines, [string]$NewLine)
    $text = ($Lines -join $NewLine) + $NewLine
    $enc = New-Object System.Text.UTF8Encoding($false)   # false = no BOM
    [System.IO.File]::WriteAllText($Path, $text, $enc)
}

function Detect-NewLine {
    param([string]$Path)
    $raw = [System.IO.File]::ReadAllText($Path)
    if ($raw -match "`r`n") { return "`r`n" }
    return "`n"
}

# Relative path from $FromDir to $ToAbs, using forward slashes.
# (Windows PowerShell 5.1 runs on .NET Framework, which has no Path.GetRelativePath.)
function Get-RelPath {
    param([string]$FromDir, [string]$ToAbs)
    $from = [System.IO.Path]::GetFullPath($FromDir).TrimEnd('\')
    $to   = [System.IO.Path]::GetFullPath($ToAbs)
    $fp = $from -split '\\'
    $tp = $to   -split '\\'
    $i = 0
    while ($i -lt $fp.Count -and $i -lt $tp.Count -and $fp[$i] -ceq $tp[$i]) { $i++ }
    $parts = New-Object System.Collections.Generic.List[string]
    for ($k = $i; $k -lt $fp.Count; $k++) { $parts.Add('..') }
    for ($k = $i; $k -lt $tp.Count; $k++) { $parts.Add($tp[$k]) }
    if ($parts.Count -eq 0) { return '.' }
    return [string]::Join('/', $parts.ToArray())
}

# Parse <File Path="..."> blocks together with the enclosing section name.
# Sections are the 8-space indented tags: Verilog / ADC_FILE / SDC_FILE / CWC_FILE / IP_FILE ...
function Get-FileBlocks {
    param([string[]]$Content, [string]$BaseDir)
    $blocks = New-Object System.Collections.Generic.List[object]
    $section = ''
    $i = 0
    while ($i -lt $Content.Count) {
        $line = $Content[$i]

        if ($line -match '^\s{8}<([A-Za-z_0-9]+)>\s*$') { $section = $Matches[1] }

        # self-closing form
        if ($line -match '<File\s+Path="([^"]+)"\s*/>\s*$') {
            $rel = $Matches[1]
            $blocks.Add([pscustomobject]@{
                Section = $section
                Rel     = $rel
                Abs     = [System.IO.Path]::GetFullPath((Join-Path $BaseDir $rel))
                Lines   = @($line)
            })
            $i++
            continue
        }

        # normal form, block ends at the first </File>
        if ($line -match '<File\s+Path="([^"]+)"\s*>\s*$') {
            $rel = $Matches[1]
            $start = $i
            $j = $i + 1
            while ($j -lt $Content.Count -and $Content[$j] -notmatch '</File>\s*$') { $j++ }
            if ($j -ge $Content.Count) { throw "unclosed <File> block at line $($start + 1)" }
            $blocks.Add([pscustomobject]@{
                Section = $section
                Rel     = $rel
                Abs     = [System.IO.Path]::GetFullPath((Join-Path $BaseDir $rel))
                Lines   = @($Content[$start..$j])
            })
            $i = $j + 1
            continue
        }

        $i++
    }
    return $blocks
}

# ---------------------------------------------------------------- locate project

$alFile = Get-ChildItem -Path $TdProjectDir -Filter '*.al' -File | Select-Object -First 1
if (-not $alFile) { throw "no .al project file found under $TdProjectDir" }
$alDir   = $alFile.DirectoryName
$prjName = $alFile.BaseName
Write-Output "project file : $($alFile.FullName)"

$runsDir = Get-ChildItem -Path $TdProjectDir -Directory -Filter '*_Runs' | Select-Object -First 1
if (-not $runsDir) { throw "no *_Runs directory found under $TdProjectDir" }
Write-Output "runs dir     : $($runsDir.FullName)"

$alContent = Read-Lines $alFile.FullName
$alBlocks  = Get-FileBlocks -Content $alContent -BaseDir $alDir
Write-Output "file blocks in .al : $($alBlocks.Count)"

# ---------------------------------------------------------------- sync each run

$runDirs = Get-ChildItem -Path $runsDir.FullName -Directory | Sort-Object Name
foreach ($rd in $runDirs) {
    $prjFile = Join-Path $rd.FullName "$prjName.prj"
    if (-not (Test-Path -LiteralPath $prjFile)) {
        Write-Output ""
        Write-Output "[$($rd.Name)] no .prj, skipped"
        continue
    }

    Write-Output ""
    Write-Output "==================== [$($rd.Name)] ===================="
    $prjContent = Read-Lines $prjFile
    $prjBlocks  = Get-FileBlocks -Content $prjContent -BaseDir $rd.FullName

    $prjAbs = @{}
    foreach ($b in $prjBlocks) { $prjAbs[$b.Abs.ToLowerInvariant()] = $true }

    $missing = @($alBlocks | Where-Object { -not $prjAbs.ContainsKey($_.Abs.ToLowerInvariant()) })
    if ($missing.Count -eq 0) {
        Write-Output "in sync, nothing missing."
        continue
    }

    Write-Output "missing $($missing.Count) entry(ies):"
    foreach ($m in $missing) { Write-Output "  [$($m.Section)] $($m.Rel)" }

    $newContent = New-Object System.Collections.Generic.List[string]
    $newContent.AddRange([string[]]$prjContent)

    foreach ($grp in ($missing | Group-Object Section)) {
        $sec = $grp.Name

        $secStart = -1
        for ($k = 0; $k -lt $newContent.Count; $k++) {
            if ($newContent[$k] -match '^\s{8}<([A-Za-z_0-9]+)>\s*$' -and $Matches[1] -eq $sec) { $secStart = $k; break }
        }
        if ($secStart -lt 0) {
            Write-Warning "[$($rd.Name)] section <$sec> not present in .prj; $($grp.Count) entry(ies) need manual handling"
            continue
        }

        $secEnd = -1
        for ($k = $secStart + 1; $k -lt $newContent.Count; $k++) {
            if ($newContent[$k] -match ('^\s{8}</' + $sec + '>\s*$')) { $secEnd = $k; break }
        }
        if ($secEnd -lt 0) {
            Write-Warning "[$($rd.Name)] section <$sec> is not closed; skipped"
            continue
        }

        $insert = New-Object System.Collections.Generic.List[string]
        foreach ($m in $grp.Group) {
            $newRel = Get-RelPath -FromDir $rd.FullName -ToAbs $m.Abs
            foreach ($ln in $m.Lines) {
                if ($ln -match '<File\s+Path="') {
                    $insert.Add(($ln -replace 'Path="[^"]*"', ('Path="' + $newRel + '"')))
                } else {
                    $insert.Add($ln)
                }
            }
        }

        $newContent.InsertRange($secEnd, $insert)

        Write-Output "  -> inserting $($grp.Count) block(s) into <$sec>:"
        foreach ($ln in $insert) { if ($ln -match '<File\s+Path="') { Write-Output "     $($ln.Trim())" } }
    }

    if ($Apply) {
        $bak = "$prjFile.pre_sync.bak"
        if (-not (Test-Path -LiteralPath $bak)) { Copy-Item -LiteralPath $prjFile -Destination $bak -Force }
        $nl = Detect-NewLine $prjFile
        Write-LinesNoBom -Path $prjFile -Lines $newContent.ToArray() -NewLine $nl
        Write-Output "  written: $prjFile  (original kept as .pre_sync.bak)"
    } else {
        Write-Output "  [dry-run] nothing written. Add -Apply to commit."
    }
}
