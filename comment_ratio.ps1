# comment_ratio.ps1 - Check comment-line ratio: cpp <= 10%, headers <= 30%
param(
    [switch]$Fail
)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$cppLimit = 10.0
$hLimit   = 30.0

function Get-CommentStats {
    param([string]$Path)

    $lines = Get-Content $Path -ErrorAction SilentlyContinue
    if ($null -eq $lines -or $lines.Count -eq 0) { return $null }

    $total   = 0
    $comment = 0
    $inBlock = $false

    foreach ($line in $lines) {
        $t = $line.Trim()
        if ($t -eq '') { continue }
        $total++

        if ($inBlock) {
            $comment++
            if ($t -match '\*/') { $inBlock = $false }
            continue
        }
        if ($t -match '^/\*') {
            $comment++
            if ($t -notmatch '\*/') { $inBlock = $true }
            continue
        }
        if ($t -match '^//') { $comment++; continue }
    }

    if ($total -eq 0) { return $null }
    [PSCustomObject]@{
        File    = $Path.Replace($root + '\', '')
        Lines   = $total
        Comment = $comment
        Pct     = [math]::Round($comment * 100.0 / $total, 1)
    }
}

$files = Get-ChildItem -Path (Join-Path $root 'PYRo'), (Join-Path $root 'Module') `
         -Recurse -Include *.cpp, *.h, *.hpp -ErrorAction SilentlyContinue |
         Where-Object { $_.FullName -notmatch '\\build\\' }

$rows = @()
foreach ($f in $files) {
    $s = Get-CommentStats -Path $f.FullName
    if ($null -eq $s) { continue }
    $limit = if ($f.Extension -eq '.cpp') { $cppLimit } else { $hLimit }
    $s | Add-Member -NotePropertyName Limit -NotePropertyValue $limit
    $s | Add-Member -NotePropertyName Over  -NotePropertyValue ($s.Pct -gt $limit)
    $rows += $s
}

$cpp = $rows | Where-Object { $_.File -like '*.cpp' }
$hdr = $rows | Where-Object { $_.File -notlike '*.cpp' }

Write-Host "=== .cpp (limit $cppLimit%) ===" -ForegroundColor Cyan
$cpp | Sort-Object Pct -Descending | ForEach-Object {
    $tag = if ($_.Over) { ' OVER' } else { '' }
    $color = if ($_.Over) { 'Red' } else { 'Gray' }
    Write-Host ("{0,6}% {1,5}/{2,-5} {3}{4}" -f $_.Pct, $_.Comment, $_.Lines, $_.File, $tag) -ForegroundColor $color
}

Write-Host ""
Write-Host "=== headers (limit $hLimit%) ===" -ForegroundColor Cyan
$hdr | Sort-Object Pct -Descending | ForEach-Object {
    $tag = if ($_.Over) { ' OVER' } else { '' }
    $color = if ($_.Over) { 'Red' } else { 'Gray' }
    Write-Host ("{0,6}% {1,5}/{2,-5} {3}{4}" -f $_.Pct, $_.Comment, $_.Lines, $_.File, $tag) -ForegroundColor $color
}

$overCpp = @($cpp | Where-Object Over).Count
$overH   = @($hdr | Where-Object Over).Count
Write-Host ""
Write-Host ("OVER: cpp {0} / header {1}" -f $overCpp, $overH)

if ($Fail -and ($overCpp + $overH) -gt 0) { exit 1 }
exit 0
