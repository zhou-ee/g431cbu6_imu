# measure.ps1 - Read IMU runtime metrics from STM32G431 (symbols auto-resolved from ELF)
param(
    [int]$SampleMs = 5000,
    [switch]$NoReset,
    [int]$StartupWaitMs = 800,
    [string]$Elf = "build\Debug\IMU_Module.elf"
)
$ErrorActionPreference = "Continue"
$ocd  = "E:\Software\Develop\Embeded\OpenOCD\xpack-openocd-0.12.0-7\bin\openocd.exe"
$nm   = "E:\Software\Develop\Embeded\Arm-GCC\bin\arm-none-eabi-nm.exe"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$elfPath = Join-Path $root $Elf

if (-not (Test-Path $elfPath)) { Write-Host "[ERROR] ELF not found: $elfPath" -ForegroundColor Red; exit 1 }

# Resolve symbols from ELF so the script survives rebuilds
$names = @(
    "imu_service_init_result","imu_service_read_ok","imu_service_cap_rounds",
    "imu_service_cap_us_max","imu_service_cap_us_avg","imu_service_err_bus",
    "imu_service_err_data","imu_service_err_ovr","imu_service_err_backpressure",
    "imu_service_err_overrun","imu_service_err_timeout"
)
$symOut = & $nm $elfPath 2>&1
$VARS = [ordered]@{}
foreach ($n in $names) {
    $hit = $symOut | Where-Object { $_ -match "\s$n$" } | Select-Object -First 1
    if ($hit -match '^([0-9a-fA-F]{8})\s+\S\s') {
        $VARS[$n -replace '^imu_service_',''] = [System.Convert]::ToUInt32($Matches[1], 16)
    }
}
if ($VARS.Count -ne $names.Count) {
    Write-Host "[ERROR] symbol resolve failed: $($VARS.Count)/$($names.Count)" -ForegroundColor Red
    exit 1
}

function Get-Metrics {
    param([bool]$DoReset)
    $ocdArgs = @("-f","interface/cmsis-dap.cfg","-f","target/stm32g4x.cfg",
                 "-c","adapter speed 1000","-c","init")
    if ($DoReset) { $ocdArgs += @("-c","reset halt") } else { $ocdArgs += @("-c","halt") }
    $keys = @($VARS.Keys)
    foreach ($k in $keys) {
        $ocdArgs += @("-c", ("read_memory 0x{0:X8} 32 1" -f $VARS[$k]))
    }
    $ocdArgs += @("-c","resume","-c","exit")
    $out = & $ocd @ocdArgs 2>&1
    $vals = [ordered]@{}; $idx = 0
    foreach ($line in $out) {
        $s = "$line".Trim()
        if ($s -match '^0x([0-9a-fA-F]+)$' -and $idx -lt $keys.Count) {
            $vals[$keys[$idx]] = [System.Convert]::ToUInt32($Matches[1], 16); $idx++
        }
    }
    return $vals
}

Write-Host "Sample interval: $SampleMs ms | symbols from $(Split-Path -Leaf $Elf)" -ForegroundColor Cyan
if ($NoReset) {
    Write-Host "No reset (keeps running)." -ForegroundColor Cyan
} else {
    Write-Host "Resetting target for clean baseline..." -ForegroundColor Cyan
    $null = Get-Metrics -DoReset $true
    Start-Sleep -Milliseconds $StartupWaitMs
}

$m1 = Get-Metrics -DoReset $false
if ($m1.Count -lt $VARS.Count) { Write-Host "[ERROR] parse failed" -ForegroundColor Red; exit 1 }

Start-Sleep -Milliseconds $SampleMs
$m2 = Get-Metrics -DoReset $false

Write-Host ""
Write-Host ("{0,-18} {1,10} {2,10} {3,10}" -f "metric","T0","T1","delta") -ForegroundColor Yellow
Write-Host ("-" * 52)
foreach ($k in $VARS.Keys) {
    $a = $m1[$k]; $b = $m2[$k]
    $d = if ($b -ge $a) { $b - $a } else { "WRAP" }
    Write-Host ("{0,-18} {1,10} {2,10} {3,10}" -f $k, $a, $b, $d)
}

Write-Host ""
if ($m2["init_result"] -ne 0) { Write-Host "[FAIL] init_result = $($m2['init_result'])" -ForegroundColor Red }
else { Write-Host "init: OK" -ForegroundColor Green }

$rounds = $m2["cap_rounds"] - $m1["cap_rounds"]
$ok     = $m2["read_ok"]     - $m1["read_ok"]
$to     = $m2["err_timeout"] - $m1["err_timeout"]
$bus    = $m2["err_bus"]     - $m1["err_bus"]
$d      = $m2["err_data"]    - $m1["err_data"]
$ovr    = $m2["err_overrun"] - $m1["err_overrun"]

if ($rounds -gt 0) {
    $rate = $ok / $rounds
    Write-Host ("rounds={0}  read_ok={1}  success={2:P2}" -f $rounds,$ok,$rate)
    Write-Host ("timeout={0}  err_bus={1}  err_data={2}  err_overrun={3}" -f $to,$bus,$d,$ovr)
    if ($to -eq 0 -and $bus -eq 0) {
        Write-Host "RESULT: PASS (no timeout/bus errors)" -ForegroundColor Green
    } else {
        Write-Host "RESULT: CHECK (timeout or bus errors present)" -ForegroundColor Yellow
    }
} else {
    Write-Host "RESULT: no rounds counted - is firmware running?" -ForegroundColor Red
}
Write-Host ("cap_us_max={0} us  cap_us_avg={1} us" -f $m2["cap_us_max"], $m2["cap_us_avg"])
