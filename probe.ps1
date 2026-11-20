# probe.ps1 - Read all IMU diagnostic counters (regression deep-dive)
param(
    [int]$SampleMs = 3000,
    [switch]$NoReset
)
$ErrorActionPreference = "Continue"
$ocd = "E:\Software\Develop\Embeded\OpenOCD\xpack-openocd-0.12.0-7\bin\openocd.exe"
$nm  = "E:\Software\Develop\Embeded\Arm-GCC\bin\arm-none-eabi-nm.exe"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$elfPath = Join-Path $root "build\Debug\IMU_Module.elf"

$names = @(
    "imu_service_init_result",
    "imu_service_read_ok","imu_service_cap_rounds",
    "imu_service_err_data","imu_service_err_backpressure","imu_service_err_overrun",
    "imu_service_vqf_ready","imu_service_algo_rounds","imu_service_vqf_consumed",
    "imu_service_vqf_err","imu_service_vqf_warn","imu_service_vqf_status",
    "imu_service_heap_free_min","imu_service_cap_us_avg","imu_service_vqf_us_avg",
    "imu_service_loop_us_avg"
)
$symOut = & $nm $elfPath 2>&1
$VARS = [ordered]@{}
foreach ($n in $names) {
    $hit = $symOut | Where-Object { $_ -match "\s$n$" } | Select-Object -First 1
    if ($hit -match '^([0-9a-fA-F]{8})\s+\S\s') {
        $VARS[$n -replace '^imu_service_',''] = [System.Convert]::ToUInt32($Matches[1], 16)
    }
}

function Get-Metrics {
    param([bool]$DoReset)
    $ocdArgs = @("-f","interface/cmsis-dap.cfg","-f","target/stm32g4x.cfg",
                 "-c","adapter speed 100","-c","init")
    if ($DoReset) { $ocdArgs += @("-c","reset halt") } else { $ocdArgs += @("-c","halt") }
    foreach ($k in $VARS.Keys) {
        $ocdArgs += @("-c", ("read_memory 0x{0:X8} 32 1" -f $VARS[$k]))
    }
    $ocdArgs += @("-c","resume","-c","exit")
    $out = & $ocd @ocdArgs 2>&1
    $vals = [ordered]@{}; $idx = 0
    $keys = @($VARS.Keys)
    foreach ($line in $out) {
        $s = "$line".Trim()
        if ($s -match '^0x([0-9a-fA-F]+)$' -and $idx -lt $keys.Count) {
            $vals[$keys[$idx]] = [System.Convert]::ToUInt32($Matches[1], 16); $idx++
        }
    }
    return $vals
}

if (-not $NoReset) { $null = Get-Metrics -DoReset $true; Start-Sleep -Milliseconds 800 }
$m1 = Get-Metrics -DoReset $false
Start-Sleep -Milliseconds $SampleMs
$m2 = Get-Metrics -DoReset $false

Write-Host ""
Write-Host ("{0,-24} {1,12} {2,12} {3,12}" -f "metric","T0","T1","delta") -ForegroundColor Yellow
Write-Host ("-" * 64)
foreach ($k in $VARS.Keys) {
    $a = $m1[$k]; $b = $m2[$k]
    $d = if ($b -ge $a) { $b - $a } else { "WRAP" }
    Write-Host ("{0,-24} {1,12} {2,12} {3,12}" -f $k, $a, $b, $d)
}

Write-Host ""
$rounds = $m2["cap_rounds"] - $m1["cap_rounds"]
$consumed = $m2["vqf_consumed"] - $m1["vqf_consumed"]
$algoR = $m2["algo_rounds"] - $m1["algo_rounds"]
$bp = $m2["err_backpressure"] - $m1["err_backpressure"]
Write-Host ("采集轮次={0}  算法轮次={1}  已消费样本={2}  背压丢弃={3}" -f $rounds,$algoR,$consumed,$bp)
if ($rounds -gt 0 -and $algoR -gt 0) {
    Write-Host ("每采集轮 -> 算法轮: {0:N2}  每算法轮消费: {1:N2}" -f ($algoR/$rounds), ($consumed/[Math]::Max($algoR,1)))
    Write-Host ("每采集轮丢弃: {0:N2}" -f ($bp/$rounds))
}
