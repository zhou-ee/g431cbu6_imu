# flash.ps1 - Flash build\Debug\IMU_Module.elf to STM32G431 via DAPLink
param(
    [string]$Elf = "build\Debug\IMU_Module.elf"
)
$ErrorActionPreference = "Continue"
$ocd = "E:\Software\Develop\Embeded\OpenOCD\xpack-openocd-0.12.0-7\bin\openocd.exe"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$elfPath = Join-Path $root $Elf

if (-not (Test-Path $elfPath)) {
    Write-Host "[ERROR] ELF not found: $elfPath" -ForegroundColor Red
    exit 1
}
Write-Host "Flashing: $elfPath" -ForegroundColor Cyan

$tcl = Join-Path $env:TEMP "flash.cfg"
$elfPosix = $elfPath -replace '\\','/'
@(
    "adapter speed 1000",
    "transport select swd",
    "init",
    "reset halt",
    "flash write_image erase `"$elfPosix`"",
    "verify_image `"$elfPosix`"",
    "reset run",
    "exit"
) | Set-Content -Path $tcl -Encoding ascii

$out = & $ocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg -f $tcl 2>&1
$out | Select-String -Pattern "wrote|verified|Error|error" | Select-Object -Last 10

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] flashing failed" -ForegroundColor Red
    $out | Select-Object -Last 15
    exit 1
}
Write-Host "Flash OK" -ForegroundColor Green
