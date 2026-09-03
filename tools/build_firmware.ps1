param(
    [string]$Sdk = 'C:\TI\mspm0_sdk_2_10_00_04',
    [string]$Compiler = 'D:\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS\bin\tiarmclang.exe',
    [string]$Sysconfig = 'C:\TI\sysconfig_1.26.2\sysconfig_cli.bat'
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$proj = Join-Path $repo 'mspm0'
$output = Join-Path $proj 'Debug'
New-Item -ItemType Directory -Force -Path $output | Out-Null
& $Sysconfig -s "$Sdk/.metadata/product.json" --script "$proj/empty.syscfg" -o $output --compiler ticlang
if ($LASTEXITCODE) { throw 'SysConfig failed' }
$flags = @('-march=thumbv6m','-mcpu=cortex-m0plus','-mfloat-abi=soft',
           '-mlittle-endian','-mthumb','-O0','-gdwarf-3',
           "-I$proj","-I$proj/Hardware","-I$proj/Control","-I$output",
           "-I$Sdk/source","-I$Sdk/source/third_party/CMSIS/Core/Include")
$defines = Get-Content -LiteralPath "$output/device.opt"
$sources = @("$proj/empty.c", "$output/ti_msp_dl_config.c",
             "$Sdk/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c")
$sources += (Get-ChildItem "$proj/Hardware","$proj/Control" -Filter '*.c').FullName
$objects = @()
foreach ($source in $sources) {
    $object = Join-Path $output (([IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
    & $Compiler @flags @defines -c $source -o $object
    if ($LASTEXITCODE) { throw "Compilation failed: $source" }
    $objects += $object
}
$compilerLib = Join-Path (Split-Path (Split-Path $Compiler -Parent) -Parent) 'lib'
$outFile = Join-Path $output 'empty_LP_MSPM0G3507_nortos_ticlang.out'
$link = @("$proj/device_linker.cmd", "-Wl,-i$Sdk/source", "-Wl,-i$output",
          "-Wl,-i$compilerLib", '-Wl,--rom_model', '-Wl,-ldevice.cmd.genlibs',
          '-Wl,-llibc.a', "-Wl,-m$output/firmware.map", '-o', $outFile)
& $Compiler @flags @defines @objects @link
if ($LASTEXITCODE) { throw 'Link failed' }
Write-Output "Firmware built: $outFile"
