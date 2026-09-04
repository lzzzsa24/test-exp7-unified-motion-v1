param(
  [switch]$LineTrackingLiftTest
)

$ErrorActionPreference = 'Stop'

$projectRoot = $PSScriptRoot
$toolRoot = 'F:\mytool\STM32CubeIDE_1.16.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623\tools\bin'
$gcc = Join-Path $toolRoot 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $toolRoot 'arm-none-eabi-objcopy.exe'
$size = Join-Path $toolRoot 'arm-none-eabi-size.exe'
$buildDirName = if ($LineTrackingLiftTest) {
  'manual-build-line-reacquire-test'
} else {
  'manual-build-unified-motion'
}
$artifactName = if ($LineTrackingLiftTest) {
  'line_reacquire_lift_test'
} else {
  'exp7_unified_motion'
}
$buildDir = Join-Path $projectRoot $buildDirName

if (-not (Test-Path -LiteralPath $gcc)) {
  throw "STM32 compiler not found: $gcc"
}

New-Item -ItemType Directory -Path $buildDir -Force | Out-Null

$includeArgs = @(
  "-I$(Join-Path $projectRoot 'Core\Inc')",
  "-I$(Join-Path $projectRoot 'Drivers\STM32F1xx_HAL_Driver\Inc')",
  "-I$(Join-Path $projectRoot 'Drivers\STM32F1xx_HAL_Driver\Inc\Legacy')",
  "-I$(Join-Path $projectRoot 'Drivers\CMSIS\Device\ST\STM32F1xx\Include')",
  "-I$(Join-Path $projectRoot 'Drivers\CMSIS\Include')"
)

$commonArgs = @(
  '-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft',
  '-DDEBUG', '-DUSE_HAL_DRIVER', '-DSTM32F103xE',
  '-O0', '-g3', '-ffunction-sections', '-fdata-sections',
  '-Wall', '-fstack-usage', '-MMD', '-MP'
) + $includeArgs
if ($LineTrackingLiftTest) {
  $commonArgs += '-DLINE_TRACKING_LIFT_TEST'
}

$coreSources = Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Core\Src') -Filter '*.c' |
  Where-Object { $_.Name -ne 'motor.c' } |
  Sort-Object Name
$halNames = @(
  'stm32f1xx_hal.c',
  'stm32f1xx_hal_cortex.c',
  'stm32f1xx_hal_dma.c',
  'stm32f1xx_hal_exti.c',
  'stm32f1xx_hal_flash.c',
  'stm32f1xx_hal_flash_ex.c',
  'stm32f1xx_hal_gpio.c',
  'stm32f1xx_hal_gpio_ex.c',
  'stm32f1xx_hal_pwr.c',
  'stm32f1xx_hal_rcc.c',
  'stm32f1xx_hal_rcc_ex.c',
  'stm32f1xx_hal_tim.c',
  'stm32f1xx_hal_tim_ex.c'
)
$halSources = $halNames | ForEach-Object {
  Get-Item -LiteralPath (Join-Path $projectRoot "Drivers\STM32F1xx_HAL_Driver\Src\$_")
}
$sources = @($coreSources) + @($halSources)
$objects = @()

for ($index = 0; $index -lt $sources.Count; ++$index) {
  $source = $sources[$index].FullName
  $object = Join-Path $buildDir ('obj_{0:D3}.o' -f $index)
  & $gcc @commonArgs '-std=gnu11' '-c' $source '-o' $object
  if ($LASTEXITCODE -ne 0) { throw "Compile failed: $source" }
  $objects += $object
}

$startup = Join-Path $projectRoot 'Core\Startup\startup_stm32f103zetx.s'
$startupObject = Join-Path $buildDir 'startup_stm32f103zetx.o'
& $gcc @commonArgs '-x' 'assembler-with-cpp' '-c' $startup '-o' $startupObject
if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
$objects += $startupObject

$elf = Join-Path $buildDir ($artifactName + '.elf')
$map = Join-Path $buildDir ($artifactName + '.map')
$hex = Join-Path $buildDir ($artifactName + '.hex')
$bin = Join-Path $buildDir ($artifactName + '.bin')
$linker = Join-Path $projectRoot 'STM32F103ZETX_FLASH.ld'
$linkArgs = @(
  '-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft',
  "-T$linker", '--specs=nosys.specs', '--specs=nano.specs',
  "-Wl,-Map=$map", '-Wl,--gc-sections', '-static'
) + $objects + @('-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group', '-o', $elf)

& $gcc @linkArgs
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
& $objcopy '-O' 'ihex' $elf $hex
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
& $objcopy '-O' 'binary' $elf $bin
if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed' }

& $size $elf
Write-Output "HEX: $hex"
Write-Output "SHA256: $((Get-FileHash -Algorithm SHA256 -LiteralPath $hex).Hash)"
