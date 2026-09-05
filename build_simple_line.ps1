param(
  [string]$ToolchainBin = 'F:\mytool\STM32CubeIDE_1.16.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623\tools\bin'
)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$buildDir = Join-Path $projectRoot 'manual-build-simple-line'
$gcc = Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe'
$size = Join-Path $ToolchainBin 'arm-none-eabi-size.exe'
foreach ($tool in @($gcc, $objcopy, $size)) {
  if (-not (Test-Path -LiteralPath $tool)) { throw "Missing tool: $tool" }
}
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$includes = @('SimpleLine\Inc', 'Core\Inc', 'Drivers\STM32F1xx_HAL_Driver\Inc',
  'Drivers\STM32F1xx_HAL_Driver\Inc\Legacy', 'Drivers\CMSIS\Device\ST\STM32F1xx\Include',
  'Drivers\CMSIS\Include') | ForEach-Object { '-I' + (Join-Path $projectRoot $_) }
$common = @('-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft', '-DUSE_HAL_DRIVER',
  '-DSTM32F103xE', '-Os', '-g3', '-ffunction-sections', '-fdata-sections',
  '-Wall', '-fstack-usage', '-MMD', '-MP') + $includes
# An explicit allowlist: no glob can silently bring an old motor owner back.
$appSources = @('SimpleLine/Src/main.c', 'SimpleLine/Src/board.c',
  'SimpleLine/Src/line_follow.c', 'SimpleLine/Src/operator_input.c', 'Core/Src/motorPWM.c')
$platformSources = @('Core/Src/system_stm32f1xx.c', 'Core/Src/stm32f1xx_hal_msp.c',
  'Core/Src/sysmem.c', 'Core/Src/syscalls.c')
$halNames = @('stm32f1xx_hal.c', 'stm32f1xx_hal_cortex.c', 'stm32f1xx_hal_dma.c',
  'stm32f1xx_hal_exti.c', 'stm32f1xx_hal_flash.c', 'stm32f1xx_hal_flash_ex.c',
  'stm32f1xx_hal_gpio.c', 'stm32f1xx_hal_gpio_ex.c', 'stm32f1xx_hal_pwr.c',
  'stm32f1xx_hal_rcc.c', 'stm32f1xx_hal_rcc_ex.c', 'stm32f1xx_hal_tim.c',
  'stm32f1xx_hal_tim_ex.c')
$halSources = $halNames | ForEach-Object { 'Drivers/STM32F1xx_HAL_Driver/Src/' + $_ }
$sources = @($appSources) + @($platformSources) + @($halSources)
$objects = @()
$manifest = @()
foreach ($relative in $sources) {
  $source = Join-Path $projectRoot $relative
  $object = Join-Path $buildDir (($relative -replace '[\\/]', '_') + '.o')
  $warnings = if ($relative -in $appSources) { @('-Wextra', '-Werror') } else { @() }
  & $gcc @common @warnings '-std=gnu11' '-c' $source '-o' $object
  if ($LASTEXITCODE -ne 0) { throw "Compile failed: $relative" }
  $objects += $object
  $manifest += [ordered]@{ path = $relative; sha256 = (Get-FileHash -LiteralPath $source).Hash }
}
$startup = Join-Path $projectRoot 'Core\Startup\startup_stm32f103zetx.s'
$startupObject = Join-Path $buildDir 'startup.o'
& $gcc @common '-x' 'assembler-with-cpp' '-c' $startup '-o' $startupObject
if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
$objects += $startupObject
$base = Join-Path $buildDir 'simple_four_line'
$elf = $base + '.elf'
$linker = Join-Path $projectRoot 'STM32F103ZETX_FLASH.ld'
$link = @('-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft', "-T$linker",
  '--specs=nosys.specs', '--specs=nano.specs', "-Wl,-Map=$base.map",
  '-Wl,--gc-sections', '-static') + $objects +
  @('-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group', '-o', $elf)
& $gcc @link
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
& $objcopy '-O' 'ihex' $elf ($base + '.hex')
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
& $objcopy '-O' 'binary' $elf ($base + '.bin')
if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed' }
& $size $elf
if ($LASTEXITCODE -ne 0) { throw 'Size check failed' }
$manifest | ConvertTo-Json -Depth 3 | Set-Content -LiteralPath (Join-Path $buildDir 'sources.json') -Encoding UTF8
foreach ($kind in @('hex', 'bin')) {
  $artifact = $base + '.' + $kind
  Write-Output "$($kind.ToUpperInvariant()): $artifact"
  Write-Output "SHA256: $((Get-FileHash -LiteralPath $artifact).Hash)"
}
