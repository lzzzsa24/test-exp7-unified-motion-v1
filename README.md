# STM32 实验七统一四轮运动控制

这是 YB-DSF01-V1.1 四轮小车（STM32F103ZETx）的当前统一运动控制基准工程。
后续改进从 Git 标签 `v1.0.0-lift-tested` 建立分支，避免覆盖已经验证的基线。

## 当前功能

- KEY1：黑线循迹 + 红外/超声波绕障。
- KEY2：仅黑线循迹。
- KEY3：编码器闭环八字轨迹，完成后停车。
- 遥控/串口 `4`：编码器闭环正方形轨迹。
- 遥控/串口 `0`：锁存停车。
- 四轮编码器、四轮独立速度/位置控制、外接 OLED 电量显示、RGB 状态灯和蜂鸣器均已整合。
- K210 当前停用，但 USART2 和视觉接口代码保留。

控制架构、引脚、模式和安全边界见 [README_UNIFIED_MOTION.md](README_UNIFIED_MOTION.md)。

## 工程结构

- `Core/Inc`、`Core/Src`：应用和驱动源码。
- `Drivers`：STM32F1 HAL/CMSIS 依赖。
- `test-exp7-unified-motion-v1.ioc`：CubeMX 配置。
- `.project`、`.cproject`、`.settings`：STM32CubeIDE 工程配置。
- `build_unified_motion.ps1`：已验证的独立构建脚本。
- `REFACTOR_VERIFICATION.md`：重构、静态检查、编译和固件证据。
- `LIFT_TEST_20260904.md`：2026-09-04 悬空测试记录与未覆盖项。
- `BASELINE_SHA256SUMS.txt`：基准 Release 固件校验和。

`Debug/` 和 `manual-build-unified-motion/` 是本地生成目录，不进入 Git 历史。
与本标签对应的 HEX/BIN 应从 GitHub Release 下载并用 SHA-256 校验。

## 构建

在 Windows PowerShell 中运行：

```powershell
.\build_unified_motion.ps1
```

脚本当前使用本机 STM32CubeIDE 1.16.0 自带的 GNU Arm 工具链；若安装位置不同，
需要调整脚本中的 `$toolRoot`。输出位于 `manual-build-unified-motion/`。

## 当前验证边界

- 电脑端编译、链接和 HEX/BIN 生成：已成功。
- 目标板写入、逐字节读回、启动：已成功。
- 四轮悬空自动测试：默认停车、完整 KEY3、正方形首边/首角、运行中停车已成功。
- 地面循迹、绕障、八字和正方形：尚未在该重构版本完成系统验收。

编码器计数达标不能等同于车身角度或轨迹达标；地面负载、轮胎打滑、电池电压和场地
都会影响结果。继续改动前请先创建分支，硬件测试后把条件和结果写入新的验证记录。

## 基准固件

标签：`v1.0.0-lift-tested`

- `exp7_unified_motion.hex`
- `exp7_unified_motion.bin`

详见 [BASELINE_SHA256SUMS.txt](BASELINE_SHA256SUMS.txt)。
