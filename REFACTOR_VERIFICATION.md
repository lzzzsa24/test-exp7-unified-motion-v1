# 统一四轮运动控制重构验证记录

验证日期：2026-09-04  
工作工程：`F:\myproject\jidian\project\test-exp7-unified-motion-v1`

## 1. 回滚与工程边界

- 原工程保持在 `F:\myproject\jidian\project\test-exp7-line-bypass-v2`。
- 完整回滚副本位于
  `F:\myproject\jidian\project\backups\test-exp7-line-bypass-v2-pre-unified-motion-20260904-170038`。
- 重新逐文件计算 SHA-256：原工程 283 个文件与回滚副本中的对应文件
  不一致数为 0；回滚副本多出的 1 个文件是备份清单。
- 重构前 HEX SHA-256：
  `69212519C6238EBD23C6D683FD2739676EDA4BC203B4227536E4BE4BDAFD9C3B`。
- 工作副本中从原工程复制来的旧 `manual-build-line-bypass-v2` 构建缓存已
  清理；原工程和回滚副本中的旧产物仍完整保留。

## 2. 已完成的软件重构

1. 新增 `Core/Inc/drive_base.h`、`Core/Src/drive_base.c`，统一负责四轮
   CPS、位置目标、PWM输出、停车和故障。
2. TIM6 与四路 AB 编码器上电后常驻；动作使用起始快照，不再反复启停
   或清零。逻辑前进修正为 M1/M2 `+1`、M3/M4 `-1`。
3. 连续速度区为每轮分段前馈 + PI + 抗积分饱和 + 目标斜坡；只允许
   无运动的落后轮获得一次错峰启动补偿。
4. 小角度、短距离、弧线内轮和末端改为读取实际剩余计数的自适应位置
   脉冲；四轮初始脉冲错开 25 ms，领先轮等待平均进度。悬空测试后又将
   自适应依据修正为通电和完整滑行的总响应，脉宽范围为1～20 ms。
5. 直线和转向按 `moved_i/target_i` 同步；已有轮到站后，其他轮只能用
   有界位置脉冲收尾，四轮均进入容差后统一停车。
6. KEY1/KEY2 的黑线模块改为只输出左右 CPS；DriveBase 为四轮分别闭环。
   丢线 360 度搜索继续由 EncoderTurn 位置控制。
7. EncoderLinear、EncoderStraight、EncoderTurn、八字、正方形和绕障段
   都已迁移到统一位置/速度执行层；原绕障距离、角度和状态参数保留为
   本轮迁移初值。
8. 电池滤波电压参与有限前馈：参考 7.8 V，限制为 85%～115%；LOW 仍
   只显示，不擅自变成欠压停车。
9. COAST 仍是 0/0；BRAKE 为非阻塞 30 ms 零输出换向保护 + 最长 22 ms
   逻辑 PWM 2400 反向转矩 + 0/0。未使用未经确认的 AT8236 1/1 状态。
10. `motor.c` 文件保留，但已从 Debug、Release 和独立脚本构建中排除。
    `.ioc` 已记录 TIM6、八个编码器 GPIO 和中断优先级；运行时暂由
    `wheel_encoder.c` 手动初始化，CubeMX 再生成前必须人工合并。

### 文件清单

- 新增：`Core/Inc/drive_base.h`、`Core/Src/drive_base.c`、
  `build_unified_motion.ps1`、`BACKUP_MANIFEST.md`、
  `README_UNIFIED_MOTION.md`、`REFACTOR_VERIFICATION.md`、
  `LIFT_TEST_20260904.md`、`test-exp7-unified-motion-v1.ioc`。
- 修改接口/控制源码：`Core/Inc/line_tracking.h`、
  `Core/Inc/motion_advanced.h`、`Core/Inc/wheel_encoder.h`、
  `Core/Inc/wheel_speed_control.h`、`Core/Src/main.c`、
  `Core/Src/motion_advanced.c`、`Core/Src/wheel_encoder.c`、
  `Core/Src/wheel_speed_control.c`、`Core/Src/wheel_speed_observer.c`、
  `Core/Src/encoder_linear.c`、`Core/Src/encoder_straight.c`、
  `Core/Src/encoder_turn.c`、`Core/Src/line_tracking.c`、
  `Core/Src/line_obstacle_bypass.c`、`Core/Src/stm32f1xx_it.c`。
- 修改工程身份/构建：`.project`、`.cproject`；旧名 IOC 和旧名构建脚本
  不保留在工作副本中。
- 更新说明：`MOTION_CONTROL_REFACTOR_HANDOFF_PLAN.md`、
  `ENCODER_SPEED_CONTROL_20260903.md`、`README_ENCODER_FIGURE8.md`、
  `README_LINE_BYPASS_V2.md`。
- 排除构建但未删除：`Core/Src/motor.c`。保持未修改：
  `Core/Src/motorPWM.c` 及其已验证方向映射。

## 3. 当前模式控制类型

| 入口 | 上层 | 底层 |
|---|---|---|
| 上电 / `0` | 锁存停车 | DriveBase 停车 |
| KEY1 / `1` | 黑线循迹 + 红外/超声波绕障 | 黑线外环 + 四轮速度内环；绕障为位置反馈与传感器状态机混合控制 |
| KEY2 / `2` | 仅黑线循迹 | 黑线外环 + 四轮速度内环；丢线搜索为位置反馈 |
| KEY3 / `3` | 一次 220 mm 半径初值八字 | 连续速度反馈 + 低速位置脉冲 + 四轮进度同步 |
| 遥控/串口 `4` | 400 mm 边长正方形 | 直线/90度转向位置反馈 + 四轮进度同步 |

K210 仍禁用，但 USART2/视觉接口没有删除；RGB 红外状态、超声波蜂鸣器、
五音节蜂鸣器及其安全优先级、OLED、电量、按键和遥控入口均保留。

## 4. 静态验收

- 直接调用 `pwm_motor*()` 的运行代码只有 `drive_base.c`；函数定义位于
  未改动的 `motorPWM.c`。
- 直接写 PWM 比较寄存器的代码只有 `motorPWM.c`。
- `motorPWM.c` SHA-256 仍为
  `D881745941F1808B2B0F4BCC99F1957806669A62D131A0E25BFFE7C66598E32F`。
- ELF 中存在全部 `DriveBase_*`、`line_tracking_compute`、
  `WheelEncoder_TIM6_IRQHandler` 符号；不存在 `motor_init` 符号。
- 只有一个 `HAL_GPIO_EXTI_Callback`，其中同时调用
  `IrRemote_EXTI_Callback(GPIO_Pin)` 和 `Ultrasonic_EXTI_Callback(GPIO_Pin)`。
- 只有一个 `TIM6_IRQHandler` 和一个 `EXTI15_10_IRQHandler`。
- 定时器用途无重叠：TIM1/TIM8=电机PWM，TIM2=超声波1 MHz计时，
  TIM6=四编码器20 kHz采样。
- `.ioc` 键名重复数为 0；工程名、Debug/Release构建路径和IOC身份均为
  `test-exp7-unified-motion-v1`。
- DriveBase、三个位置模块、循迹和绕障控制模块中没有 `HAL_Delay()`。
- 12 个新增/修改的核心 C 文件通过 GCC `-Wall -Wextra -Wshadow
  -fanalyzer -fsyntax-only` 检查，没有诊断。

## 5. 编译和产物

两条独立路径均完整编译、链接成功，没有普通 `-Wall` 警告：

| 构建路径 | text | data | bss |
|---|---:|---:|---:|
| `build_unified_motion.ps1` | 62136 | 64 | 3504 |
| CubeIDE生成的 Debug Makefile | 62140 | 64 | 3504 |

独立脚本的最终产物：

| 文件 | 字节 | SHA-256 |
|---|---:|---|
| `manual-build-unified-motion/exp7_unified_motion.elf` | 1091628 | `05FCEBD33737B744D96ABC9C08D5674549800B91E6CCFA3CB724A0EE2E5B7E4D` |
| `manual-build-unified-motion/exp7_unified_motion.hex` | 175034 | `92C54B52773A0AF7B43A6D1620E5BC951AD23BD213BAB0C788E41AFAD66D29D9` |
| `manual-build-unified-motion/exp7_unified_motion.bin` | 62204 | `EE74FA4CBFD885A2F47889C61F88AE0B823DC14225132013DCBE697A80514754` |
| `manual-build-unified-motion/exp7_unified_motion.map` | 758378 | `834250EE9FD84F078FFDCDDE6CDAC497F5198FD6BB5ECC126DE6C68CBC248628` |

CubeIDE Debug ELF：

`Debug/test-exp7-unified-motion-v1.elf`  
SHA-256：`DCA7D4BB7891BCAE5E909FF1AF6834C6050406E2797511022FC8CF13D3BA599A`

## 6. 证据边界与下一步

| 项目 | 状态 |
|---|---|
| 电脑端编译/链接 | 已成功 |
| 串口烧录和逐字节读回 | 已成功；62204字节，最后一页保留，VERIFY/GO OK |
| 四轮悬空验证 | 核心轨迹与停车测试已成功；仍有未覆盖子项 |
| 有载荷地面验证 | 本轮未执行 |

最终固件的默认停车、完整八字、正方形首边/首角和运行中停车已按
`LIFT_TEST_20260904.md` 完成悬空自动验证。逐轮低/中/高速、全部反向、
人工堵转、高速BRAKE、KEY1/KEY2传感器及地面测试尚未覆盖。反向最小PWM
当前只是独立参数表但初值沿用正向数据，15/90/360度编码器完成也不能
替代有载荷地面车身角度、八字闭合、正方形尺寸和绕障验证。
