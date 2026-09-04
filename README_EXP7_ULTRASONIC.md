# 单片机实验七：循迹、红外/超声波避障、视觉综合（独立副本）

本工程由 `test-exp7-no-line-auto-avoid` 复制而来，新增了模块化超声波
前向测距和避障安全层。原工程、`test-exp7-comprehensive` 和
`test-ultrasonic-avoid` 均未覆盖。

## 工程选择依据

目标工程是 `project/test-exp7-no-line-auto-avoid`：它的 `main.c` 同时
包含按键模式切换、四路循迹、红外避障、K210 视觉串口和电机控制；
`test-ultrasonic-avoid` 只是独立超声波演示，不是整体工程。因此本次只
修改了它的独立副本：`project/test-exp7-no-line-auto-avoid-ultrasonic`。

## 按键模式

- 上电默认或按下 `KEY1`：综合模式，保留超声波、红外和循迹；K210 当前
  已拆除，视觉串口通过 `EXP7_VISION_ENABLED 0` 暂停。
- 按下 `KEY2`：纯寻线模式。红外状态仍由 RGB 显示，但红外不控制电机；
  超声波只做被动测距/运动估计，避障和蜂鸣暂停，视觉事件被丢弃；四路
  全白时先停车保护，再持续缓慢后退，直到重新识别黑线。
- 松开按键后保持已经选择的模式；`KEY3` 当前保留，不参与模式切换。
- 从 KEY2 切回 KEY1 时，超声波从 `WAIT_SAFE` 重新取得有效距离，旧的转弯
  和视觉命令不会继续执行。

## 寻线转弯与丢线恢复

- 四路位置按 `X2(最左)、X1(左中)、X3(右中)、X4(最右)` 加权计算，多个
  探头同时压线时取平均，避免动作突然跳变。
- 只有最左 X2 识别到黑线时，先以 `2200/3400 PWM` 差速滚动 120 ms，
  再以 `3000 PWM` 原地左转约 160 ms，随后切换为 `2100/3300 PWM`
  高曲率前进；最右 X4 状态对称处理。这样不会在整个弯道持续原地转。
- 中间探头要求转向时，内侧轮至少 `2200 PWM`、外侧轮至少 `3000 PWM`，
  保证两侧都越过实车起转阈值；外侧探头触发时仍采用支点急转。
- 四路全白且已有预测方向时，先停车 70 ms，再以 `2700 PWM` 向预测方向
  原地找线，最长 550 ms；只有原地转向仍找不到黑线时才停车并改为后退。
- 无法预测方向时仍以至少 `2300 PWM` 直线后退；重新识别黑线后停车
  70 ms，并进入 250 ms 低速稳定阶段。
- 程序记录中间 X1/X3 丢线顺序：X3 先丢而 X1 保持表示左弯，X1 先丢而
  X3 保持表示右弯；同一采样同时丢失时使用丢线前最后偏差。
- 重新捕线后若 X1/X3 同时见线，稳定阶段按预测方向以 `2200/2700 PWM`
  继续转向，不再默认直行。无法预测时才以 `2200 PWM` 低速直行；稳定结束
  后，中间两路正常直行限制在 `2500 PWM`。
- KEY1 上电后从未见过黑线时仍执行无黑线直行；一旦已经进入寻线再丢失，
  优先按预测方向原地找线，超时后才后退。KEY2 从未见线且没有方向信息时
  直接后退；已有方向信息时同样优先原地转向找线。

## 本次修改

- `Core/Inc/ultrasonic.h`、`Core/Src/ultrasonic.c`：PF11 约 10 us 触发，
  PF12 双边沿 EXTI，TIM2 1 MHz 计时，40 ms 超时和 2～400 cm 合法范围；
  ECHO 使用下拉并清除残留 EXTI 标志，减少断线悬空和旧边沿造成的误判。
  小于 2 cm 的短回波按近障碍处理，大于 4 m 的长回波按无回波处理。
- `Core/Inc/ultrasonic_avoid.h`、`Core/Src/ultrasonic_avoid.c`：连续 3
  次有效结果取中值；连续 2 次有效距离不大于 25 cm 才转弯。单次漏回波
  且最近中值仍有效时只降速，不再立刻清空滤波停车；连续 3 次“无回波
  超时”后进入低速开阔区降级。近墙后先停车 120 ms、后退 300 ms、换向
  保护 60 ms，再原地转弯 500 ms；随后停车冷却 250 ms，左右方向交替。
  若转弯后仍小于 25 cm，会重新确认并继续脱困，不再永久停在墙前。
- `Core/Inc/ultrasonic_motion.h`、`Core/Src/ultrasonic_motion.c`：用毫米距离
  的三点中值估计相对固定反射面的接近速度和位移；剔除不合理跳变，回波
  超时、目标切换、红外/视觉/循迹转弯和超声波脱困时自动失效或重建参考。
- `Core/Inc/vision_uart.h`、`Core/Src/vision_uart.c`：视觉协议和运动数据回传
  代码仍保留，但 K210 拆除期间不初始化、不轮询 USART2，也不发送数据；
  避免悬空 RX 噪声误触发 `stop/left/right`。
- `Core/Inc/oled_status.h`、`Core/Src/oled_status.c`：驱动板载 0.91 寸
  128x32 OLED；使用原理图规定的 I2C1 `PB6/SCL`、`PB7/SDA`，启动时探测
  `0x3C/0x3D`，按页刷新模式、目标距离、接近速度和相对位移。找不到屏幕
  时自动停止 OLED 更新，不影响小车控制。
- `Core/Src/main.c`：调用 `Ultrasonic_Init()`、
  `UltrasonicAvoid_Init(...)` 和 `UltrasonicAvoid_Task()`；超声波前进回调
  只发布速度上限，不覆盖循迹动作；转弯和停车由超声波安全层接管。
  原红外避障动作也改为非阻塞状态机，并将实验五的 2600/2800 PWM 动作
  参数保留下来；原地旋转不再被“开阔区低速”上限误压。上电先预热约 1 s
  再标定，红外标定失败时不再无限锁死主循环，而是暂时屏蔽红外并以左右
  RGB 蓝灯闪烁提示，复位后重新标定。
- `Core/Src/ir_avoid.c`：左右红外状态分别映射到左右 RGB；无遮挡显示绿色，
  有障碍显示红色，未标定显示蓝色闪烁。红外不再直接控制蜂鸣器。
- `Core/Src/stm32f1xx_it.c`：将 PF12 合并到唯一的
  `EXTI15_10_IRQHandler`。
- `Core/Inc/main.h`：补充 PF11/PF12 宏。当前 `.ioc` 没有声明 PF11、PF12
  和 TIM2，所以它们由 `ultrasonic.c` 手动初始化；以后 CubeMX 重新生成
  时必须重新检查 User Code、中断和这些引脚，否则可能被覆盖。
- `Core/Inc/motion_advanced.h`、`Core/Src/motion_advanced.c`：增加正向
  速度上限接口，只限制 PWM 大小，不改变 `motorPWM.c` 的方向映射。

## 接线

开发板 YB-DSF01-V1.1 的超声波 J1：

| J1 | 连接 |
|---|---|
| 1 / TRIG | STM32 `PF11` |
| 2 / ECHO | STM32 `PF12`（必须先确认电平） |
| 3 / 5V | HC-SR04/VCC |
| 4 / GND | 模块 GND 与开发板 GND 共地 |

常见 HC-SR04 的 ECHO 是 5 V，STM32F1 GPIO 不能默认承受 5 V。若模块
没有 3.3 V 电平输出，请使用电阻分压或电平转换，例如 ECHO 到 PF12 串
`1 kΩ`，PF12 到 GND 接 `2 kΩ`（约 3.3 V），并以实物原理图为准。

## 避障状态流程

上电后先处于 `WAIT_SAFE`，没有连续 3 次有效测距不会前进：

```text
WAIT_SAFE --(中值距离 >= 35 cm)--> FORWARD
WAIT_SAFE/FORWARD --(25~35 cm)-----> 允许原模式动作，但限速
WAIT_SAFE/FORWARD --(连续2次 <=25 cm)
             -> 停车(120 ms) -> 后退(300 ms) -> 换向保护(60 ms)
             -> 原地旋转(500 ms)
TURNING  --------------------------> 停车 -> COOLDOWN(250 ms)
COOLDOWN --------------------------> WAIT_SAFE，重新取得3次有效测量
连续3次无回波超时 ------------------> 低速开阔区降级（约1800 PWM）
单次漏回波且有近期有效距离 ----------> 保持运行，但降为约1800 PWM
```

整体工程通过 `UltrasonicAvoid_SetNoEchoFallback(1, 3)` 启用开阔区降级：
连续三次没有 ECHO 后以慢速前进；重新收到有效回波后清空滤波器，重新取得
三次有效测量。由于“超量程”和“传感器断线”在 HC-SR04 上都表现为超时，
该策略会牺牲一部分失联安全性；需要严格失联停车时改为
`UltrasonicAvoid_SetNoEchoFallback(0, 3)`。

超声波与蜂鸣器的关系：25～35 cm 每 600 ms 短鸣一次；不大于 25 cm
或正在执行脱困动作时每 100 ms 交替鸣停；无回波/超过量程时每 1 s
短鸣一次；距离大于等于 35 cm 时关闭。视觉暂停期间不会触发 `horn`。

独立诊断灯仍保留：`WAIT_SAFE` 超时会让 `LED1` 闪烁，异常结果让 `LED2`
闪烁；视觉暂停期间不会再出现视觉 `stop` 指示。红外状态只看左右 RGB。

`FORWARD` 状态下保留红外避障和四路循迹；超声波负责前方距离安全层。
超声波不是轮速编码器，也不用于
保证 8 字轨迹精度。红外负责近距离/左右侧反射检测和原有转向动作，
超声波负责前方距离、限速；无回波降级是否启用由工程配置决定。

## 超声波相对速度与位移

单个前向超声波只能在“小车正对同一静止墙面，并大致沿声束方向直行”时
估算运动，不能确定任意路线的真实里程，也不能替代编码器：

- `D`：当前目标距离，单位 cm。
- `V`：相对目标的接近速度，单位 cm/s；正值表示靠近，负值表示远离。
- `S`：从本次有效参考点开始的相对位移，单位 cm；正值表示靠近目标。
- 连续数据尚未稳定、转弯、丢回波或目标突变时显示 `MOTION: --`。

当前 K210 已拆除，所以 `D / V / S` 串口回传暂停；这些数据现在由板载
OLED 直接显示。以后装回 K210 时，将 `Core/Src/main.c` 中
`EXP7_VISION_ENABLED` 改为 `1U`，并恢复 TX/RX 与共地即可重新启用。

## 板载 OLED 显示

右后方 0.91 寸 OLED 已作为主要数据显示设备，不依赖 K210：

```text
MODE:INTEGRATED     或 MODE:LINE ONLY
D:123.4 CM
V:+12.3 CM/S
S:+45.6 CM
```

运动估计无效时后三行显示 `---.-`。OLED 使用硬件原理图中的 I2C1：
`PB6=SCL`、`PB7=SDA`，供电为 3.3 V。当前 `.ioc` 没有声明 I2C1，因此由
`oled_status.c` 手动初始化；以后用 CubeMX 重新生成代码时必须重新检查。

## 构建与 HEX

在 PowerShell 中运行：

```powershell
& 'F:\myproject\jidian\project\test-exp7-no-line-auto-avoid-ultrasonic\build_exp7.ps1'
```

本次电脑端构建结果：

```text
HEX: F:\myproject\jidian\project\test-exp7-no-line-auto-avoid-ultrasonic\manual-build-ultrasonic-avoid\exp7_no_line_auto_avoid_ultrasonic.hex
SHA-256: A73FE1D4388947F6ADC8743D004869DE92488DD64C924042BCEA3033E8D5347B
```

这是“电脑端编译/链接成功”的证据，不代表已经烧录或实车通过。

## FlyMcu 烧录与实车验证

1. 在 FlyMcu 选择上面的 `.hex`，串口选择设备管理器显示的实际端口，
   按常用 STM32F1 设置下载；烧录完成后记录 FlyMcu 的成功提示。
2. 首次上电让车轮离地，确认 PF11 触发、PF12 接收和左右转向不会异常。
   OLED 应显示当前模式和 `D/V/S`；若显示 `---.-`，先让超声波正对固定墙面
   并保持短暂直行，等待估计稳定。
3. 轮子离地测试：前方无遮挡应经历 `WAIT_SAFE` 后进入前进；放置软质
   障碍至 25 cm 内，应停车、短暂后退、原地交替转向、冷却后重新测距。
   后退阶段没有后向测距，落地测试时必须先保证车后留有安全空间。
4. K210 当前保持拆除，先不要测试视觉指令。
5. 地面测试前确认 ECHO 已降到 3.3 V、共地、传感器朝前。先按 KEY2 在
   黑线上验证纯寻线，再按 KEY1 验证无黑线直行和红外/超声波避障。

截至本次交付，只有电脑端编译和静态重复符号检查已完成；未连接实物，
不能声称 FlyMcu 烧录、轮子离地或地面避障已经验证。

## 在其他程序复用

将 `ultrasonic.h/.c`、`ultrasonic_avoid.h/.c` 及 PF11/PF12 宏加入工程，
在唯一的 `HAL_GPIO_EXTI_Callback()` 中调用
`Ultrasonic_EXTI_Callback(GPIO_Pin)`，在唯一的 `EXTI15_10_IRQHandler()`
中调用 `HAL_GPIO_EXTI_IRQHandler(ULTRASONIC_ECHO_Pin)`；初始化后在主循环
持续调用 `UltrasonicAvoid_Task()`。若 TIM2 已被占用，应先换用空闲的
1 MHz 定时器并同步修改驱动，不能同时保留两个同名中断函数。是否允许
无回波低速前进由 `UltrasonicAvoid_SetNoEchoFallback()` 显式选择。
