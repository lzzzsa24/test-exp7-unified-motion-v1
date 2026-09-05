# 硬件与接口依据

核对日期：2026-09-05。本次核对的是本地源文件和已有实测记录，未访问串口、未重新测量接线。
参考快照为 `d2efe0e`；新分支不继承其驾驶状态机。

## 本版启用

| 部件 | 引脚/外设 | 已核实含义 |
|---|---|---|
| 主板 | YB-DSF01-V1.1 / STM32F103ZETx | 512 KiB Flash、64 KiB RAM |
| 时钟 | 外部 HSE 8 MHz，PLL ×9 | CPU 72 MHz，APB1 36 MHz，APB2 72 MHz |
| 左外探头 | X2 / PF14 | 低电平为黑线，显示位 3 |
| 左内探头 | X1 / PF13 | 低电平为黑线，显示位 2 |
| 右内探头 | X3 / PF15 | 低电平为黑线，显示位 1 |
| 右外探头 | X4 / PG0 | 低电平为黑线，显示位 0 |
| KEY1 / KEY2 / KEY3 | PG3 / PG4 / PG5 | 低电平按下；本版前两键启动、第三键停车 |
| 调试串口 | USART1 PA9 TX / PA10 RX | 115200、8N1、3.3 V TTL；串口号须现场枚举 |
| 遥控接收头 | HS0038B / PG11 / EXTI11 | NEC，DWT 计时；数字 1=0x10、2=0x11、0=0x0D、3=0x12 |
| 有源蜂鸣器 | PG12 | 高电平响；本版明确初始化 |
| LED1 / LED2 | PG13 / PG15 | 本版显示运行/丢线搜索 |
| 左 RGB R/G/B | PG1 / PE7 / PG2 | 本版红=左外，绿=左内 |
| 右 RGB R/G/B | PE2 / PE3 / PE4 | 本版绿=右内，红=右外 |

## 电机方向与 PWM

从车身上方向下看，车头向前；逻辑正数必须使车辆前进。

| 轮子 | A / B 引脚 | 定时器 A / B | 逻辑正向 | 逻辑反向 |
|---|---|---|---|---|
| M1 左前 | PC6 / PC7 | TIM8 CH1 / CH2 | A=0，B=PWM | A=PWM，B=0 |
| M2 左后 | PC8 / PC9 | TIM8 CH3 / CH4 | A=0，B=PWM | A=PWM，B=0 |
| M3 右前 | PE9 / PE11 | TIM1 CH1 / CH2 | A=PWM，B=0 | A=0，B=PWM |
| M4 右后 | PE13 / PE14 | TIM1 CH3 / CH4 | A=PWM，B=0 | A=0，B=PWM |

TIM1 完全重映射；两个定时器均为 72 MHz / 3600 = 20 kHz，ARR=3599。
左右侧机械安装导致电气极性相反，补偿只在 `Core/Src/motorPWM.c` 内实现。
新驱动保留这张映射表，加入 10 ms 零输出换向间隔。

历史悬空标定中，PWM=1800 四轮均未起转；以下三个工作点测得了有效转动，
记录同时给出了逐轮补偿。新驱动保存这些硬件数据并线性插值：

| 逻辑 PWM | M1 CCR | M2 CCR | M3 CCR | M4 CCR |
|---:|---:|---:|---:|---:|
| 2200 | 2174 | 2208 | 2210 | 2208 |
| 2400 | 2371 | 2410 | 2412 | 2408 |
| 3000 | 2962 | 3009 | 3018 | 3012 |

0 映射为 0，3599 映射为 3599。悬空标定不保证不同电池电压和地面载荷下等速。

## 已确认但本版不参与控制的接口

| 部件 | 接口 | 备注 |
|---|---|---|
| M1 编码器 AB | PD12 / PD13 | 四编码器已修复；本版不采样 |
| M2 编码器 AB | PA15 / PB3 | 正确接线，旧 PD7 单边沿替代法已废弃 |
| M3 编码器 AB | PA0 / PA1 | 本版不采样 |
| M4 编码器 AB | PB4 / PB5 | JTAG 关闭、SWD 保留 |
| 左/右红外避障 | PE5→PF9 ADC3_IN7 / PE6→PF10 ADC3_IN8 | PE5/PE6 低有效；本版置高关闭发射 |
| 超声波 J1 | PF11 TRIG / PF12 ECHO | 本版 TRIG 置低，不启动测距 |
| 外接 OLED J12 | I2C1 PB6 SCL / PB7 SDA，0x3C 或 0x3D | 128×32；本版不刷新 |
| 电池分压 | PF7 / ADC3_IN5，10k/3.3k | 本版不采集，不提供低电压保护 |
| K210 | USART2 PD5 TX / PD6 RX | 最新硬件状态为已拆除 |
| 标定 Flash 页 | 0x0807F800～0x0807FFFF | 本版不读写，链接区排除该页 |

## 本地证据

- `Core/Inc/main.h`：电机、探头、按键、蜂鸣器、RGB、超声波、遥控引脚宏。
- 历史项目 `test-exp7-line-auto-avoid-ultrasonic/Core/Inc/main.h` 和 `Core/Src/line_tracking.c`：
  X1/X3 是中间、X2/X4 是外侧；X1/X2 属于左侧；低电平检测黑线。
- 快照 `d2efe0e:Core/Src/motorPWM.c`：上述定时器重映射、正反极性和标定点。
- `MOTOR_POWER_CALIBRATION_20260903.md` 及对应 RAW 文本：历史悬空标定条件和数据。
- 快照 `d2efe0e:Core/Src/wheel_encoder.c`、`PROJECT_STATE.md`：四轮命名、修复状态、M2 接线。
- 快照的 `diagnostic_uart.c`、`ir_remote.c`、`oled_status.c`、`battery_monitor.c`：附属接口。
- `Core/Inc/stm32f1xx_hal_conf.h`、芯片 CMSIS 头文件及链接脚本：时钟、寄存器和存储器范围。

探头安装顺序仍应在新固件首次试车前用黑纸逐路确认；本次没有新的物理观测。
