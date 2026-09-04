# 超声波动态制动与连续转向修改记录

日期：2026-09-04  
目标工程：`project/test-exp7-line-bypass-v2`

## 回滚基线

- 完整备份：`project/backups/test-exp7-line-bypass-v2-before-emergency-brake-continuous-turn-20260904`
- 备份文件数：275
- 工程内回滚 HEX：`manual-build-line-bypass-v2/exp7_line_bypass_v2_before_emergency_brake_continuous_turn.hex`
- 回滚 HEX SHA-256：`80C75935FDDB6A386487F21CAD23A7BB7FF2FC98ABD0950E9565161C13A225AD`

## 新制动逻辑

- 固定停止阈值由 10 cm 提高到 20 cm，减速/释放阈值由 20 cm 提高到 35 cm。
- KEY1、KEY2 直行时启动四轮编码器被动测速；该观察器只读计数，不写电机。
- 动态紧急距离按四轮平均轮速和 140 ms 前视时间计算，限制在 20～32 cm。
- 已检测到高速后保持该峰值 220 ms，避免第一帧降速使第二帧错误退出紧急制动档。
- 典型换算：0 cps→20 cm、2000 cps→24 cm、3500 cps→27 cm、5300 cps→31 cm、8000 cps→32 cm。
- 第一帧进入动态近障区立即把速度上限降到慢速；60 ms 后第二帧仍在阈值内才正式触发，减少单次假回波误动作。10 cm 内回波立即触发。
- 一旦出现第一帧近障，动态阈值在确认完成前只允许增大、不允许因减速而后退。
- 触发前四轮平均速度不低于 3500 counts/s 时进入紧急制动档：先将全部 PWM 置零 30 ms，再以 2600 counts/s 反向闭环制动/后退，最多 40 mm。
- 普通低速触发仍保留 120 ms 停车保护和原有最大 40 mm 后退限制。

## 连续转向逻辑

- 未看到障碍侧面时，不再固定每转 15°就停车；一次连续闭环扫转最多 45°。
- 侧向红外在连续转向期间持续采样：首次稳定进入目标带或安全区时请求提前结束转向。
- 已经太靠近障碍时，连续向外修正最多 30°，到达安全边界即提前结束。
- 平行航向和返回黑线航向校正由每次最多 15°改为单段最多 45°，结束后仍按编码器实际完成角度重新计算。
- 连续转向最大轮速由 1500 提高到 1800 counts/s。
- 保留每次转向后的安全前进段；连续转向不会绕过红外过近中断、360°净转角上限、编码器失速和单动作超时保护。

## 诊断输出

绕障启动行增加：

```text
BYP2 START US V=5300 E=31
```

- `V`：触发前四轮平均速度，单位 counts/s。
- `E`：本轮动态紧急距离，单位 cm。

周期 `BYP2` 遥测增加：

- `EB=1`：本轮使用高速紧急制动档。
- `CT=1`：当前处于红外引导连续转向。

## 电脑端验证

- 整体工程编译、链接成功，未出现编译警告。
- ELF 存在 `WheelSpeedObserver_*`、`UltrasonicAvoid_SetEmergencyDistance`、`LineObstacleBypass_StartWithSpeed` 和 `EncoderTurn_RequestStop`。
- `HAL_GPIO_EXTI_Callback` 与 `EXTI15_10_IRQHandler` 各只有一个定义。
- 新增/修改的控制模块没有 `HAL_Delay()`。
- `motorPWM.c` 修改前后 SHA-256 均为 `D881745941F1808B2B0F4BCC99F1957806669A62D131A0E25BFFE7C66598E32F`，电机方向映射未改。
- 当前 HEX：`manual-build-line-bypass-v2/exp7_line_bypass_v2.hex`
- 当前 HEX SHA-256：`69212519C6238EBD23C6D683FD2739676EDA4BC203B4227536E4BE4BDAFD9C3B`
- 当前 BIN SHA-256：`1B7396A97BF4584C7C835D3D24AE2D397A3DC758EC910C7CE67A5BACC871CB60`

本次未烧录，未完成轮子离地测试和地面碰撞距离测试；上述参数是软件初值，不能当作实车避障已经通过。
