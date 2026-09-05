# 简化四路寻线分支

当前分支：`feature/simple-four-line`，基于 `d2efe0e` 新写寻线应用。
硬件为 YB-DSF01-V1.1 / STM32F103ZETx。

四路低有效探头按 **X2、X1、X3、X4**（左外、左内、右内、右外）采样；
5 ms 周期直接输出左右侧 PWM。中间微调、外侧急转、丢线限时搜索，超时停车。

- [使用方法、16 种输入决策、参数与验证范围](SimpleLine/README.md)
- [硬件映射和本地证据](SimpleLine/HARDWARE.md)
- [当前源码、候选固件和已烧录状态](PROJECT_STATE.md)

新固件上电停车，KEY1/KEY2 启动、KEY3 停车；遥控和串口 1/2 启动、0/3 停车。
本次尚未烧录，板上旧固件的按键含义由旧分支记录确定。

在仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_unified_motion.ps1
cmd /c tests\simple_line\run.cmd
```

输出为 `manual-build-simple-line/simple_four_line.hex` 和 `simple_four_line.bin`。
也可直接用 `build_simple_line.ps1 -ToolchainBin '工具链目录'` 指定编译器。

主程序及控制逻辑在 `SimpleLine`，电机极性只在 `Core/Src/motorPWM.c`。
旧 `Core/Src` 应用文件及历史说明保留作参考，不在本版构建白名单中。
CubeIDE Debug/Release 源码选择已同步；
`.ioc` 仍是旧平台参考，以脚本构建结果为本次证据。

旧分支 `feature/line-reacquire-lock` 在另一个工作区独立继续演进，本次未修改它。
本次起点的固定回退标签为 `rollback/2026-09-05-before-simple-four-line`。
本分支未推送远端。
