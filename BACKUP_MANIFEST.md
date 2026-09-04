# 统一运动工程基线与回滚清单

- 源工程：`F:\myproject\jidian\project\test-exp7-line-bypass-v2`
- 独立工作工程：`F:\myproject\jidian\project\test-exp7-unified-motion-v1`
- 完整回滚副本：`F:\myproject\jidian\project\backups\test-exp7-line-bypass-v2-pre-unified-motion-20260904-170038`
- 备份时间：2026-09-04 17:00:38（Asia/Shanghai）
- 备份核验：283 个源文件、39,957,070 字节；关键源码和固件哈希逐项一致

## 重构前固件

| 项目 | 值 |
|---|---|
| HEX | `manual-build-line-bypass-v2/exp7_line_bypass_v2.hex` |
| HEX SHA-256 | `69212519C6238EBD23C6D683FD2739676EDA4BC203B4227536E4BE4BDAFD9C3B` |
| BIN SHA-256 | `1B7396A97BF4584C7C835D3D24AE2D397A3DC758EC910C7CE67A5BACC871CB60` |
| 链接尺寸 | text=56,880；data=64；bss=3,344 |

修改工程身份后、修改源码前，`build_unified_motion.ps1` 首次独立构建所得
HEX SHA-256 仍为 `69212519...D9C3B`，证明复制和工程改名没有改变基线固件。

完整关键源码哈希见回滚副本中的 `BACKUP_MANIFEST.md`。本工程不得覆盖该
回滚目录；回滚副本也不应和工作工程同时导入同一个 STM32CubeIDE 工作区。
