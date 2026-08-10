# 固件编辑规范

!!! danger "人工须知：绝对禁止用普通编辑器打开/修改 .c/.h 文件"
    飞控固件使用 GB2312/GBK 编码，直接用 UTF-8 编辑器（VS Code、Notepad 等）修改会导致编码损坏、编译失败甚至飞行异常。

    **人工修改固件的唯一方式：用 Keil uVision 打开工程，在 Keil 内编辑。** Keil 会正确处理 GB2312 编码。

## 背景：为什么固件编码会损坏

固件源码由 Keil uVision 创建，中文注释使用 GB2312/GBK 编码。Git、现代编辑器默认 UTF-8，直接操作会导致：

- 中文注释乱码
- BOM 标记混入
- 编译时字符串长度变化（中文字符 GB2312 占2字节，UTF-8 占3字节）
- 严重时宏定义偏移导致飞行异常

详细背景和故障恢复方法见 `.Codex/memory/project_board_git_corruption_recovery.md`。

---

!!! abstract "以下为 AI Agent 专用操作，人工无需执行"
    `edit_firmware.py` 是为 AI Agent 设计的安全编辑脚本，确保 AI 在修改固件时不会破坏 GB2312 编码。人类开发者请直接使用 Keil uVision 编辑。

## AI Agent 专用：edit_firmware.py

```bash
# 查看文件编码
python edit_firmware.py show 飞控固件/FcSrc/ANO_LX.c

# 安全替换字符串（自动保持编码不变）
python edit_firmware.py replace <文件路径> <旧字符串> <新字符串>

# 验证文件编码未被破坏
python edit_firmware.py verify <文件路径>
```

---

[代码同步 →](code-sync.md)
