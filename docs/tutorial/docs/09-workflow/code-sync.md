# 代码同步

## 本地 → 板子

使用逐文件 scp 推送（不用 scp -r，避免 `__pycache__`）：

```bash
# 示例：同步修改后的 Mission_GPT.py（路径替换为你的实际板端路径）
scp drone_control/basic/Mission_GPT.py ubuntu-pi:~/<你的工作目录>/drone_control/basic/

# 或使用同步脚本
./tools/sync_to_board.sh
```

!!! warning "换行符注意"
    板子上 `.git` 存在 LF/CRLF 混用问题。scp 后先 `cat` 或 `head` 核对文件内容正常，再 commit。

## root 操作后修复权限

```bash
# root操作后必须修复（路径替换为你的实际工作目录）
sudo chown -R sunrise:sunrise ~/<你的工作目录>/
```

## 板子 → 本地（日志归档）

```bash
./tools/pull_flight_log.sh
```

---

← [固件编辑规范](firmware-edit.md) | [日志分析 →](log-analysis.md)
