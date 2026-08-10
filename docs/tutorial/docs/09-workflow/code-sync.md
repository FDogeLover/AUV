# 代码同步

本页涉及代码在本地电脑与板子之间的传输。部分操作由 AI Agent 自动执行，部分可由人工手动操作。

## 本地 → 板子（代码推送）

!!! abstract "AI Agent 专用：逐文件 scp 推送"
    AI Agent 通过逐文件 scp 将修改后的代码推送到板子（不用 `scp -r`，避免 `__pycache__`）：

    ```bash
    # 示例：同步修改后的 Mission_GPT.py（路径替换为你的实际板端路径）
    scp drone_control/basic/Mission_GPT.py ubuntu-pi:~/<你的工作目录>/drone_control/basic/
    ```

!!! example "人工操作：使用同步脚本"
    人类开发者也可以使用一键同步脚本推送代码：

    ```bash
    ./tools/sync_to_board.sh
    ```

!!! warning "换行符注意"
    板子上 `.git` 存在 LF/CRLF 混用问题。scp 后先 `cat` 或 `head` 核对文件内容正常，再 commit。

## root 操作后修复权限

!!! abstract "AI Agent 专用：chown 修复权限"
    AI Agent 执行 root 权限操作后，必须修复文件归属（路径替换为你的实际工作目录）：

    ```bash
    sudo chown -R sunrise:sunrise ~/<你的工作目录>/
    ```

## 板子 → 本地（日志归档）

!!! example "人工操作：拉取飞行日志"
    飞行结束后，人工执行一键脚本拉取板子上的飞行日志到本地归档：

    ```bash
    ./tools/pull_flight_log.sh
    ```

---

← [固件编辑规范](firmware-edit.md) | [匿名上位机使用 →](ground-station.md)
