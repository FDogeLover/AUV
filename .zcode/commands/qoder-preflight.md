---
description: 真机测试前调用 Qoder 审查 git diff，检查安全风险（用法：/qoder-preflight 或 /qoder-preflight <模块目录>）
---

用以下步骤完成起飞前审查，不要跳过：

1. 用 Bash 获取即将上板的代码变更：
   - 如果 $ARGUMENTS 非空，执行：`git -C "D:/项目与工具/Python项目/Project2" diff HEAD -- $ARGUMENTS`
   - 如果 $ARGUMENTS 为空，执行：`git -C "D:/项目与工具/Python项目/Project2" diff HEAD -- drone_control/`
   - 如果 diff 为空，改用：`git diff HEAD~1 HEAD -- drone_control/`
2. 如果 diff 超过 200 行，只保留前 200 行（截断时告知用户）
3. 用 Bash 调用 Qoder CLI：
   ```
   "C:\Users\FJJ\.qoder-cn\bin\qoderclicn\qoderclicn.exe" -p "<prompt>"
   ```
   prompt 内容：
   ```
   你是无人机飞控软件安全审查员。以下是即将部署到真实飞行板的 Python 代码 diff。
   这是一台真实飞行的无人机，代码缺陷可能导致坠机或无法降落。

   请检查以下几类问题（只报告高置信度的，给出行号）：
   1. 可能导致无人机无法正常降落的逻辑错误
   2. 状态机死锁（进入某状态后无法离开）
   3. 串口/线程资源未释放（会导致再次启动时崩溃）
   4. 超时兜底缺失（等待类操作没有超时退出）
   5. 明显的竞态条件

   diff 内容：
   <diff 内容>
   ```
4. 将 Qoder 返回的审查意见展示给用户
5. 明确问用户："确认没有问题后再 scp 上板，需要我处理其中哪条吗？"
