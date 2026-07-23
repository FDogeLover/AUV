---
description: 调用 Qoder 对指定文件做独立安全审查（用法：/qoder-review <文件路径>）
---

用以下步骤完成审查，不要跳过任何步骤：

1. 用 Read 工具读取 $ARGUMENTS 指定的文件完整内容
2. 用 Bash 调用 Qoder CLI，命令格式：
   ```
   "C:\Users\FJJ\.qoder-cn\bin\qoderclicn\qoderclicn.exe" -p "<prompt>"
   ```
   prompt 内容为：
   ```
   你是无人机飞控软件安全审查员，对以下 Python 文件做独立审查。
   只报告高置信度问题，每条包含：文件行号、问题描述、风险级别（高/中/低）。
   不要给出修改建议，只报告发现的问题。

   文件路径：$ARGUMENTS
   文件内容：
   <读取到的完整文件内容>
   ```
3. 将 Qoder 返回的审查意见原文展示给用户
4. 在审查意见下方，用一行总结：发现了几个高/中/低风险问题
