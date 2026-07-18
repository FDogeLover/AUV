---
name: feedback-background-task-completion-misleading
description: "nohup后台启动命令的\"completed\"通知只代表启动命令本身返回，不代表被启动的长时间任务(如真机飞行)已经结束"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a330d6d5-aa3a-482f-b6e1-f30a4b335cbc
---

用`ssh ... "nohup python3 main.py > log 2>&1 & echo PID=$!"`这种模式在pi上启动真机飞行脚本时，这条SSH命令本身几乎立刻返回（因为`&`把主程序丢进后台），background task工具收到的"completed"通知只代表**这条启动命令**执行完了，不代表**被启动的main.py飞行任务**已经飞完。

**Why:** 2026-07-13 circle_pole真机测试时，收到某次启动命令的"completed"通知后立刻去查日志，结果飞机其实还在环绕途中（航点还没飞完），因为通知到达和实际检查之间没有足够时间流逝。之前几次因为中间穿插了对话往来，凑巧等够了飞行需要的时间，才误以为"通知即完成"这个假设是对的。

**How to apply:** 用这种nohup模式启动真机测试后，不能只凭launch命令的完成通知就去查结果，要么：
1. 用`ssh ... "while pgrep -f 'python3 main.py' > /dev/null; do sleep 2; done"`这类真正等待进程退出的命令（但注意：如果这条wait命令自己是通过`bash -c "...python3 main.py..."`这种方式调用的，`pgrep -f`会匹配到调用自己的这条命令本身的文本，导致永远等不到——要么用`kill -0 $PID`检查具体PID，要么让wait命令的文本本身不包含"python3 main.py"这个字符串）
2. 或者直接检查具体PID是否还存在（`ps -o pid,cmd -p <PID>`），比字符串匹配更可靠
