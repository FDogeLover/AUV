---
name: feedback-ssh-root-tilde-path-bug
description: SSH到ubuntu-pi用root登录时，~会展开成/root而不是/home/sunrise，写路径必须用绝对路径
metadata: 
  node_type: memory
  type: feedback
  originSessionId: fad5c2a9-8042-44f5-b3d1-efab1a9b716b
---

SSH 到 `ubuntu-pi` 是用 `root@192.168.137.125` 登录的，但实际项目部署路径是 `/home/sunrise/Desktop/FJJ/`（属主 sunrise），不是 `/root/Desktop/FJJ/`。任何命令里写 `~/Desktop/FJJ/...` 在 root 登录的 shell 里都会展开成 `/root/Desktop/FJJ/...`，这是一个全新的、不存在的路径，会被静默 `mkdir -p` 创建出来，而不是报错——不会有任何提示告诉你写错了地方。

**Why:** 2026-07-07 同步 `basic_radar/` 到板子时用了 `ssh root@... "mkdir -p ~/Desktop/FJJ/basic_radar" && scp ... root@...:~/Desktop/FJJ/basic_radar/"`，文件全部成功传输、chown也成功，命令层面看起来一切正常，但实际建到了 `/root/Desktop/FJJ/basic_radar`，跟真正的项目目录 `/home/sunrise/Desktop/FJJ/`（有 `basic/`、`original/`、`.git`）完全是两棵不相关的目录树。用户说"为什么我没看到"才发现问题——因为`mkdir -p`+`scp`不会因为目标路径是新建的就报错或警告，这类路径错误在操作过程中完全没有失败信号，只能靠事后人工核对才能发现。

**How to apply:** 以后任何涉及 `ssh root@192.168.137.125` 的命令，路径一律写绝对路径 `/home/sunrise/Desktop/FJJ/...`，不要用 `~`。执行"限定路径可做"级别的操作（`mkdir`/`scp`/`mv`等）之后，除了原有的"用只读命令验证结果"这条规范外，还要**确认验证时用的也是绝对路径**，不能验证时也顺手写 `~`——这次的bug本身就是因为验证和操作都用了`~`，两边路径解析一致所以"看起来正确"，没能自我发现问题。跟 [[project_flight_log_backup]]、[[feedback_sync_test_data_to_local]] 一样，属于"ubuntu-pi文件操作"这个大类下的具体坑，都要小心。
