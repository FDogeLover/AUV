---
name: feedback-pi-sync-line-endings
description: scp同步Python文件到ubuntu-pi后、git commit前，必须核对每个文件原有的换行符约定
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3e206fa0-fe8a-47e7-a368-627b97bcf0c9
---

`ubuntu-pi` 上 `FJJ/.git` 仓库里换行符约定不统一：`basic_radar/Lcode/Lradar.py` 是 LF，但 `basic_radar/Mission_GPT.py`/`main.py` 历史上就是 CRLF（板子仓库自己遗留的不一致，不是本机的问题）。本机 Windows 编辑的文件基本都是 CRLF。

**Why:** 2026-07-08 同步 PoleTracker 相关改动时，scp 完直接想统一转成 LF"修一下"，结果把本来就是 CRLF 的 `Mission_GPT.py`/`main.py` 也转成了 LF，导致这两个文件在 pi 上 `git diff --stat` 显示成几百上千行改动（实际只改了几十行）——因为跟它们自己的 git 历史（CRLF）比对，换行符本身就算成整行改动。反而制造了新的不一致，得再改回来。

**How to apply:** scp 完任何文件到 pi、准备在 pi 上 `git commit` 之前，先 `git show HEAD:<file> | file -` 看这个文件在 git 历史里原来是不是 CRLF，和 `file <file>` 比对当前工作区状态，不一致就用 `sed -i 's/\r$//'`(转LF) 或 `sed -i 's/$/\r/'`(转CRLF，注意别对已经是CRLF的行重复加，最保险是先统一转LF再按需转CRLF) 按**各文件自己原有的约定**改回来，改完 `git diff --stat` 确认改动行数量级合理（对应实际改了多少行，不是整个文件），再 commit。不要假设整个仓库统一用一种换行符。

见 [[feedback_auto_sync_pi]]（同步流程本身的既有约定）。
