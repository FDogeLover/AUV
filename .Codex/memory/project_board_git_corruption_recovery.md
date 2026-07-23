---
name: project-board-git-corruption-recovery
description: ubuntu-pi上FJJ/.git仓库2026-07-15发现对象损坏，已重新init恢复(历史丢失，文件不受影响)
metadata: 
  node_type: memory
  type: project
  originSessionId: 747bf7b8-8b34-4a89-afd0-aa51c1b880fc
---

2026-07-15：往ubuntu-pi同步circle_pole改动时，`git diff --stat`异常显示近乎全文件重写(2562行变动)，排查发现`FJJ/.git`仓库有6个loose object是空文件(`66758073.../7f6bc38e.../bce7a62c.../c10a1c17.../c52e779e.../fe607960...`)，导致`git log`/`git status`/`git show`等基本命令全部报错失败。

**诊断结论**：磁盘空间正常(23G可用，不是磁盘写满)，无`.git/objects/pack/`打包文件，无`git remote`配置——没有任何本地备份数据源能恢复这6个损坏对象的原始内容，只能确认"数据已丢失"，不是"命令能修好"的问题。工作目录里的实际文件本身完好(用md5/diff内容比对确认过)，只是git自己的历史对象存储坏了。

**处理方式**：`mv .git .git.corrupted.20260715`(改名备份不删除，留着以防以后想做取证式恢复)，`git init`重新初始化，把当前工作区(364个文件)整体收进一个新的初始commit(`e85703a`)。板子历史commit记录(几个月的本地commit)从此丢失，但`FJJ/.git`本来就是"板载独立历史，不与本机仓库push/pull关联，只用于本地commit/回退"的辅助仓库(见项目CLAUDE.md约定)，真正完整历史一直在本机仓库(已推送GitHub)，不受影响。

**How to apply**：以后如果又发现board上scp同步后`git diff --stat`异常巨大(远超实际改动量级)，先怀疑git仓库本身损坏(用`git fsck --full`确认)，不要只当成换行符问题处理——两种情况现象类似(diff异常大)但处理方式完全不同(换行符问题是`sed -i 's/\r$//'`，仓库损坏需要走这个重新init流程)。
