---
enabledSkills:
  - superpowers:brainstorming
  - superpowers:writing-plans
  - drone-tools
disabledSkills:
  - loop
  - claude-api
---

# Project2 - 无人机工程

> **新 Agent 入口 → `docs/AGENTS_GUIDE.md`**（5 分钟掌握全局）
>
> **文档体系**（2026-07-24 精简）：
> - 项目结构 / 设计决策 / 已知问题详情 → `docs/` 和 `.Codex/memory/`
> - 本文件为**单一权威 Agent 配置入口**，`.claude/CLAUDE.md` 和 `.Codex/CLAUDE.md` 均指向此处

---

## 交互偏好

- **不使用交互式提问组件**（AskUserQuestion 工具）。需要用户提供信息时，直接在对话中以文字写出问题，等用户打字回复。

## 编码规范（关键约束！）

### 飞控固件 .c/.h 文件 — GB2312/GBK 编码

**绝对禁止**用 Read/Edit 工具直接编辑 `.c/.h` 文件！必须用 `edit_firmware.py`：

```bash
python edit_firmware.py show <file>       # 查看编码
python edit_firmware.py replace <f> <o> <n>  # 安全替换
python edit_firmware.py verify <file>     # 验证编码不变
```

详情见 `.Codex/memory/project_board_git_corruption_recovery.md`。

## 远程设备操作规范（SSH）

### 主机别名
- `ubuntu-pi`（动态IP，以 `~/.ssh/config` 为准，不硬编码）
- `orangepi`（192.168.137.126，离线）

### 指令分级

| 级别 | 处理方式 |
|------|---------|
| 放开 | `ls`/`cat`/`ps`/`git log`/`df` 等只读探查，随意 |
| 限路径 | `cp`/`scp`/`mkdir` 在 `~/Desktop/FJJ/` 下，执行前说明 |
| 需确认 | `rm -rf`、`git reset --hard`、批量 `chown -R` |
| 禁止 | `systemctl`/`reboot`/`apt` 系统级修改、网络配置、`kill` 非自己进程 |

### 其他约定
- root 操作后必须 `chown -R sunrise:sunrise`
- 推送代码用逐文件 `scp`，不用 `scp -r`（避免 `__pycache__`）
- 板子 `FJJ/.git` 换行符 LF/CRLF 混用，scp 后先核对再提交
- 数据归档：`./tools/pull_flight_log.sh` 一键拉取到本地 `data_archive/`
- 飞控固件只能用 `edit_firmware.py` 编辑，详情见 `.Codex/memory/project_board_git_corruption_recovery.md`

## ZCode × Qoder 协作规范

### 触发阈值

| 场景 | 是否走完整流程 |
|------|--------------|
| 参数调整、注释、重命名、单函数小改动 | ❌ 直接改 |
| bug 修复（已有测试覆盖） | ❌ 直接改 |
| **新增功能**（新状态、新传感器、新赛题模块） | ✅ 完整流程 |
| **大规模重构**（跨 3+ 文件或改变接口/状态机结构） | ✅ 完整流程 |
| 安全关键路径改动（land/QR解码/通信协议） | ✅ 完整流程（即使改动小） |

### 完整协作流程

```
Step 1  ZCode 撰写结构化计划文档 → .zcode/plans/
Step 2  Qoder Plan Review（第一轮审查）→ [高风险]/[中风险]/[低风险]/[通过]
Step 3  决策检查点 — 用户决定高风险处理方式
Step 4  按确认方案实施代码改动
Step 5  Qoder Implementation Review（第二轮审查）
Step 6  最终结论
```

**审查轮次上限：2轮**。超出说明方案本身需重新设计。

### 差异化 Prompt

| 改动类型 | 方式 |
|---------|------|
| 状态机 | `/qoder-statemachine` |
| 安全关键 | `/qoder-review` |
| 上板前 | `/qoder-preflight` |

Qoder CLI：`C:\Users\FJJ\.qoder-cn\bin\qoderclicn\qoderclicn.exe`

## 已知问题速查

> 完整详情 → `docs/known_issues.md`  |  详细记忆 → `.Codex/memory/`

| # | 问题 | 状态 | 摘要 |
|---|------|:----:|------|
| 1 | T265冷启动检测失败 | 🔴 | 概率性，物理拔插恢复 |
| 7 | 一键降落指令丢弃 | 🟡 | 2026-07-24新增DESCEND/HOVER_WAIT两级下降替代(#46)，验证中 |
| 22 | land()假阳性锁桨确认 | 🔴 | 最高优先级安全隐患，需人工目视确认电机停转 |
| 26 | HOVER_DROP后高度不恢复 | 🔴 | 飞控/IMU状态复位问题，待下次复现诊断 |
| 45 | 航向保持runaway误触 | ✅ | 2026-07-24调大阈值修复 |
| 46 | 两级降落DESCEND/HOVER_WAIT | ✅ | 2026-07-24真机验证通过 |
| T-004 | 视觉伺服精准降落（新功能） | 🟡 | 已完成设计+实现+Qoder审查，等Cyber Camera到货部署 |
| 其余 | 见 `docs/known_issues.md` | — | 完整46条 |

## 索引

```
docs/TODO.md                                         ← 待办事务总览（重要性×紧急性评分）
docs/architecture/competition_2026_airborne_architecture.md  ← 备赛版架构+视觉伺服(5.3)
docs/guides/imu_parameters_and_fusion_architecture.md ← IMU参数理解（2026-07-24）
docs/known_issues.md                                  ← 已知问题完整详情
.Codex/memory/                                        ← 59个详细记忆文件
drone_control/competition_2026_d/vision/              ← 视觉检测+跟踪+相机后端（原CyberCamera端代码已整合至此）
.agents/skills/doc-manager/                           ← 文档管理子智能体（/doc check/map/sync/todo/update）
.zcode/plans/visual_servo_landing_plan.md             ← 视觉伺服计划文档（含3版迭代）
.zcode/plans/two_stage_landing_plan.md                ← 两级降落计划文档
```
