# 推荐阅读顺序

掌握基础飞行后，按以下顺序深入：

## 第一梯队：理解架构

1. `docs/architecture/competition_2026_airborne_architecture.md` — 备赛版架构设计理念和安全边界
2. `CodeWiki/01_Architecture.md` ~ `05_Build_and_Run.md` — 固件+上位机完整代码文档
3. `docs/guides/imu_parameters_and_fusion_architecture.md` — 凌霄IMU内部EKF和融合参数
4. [匿名上位机使用指南](../09-workflow/ground-station.md) — 凌霄官方调参/调试工具（支持USB有线和数传无线连接）

## 第二梯队：理解历史决策

5. `docs/known_issues.md` — 46条已知问题完整时间线

!!! abstract "AI 内部资料（可选参考）"
    以下为 AI Agent 工作过程中产生的内部资料，人类开发者可按需参考，但不需要日常使用：

    - `.Codex/memory/MEMORY.md` — AI 记忆体系中的工程决策和实战教训
    - `.zcode/plans/` — AI 辅助设计产生的功能计划文档（30+篇）

## 第三梯队：深入具体赛题

6. `docs/competition_2026_d/communication_protocol.md` — DCP v1空地通信协议

!!! abstract "AI 设计文档（可选参考）"
    - `.zcode/plans/visual_servo_landing_plan.md` — 视觉伺服3版迭代设计
    - `.zcode/plans/two_stage_landing_plan.md` — 两级降落设计

## AI辅助开发

!!! abstract "AI Agent 专用命令"
    以下斜杠命令供 AI Agent 使用，人类开发者无需操作：

    | 技能 | 命令 | 用途 |
    |------|------|------|
    | 文档管理 | `/doc check/map/sync/todo/update` | AI维护文档一致性 |
    | 会话总结 | `/session-summary save` | AI保存/读取会话进度 |
    | Qoder协作 | `/qoder-workflow` | AI辅助新功能设计→审查→实现流程 |

---

*祝你飞行顺利！遇到问题先查 `docs/known_issues.md`，那里大概率已有答案。*
