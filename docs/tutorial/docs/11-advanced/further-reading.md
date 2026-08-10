# 推荐阅读顺序

掌握基础飞行后，按以下顺序深入：

## 第一梯队：理解架构

1. `docs/architecture/competition_2026_airborne_architecture.md` — 备赛版架构设计理念和安全边界
2. `CodeWiki/01_Architecture.md` ~ `05_Build_and_Run.md` — 固件+上位机完整代码文档
3. `docs/guides/imu_parameters_and_fusion_architecture.md` — 凌霄IMU内部EKF和融合参数

## 第二梯队：理解历史决策

4. `.Codex/memory/MEMORY.md` — 59条工程决策和实战教训（非常宝贵）
5. `docs/known_issues.md` — 46条已知问题完整时间线
6. `.zcode/plans/` — 各功能的设计计划文档（30+篇）

## 第三梯队：深入具体赛题

7. `docs/competition_2026_d/communication_protocol.md` — DCP v1空地通信协议
8. `.zcode/plans/visual_servo_landing_plan.md` — 视觉伺服3版迭代
9. `.zcode/plans/two_stage_landing_plan.md` — 两级降落设计

## AI辅助开发

| 技能 | 命令 | 用途 |
|------|------|------|
| 文档管理 | `/doc check/map/sync/todo/update` | 维护文档一致性 |
| 会话总结 | `/session-summary save` | 保存/读取会话进度 |
| Qoder协作 | `/qoder-workflow` | 新功能设计→审查→实现流程 |

---

*祝你飞行顺利！遇到问题先查 `docs/known_issues.md`，那里大概率已有答案。*
