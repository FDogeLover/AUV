# 故障排查速查

遇到问题先对照下表查找，详细根因见 `docs/known_issues.md`（46条完整记录）。

## 常见症状速查

| 症状 | 可能原因 | 立即处理 | 参考编号 |
|------|---------|---------|---------|
| T265检测不到 | 冷启动概率性失败 | 物理拔插T265 USB口 | #1 |
| 降落后电机仍在转 | land()假阳性 | **遥控上锁！手动断电！** | #22 |
| 飞行中航向偏转 | 罗盘受干扰 | 遥控接管；检查航向保持参数 | #45 |
| 突然急停坠落 | 飞控帧超时2s / T265丢帧 | 检查接线、T265连接 | #6 |
| HOVER_DROP后高度上不去 | 飞控/IMU状态复位 | 遥控接管；避免低空HOVER_DROP | #26 |
| 一键降落没反应 | CMD通道指令撞车 | 已用DESCEND/HOVER_WAIT替代；遥控可接管 | #7, #46 |
| SSH连不上板子 | 动态IP变化 | 路由器后台查IP；接显示器检查 | — |
| Keil编译报错（编码） | 文件被UTF-8编辑器修改 | 用 edit_firmware.py 检查/恢复 | git恢复 |
| 航点到达判定很慢 | precision要求高/速度大 | 检查PID参数；可切cruise测试 | — |

## 文档导航

| 需要找 | 去哪里 |
|--------|--------|
| 完整46条已知问题 | `docs/known_issues.md` |
| 工程决策和调试记录 | `.Codex/memory/MEMORY.md`（59条） |
| 待办事务 | `docs/TODO.md` |
| IMU参数理解 | `docs/guides/imu_parameters_and_fusion_architecture.md` |
| 架构设计 | `docs/architecture/competition_2026_airborne_architecture.md` |
| 代码文档 | `CodeWiki/` |

---

[进阶开发 →](../11-advanced/visual-servo.md)
