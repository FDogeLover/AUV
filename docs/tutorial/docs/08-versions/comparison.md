# 版本选择指南

项目包含多个赛题/功能版本，均基于 `basic/` 扩展。

## 版本对比

| 版本目录 | 用途 | 特色功能 | 成熟度 | 适合人群 |
|---------|------|---------|--------|---------|
| **`basic/`** | 基础自主飞行 | T265+激光+两级降落 | ✅ T-016验证 | **★ 新手必从这里开始** |
| `competition_2026/` | 2026电赛通用版 | 事件总线+截图+视频+UDP | 🟡 待实飞 | 参加电赛 |
| `competition_2026_d/` | D题陆空协同 | 空地DCP+双T265+编队 | ✅ 已实飞通过 | 做D题 |
| `warehouse_inventory/` | 立体货架盘点 | QR视觉+异步扫码 | ✅ 已验收 | 参考已完成项目 |
| `fire_patrol/` | 消防巡逻(G题) | 火情检测+激光笔+抛投 | ✅ 已实飞通过 | G题参考 |
| `circle_pole/` | 圆杆环绕 | circle_planner | ✅ 阶段1验证 | 环绕飞行参考 |

## 如何选择

1. **第一次接触项目** → 从 `basic/` 开始，跑通 DRY_RUN
2. **要参加电赛** → `competition_2026/` 或 `competition_2026_d/`
3. **参考已完成项目** → `warehouse_inventory/`

!!! tip "所有版本共享 Lcode 库"
    `Lprotocol.py`、`Lpid.py`、`heading_hold.py` 等核心库在所有版本中通用。先在 `basic/` 学会这些模块，其他版本只是在此基础上扩展业务逻辑。

!!! warning "进阶版本存在硬编码路径"
    `basic/` 代码无任何硬编码路径，可随意部署。但以下进阶版本中部分脚本写死了板端绝对路径（`/home/sunrise/Desktop/FJJ/`），使用时需手动替换为你的实际路径：

    | 版本 | 涉及文件 | 硬编码路径 |
    |------|---------|-----------|
    | `competition_2026_d/` | `rdk_oled_monitor.py`、`task2_retakeoff_bench.py` | `/home/sunrise/Desktop/FJJ` |
    | `fire_patrol/` | `rdk_imx219_jupyter_preview.py` | `/home/sunrise/imx219-captures` |
    | `warehouse_inventory/` | `laser_aim_realtime.py`、`laser_aim_check.py`、`test_gimbal_manual.py` | `~/Desktop/FJJ/warehouse_inventory` |

---

[开发工作流 →](../09-workflow/firmware-edit.md)
