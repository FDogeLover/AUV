# 运行单元测试

项目有完整的单元测试覆盖核心逻辑，所有测试无需硬件即可运行。

## 运行所有测试

```bash
cd drone_control/basic

# 全量测试
pytest

# 详细输出
pytest -v

# 单个测试文件
pytest test_heading_hold.py
pytest test_navigation_profile.py
```

## 测试覆盖

共 18+ 个测试文件，覆盖：

| 测试文件 | 覆盖内容 |
|---------|---------|
| `test_heading_hold.py` | 航向保持控制器 |
| `test_navigation_profile.py` | 航点到达判定策略 |
| `test_lprotocol.py` | 串口协议帧解析 |
| `test_lpid.py` | PID控制器 |
| `test_gpio_button.py` | GPIO按键驱动（空操作） |
| ... | 共18+个文件 |

!!! tip "跨平台"
    所有测试可在 Windows / macOS / Linux 任意平台运行，不需要硬件，`DRY_RUN` 模式下 GPIO 自动空操作。

---

← [桌面模拟飞行](dry-run.md) | [飞行前检查 →](../06-first-flight/preflight-check.md)
