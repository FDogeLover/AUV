# T-016 Basic 版本完整系统测试

2026-07-26 完成三组标准路径实飞测试（长/短/高）

- **关键发现**：短路径(0.5m)和高度变化(0.5/1.0/1.5m)全部 precision_arrival 到达，XY ≤9cm；长路径(2m 矩形)远角因 arrival_timeout_max=6.5s 偏短，WP2~WP4 以 timeout 跳点，XY 偏差 15~17cm 略超阈值
- **影响范围**：`drone_control/basic/` 导航模块；`arrival_timeout_max` 参数；`router_tests/` 标准测试航路
- **后续动作**：短距/常规飞行不受限，暂不调参，后续需要时加大 timeout

## 测试数据

| 测试 | 通过 | 关键数据 |
|------|------|---------|
| A-长路径 2m×2m | ❌ 部分 | 回到原点 17cm(阈15cm)，4/6 timeout |
| B-短路径 0.5m | ✅ | 全部 precision_arrival，XY 2~5cm |
| C-高度变化 | ✅ | 全部 precision_arrival，XY 3~9cm |

三次降落均成功锁桨，航向保持正常，Yaw 漂移 <2°。

## 相关文件

- 测试航路: `drone_control/basic/router_tests/router_t016_{long,short,height}.txt`
- 运行脚本: `drone_control/basic/run_t016_test.sh`
- 分析脚本: `tools/analyze_t016.py`
- 日志归档: `drone_control/tools/data_archive/test_data_20260726/`
- TODO 详情: `docs/TODO.md` → T-016
