# 计划：货架盘点扫描路径优化

## 问题描述 & 目标

当前每面 6 个货位的扫描路径按列交替升降：扫完一个列的上层→降到下层→升到下一列上层→...，6 个点高度变化 5 次，每次升降约耗时 3~5 秒且额外耗电。

目标：
1. **路径 S 形化**：一排扫完再换行，只升降 1 次，减少无效飞行距离
2. **到位即扫**：到达目标位置后不等待精确定位确认，立即开始扫码，缩短单点耗时

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|----------|
| **A: 仅改扫描顺序** | 改动小、风险低 | 不减少单点等待时间 |
| **B: 改顺序 + 到位即扫** | 省高度变化 + 省确认时间 | 需改两处逻辑，到位即扫可能被T265漂移影响 |

**选择方案 B**：两项优化一起做，改动范围可控。

### 扫描路径设计（以 A 面为例）

```
当前（列优先）：
  A1(顶,-1.75) → A4(底,-1.75) → A5(底,-1.20) → A2(顶,-1.20) → A3(顶,-0.70) → A6(底,-0.70)
  高度变化：1.25→0.85→0.85→1.25→1.25→0.85  （5次变化）

优化后（行优先+S形）：
  A1(顶,-1.75) → A2(顶,-1.20) → A3(顶,-0.70) → 下降 → A6(底,-0.70) → A5(底,-1.20) → A4(底,-1.75)
  高度变化：1.25→1.25→1.25→0.85→0.85→0.85  （1次变化）
```

## 改动范围

### 1. `Lcode/inventory_planner.py` — `_scan_slots()` 方法

当前按列分组（先遍历 X，列内交替 Z），改为按行分组（先遍历 Z，行内遍历 X，第二行反转 X 方向实现 S 形）。

```python
# 当前逻辑（伪代码）：
for each column X:
    for top, bottom:  # 交替高度
        scan

# 改为：
for each row Z:
    for each column X (正向或反向):  # 第二行反向
        scan
```

改动函数：
- `_scan_slots()` — 排序逻辑重写
- `_append_face()` — 确认首个航点选择逻辑兼容新顺序

### 2. `Lcode/inventory_controller.py` — 到位即扫

当前 `_advance_waypoint()` 流程：
```
到达目标位 → on_waypoint_arrived → 状态机推进 → start_scan_hold → 开始扫码
```

到位即扫：在 `on_waypoint_arrived` 中调用 `start_scan_hold` 时不等待 `_inspect_slot` 中的 arrival 确认周期，直接进入 `VISUAL_ALIGN → VERIFY_QR`。

改动函数：
- `_inspect_slot()` — 去掉不必要的 arrival 确认延迟
- 或：在 `on_waypoint_arrived` 中对 INSPECT 航点直接返回 `ENTER_SCAN`，跳过 `APPROACH_SLOT` 状态

### 3. 测试文件

- `test_warehouse_model.py` — 更新预期扫描顺序
- `test_inventory_controller.py` — 更新状态转移测试
- `test_inventory_planner.py` — 新增行优先排序测试（如果存在）

## 风险点

1. **S 形第二行反转方向时可能经过已扫过的货位** → 但视觉上 QR 码在不同行，不会重复识别，且 InventoryStore 去重
2. **到位即扫可能因 T265 漂移在错误位置开始扫码** → 扫码本身有 8 秒超时，且 `require_laser_inside=False` 时接受范围较宽，风险可控。如果漂移太大，8 秒超时后自动跳到下一格
3. **高度变化次数减少但不等于零** → 1 次升降在 1.25m ↔ 0.85m 间，约 4~6 秒，不影响安全

**回退方案**：改回 `_scan_slots()` 原逻辑只需 revert `inventory_planner.py` 的单文件改动。到位即扫可回退 `_inspect_slot()` 改动的几行。

## 验证方式

1. **单元测试**：`test_warehouse_model.py` + `test_inventory_controller.py` + `test_inventory_planner.py` 全部通过
2. **桌面验证**：`plan_face(A/B/C/D)` 输出路径人工确认 S 形正确
3. **真机验证**：A 面 6 点复飞，对比之前 4/6 成功率是否有提升
