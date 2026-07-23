# yaw_sign 标定工具设计

## 背景

`PoleTracker`（`drone_control/basic_radar/Lcode/Lradar.py`）的世界系坐标转换依赖 `yaw_sign` 参数（+1 或 -1），因为 `t265.py` 内部经过多层轴重映射+取反+欧拉角提取，`get_orientation()[2]` 的符号约定不是标准数学 CCW 正角度，具体该用哪个符号从设计阶段起就一直未标定（见 [2026-07-08-pole-tracker-world-frame-design.md](2026-07-08-pole-tracker-world-frame-design.md)）。`Mission_GPT.py` 里的 `POLE_YAW_SIGN` 常量目前是假设值 `1`，直接决定雷达悬停避障功能算出来的世界坐标准不准。

这次真机测试要标定这个值，需要一个专门的诊断工具。

## 设计

新建 `drone_control/basic_radar/calibrate_yaw_sign.py`，风格跟已有的 `radar_bench_test.py` 一致——纯诊断脚本，不解锁飞控、不用电机，只连雷达+T265两个传感器，可以在台架上做（雷达+T265刚性固定在一起，模拟真实装机状态，原地手动转动）。

**核心逻辑**：每次循环取雷达最近点（`radar.get_nearest()`，实时反馈，不经过 `PoleTracker` 多帧确认，避免转动测试时延迟）+ T265 当前位姿（`get_position()`/`get_orientation()[2]`），分别用 `yaw_sign=+1` 和 `yaw_sign=-1` 调用 `body_to_world_xy()` 算出两组候选世界坐标，实时打印对比。

**判定方法**：操作者把雷达对准一个固定目标，原地转动机体（不平移），观察哪一组世界坐标基本不随转动变化——那组对应的符号就是正确的 `yaw_sign`。

**输出格式**：单行滚动刷新（`\r` + `flush=True`，跟 `Mission_GPT.navigate()` 终端输出风格一致），包含当前机体位姿(x,y,yaw)和两组候选世界坐标。

**不需要单元测试**：`body_to_world_xy()` 本身已经在 `test_pole_tracker.py` 里覆盖过正确性（正负号、往返变换），这个脚本只是接线+循环+打印，没有独立可测的逻辑分支。

## 使用后的操作

标定完成后，操作者根据观察结果手动把 `drone_control/basic_radar/Mission_GPT.py` 里的 `POLE_YAW_SIGN` 常量改成正确值（+1 或 -1），这一步是人工决策，不在本次工具的自动化范围内。

## 不在本次范围内

- 不做平移测试（原地转动足够标定符号，不需要移动位置）
- 不自动更新 `POLE_YAW_SIGN`（人工读数、人工改常量）
- 不改动 `PoleTracker`/`Mission_GPT.py` 现有逻辑
