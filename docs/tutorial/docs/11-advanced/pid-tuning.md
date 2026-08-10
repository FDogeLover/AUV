# PID调参指南

## 当前参数（basic版，已验证）

以下参数定义在 `Lcode/Lpid.py` 中，与代码完全一致：

| 控制环 | Kp | Ki | Kd | 输出限幅 | 来源 |
|--------|-----|-----|-----|---------|------|
| XY位置环 (`type=0`) | 0.7 | 0.002 | 0.05 | ±40 cm/s | T-016验证 |
| Yaw角速度环 (`type=1`) | 1.5 | 0.0 | 0.3 | ±30 °/s | T-016验证 |

!!! info "Z高度控制"
    Z 轴没有独立的 PID 参数，复用 `type=0`（XY 位置环）的参数。高度设定值通过 `_step_ramp_z()` 做 ramp 渐变，每周期步进 1.5cm。

!!! warning "航向保持参数注意"
    航向保持（`heading_hold.py`）的 dataclass 默认值为 `kp=0.5, max_rate_dps=3`（#45 修复后的新值），但 `from_env()` 的环境变量默认值仍为旧值 `kp=0.25, max_rate_dps=1`。Mission_GPT 使用 `HeadingHoldConfig.from_env()` 构造配置——如果不设置 `DRONE_HEADING_HOLD_KP` 和 `DRONE_HEADING_HOLD_MAX_DPS` 环境变量，实际生效的是旧值 0.25 和 1。详见 [航向保持模块文档](../07-architecture/modules/heading-hold.md)。

## 调参建议

1. **先短距离测试**（0.5m以内），确认基本能到目标
2. **XY先调P**：太小到不了，太大会振荡。找到"刚好能到但不振荡"的值
3. **加D抑制振荡**：D太大会抖动，太小会有超调
4. **Z高度单独调**：高度控制独立于XY，可以单独测试
5. **航向保持最后调**：先确保XY/Z正常，再开启航向保持

## IMU融合参数

凌霄IMU内部EKF有4个融合参数（姿态/罗盘/水平面/垂直）。详见 `docs/guides/imu_parameters_and_fusion_architecture.md`。

!!! tip "罗盘遇干扰可置0"
    罗盘融合参数遇干扰可置0，消除偏航旋转。详见IMU参数文档。

## T-016 测试结果参考

| 测试项 | 结果 |
|--------|------|
| 短路径(0.5m) XY精度 | 2~5cm ✅ |
| 高度变化(0.5~1.5m) XY精度 | 3~9cm ✅ |
| 长路径(2m矩形) 回原点 | 17cm（超时导致）❌ |
| 三次降落锁桨 | 均成功 ✅ |
| 航向保持 | 正常，Yaw漂移<2° ✅ |

---

[推荐阅读顺序 →](further-reading.md)
