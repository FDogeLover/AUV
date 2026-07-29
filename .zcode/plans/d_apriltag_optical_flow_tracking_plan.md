# 计划：AprilTag 解码与光流连续跟踪

## 问题描述与目标

Cyber Camera 在实飞中会因运动模糊、透视和画面边缘造成 AprilTag 间歇解码失败，RDK 因而频繁在视觉控制和 T265 保持之间切换并产生抖动。目标是在不放宽长期失联保护的前提下，用短时光流续跟桥接解码空隙。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|---|---|---|
| 单纯延长 LOST 门限 | 改动小 | 长期使用旧目标位置，不适合移动目标 |
| 提高 AprilTag 检测分辨率 | 可能提升单帧识别率 | FPS下降，仍会间歇丢帧 |
| AprilTag + LK光流 | 身份由Tag确认，帧间连续且计算轻 | 需要限制漂移和跟踪时长 |

选择 AprilTag + LK光流。只有直接解码 ID=0 后才能初始化；光流不能独立确认身份。

## 改动范围

- `CyberCamera/boards/cybercam_d/detector.py`：加入特征点光流、RANSAC仿射估计、定期Tag重校准、跟踪失效保护。
- `CyberCamera/boards/cybercam_d/main.py`：增加光流配置参数和调试标注。
- `CyberCamera/boards/cybercam_d/test_detector_safety.py`：覆盖直接识别、光流续跟、错误ID拒绝。
- `drone_control/competition_2026_d/vision/platform_observation.py`：增加 `TEMPORAL_TRACKED` 协议标志。
- `drone_control/competition_2026_d/static_square_servo.py`：光流帧使用低速模式，直接Tag仍负责初始化。
- `drone_control/competition_2026_d/test_static_square_servo.py`：覆盖时序跟踪低速和来源互斥。
- `docs/competition_2026_d/communication_protocol.md`：记录新增标志位。

## 风险点

- 光流漂移：最长0.5秒，定期直接解码校正，RANSAC内点不足立即失效。
- 错误身份：错误ID、多Tag或歧义帧立即清除跟踪状态。
- 出画/尺度异常：四角越界、仿射尺度异常或前后向误差过大立即LOST。
- 飞行风险：光流帧速度上限0.05 m/s；完全失效后沿用T265当前位置保持。
- 回退：关闭光流参数或恢复原 `AprilTagDetector` 即可返回逐帧解码模式。

## 验证方式

- 单元测试：合成Tag平移帧验证光流中心与标志；错误ID和多Tag不得续跟。
- 快速测试：运行 Cyber Camera 与 competition_2026_d 全部pytest及compileall。
- 板端测试：先观察检测连续率/FPS，再进行静态偏置实飞，对比FULL_TRACK/LOST切换次数和速度抖动。
