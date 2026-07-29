# 计划：AprilTag与蓝色外框融合跟踪

## 目标

利用现有“25 cm蓝色正方形＋中心15 cm AprilTag”标志：Tag负责确认唯一身份，蓝色外框负责高频中心定位，解决1.0 m阶段Tag无法连续解码的问题。

## 方案

- 未锁定身份：只接受配置ID的AprilTag，蓝色不得独立进入闭环。
- 正确Tag直接解码后：锁存本次进程的目标身份。
- 锁定后：优先输出唯一、完整且位置连续的蓝色方形中心；标志为 `APRILTAG_VALID|COLOR_SHAPE_TRACKED`。
- 蓝色无效：回退到Tag直接/光流观测；全部无效则LOST。
- 错误ID、多Tag、多个蓝色候选、中心跳变或Tag/蓝框几何不一致均不得输出颜色融合结果。

## 改动范围

- Cyber `detector.py/main.py/test_detector_safety.py`
- RDK `platform_observation.py/static_square_servo.py/static_square_flight.py/test_static_square_servo.py`
- `config.json`与通信协议文档

## 安全边界

- 颜色模式不能独立建立身份。
- 颜色模式速度上限0.12 m/s，仍受加速度和jerk限制。2026-07-30持续追踪版本已关闭固定接管点软围栏，专项测试仅保留0.8 m紧急半径。
- 完全失效后T265保持丢失瞬间位置。
- 可将Cyber目标切回`apriltag`快速回退。

## 验证

- 合成组合标志测试身份锁定与颜色输出。
- 颜色单独出现、错误ID和多候选必须拒绝。
- 两端pytest/compileall与板端链路测试。
