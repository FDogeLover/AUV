# 2026-07-30 D题 AprilTag＋蓝框融合视觉伺服

## 目标

验证“25 cm蓝色方形外框＋中心ID 0 AprilTag”的融合定位，并形成简单、连续的无人机XY视觉追踪流程。

## 完成项

- Cyber Camera新增 `apriltag_blue_fusion`：正确Tag负责首次身份确认，蓝色外框负责高频中心定位，颜色失效时回退Tag直接识别和短时光流。
- VS1新增 `APRILTAG_VALID` bit 7、`TEMPORAL_TRACKED` bit 8、`COLOR_SHAPE_TRACKED` bit 9；蓝色和光流均不能独立建立身份。
- RDK接受颜色融合观测，颜色/光流限速0.12 m/s，Tag直接观测最高0.18 m/s；保持加速度、jerk、观测新鲜度和错误ID门禁。
- 修正Y轴符号：画面上方对应无人机 `+Y`，使用 `error_y=240-cy`。
- 删除运行时0.5 m软围栏对正常追踪的干预；专项测试暂保留0.8 m紧急半径。
- 实现持续追踪：1.5 m确认Tag后立即启用XY控制，下降到1.0 m期间持续接管，到达1.0 m后继续居中。
- 增加只观察实飞入口：1.5 m悬停采数时视觉不输出XY速度，最终仍使用0.15 m零停留末航点。

## 实飞结论

### 1.5 m只观察悬停

- 14.97秒内记录152帧，序号缺口0，观测频率10.09 Hz。
- 有效检测149帧（98.0%）：颜色120帧、光流24帧、Tag直接5帧、完全丢失3帧。
- 最长完全丢失约0.25秒；平均接收年龄17.3 ms。
- 颜色连续帧中心变化中位数2.24 px，融合检测具备进入低速闭环的稳定性。

### 融合闭环实飞1

- 1.5 m未居中即下降到1.0 m，目标偏移随高度降低放大并部分出画，造成长时间丢失。
- 重获后X方向正确，Y方向错误，目标被推向画面上边缘。
- 结论：修正Y符号，并避免下降期间关闭XY追踪。

### 融合闭环实飞2

- 固定1.5 m验证XY，将像素误差从约227 px降至最小31 px，下降约86%。
- XY方向和收敛正确；最大速度0.12 m/s，没有触发0.6 m旧硬围栏。
- 未满足严格居中保持条件的主要原因是旧0.5 m软围栏限制继续向目标移动。

## 当前状态

- 本地融合相关测试：`60 passed`；`compileall`和`git diff --check`通过。
- Cyber Camera与RDK运行代码已经分别部署到 `/home/pi/FJJ/cybercam_d/` 和 `/home/sunrise/Desktop/FJJ/competition_2026_d/`。
- 持续追踪并同步下降版本尚未实飞；不能把代码通过或板端编译通过写成飞行验证完成。

## 下一步

1. 实飞验证1.5 m确认后立即追踪，并在下降到1.0 m期间保持目标连续可见。
2. 核对到达1.0 m后视觉仍持续接管，中心误差继续下降且不会被基础T265航点拉回。
3. 验证视觉完全失效超过0.3秒时保持当前位置，重获后恢复追踪。
4. 静态持续追踪通过后，再使用真实小车测试直线低速伴飞。

## 相关文件

- `CyberCamera/boards/cybercam_d/detector.py`
- `CyberCamera/boards/cybercam_d/main.py`
- `drone_control/competition_2026_d/static_square_servo.py`
- `drone_control/competition_2026_d/static_square_flight.py`
- `docs/competition_2026_d/communication_protocol.md`
- `docs/competition_2026_d/plans/01_cybercamera_vision.md`
- `docs/competition_2026_d/plans/02_uav_control.md`
