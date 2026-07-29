# 计划：D题中心 AprilTag 主定位

## 问题描述与目标

Cyber Camera 的蓝色方块检测受 GC2093 自动白平衡影响，第三次实飞出现明显偏色并导致有效检测率为零。现在用灰度 AprilTag 检测替代颜色阈值作为正式主定位，使白平衡漂移不再影响中心定位，同时保留现有蓝色方块测试模式和双圆环/十字正式图案检测作为独立模式或后备能力。

本轮目标限定为“检测与静态视觉伺服输入打通”：Cyber Camera 识别指定的单个中心 Tag，经现有 VS1 协议发送，RDK 能把 `APRILTAG_VALID` 观测送入现有四状态卡尔曼与安全视觉伺服。暂不改动态降落状态机，也不根据 Tag 姿态控制无人机偏航。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|----------|
| OpenCV `cv2.aruco` 的 AprilTag 字典 | Cyber Camera 已实测 OpenCV 4.10.0、`aruco` 和 `DICT_APRILTAG_36H11` 可用，无新增板端依赖 | 需要正确配置 Tag family 和 ID；远距最小像素尺寸仍需实测 |
| 安装 `apriltag`/`pupil_apriltags` | 专用解码器可调参数较多 | 板端当前未安装，增加部署、ABI和性能风险 |
| 继续蓝色阈值并做颜色自适应 | 可复用现有代码 | 仍依赖 ISP 输出，无法从根本上规避白平衡启动不稳定 |

**选择 OpenCV `cv2.aruco` 方案。** 原因是板端已验证可用，不增加安装步骤，且灰度检测能直接绕开颜色失真。已于 2026-07-29 在 Cyber Camera 执行能力检查，结果为 OpenCV 4.10.0、`cv2.aruco=True`、`DICT_APRILTAG_36H11` 和 `ArucoDetector` 均存在。默认使用已打印 Tag 对应的 `tag36h11`、ID 0；打印总尺寸150 mm，其中有效 Tag 尺寸120 mm。Tag ID仍可通过启动参数修改，但控制闭环只接受本次进程启动时配置的唯一 ID。

## 固定接口与安全策略

### 目标来源互斥

- RDK 配置增加 `vision_target_source`，只允许 `apriltag` 或 `blue_square`，默认 `apriltag`，不存在“同时接受”状态。
- `apriltag` 模式只接受 `APRILTAG_VALID`，明确拒绝 `SURROGATE_SQUARE`；`blue_square` 模式只接受 `SURROGATE_SQUARE`，明确拒绝 `APRILTAG_VALID`。两位同时出现视为协议异常并拒绝。
- 目标来源在进程启动时读取并固定，运行中不热切换；改变来源后必须重启 RDK 测试进程并重新执行通信与静态检测门禁。
- 两种来源都沿用连续3帧确认后才更新卡尔曼，切换进程或 Cyber Camera `stream_id` 变化会清空确认队列和跟踪器状态。

### AprilTag质量公式

对唯一的指定 ID 候选计算四条边长度 `s0..s3`、平均边长 `mean_side`、四角到最近画面边界的最小距离 `border`、四边形面积 `area` 和最小外接矩形面积 `rect_area`。先执行硬拒绝：四边形非凸、`mean_side < min_side_px`、任意角点越界、面积非正或指定 ID 不唯一时不置 `APRILTAG_VALID`。

默认 `min_side_px=18`，各分量均限制到 `[0,1]`：

- `size_score = clip((mean_side - 18) / 22)`，平均边长40 px达到满分；
- `margin_score = clip(border / (0.35 * mean_side))`；
- `consistency_score = clip(1 - (max(si)-min(si)) / (0.30 * mean_side))`；
- `shape_score = clip((area / rect_area - 0.55) / 0.35)`。

最终 `quality = round(100 * min(size_score, margin_score, consistency_score, shape_score))`，闭环默认要求 `quality >= 55`。该公式以最差几何分量限制总质量，不使用颜色信息。静态板测若显示真实1.5 m照片因正常透视被系统性压低，只允许基于数据调整配置阈值，不在代码中绕过硬拒绝或丢失归零。

### 多目标规则

- 只有“画面中解码出且通过几何硬拒绝的 Tag 恰好一张，并且其 ID 等于配置 ID”时才置 `APRILTAG_VALID`。
- 出现错误 ID、正确 ID与错误 ID共存、正确 ID多实例，全部输出 `found=False + AMBIGUOUS`；不选择最大候选。
- 部分遮挡但仍能解码的候选必须继续通过上述质量公式；质量不足时输出无效，不能依赖解码成功直接接管。

### 部署与回滚顺序

1. 飞机不上桨，先部署 RDK：RDK 能解析 bit 7，但仍按 `vision_target_source` 互斥拒绝未选来源；验证旧蓝方块 VS1 帧仍可解析且不会误接管。
2. 再部署 Cyber Camera：以 `--target apriltag --tag-family tag36h11 --tag-id 0` 启动，验证 VS1、PING/PONG和调试帧。
3. 两端版本验证通过后才允许装桨。任何一步失败，先停止视觉测试进程；Cyber Camera 回退到旧 `--target blue_square`，RDK 回退 `vision_target_source=blue_square`，两端重启并重新检查OLED双向OK。
4. 不允许 Camera 新版与 RDK 旧版进入实飞；版本不确定时按无视觉能力处理，不起飞。

## 改动范围

- `CyberCamera/boards/cybercam_d/detector.py`
  - 增加 `APRILTAG_VALID = 1 << 7`。
  - 增加单 Tag 检测器：BGR/灰度输入统一转灰度，只接受配置 family 和 ID。
  - 输出 Tag 四角中心、四边平均像素长度、平面航向、质量和调试多边形。
  - 目标 ID 不存在、多张同 ID、候选冲突或尺寸过小时输出无效/歧义，不默认选择最大 Tag。
- `CyberCamera/boards/cybercam_d/main.py`
  - 增加 `apriltag` 目标模式，以及 `--tag-family`、`--tag-id` 参数。
  - 调试图片沿用现有标注和每秒记录机制。
  - 初始化时检查 OpenCV AprilTag 字典能力；能力缺失则启动失败，不静默降级到颜色检测。
- `drone_control/competition_2026_d/vision/platform_observation.py`
  - 同步增加 `APRILTAG_VALID` 协议标志。
- `drone_control/competition_2026_d/static_square_servo.py`
  - 将现有安全伺服扩展为可配置接受 AprilTag；像素误差、卡尔曼、限速、限加速度、围栏和丢失归零逻辑保持不变。
  - 蓝色方块仅在显式测试配置下继续接受。
- `drone_control/competition_2026_d/static_square_flight.py`
  - 从配置读取允许的视觉目标类型，日志明确记录 AprilTag/蓝方块来源。
- `drone_control/competition_2026_d/config.json`
  - 增加 `vision_target_source="apriltag"`、AprilTag family、ID、`min_side_px=18` 和 `min_quality=55`；目标来源互斥，不改当前飞行速度和围栏参数。
- `drone_control/competition_2026_d/test_vision_core.py`
  - 生成合成 AprilTag 图验证中心、边长、航向、指定 ID、错误 ID、多目标歧义、彩色偏色不影响检测。
- `drone_control/competition_2026_d/test_static_square_servo.py`
  - 增加 `APRILTAG_VALID` 进入完整跟踪、蓝方块禁用时被拒绝、丢失后速度归零的回归测试。
- `docs/competition_2026_d/communication_protocol.md` 与 `docs/competition_2026_d/plans/01_cybercamera_vision.md`
  - 将 bit 7 从“预留”更新为已实现，并记录实际启动和验证门禁。

## 风险点

- 安全隐患：错误 Tag 或多 Tag 被误选会使无人机朝错误目标运动。画面中只有唯一指定 ID 且没有任何其他合格Tag时才接受；冲突时输出 `AMBIGUOUS`，RDK 归零。
- 边界条件：Tag 过小、出画、强反光、运动模糊或过曝时灰度检测仍会丢失。当前融合专项测试使用 `0.3 s` 新鲜度门限、0.8 m紧急半径和任务超时退出。
- 协议一致性：Cyber Camera 与 RDK 必须同时定义 bit 7；VS1 帧格式不变，避免影响串口兼容性。
- 坐标风险：沿用第二次实飞确认的 X 映射和已修正的 Y 映射；首次 AprilTag 实飞只做 10–15 cm 单轴偏移验证。
- 质量风险：OpenCV ArUco 不直接给统一的判决 margin。本轮严格使用“固定接口与安全策略”中的四分量最小值公式和55分阈值；调整必须来自板测数据并保留硬拒绝。
- 回退方案：启动参数切回 `--target blue_square` 或 `--target formal`；RDK 配置可只接受蓝方块。若 AprilTag 板端帧率不足，停止闭环，不降低安全门限强行实飞。
- 现有未提交修改：实现前后分别检查 `git diff`，在当前工作树上增量编辑，保留 `static_square_servo.py` 的Y误差 `observation.cy - image_cy`、`config.json` 焦距570 px和既有方向回归测试；不为了本功能回退或覆盖它们。因Y修正尚未获得有效视觉实飞数据，本轮不先行提交该基线。`drone_control/basic/fc_log.log`继续不纳入提交。

## 验证方式

- 单元测试：用 `cv2.aruco.generateImageMarker()` 生成 tag36h11 图像，覆盖正确 ID、错误 ID、旋转、偏色、两 Tag、过小 Tag、质量公式边界；协议 bit 7 往返解析；RDK 安全伺服来源门禁。合成图只验证算法与几何变换，不作为真实打印质量证据；板端静态测试必须保存至少一组真实打印 Tag 图作为集成验证基准。
- 板端静态测试：Cyber Camera 外接 CSI，关闭显示，用每秒一张调试记录验证 1.5 m 视角的中心、边长、质量、FPS和 VS1 连续性。
- 通信测试：RDK OLED 必须同时显示 `CAM>RDK:OK` 与 `RDK>CAM:OK`；确认 `parse_errors=0`、PONG正常、AprilTag帧 bit 7 正确。
- 首次闭环：高度 1.5 m，Tag 相对画面中心只偏移 10–15 cm，先测单轴；当前最大直接/回退速度为0.18/0.12 m/s，专项测试紧急半径0.8 m；视觉失效超过0.3秒交回T265保持。
- 实飞准入：起飞前静态连续检测至少 1 s、ID正确、质量过阈值、双向通信OK、T265置信度满足现有要求；任一不满足则不接管视觉速度。
