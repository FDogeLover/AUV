# 会话总结 — 2026-07-24

**分支：** `main`
**提交：** `1b70f41`
**会话目标：** 视觉伺服精准降落模块的完整设计、审核、实现、校验

---

## 📋 完成项

- **创建 `docs/TODO.md`** — 待办事务总览系统，含重要性(I)×紧急性(U)=优先级(P)评分，14条→精简为9条活跃待办
- **整理 `docs/` 目录** — 新建 `architecture/` 和 `guides/` 子目录，根目录散落文件归类
- **视觉伺服精准降落模块（完整 qoder-workflow 6步）**：
  - `vision/servo_controller.py` — tick-based IBVS 控制器，接受 `Detection` 对象
  - `vision/cyber_cam_reader.py` — UART 后台线程读取 + ASCII 协议解析
  - `vision/square_detector.py` — 桌面调试用 OpenCV 黑色方块检测
  - `Mission_GPT.py` — 新增 `VISUAL_SERVO` 原生状态，30ms tick 内同步执行
  - 20 个单元测试全部通过
  - Qoder 实现审查：[实现符合计划]
- **CyberCamera 目录重组**：
  - `boards/cybercam/` — Cyber CAM（核桃派）端代码（main.py, detector.py, protocol.py, calib.py）
  - `k230/` — 已有 K230 代码
- **架构修正**：Cyber Camera 是独立核桃派板（UART 连接），非 USB 摄像头
  - 板上确认：USB 普通摄像头（Realtek 1280×720）+ IMX219 CSI 摄像头，均无处理模块
  - CP210x UART Bridge 是 Cyber Camera 的连接口预留
- **更新 CLAUDE.md**（.claude 和 .Codex 两处）— 索引添加视觉伺服引用
- **更新 TODO.md** — T-004 状态改为「待部署」
- **更新 ZCode 记忆** — 新增 visual-servo topic 记忆
- **更新 `sync-to-ubuntu-pi.sh`** — 增加 `vision/` 子目录同步

## 🚧 未完成 / 待办

- [ ] Cyber Camera（核桃派）到货后部署 `boards/cybercam/` 代码
- [ ] 跑 `calib.py` 标定 `focal_length_px`
- [ ] 台架验证 UART 读帧 + 完整视觉伺服回路
- [ ] T-007 串口断线重连通知
- [ ] T-008 串口路径硬编码 → 配置文件

## 🔧 修改的文件

### 新增文件
- `CyberCamera/README.md` — 目录说明 + 通信协议
- `CyberCamera/boards/cybercam/main.py` — 入口：捕获→检测→UART 发送
- `CyberCamera/boards/cybercam/detector.py` — 黑色方块检测（1920×1080）
- `CyberCamera/boards/cybercam/protocol.py` — ASCII 协议编解码
- `CyberCamera/boards/cybercam/calib.py` — 焦距标定工具
- `drone_control/competition_2026/vision/__init__.py`
- `drone_control/competition_2026/vision/servo_controller.py` — tick-based IBVS
- `drone_control/competition_2026/vision/cyber_cam_reader.py` — UART reader
- `drone_control/competition_2026/vision/square_detector.py` — OpenCV 检测
- `drone_control/competition_2026/vision/test_servo_controller.py` (10 tests)
- `drone_control/competition_2026/vision/test_square_detector.py` (10 tests)
- `docs/TODO.md` — 待办事务总览

### 修改文件
- `drone_control/competition_2026/Mission_GPT.py` — VISUAL_SERVO 状态 + speed limit ±20cm/s
- `sync-to-ubuntu-pi.sh` — 增加 vision/ 子目录同步
- `.claude/CLAUDE.md` — 索引 + 已知问题速查更新
- `.Codex/CLAUDE.md` — 索引更新
- `docs/architecture/competition_2026_airborne_architecture.md` — 新增 5.3 视觉伺服
- `docs/TODO.md` — T-004 状态更新

## 💡 下一步建议

1. Cyber Camera（核桃派）到货后，部署 `boards/cybercam/` 代码并验证
2. 桌面测试：用 USB 摄像头 + `FrameDetectorAdapter` 验证检测收敛逻辑
3. 推进 T-011 台架预检 + T-012/T-013 飞行测试
4. T-008 串口路径硬编码→配置文件（小改动，大收益）

## 🔗 相关上下文

- Qoder 审查：视觉伺服 v3 计划 + 实现均通过
- 板上确认有 USB 普通摄像头 + IMX219 CSI 摄像头
- Cyber Camera（核桃派）通过 CP210x UART Bridge 连接，预计
- sync 脚本已支持 `vision/` 子目录

## 📊 Git 状态

- 当前分支：`main`
- 相比开始的新提交：4（6332543, c32a471, 86a8c38, c4dafef 等）
- 未提交改动：CLAUDE.md 更新（.claude/.Codex）+ TODO.md 状态更新
