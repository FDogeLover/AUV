# 2026-07-23 competition_2026 完整地面准备 — 从 Codex 中断到板载硬件验证

## 第一阶段：接续 Codex 中断的工作

Codex 会话 `019f8766-d678-7d20-bd43-cb8d9bdc4a86`（2026-07-22~23）因用量配额耗尽中断，
当时 airborne_video.py、video_backends.py 和 test_airborne_video.py 已写入磁盘但未验证提交。
后续其他会话补充了 action_executor、drone_link、mission_outcome、preflight 等模块，
以及 gpio_led/mission_events 的修改——均未提交。

### 完成

1. ✅ 验证所有文件完整性（14个文件涉及新增/修改）
2. ✅ 运行全套测试：138 passed, 1 skipped
3. ✅ 编译检查：`compileall -q` 通过
4. ✅ 更新 README
5. ✅ 提交并推送（`933d813`）

## 第二阶段：架构文档

- `docs/architecture/competition_2026_airborne_architecture.md`（504 行）
  - 设计原则、模块总览、启动/关闭时序、事件总线、可选后台服务
  - 通信协议（UDP 遥测帧 + UDP-JPEG 分片）、安全边界、配置参考

## 第三阶段：板载硬件测试脚本

### `hardware_preflight.py` — 只读硬件预检
检查 Python 版本、GPIO 模块、T265 枚举、飞控串口收帧、磁盘空间、可执行文件。
绝不发送解锁/起飞指令。退出码 0 = 全部通过。

### `link_hardware_check.py` — UDP 链路测试
本机回环收发、CRC 校验、JPEG 分片编解码、事件序列化、HMAC 签名、可选局域网对端。
（回环测试接收端 CRC 偏移 bug 在板载测试中发现并修复。）

### `video_hardware_check.py` — 摄像头测试
OpenCV 可用性、摄像头枚举、帧读取、帧率统计、可选 JPEG 截图保存。

### `sync-to-ubuntu-pi.sh` 扩展
新增 `comp` / `all` 目标，支持同步 competition_2026 目录到板子。

## 第四阶段：板载验证（ubuntu-pi）

| 项目 | 结果 |
|------|------|
| 全量单元测试 | ✅ 139 passed |
| T265 枚举 | ✅ Intel RealSense T265 (serial: 929122110888) |
| 飞控串口 | ✅ /dev/ttyS6，99 samples/5s |
| GPIO 模块 | ✅ RPi.GPIO |
| UDP 回环测试 | ✅ 全部 PASS（含 CRC 修复后） |
| 事件序列化 | ✅ 153B round-trip |
| 磁盘空间 | ✅ 20 GB 剩余 |

## 最终提交链

```
0bf71ab fix: hardware_preflight 飞控检测改用非零字段 + pytest降级可选
9ab9edf fix: UDP回环CRC偏移修复 + sync脚本扩展
e98cbd0 feat: 板载硬件测试脚本 x3
6e4b365 docs: competition_2026 无人机端架构文档
933d813 feat: 机载视频/动作执行/预检/结果跟踪/链路模块
e674650 docs: 更新备赛架构完成状态
```

## 模块状态

| 模块 | 文件 | 状态 |
|------|------|------|
| VideoSource 接口 | `Lcode/video_source.py` | 已有 |
| 机载视频生命周期 | `Lcode/airborne_video.py` | Codex 写入 |
| OpenCV/UDP-JPEG 后端 | `Lcode/video_backends.py` | Codex 写入 |
| 到达点位截图 | `Lcode/waypoint_snapshot.py` | 已有 |
| 航点事件总线 | `Lcode/mission_events.py` | 已有 + 扩展 |
| 任务会话 | `Lcode/mission_session.py` | 已有 |
| 航线规划 | `Lcode/competition_plan.py` | 已有 |
| 动作执行器 | `Lcode/action_executor.py` | 新增 ✓ |
| 飞行前检查 | `Lcode/preflight.py` | 新增 ✓ |
| 任务结果跟踪 | `Lcode/mission_outcome.py` | 新增 ✓ |
| 无人机链路 | `Lcode/drone_link.py` | 新增 ✓ |
| GPIO LED（含租约） | `Lcode/gpio_led.py` | 已有 + 增强 |
| 板载硬件预检 | `hardware_preflight.py` | 新增 ✓ |
| UDP 链路测试 | `link_hardware_check.py` | 新增 ✓ |
| 摄像头测试 | `video_hardware_check.py` | 新增 ✓ |
