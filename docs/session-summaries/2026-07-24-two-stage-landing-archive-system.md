# 会话总结 — 2026-07-24

**分支：** `main`
**提交：** `1b70f41`
**会话目标：** 解决罗盘干扰导致的偏航旋转问题 → IMU参数理解 → 降落可靠性改进（两级下降）→ 航向保持修复 → 飞行日志工具链 → 文档体系精简

---

## 📋 完成项

### 架构理解
- ✅ 凌霄IMU 四个融合参数（姿态/罗盘/惯导水平/惯导垂直）的作用范围已梳理
- ✅ 确认飞控侧不做高度二次融合，IMU 不输出高度值只输出 vel_z
- ✅ 创建 `docs/guides/imu_parameters_and_fusion_architecture.md`

### 两级降落（DESCEND/HOVER_WAIT）
- ✅ 新增 DESCEND 状态：水平开环、ramp 递减高度(~15cm/s)、专用 is_near_ground() 贴地检测
- ✅ 新增 HOVER_WAIT 状态：超时后悬停等人工介入，不关串口
- ✅ land() 改为循环发 se_fc[7]=101(FC_Lock)+双确认，不再依赖 OneKey_Land CMD
- ✅ Qoder 两轮审查通过（Plan Review 3🔴+3🟡+2🟢 → Implementation Review 4偏差）
- ✅ 真机验证（3次飞行+矩形路径）：成功锁桨，Yaw 正常
- ✅ 67 个单元测试全部通过

### 航向保持修复
- ✅ kp=0.25→0.5, max_rate_dps=1→3, runaway_growth_deg=3.0→15.0
- ✅ 真机验证：全程 heading_fault_reason=null，Yaw 漂移仅 +1.5°

### 飞行日志工具链
- ✅ `tools/flight_log_analyzer.py` — 一键分析 flight_data.jsonl
- ✅ `tools/pull_flight_log.sh` — 从板子拉取日志到本地归档，自动清空板子
- ✅ 数据只存在本地，板子不留副本

### 历史数据迁移
- ✅ 板子 `~/Desktop/FJJ/test_data/` (~416MB, 2026-07-05 至 2026-07-22) 全部拉到本地
- ✅ 清理板子空间

### 文档体系精简
- ✅ `.claude/CLAUDE.md` 从 ~230 行瘦身到 ~40 行
- ✅ `.Codex/CLAUDE.md` 同样精简
- ✅ `docs/AGENTS_GUIDE.md` 作为新 Agent 统一入口
- ✅ `.Codex/memory/` 48 → 50 文件，索引全覆盖
- ✅ 修复 known_issues 编号冲突（#8/#9 → #45/#46）

## 🚧 未完成 / 待办

- [ ] T-015: 两级降落同步到 basic_radar / competition 版本
- [ ] T-016: Basic 版本完整系统测试（长/短/高三组路径，明天执行）

## 🔧 修改的文件

### 上位机 (Python)
- `drone_control/basic/Mission_GPT.py` — 新增 DESCEND/HOVER_WAIT 状态、descend()、hover_wait()、is_near_ground()
- `drone_control/basic/Lcode/heading_hold.py` — KP/max_rate/runaway_growth 调优，KP上限 0.5→1.0
- `drone_control/basic/test_land_logging.py` — 适配新 landing 逻辑 + 9 个新测试
- `drone_control/basic/test_heading_hold.py` — 适配新默认参数
- `drone_control/basic/router.txt` — 矩形路径等测试配置

### 工具
- `tools/flight_log_analyzer.py` — 新建
- `tools/pull_flight_log.sh` — 新建
- `tools/analyze_flight.py` — 新建（旧版，已被 flight_log_analyzer.py 替代）

### 文档
- `docs/AGENTS_GUIDE.md` — 新建，Agent 统一入口
- `docs/guides/imu_parameters_and_fusion_architecture.md` — 新建
- `.zcode/plans/two_stage_landing_plan.md` — 新建
- `docs/known_issues.md` — 新增 #45(航向保持)、#46(两级降落)
- `docs/TODO.md` — 新增 T-015、T-016
- `.claude/CLAUDE.md` — 精简重构
- `.Codex/CLAUDE.md` — 精简重构
- `.Codex/memory/` — 新增3文件 + 更新2文件

## 💡 下一步建议

1. **明天执行 T-016**：长路径(2m矩形) → 短路径(0.5m) → 高度变化(0.5→1.5m)，用 flight_log_analyzer.py 逐次分析
2. 验证通过后将两级降落同步到 `basic_radar/` 和 `competition_2026/`

## 📊 Git 状态

- 当前分支：`main`
- 相比上次总结的新提交：17 个
- 未提交改动：无（clean）
