# 光流 vs T265 手持晃动测试 — 设计

## 背景

之前用 `analyze_of_t265_correlation.py` 分析真实飞行日志（81个有效样本）得出"交叉轴相关性峰值 0.24 > 同轴相关性峰值 0.15"，初步支持 CLAUDE.md 已知问题第2条"光流坐标系约90°旋转"的假设，但样本来自一次普通飞行任务，晃动幅度小、样本量刚过警戒线（30），结论强度有限。需要一次专门设计的、动作更充分的手持晃动测试来获得更干净的数据验证同一假设。

## 目标

采集一段专门的手持晃动数据，用现有分析脚本重新验证坐标旋转假设，不引入新的分析逻辑。

## 关键约束：日志采样频率

`drone_control/basic/Mission_GPT.py` 中 `FLIGHT_LOG_INTERVAL = 1.0`（秒），日志写入频率固定 1Hz，与飞控下行帧的实际到达率（~50Hz）无关，也与 `DRONE_DRY_RUN` 无关（DRY_RUN 只影响是否解锁电机，不影响状态机和日志线程）。手持晃动通常包含 1-3Hz 的运动分量，1Hz 采样会造成欠采样/混叠，使相关系数失真。

**处理方式**：测试前临时把 `ubuntu-pi` 上部署的 `basic/Mission_GPT.py` 里这一常量从 `1.0` 改成 `0.05`，测试完成后改回 `1.0`。这是唯一涉及的代码改动，可逆，不提交到本机仓库（只改 `ubuntu-pi` 上的部署副本）。

## 测试流程

1. **改小日志间隔**：SSH 到 `ubuntu-pi`，编辑 `~/Desktop/FJJ/basic/Mission_GPT.py`，把 `FLIGHT_LOG_INTERVAL = 1.0` 改成 `FLIGHT_LOG_INTERVAL = 0.05`
2. **清空旧日志**：把 `~/Desktop/FJJ/basic/flight_data.jsonl` 重命名为 `flight_data_prev.jsonl.bak`（避免新旧数据追加混淆），保留旧文件而非删除
3. **启动采集**：在 `ubuntu-pi` 上 `cd ~/Desktop/FJJ/basic && DRONE_DRY_RUN=1 python main.py`（DRY_RUN 模式，电机不解锁，纯采集姿态/T265/光流数据）
4. **执行动作**：手持机体做连续混合方向晃动（不区分轴，各方向随意动），持续约 20-30 秒，保持光线充足以维持 T265 视觉跟踪不丢失
5. **停止采集**：`Ctrl+C` 停止脚本（或等状态机自然走完 `router.txt` 航点）
6. **拷回数据**：`scp` 把 `~/Desktop/FJJ/basic/flight_data.jsonl` 拷贝到本地
7. **还原日志间隔**：把 `ubuntu-pi` 上的 `FLIGHT_LOG_INTERVAL` 改回 `1.0`
8. **分析**：本地用已有的 `drone_control/tools/analyze_of_t265_correlation.py` 跑拷回的日志，对比新旧两次结论是否一致

## 非目标

- 不新增分析脚本功能，不新增采集脚本
- 不做轴分段标记（用户已确认不需要区分单轴晃动阶段）
- 不永久修改 `FLIGHT_LOG_INTERVAL` 默认值
- 不修改飞控固件或协议

## 执行者与角色划分

- 步骤 1、2、3、7（SSH 操作、改常量、清理旧日志、还原常量）：可由 Claude 直接执行（限定路径 `~/Desktop/FJJ/basic/` 内，改文件前后各用只读命令验证）
- 步骤 4（物理晃动机体）：必须由用户本人完成，Claude 无法代劳
- 步骤 5、6、8：视情况由 Claude 或用户配合完成（Claude 可执行 scp 拷贝和跑分析脚本，用户负责在物理测试期间告知"开始"/"结束"）

## 验证

测试完成后跑 `analyze_of_t265_correlation.py`，确认：
- 有效样本数明显高于上次的 81（因为采样率从 1Hz 提到 20Hz，同样 20-30 秒时长应有几百个样本）
- 表格数值仍在 `[0,1]` 范围，无崩溃
- 结论文字与上次方向是否一致（交叉轴 vs 同轴哪个更强），作为对假设的二次验证
