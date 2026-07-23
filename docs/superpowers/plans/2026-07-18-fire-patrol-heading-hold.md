# fire_patrol 起飞航向保持修改方案

> 日期：2026-07-18  
> 状态：已实施并完成两轮开启组真机复测；用户按当前赛题肉眼效果验收，2026-07-18决定默认开启并推广到basic
> 范围：仅 `drone_control/fire_patrol/`；不修改飞控固件、串口协议和其他任务版本

## 1. 目标

让无人机在起飞、巡航、火情接近、悬停抛投和恢复巡航期间保持起飞时的机头方向，抑制可见 yaw 自转，同时满足以下约束：

- 不改变已经用真机日志验证过的 XY 指令坐标关系；
- 不修改凌霄 IMU 或 STM32 固件；
- T265 异常时自动退回 `yaw_dps=0`，不阻断原有基本飞行；
- 未经真机 A/B 验证前默认关闭，不能直接成为正式飞行默认行为；
- LAND、END 和 emergency 阶段不启用外层航向保持。

> 后续决策：下述“默认关闭、完成3+3后再启用”是方案制定时的保守门槛。实际完成1次关闭基线和2次开启复测后，定量结果呈“净漂移和典型横向误差改善、yaw全范围与最坏峰值未稳定改善”；用户结合两轮肉眼无明显yaw旋转，按本赛题目标接受并决定默认开启。安全回退`DRONE_HEADING_HOLD=0`及全部故障保护保持不变。

## 2. 已确认的根因线索

当前代码存在两套不一致的 yaw 指令符号：

```python
# takeoff(): 直接发送 PID 输出
self.se_fc[6] = vyaw + sp_side

# navigate(): PID 输出又被取反一次
self.set_speed(vx, vy, -vyaw, int(self._ramp_z_cm))
```

`simple_pid` 的输出是 `Kp * (setpoint - current)`。当目标为 `0°`、当前 yaw 为 `+5°` 时，PID 已经输出负角速度；`navigate()` 再取负会把它变成正角速度，使偏差继续增大。

现有证据链一致支持“额外负号错误”这一判断：

1. 转盘测试确认 T265 yaw 正值代表逆时针；
2. 非闭环真机脉冲确认正 `yaw_dps` 产生正 yaw、负 `yaw_dps` 产生负 yaw；
3. `simple_pid(setpoint=0, current=+5)` 输出负值；
4. 历史 Kp 递进测试中的 `navigate()` 始终保留了 `-vyaw`；
5. 历史日志记录的是取反前的 `vyaw`，不是串口实际发送值。

因此，2026-07-12 得到的“Kp=0.3 稳定、0.4 有界、0.5 发散”不能继续当作正确负反馈的稳定性边界。它更像是正反馈强度逐渐升高后的表现，需要以修正后的控制链重新建立基线。

## 3. 总体设计

采用“外层航向角保持 + 现有凌霄 IMU 内层角速度环”的级联结构：

```text
起飞前锁存的 T265 yaw
          │
          ▼
wrap(target - current) 到 [-180°, 180°)
          │
          ▼
低增益 P 控制 + 死区 + 整数量化 + 角速度限幅
          │
          ▼
com_yaw / yaw_dps（整数 °/s）
          │
          ▼
凌霄 IMU 现有角速度内环
```

不复用当前 `yaw_pid` 的“喂弧度后靠整数截断静默关闭”机制，改为独立、显式、可单元测试的控制器。

## 4. 文件范围

新增：

- `drone_control/fire_patrol/Lcode/heading_hold.py`
- `drone_control/fire_patrol/test_heading_hold.py`
- 可选：`drone_control/tools/analyze_heading_hold.py`
- 可选：`drone_control/tools/test_analyze_heading_hold.py`

修改：

- `drone_control/fire_patrol/Mission_GPT.py`
- `drone_control/fire_patrol/test_fire_mission_state_machine.py`
- 真机验证后再更新 `.claude/CLAUDE.md` 和 `docs/known_issues.md`

不修改：

- `ANO_LX_FC_倾角保护版/**`
- `drone_control/basic/**`
- `drone_control/basic_radar/**`
- `drone_control/circle_pole/**`
- `drone_control/original/**`
- 上下行串口帧格式

## 5. HeadingHoldController 设计

### 5.1 配置

使用独立配置对象，业务代码从环境变量构造，单元测试直接注入参数，避免测试依赖模块重载：

```python
@dataclass(frozen=True)
class HeadingHoldConfig:
    enabled: bool = False
    kp: float = 0.25
    deadband_deg: float = 1.5
    max_rate_dps: int = 1
    fault_error_deg: float = 8.0
    runaway_window_s: float = 1.0
    runaway_growth_deg: float = 3.0
```

对应环境变量：

- `DRONE_HEADING_HOLD=0|1`
- `DRONE_HEADING_HOLD_KP=0.25`
- `DRONE_HEADING_HOLD_DEADBAND_DEG=1.5`
- `DRONE_HEADING_HOLD_MAX_DPS=1`

所有参数必须校验范围；非法值应在任务创建时明确报错，不能静默接受。第一阶段限制：

- `0 < kp <= 0.5`
- `0.5 <= deadband_deg <= 5.0`
- `1 <= max_rate_dps <= 3`

故障阈值先保持代码常量，避免真机时随手通过环境变量放宽保护。

### 5.2 接口

```python
controller.arm(current_yaw_rad, now)
status = controller.update(current_yaw_rad, confidence, now)
controller.disarm(reason)
controller.reset_for_new_mission()
```

`update()` 返回结构化状态，而不是只返回整数：

```python
HeadingHoldStatus(
    command_dps,
    enabled,
    armed,
    target_deg,
    current_deg,
    error_deg,
    degraded_reason,
    fault_reason,
)
```

这样飞行日志记录的是实际发送值和完整控制状态，不再把中间变量误记为发送值。

### 5.3 目标锁存

- 起飞警示灯结束后、写入 `task_sta=1` 之前读取 T265 yaw；
- 仅当 T265 已运行且当前 confidence 不低于 2 时 `arm()`；
- 目标为这一时刻的当前 yaw，而不是固定数学 `0°`；
- 一次任务只锁存一次，PATROL、APPROACH、HOVER_DROP、RECOVER_HEIGHT 不重新锁存；
- T265 起飞前不可用时，航向保持保持未 armed，基本飞行继续沿用 `yaw_dps=0`。

### 5.4 误差、符号和量化

```python
error_deg = wrap_deg(target_deg - current_deg)
raw_dps = kp * error_deg
```

命令必须与 `error_deg` 同号：当前 yaw 比目标偏正时，发送负角速度；偏负时发送正角速度。接入 `set_speed()` 时直接传递控制器的 `command_dps`，禁止再写 `-command_dps`。

协议只能发送整数度/秒，因此明确采用以下规则：

1. `abs(error_deg) <= deadband_deg`：发送 0；
2. 死区外且 `abs(raw_dps) < 1`：按误差方向发送最小 `±1°/s`；
3. 其余情况做对称取整；不能使用对负数和正数不对称的截断；
4. 最后限制到 `±max_rate_dps`。

第一阶段 `max_rate_dps=1` 实质上是带死区的保守恒速纠偏。只有首轮验证方向、回落和无振荡后，才测试 2°/s；未经数据支持不超过 3°/s。

### 5.5 降级和故障锁存

必须区分两类情况：

**短暂数据降级，不锁存 fault：**

- confidence < 2；
- T265 暂时无新姿态；
- 控制器尚未 arm。

处理方式是当前 tick 输出 `0°/s`、保留原目标并记录 `degraded_reason`。数据恢复后，只有 `abs(error_deg) < fault_error_deg` 才恢复纠偏；如果失联期间已经偏转过大，则转为锁存 fault，避免突然大幅追赶旧目标。

**闭环疑似失控，锁存 fault：**

- `abs(error_deg) >= 8°`；或
- 连续输出非零、命令方向保持一致时，在 1 秒窗口内绝对误差反而增加至少 3°。

锁存后，本次任务永久输出 `0°/s`，只能由新任务 `reset_for_new_mission()` 后重新 arm。航向环 fault 只关闭外层 yaw 修正，不等于整机上锁或任务 emergency。

运行窗口判断必须使用时间戳和角度回绕后的误差，不能简单按固定 tick 数计算，也不能跨越命令反向或低置信度区间累计。

## 6. Mission_GPT.py 接入

### 6.1 初始化

- 创建 `HeadingHoldController`；
- 删除 `fire_patrol` 对旧 `self.yaw_pid` 的航向保持依赖；
- 保留 XY PID 不变；
- 每次新任务启动时重置 controller 状态。

### 6.2 TAKEOFF

1. 完成现有起飞警示灯；
2. 读取可信 T265 yaw 并锁存目标；
3. 再写入 `se_fc[2]` 触发起飞；
4. 起飞确认循环中每 tick 调用 controller；
5. 将返回的实际 `command_dps` 原样写入 `se_fc[6]`。

### 6.3 NAVIGATE 与子状态

`navigate(pos, yaw)` 顶部只计算一次 `heading_status` 和 `yaw_cmd`，然后显式传给：

- PATROL 正常导航；
- `_do_approach(pos, yaw_cmd)`；
- `_do_hover_drop(pos, yaw_cmd)`；
- `_do_recover_height(pos, yaw_cmd)`。

CONFIRM_WARN 不生成新的运动指令，不重新 arm。LAND、END、`stop_all()` 和 `emergency()` 显式清零 yaw，并 disarm 控制器。

不允许在 `set_speed()` 内隐式覆盖调用者传入的 yaw，否则以后新增主动转向动作时会出现优先级不透明的问题。

### 6.4 日志

TAKEOFF、NAVIGATE、APPROACH、HOVER_DROP 和 RECOVER_HEIGHT 至少记录：

- `heading_hold_enabled`
- `heading_hold_armed`
- `heading_target_deg`
- `t265_yaw_deg`
- `heading_error_deg`
- `yaw_cmd_sent`
- `heading_degraded_reason`
- `heading_fault_reason`

原 `vyaw` 字段改名或停止使用，避免历史上“记录取反前值、实际发送相反”的歧义。终端只增加紧凑信息，例如：

```text
yaw=+3.2° err=-3.1° cmd=-1°/s
```

## 7. 实施步骤

### Task 1：纯控制器测试先行

新增 `test_heading_hold.py`，先覆盖：

- 当前 `+5°`、目标 `0°` 时命令为负；
- 当前 `-5°` 时命令为正；
- `+179°/-179°` 两侧的最短路径回绕；
- 死区内输出 0；
- 死区外最小整数输出；
- 正负对称量化；
- 最大角速度限幅；
- disabled、未 arm、低 confidence 输出 0；
- 低 confidence 恢复且误差较小时保留原目标；
- 恢复时误差过大转为 fault；
- 硬误差和反向增长 fault；
- fault 在本次任务内锁存；
- 新任务 reset 后才能重新 arm；
- `arm()` 重复调用不能悄悄改变目标。

然后实现 `Lcode/heading_hold.py` 使测试通过。

### Task 2：接入 TAKEOFF 和 PATROL

- 起飞前锁存目标；
- 替换旧 yaw PID；
- 删除 `navigate()` 的额外负号；
- 添加集成回归测试：当前 yaw 为 `+5°` 时，`set_speed()` 实际收到的 yaw 必须为负；
- 测试 takeoff 和 navigate 使用同一符号规则。

### Task 3：覆盖 fire_patrol 子状态

逐一修改 APPROACH、HOVER_DROP、RECOVER_HEIGHT 的方法参数和 `set_speed()` 调用，验证同一个目标和命令贯穿全过程。补充状态机测试，确保：

- 子状态不会重新锁存目标；
- 子状态不会把 yaw 命令重置为 0；
- CONFIRM_WARN 不篡改控制器；
- LAND、END、emergency 必须输出 0。

### Task 4：日志与失控诊断

- 所有相关状态写入实际发送命令和控制器状态；
- fault 只记录一次状态切换事件，避免每 tick 刷屏；
- 保留每 tick/节流后的状态字段供离线分析；
- 加测试确认日志中的 `yaw_cmd_sent` 等于 `se_fc[6] - sp_side`。

### Task 5：桌面验证

运行：

```bash
cd drone_control/fire_patrol
python -m pytest test_heading_hold.py -v
python -m pytest test_fire_mission_state_machine.py test_yaw_unit_fix.py -v
python -m pytest -q
```

随后使用 `DRONE_DRY_RUN=1 DRONE_HEADING_HOLD=1`：

- 不解锁飞控；
- 人工缓慢转动 T265/机体；
- 检查当前偏正时 `yaw_cmd_sent` 为负、偏负时为正；
- 检查越过 ±180° 时命令不跳成长路径；
- 检查低置信度时输出立即归零。

注意：DRY_RUN 只能验证软件符号链和实际写入值；物理执行方向已经由历史开环脉冲验证，最终闭环稳定性仍必须靠真机测试确认。

### Task 6：真机 A/B 验证

每次试飞前归档旧日志。先使用短时、低高度、单点悬停路线，不运行完整消防任务。

1. 关闭航向保持，至少 3 次相同条件基线；
2. 开启航向保持：`Kp=0.25`、死区 `1.5°`、限幅 `1°/s`，至少 3 次；
3. 人工全程观察；出现持续同向 yaw 增长、明显摆振或 XY/姿态异常立即停止；
4. 对比以下指标：
   - yaw 最大绝对误差；
   - yaw 标准差；
   - 净漂移；
   - yaw 角速度 P95 和标准差；
   - 绝对误差积分 IAE；
   - yaw 命令反向次数；
   - XY 误差、roll/pitch 峰值。

验收条件：

- 开启后不能出现持续同向增长或 fault；
- yaw 峰值、标准差、净漂移或 IAE 至少有明确、可重复的改善；
- yaw 角速度不能因频繁纠偏显著增大；
- XY 误差和 roll/pitch 不得明显恶化；
- 现场观察无可见连续旋转或左右摆振。

只有第一阶段通过，才单独把限幅改到 `2°/s` 复测；每次只调整一个参数。Kp 是否提高到 0.3 必须由数据决定，不能与限幅同时修改。

### Task 7：结论与推广

- 真机验证成功后，更新 `docs/known_issues.md` 问题16，记录额外负号、修复版本和 A/B 数据；
- `.claude/CLAUDE.md` 速查表只更新一行摘要；
- 再单独决定是否默认启用；
- 再单独规划向 basic/basic_radar/circle_pole/original 同步，不能在 fire_patrol 尚未验证时批量复制。

## 8. 提交拆分

建议按以下边界提交：

1. `fire_patrol: 新增航向保持纯控制器和单元测试`
2. `fire_patrol: 接入起飞与全部导航子状态`
3. `fire_patrol: 补充航向保持日志和故障诊断`
4. `tools: 新增航向保持A/B日志分析`（如实现）
5. `docs: 记录yaw额外负号修复与真机验证结论`（真机验证后）

## 9. 风险与回退

| 风险 | 控制措施 | 回退方式 |
|---|---|---|
| 符号仍有遗漏 | 纯函数测试 + Mission实际发送值测试 + DRY_RUN | `DRONE_HEADING_HOLD=0` |
| 量化导致来回摆动 | 1.5°死区、初始仅±1°/s | 增大死区或关闭功能 |
| T265短暂掉线 | 当前tick输出0、保留目标 | 误差过大时锁存fault |
| 闭环方向异常 | 1秒反向增长监测、8°硬阈值 | 本次任务永久输出0 |
| 子状态绕过航向保持 | 显式传参 + 每个状态集成测试 | 关闭功能恢复原行为 |
| 修改影响其它版本 | 第一阶段只改fire_patrol | 不同步其它目录 |

## 10. Claude 独立审查与 Codex 复评

本方案按 `.claude/CLAUDE.md` 的 Codex–Claude 协作流程，经 `claude -p --model sonnet --tools "" --no-session-persistence` 只读审查。

| Claude 意见 | Codex 决策 | 理由/纳入方式 |
|---|---|---|
| 符号链是最高风险，必须验证 | 采纳 | 增加纯函数、Mission实际发送值和DRY_RUN三层验证；实现中明确删除额外负号 |
| 低confidence可能导致“整机disarm”，应加2秒宽限 | 不采纳 | 初稿设计只是航向环当前tick输出0，不触发整机disarm，也不锁存fault；已有起飞前tracking检查 |
| 数据恢复后的策略未定义 | 采纳 | 明确短暂降级保留目标；恢复时误差小才继续，误差过大锁存fault，不自动重设目标 |
| 各状态可能有主动yaw需求，应逐状态arm/disarm | 不采纳 | 审查混入了circle_pole场景；本次fire_patrol没有主动转向需求。采用单一目标显式传参，LAND/END/emergency关闭 |
| P控制可能有稳态误差，A/B应增加IAE | 采纳 | 验收指标新增绝对误差积分IAE和yaw角速度平滑度 |
| 日志应区分confidence与其它fault | 采纳 | 分设`degraded_reason`与`fault_reason` |
| 提高限幅时应同时提高Kp | 不采纳 | 为保持因果可辨识，每轮只能改一个参数；限幅和Kp必须分别验证 |

## 11. 完成定义

方案的“代码完成”与“功能完成”必须分开：

- **代码完成**：新增控制器、全部状态接入、日志和单测完成，fire_patrol 全量测试通过；后续用户验收决定已将默认值改为开启；
- **功能完成**：至少 3+3 次可比较 A/B 真机测试证明 yaw 稳定性可重复改善，且 XY、roll/pitch 无明显回归；
- 原计划3+3门槛未补满；最终默认开启属于当前赛题的用户验收决策，不能把定量结论外推为所有任务场景最优。
