---
name: project-two-stage-landing
description: 两级降落 DESCEND/HOVER_WAIT 替代 OneKey_Land CMD 通道
metadata: 
  node_type: memory
  type: project
  originSessionId: sess_444a2a84-dc30-4300-83d4-a618a8ba533e
---

# 两级降落（2026-07-24）

## 为什么改

OneKey_Land() 走 CMD 通道（`dt.wait_ck`），可能被其他指令撞车导致降落指令被静默丢弃，且近地时 T265 位置反馈漂移可能导致落地偏移。

## 方案

```
NAVIGATE → DESCEND → LAND → END
                ↕ 超时
           HOVER_WAIT (等人工介入)
```

- **DESCEND**：水平开环（`set_speed(vx=0, vy=0, yaw=航向保持, z=ramp ~15cm/s`），不依赖 T265 位置反馈
  - 入口条件：全部航点完成 + 高度 ≤ 20cm（安全分界线，实测可靠）
  - 贴地检测：专用 `is_near_ground()`（阈值 8cm，原始激光值不过滤），300ms 去抖（10帧×30ms）
  - 超时 20s → HOVER_WAIT
- **HOVER_WAIT**：超时后悬停，不关串口（维持 T265 速度参考让固件能持续悬停），等人工遥控器接管
- **LAND**：循环发 `se_fc[7]=101`（FC_Lock）+ `unlock_sta`/`motor_pwm_mask` 双确认，不再调用 OneKey_Land()
- 固件侧近地强制锁定（`of_alt_cm<10` 持续 1 秒 → PWM 直写）作为兜底

## 关键常量

| 常量 | 值 | 说明 |
|------|-----|------|
| DESCEND_SAFE_HEIGHT_CM | 20 | 位置闭环/开环分界线 |
| DESCEND_LOCK_HEIGHT_CM | 8 | 锁桨触发阈值（静态激光~6cm） |
| DESCEND_RAMP_STEP | 0.45 | 递减步长 cm/30ms ≈ 15cm/s |
| DESCEND_CONFIRM_COUNT | 10 | 贴地去抖帧数 |
| DESCEND_TIMEOUT_S | 20 | 垂降超时 |

## 真机验证

- 2026-07-24 三次飞行（2 次短路径 + 1 次 1m 矩形路径）全部成功
- 降落点 XY 偏差 <10cm（偏差来自 NAVIGATE 阶段 T265 漂移，DESCEND 不恶化）
- 更高精度需视觉伺服

## router.txt

最后一个航点 Z=0 表示"飞到此后启动 DESCEND"，不需要 OneKey_Land CMD。

## 相关文件

- `drone_control/basic/Mission_GPT.py` — `descend()`, `hover_wait()`, 修改后的 `land()`
- `.zcode/plans/two_stage_landing_plan.md` — 完整计划 + Qoder 审查记录
- `docs/known_issues.md #9` — 详细测试记录
