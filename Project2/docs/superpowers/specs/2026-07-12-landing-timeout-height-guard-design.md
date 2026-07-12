# 一键降落纯超时兜底加高度判断 — 设计文档

日期：2026-07-12

## 背景

`ANO_LX_FC_倾角保护版/FcSrc/User_Task.c` 的 `UserTask_OneKeyCmd()` 里，一键降落目前有三条独立的锁桨(`FC_Lock()`)路径：

1. 凌霄IMU自己的`OneKey_Land()` CMD内部序列（完全黑盒，通过CMD 0x06把`unlock_sta`报回来）
2. 近地强制锁定（`ano_of.of_alt_cm<10`连续满足`landing_cnt>=50`约1秒才触发）
3. 纯超时兜底（2026-07-10新增，`land_timeout_cnt>=500`约10秒，**不检查任何高度/姿态/速度**，无条件强制锁桨）

2026-07-12用一次真实的"先爬升后骤降"异常飞行数据反推，坐实了③号纯超时兜底完全不检查高度这个隐患：如果触发降落后10秒内飞机因为某种原因（比如已修复的`vel_z`未重置bug，或其他未知原因）仍处于明显的空中高度，这条兜底会无条件切断全部电机PWM，等同于自由落体/坠机，而不是继续悬停等待人工接管这种相对安全的失败模式。详见 `.claude/CLAUDE.md` 已知问题7/9。

## 目标

给③号纯超时兜底加一个高度判断：只有高度确实已经比较低、锁桨物理风险小的时候才允许它强制锁桨；高度仍然偏高时改为放弃这次自动锁定尝试，转为等待人工接管，而不是无条件切断动力。

## 设计

### 固件端（`User_Task.c`）

新增静态标志 `land_timeout_gaveup_f`（u8），随其他降落状态变量一起在每次新任务开始时（`one_key_mission_f` 0→1 边沿）清零。

修改现有 `land_timeout_cnt >= 500`（约10秒@50Hz）分支，加上高度判断和一次性判定（latch）：

```c
if (land_timeout_cnt >= 500 && land_timeout_gaveup_f == 0)  // 只判定一次
{
    if (ano_of.work_sta && ano_of.of_alt_cm <= 50)  // 高度数据有效且≤0.5m，视为安全
    {
        FC_Lock();
        pwm_to_esc.pwm_m1 = 0;
        pwm_to_esc.pwm_m2 = 0;
        pwm_to_esc.pwm_m3 = 0;
        pwm_to_esc.pwm_m4 = 0;
        landing_f = 1;
    }
    else  // 高度未知(work_sta==0)或明显偏高(>0.5m)，视为不安全，宁可错杀不错放
    {
        land_timeout_gaveup_f = 1;  // 永久放弃这条路径的自动锁定权，不再重新判定
    }
}
```

**关键决策**：
- 高度阈值 0.5m（`of_alt_cm<=50`），高于此值不允许纯超时路径强制锁桨。
- 高度数据无效（`work_sta==0`）时同样视为不安全，不锁桨。
- 一旦判定为不安全，`land_timeout_gaveup_f`置1后**永久生效**，不会在后续tick里因为高度恰好降下来而自动补锁——纯粹等待人工介入（或近地强制锁定②独立生效）。
- 近地强制锁定②完全不受这次改动影响，继续独立运行；如果飞机后续（不管什么原因）真的降到10cm以下并保持住，②仍会正常触发锁定。这次改动只处理③号路径本身的盲目性。

### 协议扩展（复用已有字节的空闲位）

frame2（0x02调试扩展帧）里承载`motor_pwm_mask`的字节只用了bit0~3（对应m1~m4电机PWM非零标记），bit4~7空闲。新增`land_timeout_gaveup_f`状态复用bit4，不需要再次扩展帧长度（避免重复走"扩帧字节数→改Python侧length判断"这套流程）。

`my_protocol.c`打包frame2时，在原有拼装`motor_pwm_mask`字节的地方顺便把`land_timeout_gaveup_f`打进bit4。

### Python 端

**`Lprotocol.py`**：解析frame2时，从原有字节里额外拆出bit4，存入`self.debug_data["land_timeout_gaveup"]`（布尔值）。

**`Mission_GPT.py` 的 `land()`**：
1. 等待循环里每轮读取`debug_data.get("land_timeout_gaveup")`
2. 第一次检测到为`True`时打印一条warning日志（例如"降落纯超时兜底判定高度仍偏高，已放弃自动锁桨，需要人工介入"），只打一次，不重复刷屏
3. 检测到这个状态后，**跳过`LAND_CONFIRM_TIMEOUT_S`(25秒)的退出判断**——继续循环、持续`set_speed(0,0,0,ramp)`维持T265速度参考，直到：
   - 近地强制锁定②最终生效（`unlock_sta`+`motor_pwm_mask`都归零，走现有的双条件确认+去抖流程正常退出），或
   - 操作者手动`Ctrl+C`中止脚本
4. 字段为`None`（老固件/还未收到过帧2）时不影响现有逻辑，正常走25秒超时——保证向后兼容，不依赖新固件也能跑（只是没有这层保护）。

**为什么需要这一步**：如果固件侧已经进入"等人工接管"状态，但Python侧仍按旧的25秒超时逻辑关闭串口退出，会切断凌霄IMU定点悬停依赖的T265速度参考（CMD 0x33），在飞机还需要稳定悬停等待人工接管的时候，反而先破坏了它悬停的能力——这跟固件设计的"等人工介入"意图直接冲突，必须联动处理。

## 测试策略

**固件**：无法自动化测试，沿用项目既有流程——`edit_firmware.py replace`精确字节匹配编辑→`edit_firmware.py verify`确认编码→大括号配对检查→Keil编译烧录→是否需要拆桨台架测试待实施阶段决定→真机验证。

**Python**：TDD补单元测试，覆盖：
- `Lprotocol.py`：frame2解析正确拆出`land_timeout_gaveup`位（bit4=1/0两种情况），不影响`motor_pwm_mask`(bit0-3)原有解析
- `Mission_GPT.py`的`land()`：
  - 检测到`land_timeout_gaveup=True`时打印一次警告日志（不重复）
  - 检测到该状态后，即使超过`LAND_CONFIRM_TIMEOUT_S`也不退出循环（持续调用`set_speed`）
  - 该字段为`None`时行为不变（仍按25秒超时退出）
  - 该字段从`True`变化过程中，`unlock_sta`+`motor_pwm_mask`双双归零时仍能正常确认退出（验证②号路径生效时的正常收尾不受影响）

## 范围外（明确不做）

- 不在这次改动里实现"超时时自动改为慢速下降"（已在设计讨论中排除，选择的是纯悬停等待人工接管）
- 不改动近地强制锁定②本身的逻辑
- 不改动凌霄IMU自己的`OneKey_Land()`CMD序列（黑盒，不可控）
- 不在这次改动里处理`basic/`、`original/`两个版本的Python同步（先在`basic_radar/`验证，参照项目惯例后续再考虑）
