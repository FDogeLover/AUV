# basic 导航模式与航向保持

`drone_control/basic`保留原有精确定点行为，同时提供面向长距离巡航的掠过式航点模式。航向保持与导航模式相互独立，两种模式都默认保持起飞时机头方向。

## 默认行为

不设置环境变量时：

- `DRONE_NAV_PROFILE=precision`：全部航点使用原有精确到达逻辑；
- `DRONE_HEADING_HOLD=1`：起飞解锁前锁存当前T265 yaw并全程低速纠偏；
- `LAND`、`END`和`emergency`不运行航向保持，yaw指令明确归零。

需要回退航向控制时使用：

```bash
DRONE_HEADING_HOLD=0 python3 main.py
```

## 掠过式巡航

```bash
DRONE_NAV_PROFILE=cruise python3 main.py
```

默认首1个和尾1个航点保持精确模式，中间航点进入15cm欧氏圆半径并连续保持3个控制周期后立即切换，不要求速度归零、不检查Z、不停留。第一个精确点用于保证先原地爬升，最后一个精确点用于保证稳定到达降落准备位置。

巡航单段超时取以下较大值：

```text
25秒
5秒 + 航段初始水平距离 / 0.20m/s
```

因此已验证的4m量级航段仍使用25秒，未来更长路线会自动获得更长时间预算。

## 可选配置

| 环境变量 | 默认值 | 说明 |
|---|---:|---|
| `DRONE_NAV_PROFILE` | `precision` | `precision`或`cruise` |
| `DRONE_CRUISE_PRECISION_HEAD` | `1` | 巡航路线开头保留的精确航点数 |
| `DRONE_CRUISE_PRECISION_TAIL` | `1` | 巡航路线末尾保留的精确航点数，最小为1 |
| `DRONE_CRUISE_RADIUS_M` | `0.15` | 中间航点掠过半径，米 |
| `DRONE_CRUISE_CONFIRM_CYCLES` | `3` | 连续进入半径的控制周期数 |
| `DRONE_CRUISE_TIMEOUT_S` | `25` | 巡航单段基础超时下限，秒 |
| `DRONE_CRUISE_MIN_PROGRESS_MPS` | `0.20` | 距离缩放使用的保守进度速度，米/秒 |
| `DRONE_CRUISE_TIMEOUT_MARGIN_S` | `5` | 距离缩放额外时间，秒 |
| `DRONE_HEADING_HOLD` | `1` | 航向保持总开关，`0`可回退 |
| `DRONE_HEADING_HOLD_KP` | `0.25` | 航向外环比例增益 |
| `DRONE_HEADING_HOLD_DEADBAND_DEG` | `1.5` | 航向死区，度 |
| `DRONE_HEADING_HOLD_MAX_DPS` | `1` | yaw角速度限幅，度/秒 |

所有配置在任务对象创建时校验，非法值会直接报错，不会静默采用危险参数。

## 当前验证边界

- `fire_patrol`的航向保持和15cm巡航判据已有完整路线真机数据；
- 迁移后的`basic`已完成单元测试、状态机回归和语法检查；
- `basic`双模式尚未单独真机飞行，第一次用于新赛题时仍需分别验证目标路线；
- 每次真实起飞仍必须重新取得现场安全确认。
