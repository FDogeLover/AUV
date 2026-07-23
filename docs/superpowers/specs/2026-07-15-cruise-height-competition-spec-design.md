# 巡航高度改为150cm(赛题合规) 设计

日期：2026-07-15
模块：`drone_control/circle_pole`

## 背景/目的

赛题要求无人机垂直起飞到150±10cm巡航高度（见`docs/superpowers/specs/2026-07-13-circle-pole-design.md`第7行赛题原文）。当前`patrol_router.txt`/`landing_router.txt`全程z=1.2m(120cm)，是阶段1测试空间的暂定值，低于140~160cm要求区间，从未更新过。这次只改高度，环绕半径`POLE_CIRCLE_RADIUS_M`保持0.7m不变（净空约30cm，仍低于赛题40~60cm要求，用户决定暂不处理，留到以后专项测试再调）。

## 改动范围

**纯配置层面，不涉及任何控制逻辑代码改动**：

1. `patrol_router.txt`：全部8个航点z从`1.2`改成`1.5`
2. `landing_router.txt`：第一个航点(巡航高度)z从`1.2`改成`1.5`，第二个航点(降落下降点)保持`0.2`不变
3. `POLE_CIRCLE_RADIUS_M`不改，维持`0.7`

`CIRCLING`/`APPROACHING`阶段用的`self._cruise_z`是从`patrol_router.txt`第一个航点的z值读取的（`Mission_GPT.py`里`self._cruise_z = self.targets[0][2] if self.targets else put_height / 100`），改了航点文件后会自动传播到这两个阶段，不需要额外改代码常量。

## 已排查、确认不受影响的部分

- `POLE_DANGER_DIST_M`(0.75m)/`POLE_RESUME_DIST_M`(0.9m)悬停避让阈值：是水平距离阈值，跟飞行高度无关，不用改
- `TAKEOFF_LIFTOFF_CM`(35cm)：只是起飞盲飞离地这一小段的高度，独立于最终巡航高度，不用改
- `LASER_HEIGHT_MAX_M`(10.0m)：激光高度合理性上限，远高于1.5m，不受影响
- 雷达`max_range_mm`(1200mm)：是水平探测距离，不是高度相关参数，代码里"~1.2m"字样纯属巧合(雷达探测距离和旧巡航高度数值恰好一样)，不需要改动
- `POLE_CIRCLE_RADIUS_M`/`POLE_CIRCLE_EXCLUDE_MARGIN_M`/`APPROACH_CIRCLE_TRIGGER_DIST_M`等半径相关常量：本次不涉及，维持现状

## 未解决/明确保留的已知偏差

赛题要求环绕净空40~60cm，当前`POLE_CIRCLE_RADIUS_M=0.7m`反推净空约30cm，本次不处理，`docs/known_issues.md`或`CLAUDE.md`的已知问题表继续保留这一条不变。

## 测试要求

这纯粹是配置改动，实际效果只能靠真机验证。下次真机测试要重点确认新高度(1.5m)下：
- 到达确认（`xy_thresh`/`posthreshold_z`等阈值）是否依然有效
- 雷达探测杆塔的实际效果是否受飞行高度影响（雷达实际安装角度/杆塔在不同高度的反射特性可能不同，没有代码层面耦合但物理上未验证过）
- 悬停避让触发距离在新高度下的实际表现
- 起飞/降落过程在更高目标高度下是否正常（爬升/下降耗时变长，但`TAKEOFF_LIFTOFF_CM`独立于此不受影响）

不需要新增单元测试——航点文件是纯数据。`grep`确认过`test_approaching_state.py`等测试文件里确实有大量`1.2`字样，但那些是传给`navigate(pos, yaw)`的**当前位置**模拟值(测试自己构造的T265位置读数)，不是读取`patrol_router.txt`的目标航点，两者相互独立，改航点文件高度不会影响这些测试的断言逻辑。实现时仍要跑一次完整测试套件确认这个判断没错，不能只凭代码阅读断言。
