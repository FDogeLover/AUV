# PATROL→APPROACHING触发新增"杆子在正前方"条件 设计

日期：2026-07-15
模块：`drone_control/circle_pole`

## 背景/目的

当前`_update_trigger_candidate()`（`Mission_GPT.py`）的PATROL→APPROACHING触发条件只看"视觉持续确认颜色(且未被环绕去重)满`POLE_TRIGGER_CONFIRM_S`(0.3秒)"，完全不管杆子在画面里的位置——哪怕杆子只是在画面边缘一闪而过（大方位角），只要颜色识别持续0.3秒就会打断巡航路线、触发APPROACHING。这次要新增一个约束：只有当杆子大致在摄像头正前方时，颜色确认计时才成立，保证飞机执行既定巡航路径直到自然经过杆子正前方，才切入APPROACHING。

## 设计

**改动范围**：只改`_update_trigger_candidate()`里`candidate`的判定条件，不改其他任何逻辑。

现在：
```python
candidate = None
if vision_fresh and vision_color is not None and not self._color_already_circled(vision_color):
    candidate = vision_color
```

改成：
```python
candidate = None
if (vision_fresh and vision_color is not None
        and not self._color_already_circled(vision_color)
        and vision["dx_px"] is not None
        and abs(vision["dx_px"]) < APPROACH_CENTERED_DX_PX):
    candidate = vision_color
```

**"正前方"阈值**：复用现有常量`APPROACH_CENTERED_DX_PX=100`（约±5.2°方位角，`atan(100/1100)`），跟APPROACHING阶段内部"雷达坐标未冻结时居中才前进"用同一个数，语义一致（"居中"），不新增常量。

**时间窗口配合**：不需要额外维护"居中"这个独立状态或计时器。现有的滞回逻辑本来就是"candidate变了就重置计时器"（`if self._trigger_candidate != candidate: 重置`）——只要`dx_px`偏出±100px范围，`candidate`自动变回`None`，现有代码原样触发重置。天然实现"颜色确认和居中必须同时成立才计时"：任意一帧`dx_px`偏出范围，正在累积的0.3秒确认窗口就作废，需要重新从居中的那一刻开始计时。

**行为**：巡航路线上只要杆子暂时不在正前方（`_trigger_candidate`会保持`None`或不断被重置），PATROL继续飞既定路线，不会被打断；一旦飞机的位置/朝向让杆子自然落入±100px中心区域，且颜色持续0.3秒不变，才触发APPROACHING。飞机不会主动转向去对准杆子——纯粹是"路线/朝向自然经过正前方时才触发"的被动判定，不新增任何主动寻的行为。

## 边界情况

- `vision["dx_px"]`为`None`（颜色检测到了但没有有效质心，理论上不应该发生，`detect_target()`保证`color`非`None`时`dx_px`也非`None`）：按新条件直接判定不满足，不触发计时，行为上等同于"没检测到"，安全默认。
- 颜色已被环绕去重（`_color_already_circled`）+ 恰好居中：跟现有行为一致，`candidate`仍为`None`，不受本次改动影响。

## 测试

新增3个测试用例（`test_approaching_state.py`的`TestPatrolTriggerIsVisionOnly`类）：
1. `dx_px`偏出±100px范围：颜色持续确认也不应该开始计时（`_trigger_candidate`保持`None`）
2. 从偏离中心过渡到居中：计时器应该从"变成居中"那一刻开始算，不是从"颜色第一次出现"那一刻算
3. 计时过程中途偏出中心：应该导致确认窗口作废，不触发APPROACHING（即使后续又重新居中，也要重新计满0.3秒）

现有测试（`_FakeVision`默认`dx_px=0.0`，本来就是居中场景）预期不受影响，全部应该继续通过，不需要修改。

## 范围外（不做）

- 不新增主动"转向对准杆子"的行为，只是被动过滤触发条件
- 不改变APPROACHING阶段内部的居中/雷达接管/环绕逻辑
- 不改变`APPROACH_CENTERED_DX_PX`阈值本身的数值（复用现有值，不重新标定）
