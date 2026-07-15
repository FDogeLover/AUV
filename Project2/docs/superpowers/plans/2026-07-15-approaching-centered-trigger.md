# PATROL触发新增"正前方"条件 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给`drone_control/circle_pole`的PATROL→APPROACHING触发条件新增一个"杆子在正前方"约束，避免飞机在杆子只是从画面边缘一闪而过时就打断巡航路线。

**Architecture:** 只改`Mission_GPT.py`里`_update_trigger_candidate()`方法的`candidate`判定条件，复用现有的`APPROACH_CENTERED_DX_PX`阈值和现有的滞回计时机制(candidate变化就重置计时器)，不新增状态、不新增常量、不改变其他任何逻辑。

**Tech Stack:** Python 3.10 / pytest

参考设计文档：[`docs/superpowers/specs/2026-07-15-approaching-centered-trigger-design.md`](../specs/2026-07-15-approaching-centered-trigger-design.md)

---

## 背景（写代码前必读）

- 项目路径：`drone_control/circle_pole/`
- 目标方法：`Mission_GPT.py`里`mission`类的`_update_trigger_candidate()`（约第821-855行），负责PATROL态下判断要不要触发APPROACHING。当前逻辑：
  1. 拿视觉最新结果`vision = self.pole_vision.latest()`（返回`{"dx_px": ..., "color": ..., "t": ...}`）
  2. 视觉新鲜(`vision_fresh`，`t`在`POLE_VISION_STALE_S`内)+颜色非空+该颜色没被环绕去重(`_color_already_circled`)，才算候选`candidate`
  3. 候选和上次不一样(包括从有变无、从无变有、颜色变了)，重置计时器`_trigger_candidate_since`为现在，本帧不触发
  4. 候选跟上次一样且持续时间超过`POLE_TRIGGER_CONFIRM_S`(0.3秒)，调用`_start_approaching(candidate)`触发APPROACHING
- 现有常量（文件顶部已定义，直接可用，不用新增import）：
  - `APPROACH_CENTERED_DX_PX`（第114行附近，当前值100）：像素级"居中"阈值，本次直接复用
  - `POLE_TRIGGER_CONFIRM_S`（第108行附近，0.3）：颜色确认时长，不改
  - `POLE_VISION_STALE_S`（第107行附近，0.5）：视觉新鲜度，不改
- 测试文件：`test_approaching_state.py`，相关测试类是`TestPatrolTriggerIsVisionOnly`（约第45-110行），里面的`_FakeVision`类（约第31-42行）：
  ```python
  class _FakeVision:
      def __init__(self, dx_px=0.0, color="red", fresh=True):
          self._dx_px = dx_px
          self._color = color
          self._fresh = fresh
          self.locked_color = None

      def latest(self):
          t = time.time() if self._fresh else 0.0
          return {"dx_px": self._dx_px, "color": self._color, "t": t}

      def set_locked_color(self, color):
          self.locked_color = color
  ```
  默认`dx_px=0.0`（本来就居中），所以`TestPatrolTriggerIsVisionOnly`类现有的6个测试用例改动后应该全部继续通过，不需要修改。
- `_make_mission()`辅助函数（测试文件约第22-26行）：
  ```python
  def _make_mission(radar_obj=None, pole_vision_obj=None):
      re_fc = [0] * 14
      se_fc = [0] * 11
      return mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None,
                     radar_obj=radar_obj, pole_vision_obj=pole_vision_obj)
  ```
- 运行测试命令：`cd drone_control/circle_pole && python -m pytest test_approaching_state.py -v`

---

## Task 1: `_update_trigger_candidate()`新增居中条件

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py:836-838`
- Modify: `drone_control/circle_pole/test_approaching_state.py`（在`TestPatrolTriggerIsVisionOnly`类里追加3个测试方法）

- [ ] **Step 1: 写3个失败的测试**

在`test_approaching_state.py`的`TestPatrolTriggerIsVisionOnly`类末尾（`test_color_change_resets_confirm_window`方法后面，注意保持在这个类的缩进内）追加：

```python

    def test_off_center_vision_does_not_start_trigger_timer(self):
        """杆子颜色确认了但不在正前方(|dx_px|>=APPROACH_CENTERED_DX_PX)，
        不应该开始累积确认计时——巡航路线上杆子只是从画面边缘一闪而过时，
        不该打断巡航路线。"""
        vision = _FakeVision(color="red", dx_px=APPROACH_CENTERED_DX_PX + 50)
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate is None
        assert m._trigger_candidate_since is None

    def test_becoming_centered_starts_timer_from_that_moment(self):
        """先偏离中心(不计时)，之后进入居中范围，确认计时器应该从"变成
        居中"这一刻才开始算，不是从颜色第一次出现那一刻算。"""
        vision = _FakeVision(color="red", dx_px=APPROACH_CENTERED_DX_PX + 50)
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m._trigger_candidate is None

        vision._dx_px = 0.0  # 变成居中
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m._trigger_candidate == "red"
        since_when_centered = m._trigger_candidate_since
        assert since_when_centered is not None

        # 计时未满0.3秒，不应该触发
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

        # 计时满0.3秒后才应该触发
        m._trigger_candidate_since = time.time() - POLE_TRIGGER_CONFIRM_S - 0.01
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "APPROACHING"

    def test_drifting_off_center_mid_confirm_resets_timer(self):
        """确认窗口进行到一半时杆子偏出中心，应该导致这次确认作废，不能
        沿用之前累积的时长直接触发。"""
        vision = _FakeVision(color="red", dx_px=0.0)
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m._trigger_candidate == "red"

        # 手动把计时器往前拨，模拟已经过去了一段时间(但还没到0.3秒的边界)
        m._trigger_candidate_since = time.time() - (POLE_TRIGGER_CONFIRM_S - 0.05)

        vision._dx_px = APPROACH_CENTERED_DX_PX + 50  # 中途偏出中心
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate is None
        assert m._trigger_candidate_since is None

        # 就算立刻重新居中，也必须从这一刻重新计满0.3秒，不能因为"之前已经
        # 攒了一部分时长"就提前触发
        vision._dx_px = 0.0
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate == "red"
```

- [ ] **Step 2: 运行测试确认新增的3个失败**

Run: `cd drone_control/circle_pole && python -m pytest test_approaching_state.py::TestPatrolTriggerIsVisionOnly -v`
Expected: 现有6个测试PASS，新增的3个测试FAIL（因为`_update_trigger_candidate()`还没有居中判断，偏离中心的候选目前仍然会被计时/触发）

- [ ] **Step 3: 修改`_update_trigger_candidate()`**

在`Mission_GPT.py`里找到（第836-838行）：

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

- [ ] **Step 4: 运行测试确认全部通过**

Run: `cd drone_control/circle_pole && python -m pytest test_approaching_state.py::TestPatrolTriggerIsVisionOnly -v`
Expected: 9 passed（原有6个+新增3个）

- [ ] **Step 5: 运行全部测试确认没有破坏其他模块**

Run: `cd drone_control/circle_pole && python -m pytest -q`
Expected: 全部通过（此前是143 passed，这次应该是146 passed，因为只新增了3个测试，没有改变任何其他行为）

- [ ] **Step 6: Commit**

```bash
cd drone_control/circle_pole
git add Mission_GPT.py test_approaching_state.py
git commit -m "circle_pole: PATROL触发APPROACHING新增杆子在正前方约束"
```

---

## Task 2: 同步到板子

**Files:**
- 无代码修改，只做部署+验证

- [ ] **Step 1: 同步改动到板子**

Run:
```bash
scp "drone_control/circle_pole/Mission_GPT.py" ubuntu-pi:/home/sunrise/Desktop/FJJ/circle_pole/Mission_GPT.py
scp "drone_control/circle_pole/test_approaching_state.py" ubuntu-pi:/home/sunrise/Desktop/FJJ/circle_pole/test_approaching_state.py
```
Expected: 无报错

- [ ] **Step 2: 板子上转换换行符（板子仓库历史上这两个文件是LF，本机Windows编辑后scp传过去会变成CRLF）**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ/circle_pole && sed -i 's/\r$//' Mission_GPT.py test_approaching_state.py"`
Expected: 无报错

- [ ] **Step 3: 核对diff规模合理**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ && git diff --stat -- circle_pole/Mission_GPT.py circle_pole/test_approaching_state.py"`
Expected: 改动行数应该是个位数到几十行的量级（Task 1只改了3行代码+新增约60行测试代码），不应该出现几百行的整文件重写——如果看到异常大的diff，说明换行符转换没生效，回到Step 2重新处理

- [ ] **Step 4: 板子上跑测试**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ/circle_pole && python3 -m pytest -q -p no:anyio 2>&1 | tail -10"`
Expected: 全部通过（146 passed，跟本机一致；`-p no:anyio`是board上pytest环境已知的插件冲突绕过方式，跟这次改动无关，是环境本身的既有问题）

- [ ] **Step 5: 板子上git commit（不push，独立历史）**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ && git add circle_pole/Mission_GPT.py circle_pole/test_approaching_state.py && git commit -m 'circle_pole: PATROL触发APPROACHING新增杆子在正前方约束'"`
Expected: commit成功

- [ ] **Step 6: 本机push**

Run: `git push`
Expected: push成功（先确认`git status`工作区干净，Task 1的commit已经在本机仓库里）
