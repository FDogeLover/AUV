# Drone Stability Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix Z-axis height PID (positional PID + tilt compensation + integral separation), add closed-loop takeoff with yaw stabilization, and implement smooth height ramp between waypoints with different altitudes.

**Architecture:** Two independent subsystems in one plan. (1) Firmware: rewrite `height_set()` in `my_protocol.c` from incremental PID to positional PID, add file-scope state vars, reset in `PID_init()`. (2) Python: add `_step_ramp_z()` helper + `_ramp_z_cm` state to `Mission_GPT`, rewrite `takeoff()` as a height-feedback loop with yaw control, update `navigate()` to use ramp. XY PID D term raised from 0.00 to 0.05 in `Lpid.py`.

**Tech Stack:** C / Keil MDK (STM32F407, GB2312 encoding), Python 3.x, simple_pid, pytest, unittest.mock

---

## Files Changed

| File | Action | Scope |
|---|---|---|
| `ANO_LX_FC_倾角保护版/Mycode/my_protocol.c` | Modify | Add state vars, rewrite `height_set()`, reset in `PID_init()` |
| `ANO_LX_FC_倾角保护版/Mycode/my_protocol.h` | Modify | Update `height_set` declaration parameter name |
| `drone_control/basic/Lcode/Lpid.py` | Modify | D term default 0.05, configurable constructor params |
| `drone_control/basic/Mission_GPT.py` | Modify | `_ramp_z_cm` state, `_step_ramp_z()`, rewrite `takeoff()`, update `navigate()` |
| `drone_control/test_stability.py` | Create | pytest unit tests for all Python changes |

---

## ⚠️ GB2312 Encoding Constraint (firmware files)

`my_protocol.c` and `my_protocol.h` are GB2312 encoded. **NEVER use the Read/Edit tools on these files** — doing so corrupts Chinese comments. ALL edits must go through the Bash tool using `python` with explicit encoding.

---

## Part 1 — Firmware

### Task 1: Rewrite `height_set()` in `my_protocol.c`

**Files:**
- Modify: `ANO_LX_FC_倾角保护版/Mycode/my_protocol.c`
- Modify: `ANO_LX_FC_倾角保护版/Mycode/my_protocol.h`

- [ ] **Step 1: Verify includes already present**

```bash
python -c "
raw = open('ANO_LX_FC_倾角保护版/Mycode/my_protocol.c', 'rb').read()
text = raw.decode('gbk', errors='replace')
lines = text.splitlines()[:6]
for l in lines: print(l)
"
```

Expected output must include both:
```
#include "Ano_Math.h"
#include "ANO_LX.h"
```
These provide `my_cos()`, `my_sqrt()`, and `fc_att`. If either is missing, add it before proceeding.

- [ ] **Step 2: Write the edit script to a temp file**

Create `_patch_height_set.py` in the project root with the following content:

```python
# _patch_height_set.py  — run with: python _patch_height_set.py
import re

path = 'ANO_LX_FC_倾角保护版/Mycode/my_protocol.c'
raw  = open(path, 'rb').read()
text = raw.decode('gbk', errors='replace')

# ---- 1. Insert file-scope PID state variables after last #include ----
state_block = (
    '\r\n'
    '/* Height positional PID state */\r\n'
    'static s16 s_height_integral = 0;\r\n'
    'static s16 s_height_err_last  = 0;\r\n'
)
# Find position after last #include line
last_inc = max(m.end() for m in re.finditer(r'#include\s+"[^"]+"\r?\n', text))
text = text[:last_inc] + state_block + text[last_inc:]

# ---- 2. Replace the active height_set() function body ----
NEW_FUNC = (
    's16 height_set(u32 height, u16 height_target)\r\n'
    '{\r\n'
    '    /* Tilt compensation: convert slant range to vertical height */\r\n'
    '    {\r\n'
    '        float rol_deg = fc_att.st_data.rol_x100 / 100.0f;\r\n'
    '        float pit_deg = fc_att.st_data.pit_x100 / 100.0f;\r\n'
    '        float tilt_deg = my_sqrt(rol_deg * rol_deg + pit_deg * pit_deg);\r\n'
    '        if (tilt_deg > 45.0f) tilt_deg = 45.0f;\r\n'
    '        float tilt_rad = tilt_deg * 0.0174533f;\r\n'
    '        height = (u32)((float)height * my_cos(tilt_rad));\r\n'
    '    }\r\n'
    '\r\n'
    '    s16 err = (s16)height_target - (s16)height;\r\n'
    '\r\n'
    '    /* Integral separation: disable integral when error > 200 cm */\r\n'
    '    s16 i_term;\r\n'
    '    if (err > 200 || err < -200) {\r\n'
    '        i_term = 0;\r\n'
    '        s_height_integral = 0;\r\n'
    '    } else {\r\n'
    '        s_height_integral += err;\r\n'
    '        if (s_height_integral >  100) s_height_integral =  100;\r\n'
    '        if (s_height_integral < -100) s_height_integral = -100;\r\n'
    '        i_term = (s16)(0.05f * s_height_integral);\r\n'
    '    }\r\n'
    '\r\n'
    '    /* Positional PID: Kp=0.8, Ki=0.05, Kd=0.2 */\r\n'
    '    s16 output = (s16)(0.8f * err + i_term + 0.2f * (err - s_height_err_last));\r\n'
    '    s_height_err_last = err;\r\n'
    '\r\n'
    '    if (output >  30) output =  30;\r\n'
    '    if (output < -30) output = -30;\r\n'
    '    return output;\r\n'
    '}\r\n'
)

# Match the active (non-commented) height_set function
pattern = re.compile(
    r's16 height_set\(u32 height,u16 height_set\)\r?\n\{[^}]*(?:\{[^}]*\}[^}]*)?\}',
    re.DOTALL
)
match = pattern.search(text)
if not match:
    print('ERROR: could not find height_set function')
    exit(1)

print(f'Replacing chars {match.start()}..{match.end()}')
text = text[:match.start()] + NEW_FUNC + text[match.end():]

# ---- 3. Add state reset inside PID_init() ----
RESET_LINES = (
    '    /* Reset height PID state */\r\n'
    '    s_height_integral = 0;\r\n'
    '    s_height_err_last  = 0;\r\n'
)
# Find closing brace of PID_init
pid_init_pos = text.find('void PID_init()')
if pid_init_pos == -1:
    print('ERROR: PID_init not found')
    exit(1)
brace_start = text.index('{', pid_init_pos)
depth, i = 0, brace_start
while i < len(text):
    if text[i] == '{': depth += 1
    elif text[i] == '}':
        depth -= 1
        if depth == 0:
            text = text[:i] + RESET_LINES + text[i:]
            break
    i += 1

open(path, 'wb').write(text.encode('gbk'))
print('Done.')
```

- [ ] **Step 3: Run the patch script**

```bash
python _patch_height_set.py
```

Expected output:
```
Replacing chars XXXXX..XXXXX
Done.
```

- [ ] **Step 4: Verify the changes**

```bash
python -c "
raw = open('ANO_LX_FC_倾角保护版/Mycode/my_protocol.c', 'rb').read()
text = raw.decode('gbk', errors='replace')
checks = {
    'state vars added':       's_height_integral' in text,
    'tilt compensation':      'tilt_deg' in text,
    'integral separation':    'Integral separation' in text,
    'positional PID':         'Positional PID' in text,
    'PID_init reset':         text.count('s_height_integral = 0') >= 2,
    'old func removed':       'height_PID.actual' not in text,
}
all_ok = True
for k, v in checks.items():
    print(f'  {\"OK\" if v else \"FAIL\"}: {k}')
    if not v: all_ok = False
exit(0 if all_ok else 1)
"
```

Expected: all lines show `OK`.

- [ ] **Step 5: Update header declaration**

```bash
python -c "
path = 'ANO_LX_FC_倾角保护版/Mycode/my_protocol.h'
raw  = open(path, 'rb').read()
text = raw.decode('gbk', errors='replace')
old  = 's16 height_set(u32 height,u16 height_set);'
new  = 's16 height_set(u32 height, u16 height_target);'
if old not in text:
    print('WARNING: old decl not found, check manually')
else:
    text = text.replace(old, new)
    open(path, 'wb').write(text.encode('gbk'))
    print('Header updated.')
"
```

- [ ] **Step 6: Compile in Keil — verify 0 errors**

Open `ANO_LX_FC_倾角保护版/ProjectSTM32F407/*.uvprojx` in Keil MDK.
Press **F7** (Build). Expected: `0 Error(s), 0 Warning(s)` (or same warning count as before the change).

If `my_cos`/`my_sqrt` show type mismatch, check `Ano_Math.h` — cast the argument:
`my_cos((u16)(tilt_rad * 10000))` if the function takes integer degrees×100.

- [ ] **Step 7: Commit**

```bash
cd /d/项目与工具/Python项目/Project2
git add "ANO_LX_FC_倾角保护版/Mycode/my_protocol.c" \
        "ANO_LX_FC_倾角保护版/Mycode/my_protocol.h"
git rm --cached _patch_height_set.py 2>/dev/null || true
rm _patch_height_set.py
git commit -m "fc: rewrite height_set() with positional PID, tilt compensation, integral separation"
```

---

## Part 2 — Python

> Run all Python commands from `drone_control/` directory unless stated otherwise.
> `_last_laser_height_cm` is in **meters** (named misleadingly — e.g., `1.0` = 1 m = 100 cm).

### Task 2: `Lpid.py` — add D term default, configurable params

**Files:**
- Modify: `drone_control/basic/Lcode/Lpid.py`
- Create: `drone_control/basic/test_stability.py`

- [ ] **Step 1: Write failing tests**

Create `drone_control/basic/test_stability.py`:

```python
import pytest
import sys, os
sys.path.insert(0, os.path.dirname(__file__))


# ════════════════════ Lpid tests ════════════════════

class TestLpid:
    def test_xy_pid_default_d_is_0_05(self):
        from Lcode.Lpid import PID
        pid = PID(type=0)
        assert pid.xyd == 0.05

    def test_xy_pid_simple_pid_receives_d_term(self):
        from Lcode.Lpid import PID
        pid = PID(type=0)
        assert pid.pid.Kd == pytest.approx(0.05)

    def test_custom_xy_params_override_defaults(self):
        from Lcode.Lpid import PID
        pid = PID(type=0, target=0.5, p=1.0, i=0.01, d=0.1)
        assert pid.pid.Kp == pytest.approx(1.0)
        assert pid.pid.Ki == pytest.approx(0.01)
        assert pid.pid.Kd == pytest.approx(0.1)

    def test_yaw_pid_params_unchanged(self):
        from Lcode.Lpid import PID
        pid = PID(type=1)
        assert pid.yawp == 1.5
        assert pid.yawi == 0.0
        assert pid.yawd == 0.3

    def test_custom_yaw_params(self):
        from Lcode.Lpid import PID
        pid = PID(type=1, p=2.0, d=0.5)
        assert pid.pid.Kp == pytest.approx(2.0)
        assert pid.pid.Kd == pytest.approx(0.5)
```

- [ ] **Step 2: Run — verify FAIL**

```bash
cd drone_control/basic && python -m pytest test_stability.py::TestLpid -v 2>&1 | tail -10
```

Expected: `FAILED` on `test_xy_pid_default_d_is_0_05` (current value is 0.00).

- [ ] **Step 3: Implement**

Replace `drone_control/Lcode/Lpid.py` entirely:

```python
import simple_pid


class PID:
    def __init__(self, type=0, target=0, p=None, i=None, d=None) -> None:
        self.xyp = 0.7
        self.xyi = 0.002
        self.xyd = 0.05
        self.yawp = 1.5
        self.yawi = 0.0
        self.yawd = 0.3
        self.xylimit = 40
        self.yawlimit = 30

        if type == 0:
            kp = p if p is not None else self.xyp
            ki = i if i is not None else self.xyi
            kd = d if d is not None else self.xyd
            self.pid = simple_pid.PID(kp, ki, kd, target)
            self.pid.output_limits = (-self.xylimit, self.xylimit)
        else:
            kp = p if p is not None else self.yawp
            ki = i if i is not None else self.yawi
            kd = d if d is not None else self.yawd
            self.pid = simple_pid.PID(kp, ki, kd, target)
            self.pid.output_limits = (-self.yawlimit, self.yawlimit)

    def set_target(self, target):
        self.pid.setpoint = target

    def get_pid(self, current):
        return self.pid(current)

    def reset(self):
        self.pid.reset()
```

- [ ] **Step 4: Run — verify PASS**

```bash
cd drone_control/basic && python -m pytest test_stability.py::TestLpid -v 2>&1 | tail -10
```

Expected: `5 passed`.

- [ ] **Step 5: Commit**

```bash
cd /d/项目与工具/Python项目/Project2
git add drone_control/Lcode/Lpid.py \
        drone_control/test_stability.py
git commit -m "drone_control: Lpid XY D term 0.05, configurable constructor params"
```

---

### Task 3: `Mission_GPT.py` — add `_ramp_z_cm` + `_step_ramp_z()`

**Files:**
- Modify: `drone_control/Mission_GPT.py`
- Modify: `drone_control/test_stability.py`

- [ ] **Step 1: Write failing tests**

Append to `drone_control/test_stability.py`:

```python
# ════════════════════ Mission ramp tests ════════════════════

from unittest.mock import MagicMock, patch


def make_mission():
    """Minimal mission instance: no real serial/T265/K230."""
    from Mission_GPT import mission as MissionClass
    re_fc  = [0, 0, 0, 0, 0]
    se_fc  = [170, 2, 0, 128, 128, 120, 128, 0, 128, 0, 255]
    re_dmz = [('A9', 'B1'), ('A10', 'B2'), ('A11', 'B3')]
    se_dmz = [0xAA, 0, 0xFF, 0, 0xFF]
    realsense  = MagicMock()
    k230       = MagicMock()
    serial_fc  = MagicMock()
    serial_fc._last_laser_height_cm = 0.0
    waypoints  = [[0.0, 0.0, 1.0], [0.5, 0.0, 1.2]]
    with patch.object(MissionClass, 'load_waypoints', return_value=waypoints):
        m = MissionClass(re_fc, se_fc, re_dmz, se_dmz, realsense, k230, serial_fc)
    return m


class TestMissionRamp:
    def test_ramp_z_initialized_to_zero(self):
        m = make_mission()
        assert m._ramp_z_cm == 0.0

    def test_step_ramp_increases_toward_target(self):
        m = make_mission()
        m._ramp_z_cm = 90.0
        m._step_ramp_z(100)
        assert m._ramp_z_cm == pytest.approx(91.5)

    def test_step_ramp_decreases_toward_target(self):
        m = make_mission()
        m._ramp_z_cm = 110.0
        m._step_ramp_z(100)
        assert m._ramp_z_cm == pytest.approx(108.5)

    def test_step_ramp_clamps_when_within_step(self):
        m = make_mission()
        m._ramp_z_cm = 99.2
        m._step_ramp_z(100)
        assert m._ramp_z_cm == pytest.approx(100.0)

    def test_navigate_sends_ramp_z_not_direct_target(self):
        import time
        m = make_mission()
        m._ramp_z_cm = 90.0
        m.target_index = 0
        m.t265_ok = True
        m.arrival_start_time = time.time()
        m.realsense.get_tracking_confidence.return_value = 3
        m.realsense.get_velocity.return_value = [0.0, 0.0, 0.0]

        m.navigate([0.0, 0.0, 0.9], 0.0)

        # se_fc[5] must reflect ramp value (91), not direct target_z (100)
        assert m.se_fc[5] == 91
```

- [ ] **Step 2: Run — verify FAIL**

```bash
cd drone_control && python -m pytest test_stability.py::TestMissionRamp -v 2>&1 | tail -10
```

Expected: `FAILED` (`_ramp_z_cm`, `_step_ramp_z` not defined).

- [ ] **Step 3: Add constant to `Mission_GPT.py`**

After the existing module-level constants block (after `FLIGHT_LOG_INTERVAL = ...`), add:

```python
RAMP_STEP = 1.5        # cm per frame at 30 ms cycle ≈ 50 cm/s climb/descent rate
```

- [ ] **Step 4: Add `_ramp_z_cm` to `__init__()`**

In `mission.__init__()`, after the line `self.last_target_index = -1`, add:

```python
        # Height ramp state (cm); steps toward target each navigate() frame
        self._ramp_z_cm = 0.0
```

- [ ] **Step 5: Add `_step_ramp_z()` method**

After the `limit()` method, add:

```python
    def _step_ramp_z(self, target_z_cm: float):
        if self._ramp_z_cm < target_z_cm - RAMP_STEP:
            self._ramp_z_cm += RAMP_STEP
        elif self._ramp_z_cm > target_z_cm + RAMP_STEP:
            self._ramp_z_cm -= RAMP_STEP
        else:
            self._ramp_z_cm = target_z_cm
```

- [ ] **Step 6: Update `navigate()` to use ramp**

In `navigate()`, find the call to `self.set_speed(vx, vy, -vyaw, target_z)` (near the confidence check) and replace it with:

```python
        self._step_ramp_z(target_z)
        self.set_speed(vx, vy, -vyaw, int(self._ramp_z_cm))
```

(`target_z` is already defined as `int(target[2] * 100)` earlier in the same method.)

- [ ] **Step 7: Run — verify PASS**

```bash
cd drone_control && python -m pytest test_stability.py::TestMissionRamp -v 2>&1 | tail -10
```

Expected: `5 passed`.

- [ ] **Step 8: Commit**

```bash
cd /d/项目与工具/Python项目/Project2
git add drone_control/Mission_GPT.py \
        drone_control/test_stability.py
git commit -m "drone_control: add height ramp (_ramp_z_cm, _step_ramp_z) in navigate()"
```

---

### Task 4: `Mission_GPT.py` — rewrite `takeoff()` as closed-loop

**Files:**
- Modify: `drone_control/Mission_GPT.py`
- Modify: `drone_control/test_stability.py`

- [ ] **Step 1: Write failing tests**

Append to `drone_control/test_stability.py`:

```python
# ════════════════════ Takeoff tests ════════════════════

class TestTakeoff:
    def test_takeoff_sets_task_sta_to_1(self):
        """se_fc[2] must be set to 1 immediately."""
        import time as real_time
        m = make_mission()
        m.t265_ok = False

        tick = [0.0]
        def fake_time():
            tick[0] += 2.0          # advance 2 s per call → timeout in ~8 calls
            return tick[0]

        with patch('Mission_GPT.time.sleep', lambda _: None), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m.se_fc[2] == 1

    def test_takeoff_transitions_to_navigate(self):
        """State must be NAVIGATE after takeoff() returns."""
        m = make_mission()
        m.t265_ok = False

        tick = [0.0]
        def fake_time():
            tick[0] += 2.0
            return tick[0]

        with patch('Mission_GPT.time.sleep', lambda _: None), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m.state == "NAVIGATE"

    def test_takeoff_initializes_ramp_z_to_first_waypoint(self):
        """_ramp_z_cm must equal targets[0][2]*100 when takeoff exits."""
        m = make_mission()
        m.t265_ok = False

        tick = [0.0]
        def fake_time():
            tick[0] += 2.0
            return tick[0]

        with patch('Mission_GPT.time.sleep', lambda _: None), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m._ramp_z_cm == pytest.approx(100.0)   # targets[0][2]=1.0 m → 100 cm

    def test_takeoff_exits_early_on_height_confirmed(self):
        """If laser height matches target for 10 frames, exit before timeout."""
        m = make_mission()
        m.t265_ok = False
        m.serial_fc_ref._last_laser_height_cm = 1.0   # 1.0 m = 100 cm (target)

        call_count = [0]
        tick = [0.0]
        def fake_time():
            tick[0] += 0.03        # realistic 30 ms per frame — stays well within 15 s
            return tick[0]

        def fake_sleep(t):
            call_count[0] += 1

        with patch('Mission_GPT.time.sleep', fake_sleep), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m.state == "NAVIGATE"
        assert call_count[0] <= 20   # must exit in ≤ 20 frames (10 confirm + margin)
```

- [ ] **Step 2: Run — verify FAIL**

```bash
cd drone_control && python -m pytest test_stability.py::TestTakeoff -v 2>&1 | tail -12
```

Expected: `FAILED` (current `takeoff()` has no height feedback or state transition inside the method).

- [ ] **Step 3: Add constants to `Mission_GPT.py`**

After `RAMP_STEP = 1.5`, add:

```python
TAKEOFF_CONFIRM_NEED = 10     # consecutive frames within ±10 cm of target
TAKEOFF_TIMEOUT_S    = 15.0   # force transition to NAVIGATE after this
```

- [ ] **Step 4: Replace `takeoff()` method**

Replace the existing `takeoff()` method with:

```python
    def takeoff(self):
        logger.info("takeoff: started")

        with lock:
            self.se_fc[2] = 1   # trigger FC: unlock + mode switch

        target_h_cm = float(self.targets[0][2] * 100)
        confirm_count = 0
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start

            # Yaw stabilization during climb
            if self.t265_ok:
                try:
                    yaw = self.realsense.get_orientation()[2]
                    vyaw = int(self.limit(self.yaw_pid.get_pid(yaw) * VEL_SCALE, 30))
                    with lock:
                        self.se_fc[6] = vyaw + sp_side
                except Exception:
                    pass

            # Height confirmation (note: _last_laser_height_cm is in metres)
            with lock:
                laser_m = self.serial_fc_ref._last_laser_height_cm \
                          if self.serial_fc_ref else 0.0
            laser_cm = laser_m * 100.0

            if laser_cm > 5.0 and abs(laser_cm - target_h_cm) <= 10.0:
                confirm_count += 1
            else:
                confirm_count = 0

            if confirm_count >= TAKEOFF_CONFIRM_NEED:
                logger.info(f"takeoff: height confirmed {laser_cm:.0f} cm")
                break

            if elapsed >= TAKEOFF_TIMEOUT_S:
                logger.warning("takeoff: timeout, proceeding anyway")
                break

            time.sleep(0.03)

        # Seed ramp at first waypoint height so navigate() starts smooth
        self._ramp_z_cm = target_h_cm
        self.state = "NAVIGATE"
```

- [ ] **Step 5: Run — verify PASS**

```bash
cd drone_control && python -m pytest test_stability.py::TestTakeoff -v 2>&1 | tail -12
```

Expected: `4 passed`.

- [ ] **Step 6: Commit**

```bash
cd /d/项目与工具/Python项目/Project2
git add drone_control/Mission_GPT.py \
        drone_control/test_stability.py
git commit -m "drone_control: takeoff() closed-loop height confirmation + yaw stabilization"
```

---

### Task 5: Full test suite + smoke check

**Files:** (no new changes)

- [ ] **Step 1: Run complete test file**

```bash
cd drone_control && python -m pytest test_stability.py -v 2>&1 | tail -20
```

Expected: `14 passed` (5 Lpid + 5 Ramp + 4 Takeoff). Zero failures.

- [ ] **Step 2: Verify imports from `main.py` still work**

```bash
cd drone_control && python -c "
from Lcode.Lpid import PID
from Lcode.Lprotocol import Serial_fc, Serial_dmz
from Lcode.global_variable import sp_side, lock
print('imports OK')
"
```

Expected: `imports OK`

- [ ] **Step 3: Check branch log**

```bash
cd /d/项目与工具/Python项目/Project2
git log --oneline refactor/stability-v1 ^main
```

Expected (5 commits):
```
xxxxxxx drone_control: takeoff() closed-loop height confirmation + yaw stabilization
xxxxxxx drone_control: add height ramp (_ramp_z_cm, _step_ramp_z) in navigate()
xxxxxxx drone_control: Lpid XY D term 0.05, configurable constructor params
xxxxxxx fc: rewrite height_set() with positional PID, tilt compensation, integral separation
xxxxxxx docs: 添加稳定性重构设计文档
```

- [ ] **Step 4: Final commit (if any unstaged files remain)**

```bash
cd /d/项目与工具/Python项目/Project2
git status
# If clean, nothing to do. Otherwise:
# git add <file> && git commit -m "drone_control: cleanup after stability refactor"
```

---

## Verification Checklist (post-implementation)

| Item | How to verify |
|---|---|
| Keil build passes | F7 → 0 errors |
| height_set uses positional PID | `python _patch_height_set.py` verify script |
| XY PID has D=0.05 | `pytest test_stability.py::TestLpid` |
| Height ramp works | `pytest test_stability.py::TestMissionRamp` |
| Takeoff is closed-loop | `pytest test_stability.py::TestTakeoff` |
| No import regressions | Step 2 of Task 5 |
| 5 commits on branch | `git log --oneline refactor/stability-v1 ^main` |
