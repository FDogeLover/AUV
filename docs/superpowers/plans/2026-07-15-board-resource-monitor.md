# 板载资源监控（CPU/内存/温度）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给`drone_control/circle_pole`任务运行时新增一个独立的后台线程，每秒采样一次CPU/内存/温度，写进现有的`flight_data.jsonl`，用于事后分析任务运行中的板子负载情况。

**Architecture:** 新增`Lcode/resource_monitor.py`里的`ResourceMonitor`类，用`psutil`采样，通过`start(log_file, log_lock)`/`stop()`两个方法接入`Mission_GPT.py`里`mission.start()`/`mission.stop_all()`的现有生命周期。因为主循环线程和资源监控线程会并发写同一个文件对象，顺带给`Mission_GPT.py`里全部8处现有的`_log_file.write()+flush()`调用点补上一把共享锁（`self._log_lock`），避免两个线程的写入交错把JSON行拆坏。

**Tech Stack:** Python 3.10（板子）/ pytest（测试）/ `psutil`（新依赖）

参考设计文档：[`docs/superpowers/specs/2026-07-15-board-resource-monitor-design.md`](../specs/2026-07-15-board-resource-monitor-design.md)

---

## 背景（写代码前必读）

- 项目路径：`drone_control/circle_pole/`
- 主状态机文件：`Mission_GPT.py`，`mission`类里`self._log_file`是一个打开的文件对象（`open(path + "/flight_data.jsonl", "a")`），任务运行期间由**多个不同代码位置**分别调用`self._log_file.write(json.dumps({...}) + "\n")`紧接着`self._log_file.flush()`来追加一行JSON。
- `mission.__init__()`（约第171-220行）目前在第215行`self._log_file = None`，第216行`self._last_log_time = 0.0`。
- `mission.start()`（约第280行开始）在第320-326行打开`_log_file`并写第一行`{"event": "task_start"}`，然后在第328行`threading.Thread(target=self.loop, daemon=True).start()`启动主循环线程。
- `mission.stop_all()`（约第1208-1222行）在第1210-1214行关闭`_log_file`，然后第1215-1221行发送解锁指令，最后第1222行`self.task_running = False`。
- 现有全部8处`_log_file.write()+flush()`调用点（本计划要给这8处全部套上锁）：
  1. `start()`第323-324行：`{"event": "task_start"}`
  2. `takeoff()`第425-433行：起飞阶段yaw记录
  3. `navigate()`悬停避让分支，第565-580行
  4. `navigate()` T265丢失分支，第604-615行
  5. `navigate()`主日志块，第703-721行
  6. `_log_approaching_telemetry()`第881-899行
  7. `land()`第1171-1182行
  8. （新增）`ResourceMonitor`的采样写入——不在`Mission_GPT.py`里，但也要用同一把`self._log_lock`
- 板子（ubuntu-pi）上`psutil`已安装（5.9.0），CPU温度读法已验证：`psutil.sensors_temperatures()`返回`{'pvt': [shwtemp(current=46.3,...), shwtemp(current=44.7,...)]}`，取`['pvt'][0].current`。
- 本机（Windows开发机）**没有装`psutil`**，需要先`pip install psutil`才能跑测试。
- 测试约定：`pytest`，class-based（比如`test_approaching_state.py`里的`TestApproachingTelemetryLogging`），用`monkeypatch` fixture做mock，不用`unittest.mock.patch`装饰器风格。测试文件顶部固定这三行加入模块搜索路径：
  ```python
  import os
  import sys
  sys.path.insert(0, os.path.dirname(__file__))
  ```
- 运行测试命令：`cd drone_control/circle_pole && python -m pytest <file> -v`

---

## File Structure

- Create: `drone_control/circle_pole/Lcode/resource_monitor.py` — `ResourceMonitor`类，独立文件，只依赖`psutil`+标准库，不依赖`Mission_GPT.py`任何东西，可独立测试
- Create: `drone_control/circle_pole/test_resource_monitor.py` — 单元测试
- Modify: `drone_control/circle_pole/Mission_GPT.py` — 接入`ResourceMonitor`+补充`self._log_lock`并包住8处写入点
- Modify: `drone_control/circle_pole/requirements.txt` — 加`psutil`依赖

---

## Task 1: 本机安装psutil依赖

**Files:**
- Modify: `drone_control/circle_pole/requirements.txt`

- [ ] **Step 1: 本机安装psutil（跑后面测试要用）**

Run: `pip install psutil`
Expected: 安装成功，输出结尾类似 `Successfully installed psutil-x.x.x`

- [ ] **Step 2: 验证能正常import并拿到系统信息**

Run: `python -c "import psutil; print(psutil.cpu_percent()); print(psutil.virtual_memory().percent)"`
Expected: 打印两个数字（浮点数），不报错

- [ ] **Step 3: 把psutil加进requirements.txt**

把文件内容改成：

```
numpy>=1.21.0
simple_pid>=0.7.0
pyserial>=3.5
opencv-python>=4.5.0
psutil>=5.9.0
```

- [ ] **Step 4: Commit**

```bash
cd drone_control/circle_pole
git add requirements.txt
git commit -m "circle_pole: 新增psutil依赖(板载资源监控用)"
```

---

## Task 2: ResourceMonitor类 — 采样单帧数据

**Files:**
- Create: `drone_control/circle_pole/Lcode/resource_monitor.py`
- Test: `drone_control/circle_pole/test_resource_monitor.py`

先写`_sample_once()`（单次采样+写入），不涉及线程启停。

- [ ] **Step 1: 写失败的测试**

创建`drone_control/circle_pole/test_resource_monitor.py`，写入：

```python
"""ResourceMonitor单元测试。

运行（先确保已 pip install pytest psutil）：
    cd drone_control/circle_pole && python -m pytest test_resource_monitor.py -v
"""
import io
import json
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.resource_monitor import ResourceMonitor


class FakeTemps:
    def __init__(self, current):
        self.current = current


class FakeProc:
    def __init__(self, cpu_pct=41.0, rss_mb=210.3):
        self._cpu_pct = cpu_pct
        self._rss_bytes = rss_mb * 1024 * 1024

    def cpu_percent(self, interval=None):
        return self._cpu_pct

    def memory_info(self):
        class Mem:
            pass
        mem = Mem()
        mem.rss = self._rss_bytes
        return mem


class FakeVirtualMemory:
    def __init__(self, percent=55.2, used_mb=890.5):
        self.percent = percent
        self.used = used_mb * 1024 * 1024


class TestSampleOnce:
    def test_writes_expected_fields(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 62.3)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(
            rm_module.psutil, "sensors_temperatures",
            lambda: {"pvt": [FakeTemps(46.319), FakeTemps(44.707)]},
        )

        monitor = ResourceMonitor()
        monitor._proc = FakeProc(cpu_pct=41.0, rss_mb=210.3)
        log_file = io.StringIO()
        monitor._log_file = log_file
        monitor._log_lock = threading.Lock()

        monitor._sample_once()

        lines = log_file.getvalue().strip().split("\n")
        assert len(lines) == 1
        entry = json.loads(lines[0])
        assert entry["event"] == "resource"
        assert entry["cpu_percent_sys"] == 62.3
        assert entry["cpu_percent_proc"] == 41.0
        assert entry["mem_percent_sys"] == 55.2
        assert entry["mem_used_mb_sys"] == pytest.approx(890.5, abs=0.1)
        assert entry["mem_rss_mb_proc"] == pytest.approx(210.3, abs=0.1)
        assert entry["cpu_temp_c"] == pytest.approx(46.3, abs=0.1)
        assert "t" in entry

    def test_missing_temp_sensor_gives_none(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 10.0)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(rm_module.psutil, "sensors_temperatures", lambda: {})

        monitor = ResourceMonitor()
        monitor._proc = FakeProc()
        log_file = io.StringIO()
        monitor._log_file = log_file
        monitor._log_lock = threading.Lock()

        monitor._sample_once()

        entry = json.loads(log_file.getvalue().strip())
        assert entry["cpu_temp_c"] is None

    def test_psutil_exception_does_not_raise(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        def boom(interval=None):
            raise RuntimeError("psutil炸了")

        monkeypatch.setattr(rm_module.psutil, "cpu_percent", boom)

        monitor = ResourceMonitor()
        monitor._proc = FakeProc()
        log_file = io.StringIO()
        monitor._log_file = log_file
        monitor._log_lock = threading.Lock()

        monitor._sample_once()  # 不应该抛异常

        assert log_file.getvalue() == ""
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd drone_control/circle_pole && python -m pytest test_resource_monitor.py -v`
Expected: `ModuleNotFoundError: No module named 'Lcode.resource_monitor'` 或 `ImportError`

- [ ] **Step 3: 写最小实现**

创建`drone_control/circle_pole/Lcode/resource_monitor.py`：

```python
"""板载CPU/内存/温度采样，独立后台线程周期性写入flight_data.jsonl。

circle_pole阶段2的视觉识别(pole_vision)/雷达监听(Lradar)/T265 SDK都是同一个
Python进程里的线程，本进程CPU%已经能回答"是circle_pole自己在吃CPU还是板子上
其他东西"，线程级CPU拆分收益不够抵消/proc读取+C扩展线程无法命名的复杂度，
本模块只做进程级+系统级采样。见
docs/superpowers/specs/2026-07-15-board-resource-monitor-design.md。
"""
import json
import os
import threading
import time

import psutil

SAMPLE_INTERVAL_S = 1.0


class ResourceMonitor:
    def __init__(self):
        self._thread = None
        self._stop_flag = threading.Event()
        self._log_file = None
        self._log_lock = None
        self._proc = None

    def start(self, log_file, log_lock):
        self._log_file = log_file
        self._log_lock = log_lock
        self._stop_flag.clear()
        # psutil.Process必须只创建一次并复用同一实例——cpu_percent()靠同一
        # 实例前后两次调用的时间差算百分比，每次新建实例读数会一直是0。
        self._proc = psutil.Process(os.getpid())
        # cpu_percent系接口首次调用是"热身"，返回值无意义，直接丢弃。
        psutil.cpu_percent(interval=None)
        self._proc.cpu_percent(interval=None)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_flag.set()
        if self._thread is not None:
            # stop_all()只在任务结束时调一次，不在实时控制路径上，阻塞至多
            # 2秒没有性能代价，但能保证stop()返回后线程真的不会再碰
            # _log_file，避免调用方紧接着close()文件时跟仍在跑最后一轮采样
            # 的线程产生"写已关闭文件"的竞态。
            self._thread.join(timeout=2.0)

    def _loop(self):
        while not self._stop_flag.is_set():
            self._sample_once()
            time.sleep(SAMPLE_INTERVAL_S)

    def _sample_once(self):
        try:
            cpu_temp_c = None
            temps = psutil.sensors_temperatures()
            if temps.get("pvt"):
                cpu_temp_c = temps["pvt"][0].current

            vm = psutil.virtual_memory()
            entry = {
                "event": "resource",
                "t": round(time.time(), 3),
                "cpu_percent_sys": psutil.cpu_percent(interval=None),
                "cpu_percent_proc": self._proc.cpu_percent(interval=None),
                "mem_percent_sys": vm.percent,
                "mem_used_mb_sys": round(vm.used / 1024 / 1024, 1),
                "mem_rss_mb_proc": round(self._proc.memory_info().rss / 1024 / 1024, 1),
                "cpu_temp_c": round(cpu_temp_c, 1) if cpu_temp_c is not None else None,
            }
            if self._log_file:
                with self._log_lock:
                    self._log_file.write(json.dumps(entry) + "\n")
                    self._log_file.flush()
        except Exception:
            pass
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd drone_control/circle_pole && python -m pytest test_resource_monitor.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
cd drone_control/circle_pole
git add Lcode/resource_monitor.py test_resource_monitor.py
git commit -m "circle_pole: 新增ResourceMonitor单帧采样(CPU/内存/温度)"
```

---

## Task 3: ResourceMonitor线程启停

**Files:**
- Modify: `drone_control/circle_pole/test_resource_monitor.py`（追加测试类）
- Modify: `drone_control/circle_pole/Lcode/resource_monitor.py`（Task 2已实现完整，这里只是补测试验证线程行为，预期不需要再改实现代码）

- [ ] **Step 1: 写失败的测试（线程启停）**

在`test_resource_monitor.py`末尾追加：

```python
class TestStartStop:
    def test_start_produces_samples_stop_halts_them(self, monkeypatch):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module, "SAMPLE_INTERVAL_S", 0.02)
        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 5.0)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(rm_module.psutil, "sensors_temperatures", lambda: {})
        monkeypatch.setattr(
            rm_module.psutil, "Process",
            lambda pid: FakeProc(cpu_pct=5.0, rss_mb=100.0),
        )

        monitor = ResourceMonitor()
        log_file = io.StringIO()
        log_lock = threading.Lock()

        monitor.start(log_file, log_lock)
        time.sleep(0.1)  # 至少经过几个SAMPLE_INTERVAL_S=0.02周期
        monitor.stop()

        lines_after_stop = log_file.getvalue().strip().split("\n")
        assert len(lines_after_stop) >= 2
        for line in lines_after_stop:
            entry = json.loads(line)
            assert entry["event"] == "resource"

        count_right_after_stop = len(lines_after_stop)
        time.sleep(0.1)  # 再等一轮周期，确认线程真的停了，没有继续写
        lines_later = log_file.getvalue().strip().split("\n")
        assert len(lines_later) == count_right_after_stop
```

在文件顶部`import threading`下面补一行`import time`（如果Step 1的import列表里还没有的话，检查现有文件头）。

- [ ] **Step 2: 运行测试确认失败或通过**

Run: `cd drone_control/circle_pole && python -m pytest test_resource_monitor.py::TestStartStop -v`
Expected: 由于Task 2的`ResourceMonitor.start()/stop()`已经实现完整，这个测试大概率直接PASS——如果PASS就跳过Step 3直接进Step 4；如果FAIL，先看报错信息按需修`resource_monitor.py`（比如`psutil.Process`被mock成lambda返回`FakeProc`时，`start()`里`self._proc = psutil.Process(os.getpid())`要能正确拿到mock返回值）

- [ ] **Step 3: 如有必要修复实现**

（仅当Step 2失败时执行）根据失败信息修`Lcode/resource_monitor.py`，重新跑Step 2直到PASS

- [ ] **Step 4: 运行全部resource_monitor测试确认通过**

Run: `cd drone_control/circle_pole && python -m pytest test_resource_monitor.py -v`
Expected: 4 passed

- [ ] **Step 5: Commit**

```bash
cd drone_control/circle_pole
git add test_resource_monitor.py Lcode/resource_monitor.py
git commit -m "circle_pole: 验证ResourceMonitor线程启停行为"
```

---

## Task 4: Mission_GPT.py接入`self._log_lock`，包住全部8处写入点

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`

这一步先只加锁，不接入`ResourceMonitor`（下一个Task再接），保证改动可以独立跑现有测试验证不破坏行为。

- [ ] **Step 1: 在`__init__`里新增`self._log_lock`**

在`Mission_GPT.py`第215-216行（`self._log_file = None` / `self._last_log_time = 0.0`）后面加一行：

```python
        self._log_file = None
        self._last_log_time = 0.0
        self._log_lock = threading.Lock()
```

- [ ] **Step 2: 包住第1处写入点 — `start()`第323-324行**

原代码：
```python
            self._log_file = open(path + "/flight_data.jsonl", "a")
            self._log_file.write(json.dumps({"event": "task_start"}) + "\n")
            self._log_file.flush()
```

改成：
```python
            self._log_file = open(path + "/flight_data.jsonl", "a")
            with self._log_lock:
                self._log_file.write(json.dumps({"event": "task_start"}) + "\n")
                self._log_file.flush()
```

- [ ] **Step 3: 包住第2处写入点 — `takeoff()`第423-434行**

原代码（第423-434行，注意`try:`本身不动，只把`write`+`flush`两行套进`with`）：
```python
            if self._log_file:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(time.time(), 3),
                        "state": "TAKEOFF",
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "fc_yaw_deg": round(fc_yaw_deg, 2),
                        "vyaw": vyaw,
                        "laser_cm": round(laser_cm, 1),
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
```

改成：
```python
            if self._log_file:
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(time.time(), 3),
                            "state": "TAKEOFF",
                            "t265_yaw_deg": round(math.degrees(yaw), 2),
                            "fc_yaw_deg": round(fc_yaw_deg, 2),
                            "vyaw": vyaw,
                            "laser_cm": round(laser_cm, 1),
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
```

- [ ] **Step 4: 包住第3处写入点 — `navigate()`悬停避让分支第562-583行**

找到（第562-583行）：
```python
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                tv = self.realsense.get_velocity() if (self.t265_ok and self.realsense) else (0.0, 0.0, 0.0)
                try:
                    self._log_file.write(json.dumps({
```
...一直到...
```python
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
```

把`try:`里的`self._log_file.write(...)`到`self._log_file.flush()`这两行整体缩进一级，外面套`with self._log_lock:`。即改成：
```python
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                tv = self.realsense.get_velocity() if (self.t265_ok and self.realsense) else (0.0, 0.0, 0.0)
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(now, 3),
                            "state": self.state,
                            "target_idx": self.target_index,
                            "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                            "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                            "vx": vx, "vy": vy, "vyaw": 0,
                            "t265_yaw_deg": round(math.degrees(yaw), 2),
                            "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                            "height_setpoint_cm": round(self._ramp_z_cm, 1),
                            "pole_hover": self._pole_hovering,
                            "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                            "hover_hold_pos": list(self._hover_hold_pos),
                            "confirmed_poles": confirmed_poles_list,
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
```

（字段内容原样保留，只是缩进+套锁，跟现有第565-579行的字段列表逐字对应，实现时直接对照原文件改，不要凭空重打字段）

- [ ] **Step 5: 包住第4处写入点 — `navigate()` T265丢失分支第601-618行**

原代码：
```python
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "target_idx": self.target_index,
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                        "vx": 0, "vy": 0, "vyaw": 0,
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "t265_confidence_lost": True,
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
```

改成：
```python
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(now, 3),
                            "state": self.state,
                            "target_idx": self.target_index,
                            "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                            "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                            "vx": 0, "vy": 0, "vyaw": 0,
                            "t265_yaw_deg": round(math.degrees(yaw), 2),
                            "height_setpoint_cm": round(self._ramp_z_cm, 1),
                            "t265_confidence_lost": True,
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
```

- [ ] **Step 6: 包住第5处写入点 — `navigate()`主日志块第700-724行**

原代码：
```python
        now = time.time()
        if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
            try:
                self._log_file.write(json.dumps({
                    "t": round(now, 3),
                    "state": self.state,
                    "target_idx": self.target_index,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "vx": vx, "vy": vy, "vyaw": vyaw,
                    "t265_yaw_deg": round(math.degrees(yaw), 2),
                    "fc_yaw_deg": round(fc_yaw_deg, 2),
                    "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                    "of1_vel_cms": [of1_dx, of1_dy],
                    "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                    "height_setpoint_cm": round(self._ramp_z_cm, 1),
                    "of_status": [of_quality, of_link_sta, of_work_sta],
                    "pole_hover": self._pole_hovering,
                    "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                    "confirmed_poles": confirmed_poles_list,
                }) + "\n")
                self._log_file.flush()
            except Exception:
                pass
            self._last_log_time = now
```

改成：
```python
        now = time.time()
        if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
            try:
                with self._log_lock:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "target_idx": self.target_index,
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                        "vx": vx, "vy": vy, "vyaw": vyaw,
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "fc_yaw_deg": round(fc_yaw_deg, 2),
                        "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                        "of1_vel_cms": [of1_dx, of1_dy],
                        "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "of_status": [of_quality, of_link_sta, of_work_sta],
                        "pole_hover": self._pole_hovering,
                        "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                        "confirmed_poles": confirmed_poles_list,
                    }) + "\n")
                    self._log_file.flush()
            except Exception:
                pass
            self._last_log_time = now
```

- [ ] **Step 7: 包住第6处写入点 — `_log_approaching_telemetry()`第880-902行**

原代码：
```python
        try:
            self._log_file.write(json.dumps({
                "t": round(now, 3),
                "state": self.state,
                "nav_mode": self.nav_mode,
                "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                "vx": vx, "vy": vy,
                "t265_yaw_deg": round(math.degrees(yaw), 2),
                "fc_yaw_deg": round(fc_yaw_deg, 2),
                "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                "dx_px": dx_px,
                "azimuth_deg": round(math.degrees(azimuth_rad), 2) if azimuth_rad is not None else None,
                "vision_fresh": vision_fresh,
                "vision_age_s": round(vision_age_s, 3) if vision_age_s is not None else None,
                "pole_hover": self._pole_hovering,
                "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                "confirmed_poles": confirmed_poles_list or [],
            }) + "\n")
            self._log_file.flush()
        except Exception:
            pass
        self._last_log_time = now
```

改成：
```python
        try:
            with self._log_lock:
                self._log_file.write(json.dumps({
                    "t": round(now, 3),
                    "state": self.state,
                    "nav_mode": self.nav_mode,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "vx": vx, "vy": vy,
                    "t265_yaw_deg": round(math.degrees(yaw), 2),
                    "fc_yaw_deg": round(fc_yaw_deg, 2),
                    "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                    "dx_px": dx_px,
                    "azimuth_deg": round(math.degrees(azimuth_rad), 2) if azimuth_rad is not None else None,
                    "vision_fresh": vision_fresh,
                    "vision_age_s": round(vision_age_s, 3) if vision_age_s is not None else None,
                    "pole_hover": self._pole_hovering,
                    "pole_dist": round(pole_dist, 3) if pole_dist is not None else None,
                    "confirmed_poles": confirmed_poles_list or [],
                }) + "\n")
                self._log_file.flush()
        except Exception:
            pass
        self._last_log_time = now
```

- [ ] **Step 8: 包住第7处写入点 — `land()`第1168-1184行**

原代码：
```python
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "pos": [round(land_pos[0], 4), round(land_pos[1], 4), round(land_pos[2], 4)],
                        "t265_yaw_deg": round(math.degrees(land_yaw), 2),
                        "t265_vel": [round(land_tv[0], 4), round(land_tv[1], 4)],
                        "raw_imu": [round(v, 4) for v in land_raw_imu],
                        "unlock_sta": unlock_sta,
                        "motor_pwm_mask": motor_pwm_mask,
                        "motor_pwm_mask_t": motor_pwm_mask_t,
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
```

改成：
```python
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    with self._log_lock:
                        self._log_file.write(json.dumps({
                            "t": round(now, 3),
                            "state": self.state,
                            "pos": [round(land_pos[0], 4), round(land_pos[1], 4), round(land_pos[2], 4)],
                            "t265_yaw_deg": round(math.degrees(land_yaw), 2),
                            "t265_vel": [round(land_tv[0], 4), round(land_tv[1], 4)],
                            "raw_imu": [round(v, 4) for v in land_raw_imu],
                            "unlock_sta": unlock_sta,
                            "motor_pwm_mask": motor_pwm_mask,
                            "motor_pwm_mask_t": motor_pwm_mask_t,
                        }) + "\n")
                        self._log_file.flush()
                except Exception:
                    pass
```

（注意：这段原代码在文件里`except Exception: pass`之后紧跟的是同一个`if`块外的其他逻辑，不属于这处写入点，不要改动`except`之后的代码）

- [ ] **Step 9: 运行现有全部测试确认没有破坏行为**

Run: `cd drone_control/circle_pole && python -m pytest -q`
Expected: 全部通过（此前是134 passed，这次应该还是134 passed，因为只是给写入操作加锁，没有改变任何字段内容或控制逻辑）

- [ ] **Step 10: Commit**

```bash
cd drone_control/circle_pole
git add Mission_GPT.py
git commit -m "circle_pole: 全部8处飞行日志写入点加锁(为并发资源监控线程做准备)"
```

---

## Task 5: 接入ResourceMonitor到mission的启停生命周期

**Files:**
- Modify: `drone_control/circle_pole/Mission_GPT.py`

- [ ] **Step 1: 加import**

在`Mission_GPT.py`顶部现有import块（第16-22行左右，`from Lcode.pole_vision import azimuth_from_dx`那一行附近）加一行：

```python
from Lcode.resource_monitor import ResourceMonitor
```

- [ ] **Step 2: 在`__init__`里创建`ResourceMonitor`实例**

在Task 4 Step 1加的`self._log_lock = threading.Lock()`后面加一行：

```python
        self._log_lock = threading.Lock()
        self._resource_monitor = ResourceMonitor()
```

- [ ] **Step 3: 在`start()`里启动监控**

找到`start()`方法里打开`_log_file`并写`task_start`那段（Task 4 Step 2改完后的样子），紧接着加一行：

```python
            self._log_file = open(path + "/flight_data.jsonl", "a")
            with self._log_lock:
                self._log_file.write(json.dumps({"event": "task_start"}) + "\n")
                self._log_file.flush()
        except Exception:
            pass

        self._resource_monitor.start(self._log_file, self._log_lock)

        threading.Thread(target=self.loop, daemon=True).start()
```

（注意：`self._resource_monitor.start(...)`要放在`try/except`块**外面**、`threading.Thread(target=self.loop, ...).start()`**之前**——`_log_file`此时已经保证是打开状态或者是`None`（如果`open()`失败被上面的`except`吞掉），`ResourceMonitor.start()`内部对`_log_file`是`None`的情况已经在`_sample_once()`里用`if self._log_file:`判断过，不会崩）

- [ ] **Step 4: 在`stop_all()`里停止监控（必须在关闭`_log_file`之前）**

找到`stop_all()`方法（第1208-1222行左右）：

```python
    def stop_all(self):
        logger.info("任务结束")
        try:
            if self._log_file:
                self._log_file.close()
        except Exception:
            pass
```

改成：

```python
    def stop_all(self):
        logger.info("任务结束")
        self._resource_monitor.stop()
        try:
            if self._log_file:
                self._log_file.close()
        except Exception:
            pass
```

- [ ] **Step 5: 运行现有全部测试确认没有破坏行为**

Run: `cd drone_control/circle_pole && python -m pytest -q`
Expected: 全部通过（134 passed，`ResourceMonitor`在测试环境下`start()`会真的起一个线程，但现有测试构造的`mission`对象大多不调用`start()`/`stop_all()`，不会触发新代码路径；需要确认这一点——如果某个现有测试确实调用了`start()`，检查该测试是否因为多起了一个线程导致意外副作用，如有需要在该测试里在`teardown`阶段调用`mission._resource_monitor.stop()`清理）

- [ ] **Step 6: Commit**

```bash
cd drone_control/circle_pole
git add Mission_GPT.py
git commit -m "circle_pole: 接入ResourceMonitor到mission启停生命周期"
```

---

## Task 6: 集成测试 — mission.start()/stop_all()真实驱动ResourceMonitor

**Files:**
- Modify: `drone_control/circle_pole/test_resource_monitor.py`（追加集成测试）

- [ ] **Step 1: 写失败的测试**

在`test_resource_monitor.py`末尾追加（复用`test_land_logging.py`里`_make_mission_for_land()`的构造模式，但这里不需要`FakeRealsense`，因为只测日志文件里有没有出现resource事件）：

```python
class TestMissionIntegration:
    def test_mission_start_stop_writes_resource_events(self, monkeypatch, tmp_path):
        import Lcode.resource_monitor as rm_module

        monkeypatch.setattr(rm_module, "SAMPLE_INTERVAL_S", 0.02)
        monkeypatch.setattr(rm_module.psutil, "cpu_percent", lambda interval=None: 5.0)
        monkeypatch.setattr(rm_module.psutil, "virtual_memory", lambda: FakeVirtualMemory())
        monkeypatch.setattr(rm_module.psutil, "sensors_temperatures", lambda: {})

        from Mission_GPT import mission

        re_fc = [0] * 14
        se_fc = [0] * 11
        m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=None)

        # start()内部逻辑较多(涉及T265置信度确认等待、input()人工确认等)，
        # 这里不整体调用start()，只单独驱动跟本测试相关的两行(打开文件+起监控)，
        # 复刻start()里第320行之后的行为，避免测试卡在input()等待用户输入上。
        log_path = tmp_path / "flight_data.jsonl"
        m._log_file = open(log_path, "a")
        m._resource_monitor.start(m._log_file, m._log_lock)

        import time
        time.sleep(0.1)

        m.stop_all()

        content = log_path.read_text()
        lines = [json.loads(line) for line in content.strip().split("\n") if line]
        resource_lines = [e for e in lines if e.get("event") == "resource"]
        assert len(resource_lines) >= 2
```

- [ ] **Step 2: 运行测试确认通过**

Run: `cd drone_control/circle_pole && python -m pytest test_resource_monitor.py::TestMissionIntegration -v`
Expected: PASS。如果FAIL，检查`mission.__init__`是否真的创建了`self._resource_monitor`（Task 5 Step 2），以及`stop_all()`是否调用了`self._resource_monitor.stop()`（Task 5 Step 4）

- [ ] **Step 3: 运行全部测试确认通过**

Run: `cd drone_control/circle_pole && python -m pytest -q`
Expected: 全部通过

- [ ] **Step 4: Commit**

```bash
cd drone_control/circle_pole
git add test_resource_monitor.py
git commit -m "circle_pole: 新增mission.start()/stop_all()驱动ResourceMonitor的集成测试"
```

---

## Task 7: 同步到板子并真机冒烟测试

**Files:**
- 无代码修改，只做部署+验证

- [ ] **Step 1: 确认板子上psutil可用**

Run: `ssh ubuntu-pi "python3 -c 'import psutil; print(psutil.__version__)'"`
Expected: 打印版本号（此前确认过是5.9.0），不报错

- [ ] **Step 2: 同步改动到板子**

Run:
```bash
scp "drone_control/circle_pole/Lcode/resource_monitor.py" ubuntu-pi:/home/sunrise/Desktop/FJJ/circle_pole/Lcode/resource_monitor.py
scp "drone_control/circle_pole/test_resource_monitor.py" ubuntu-pi:/home/sunrise/Desktop/FJJ/circle_pole/test_resource_monitor.py
scp "drone_control/circle_pole/Mission_GPT.py" ubuntu-pi:/home/sunrise/Desktop/FJJ/circle_pole/Mission_GPT.py
scp "drone_control/circle_pole/requirements.txt" ubuntu-pi:/home/sunrise/Desktop/FJJ/circle_pole/requirements.txt
```
Expected: 无报错

- [ ] **Step 3: 板子上核对换行符约定**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ && git diff --stat -- circle_pole/Mission_GPT.py circle_pole/requirements.txt; file circle_pole/Mission_GPT.py circle_pole/Lcode/resource_monitor.py circle_pole/test_resource_monitor.py"`
Expected: `Mission_GPT.py`/`requirements.txt`应保持跟`git show HEAD:<file> | file -`一致的换行符约定（历史上是CRLF，见项目CLAUDE.md"Pi sync line endings"），`git diff --stat`改动行数应该跟本次实际改动量级相符（不应该出现整文件几百上千行的假性重写）；新文件`resource_monitor.py`/`test_resource_monitor.py`是新增文件，没有历史约定，保持scp传过去的原样即可

- [ ] **Step 4: 板子上跑单元测试**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ/circle_pole && python3 -m pytest test_resource_monitor.py -v"`
Expected: 全部通过

- [ ] **Step 5: 板子上跑全部测试确认没有破坏其他模块**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ/circle_pole && python3 -m pytest -q"`
Expected: 全部通过

- [ ] **Step 6: 板子上git commit（不push，独立历史）**

Run: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ && git add circle_pole/Lcode/resource_monitor.py circle_pole/test_resource_monitor.py circle_pole/Mission_GPT.py circle_pole/requirements.txt && git commit -m 'circle_pole: 新增板载资源监控(CPU/内存/温度)'"`
Expected: commit成功

- [ ] **Step 7: 真机/台架冒烟测试（不需要真的起飞，跑一次DRY_RUN或短时间任务观察日志即可）**

具体运行方式跟当前测试流程一致（本计划不覆盖真机飞行安全确认流程，由使用本计划的人按项目现有规范执行）。核心验证点：任务运行期间`flight_data.jsonl`里每隔约1秒出现一行`{"event": "resource", ...}`，任务结束后新的resource行不再出现（`stop_all()`已生效），且原有的位置遥测行（`state`/`nav_mode`等字段的行）没有出现JSON格式损坏（证明加锁生效、没有写入交错）。

Run（任务结束后检查）: `ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ/circle_pole && python3 -c \"import json; [json.loads(l) for l in open('flight_data.jsonl')]; print('全部行JSON格式正常')\""`
Expected: 打印"全部行JSON格式正常"，不报`json.decoder.JSONDecodeError`

---

## Task 8: 同步测试数据/文档收尾

**Files:**
- 无代码修改

- [ ] **Step 1: 把Task 7冒烟测试产生的flight_data.jsonl（如果是新的一次任务运行）归档**

按项目现有归档约定（`FJJ/test_data/<版本>_<日期>/`），如果Task 7只是跑了极短的DRY_RUN/台架测试不产生有意义的飞行数据，此步骤可以跳过——归档与否由执行者根据实际测试性质判断，不是强制步骤。

- [ ] **Step 2: 确认本机分支状态干净**

Run: `git status`
Expected: working tree clean（Task 1-6的commit都已经在本机仓库里）

- [ ] **Step 3: push到远程**

Run: `git push`
Expected: push成功
