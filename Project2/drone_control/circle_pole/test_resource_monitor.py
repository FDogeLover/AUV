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
            raising=False,  # Windows开发机上psutil无此属性(Linux专属)，板载环境(ubuntu-pi)存在
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
        monkeypatch.setattr(rm_module.psutil, "sensors_temperatures", lambda: {}, raising=False)

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
