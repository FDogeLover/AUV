"""land() 降落等待期间飞行日志记录的单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_land_logging.py -v
"""
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import mission


class FakeRealsense:
    """伪造T265接口，land()会在等待循环里自己重新采样，不依赖外部传入的旧值。"""
    def __init__(self, pos, yaw, vel=(0.0, 0.0, 0.0)):
        self._pos = pos
        self._yaw = yaw
        self._vel = vel

    def get_position(self):
        return list(self._pos)

    def get_orientation(self):
        return [0.0, 0.0, self._yaw]

    def get_velocity(self):
        return list(self._vel)


def _make_mission_for_land():
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=None)
    m.t265_ok = True
    m.realsense = FakeRealsense(pos=(0.12, -0.05, 0.03), yaw=0.01)
    m.state = "LAND"
    return m


class TestLandLogging:
    def test_land_logs_position_when_immediately_unlocked(self):
        """land()原本从触发到确认/超时全程不写任何飞行日志，导致降落物理下降过程
        完全没有位置数据(2026-07-08真机测试发现)。这里验证哪怕最简单的"一进来就
        已经上锁"这种情况，也至少要记录一条日志。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0  # 已经上锁，land()应该立刻确认退出

        m.land()

        m._log_file.seek(0)
        lines = [line for line in m._log_file.readlines() if line.strip()]
        assert len(lines) >= 1
        entry = json.loads(lines[-1])
        assert entry["state"] == "LAND"
        assert entry["pos"] == [0.12, -0.05, 0.03]
