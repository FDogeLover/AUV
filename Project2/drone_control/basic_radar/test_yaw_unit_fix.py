"""yaw 修正回路弧度/角度单位不匹配 — 已修复且已投入使用，本文件测试"应急回滚路径"。

背景：`yaw_pid` 用的增益是按"角度"量级设计的，但 `navigate()`/`takeoff()` 里
早期版本喂给它的 `yaw` 参数是T265原始朝向弧度，几度误差换算成弧度后PID输出
被 `int(...)` 截断成0——这条回路在正常飞行范围内完全不发指令。2026-07-08修复
成喂角度后，2026-07-09用原始Kp=1.5首次真机验证触发了近90°失控事故，因此
长期回退为喂弧度的"恒输出≈0"安全状态。

2026-07-12递进式真机复测(单独会话+小步长+人工全程待命)确认了稳定性边界在
[0.45,0.5]之间：Kp=0.3/0.4/0.45均收敛(0.45最干净)，Kp=0.5确认无界发散。据此
把`YAW_TEST_KP`默认值改为0.45，yaw修正回路正式投入使用(见test_yaw_test_gain.py)。

**这个文件的用途**：验证应急回滚路径——`DRONE_YAW_TEST_KP=0`必须精确恢复到
喂弧度、恒输出≈0的旧安全状态，供真机观察到异常时快速回退使用。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_yaw_unit_fix.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import Mission_GPT as mg
from Mission_GPT import mission


class FakeRealsenseYaw:
    def __init__(self, confidence=3, vel=(0.0, 0.0, 0.0)):
        self._confidence = confidence
        self._vel = vel

    def get_tracking_confidence(self):
        return self._confidence

    def get_velocity(self):
        return list(self._vel)


def _make_mission():
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=None)
    m.t265_ok = True
    m.realsense = FakeRealsenseYaw()
    return m


class TestYawEmergencyRollback:
    def test_realistic_yaw_error_produces_zero_correction_when_rolled_back(self, monkeypatch):
        """应急回滚(DRONE_YAW_TEST_KP=0，此处monkeypatch模拟)：今天真机实测过的
        yaw误差(6.12度)，换算成弧度后PID输出应被int()截断成0——精确恢复
        2026-07-09事故后维持过的安全(但无效)状态。如果这个断言开始失败，
        说明回滚路径本身坏了，必须先修好才能信任它作为应急手段。"""
        monkeypatch.setattr(mg, "YAW_TEST_KP", 0)
        m = _make_mission()
        target = m.targets[0]  # 默认航点，跟pos一致，只有yaw有误差
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        vyaw_sent = sent[0][2]
        assert vyaw_sent == 0

    def test_small_yaw_error_still_zero_when_rolled_back(self, monkeypatch):
        """回滚状态下，误差很小(0.5度)时输出仍然是0，跟旧的死区行为一致。"""
        monkeypatch.setattr(mg, "YAW_TEST_KP", 0)
        m = _make_mission()
        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(0.05)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        assert sent[0][2] == 0
