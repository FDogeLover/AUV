"""yaw 修正回路弧度/角度单位不匹配的回归测试。

背景：`yaw_pid` 用的增益(Kp=1.5等)是按"角度"量级设计的，但 `navigate()`/
`takeoff()` 里喂给它的 `yaw` 参数是T265原始朝向，单位是弧度。几度的误差
换算成弧度只有0.05-0.1量级，PID输出乘VEL_SCALE后还是远小于1，最后
`int(...)`截断成0——导致这条回路在正常飞行范围内(误差几度)完全不发指令。
2026-07-08真机飞行801帧`vyaw`全部为0印证了这一点。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_yaw_unit_fix.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

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


class TestYawUnitFix:
    def test_realistic_yaw_error_produces_nonzero_correction(self):
        """今天真机实测过的yaw误差(6.12度)，修复前PID输出被int()截断成0，
        修复后应该能产生非零的yaw修正指令。"""
        m = _make_mission()
        target = m.targets[0]  # 默认航点，跟pos一致，只有yaw有误差
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        vyaw_sent = sent[0][2]
        assert vyaw_sent != 0

    def test_small_yaw_error_still_zero(self):
        """误差很小(0.5度)时输出仍然可能是0，这是合理的(死区)，不是bug。"""
        m = _make_mission()
        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(0.05)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        # 不断言具体值，只是记录这个边界情况供参考
