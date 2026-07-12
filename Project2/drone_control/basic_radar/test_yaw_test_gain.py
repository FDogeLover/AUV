"""问题16：yaw修正回路增益开关(YAW_TEST_KP)。

2026-07-12递进式真机复测确认稳定性边界在[0.45,0.5]之间(0.3/0.4/0.45均收敛)，
但0.45的"默认行为"复测样本出现了跟第一次不一致的结果(峰值10.38°未收敛+可见
旋转)，run-to-run方差偏大；0.4的唯一样本更保守(全程未观察到可见旋转)。
默认值已从0(禁用)改为0.4(启用，喂角度+此增益闭环)。设置环境变量
DRONE_YAW_TEST_KP=0可临时回退到旧的"喂弧度，不修正"安全状态
(应急回滚用，见test_yaw_unit_fix.py的回归守卫)。

运行：
    cd drone_control/basic_radar && python -m pytest test_yaw_test_gain.py -v
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


class TestYawCorrectionEnabledByDefault:
    def test_default_kp_is_0p4(self):
        """模块级默认值必须是0.4(问题16真机验证过、比0.45更保守的边界内数值)，不能被意外改动。"""
        assert mg.YAW_TEST_KP == 0.4

    def test_mission_flagged_enabled_by_default(self):
        m = _make_mission()
        assert m._yaw_test_enabled is True

    def test_default_produces_nonzero_correction(self):
        """默认配置下，今天真机实测过的yaw误差(6.12度)应该能产生非零修正指令
        ——这是"默认已启用"的直接验证，不再是恒输出0的旧安全回退状态。"""
        m = _make_mission()
        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        assert sent[0][2] != 0


class TestYawCorrectionCanBeDisabled:
    def test_env_zero_disables_and_produces_zero_correction(self, monkeypatch):
        """应急回滚路径：显式设置DRONE_YAW_TEST_KP=0(即monkeypatch模块常量为0)，
        必须精确回退到test_yaw_unit_fix.py验证过的旧安全状态(喂弧度，恒输出0)。"""
        monkeypatch.setattr(mg, "YAW_TEST_KP", 0)
        m = _make_mission()
        assert m._yaw_test_enabled is False

        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert sent[0][2] == 0

    def test_other_gain_values_still_enable_correction(self, monkeypatch):
        """非0的任意增益(比如问题16测过的0.3)都应保持启用状态，不只是默认值0.4。"""
        monkeypatch.setattr(mg, "YAW_TEST_KP", 0.3)
        m = _make_mission()
        assert m._yaw_test_enabled is True

        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert sent[0][2] != 0
