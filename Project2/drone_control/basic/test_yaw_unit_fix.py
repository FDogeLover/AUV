"""yaw 修正回路弧度/角度单位不匹配 — 修复已回退，本文件现在测试"回退后的安全状态"。

背景：`yaw_pid` 用的增益(Kp=1.5等)是按"角度"量级设计的，但 `navigate()`/
`takeoff()` 里喂给它的 `yaw` 参数是T265原始朝向，单位是弧度。几度的误差
换算成弧度只有0.05-0.1量级，PID输出乘VEL_SCALE后还是远小于1，最后
`int(...)`截断成0——导致这条回路在正常飞行范围内(误差几度)完全不发指令。
2026-07-08真机飞行801帧`vyaw`全部为0印证了这一点。

2026-07-08曾经把喂给yaw_pid的参数从弧度改成`math.degrees(yaw)`，修复了这个
截断问题。但2026-07-09真机首次验证这个修复时触发了严重的yaw失控事故(飞机
持续同方向偏转近80-90度，人工介入才控制住)——已紧急回退成喂弧度，恢复到
"yaw修正回路事实上恒输出≈0"的已知安全状态。根本原因(疑似vyaw符号在物理
执行层面跟文档约定不一致，或者跟持续闭环运行的动态特性有关)截至2026-07-09
仍未定位，转盘验证+开环脉冲测试排除了"简单符号反了"这个假设，但没有找到能
安全重新启用的证据。详见 CLAUDE.md 已知问题16。

**这个文件的用途已经改变**：不再验证"修复生效"，而是作为回退状态的回归
守卫——如果以后有人不小心把 `navigate()`/`takeoff()` 里的
`self.yaw_pid.get_pid(yaw)` 又改回 `math.degrees(yaw)`，这里的测试会失败，
提醒这个改动需要先解决问题16记录的失控根因，不能直接重新启用。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic && python -m pytest test_yaw_unit_fix.py -v
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
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
    m.t265_ok = True
    m.realsense = FakeRealsenseYaw()
    return m


class TestYawUnitFixReverted:
    def test_realistic_yaw_error_still_produces_zero_correction(self):
        """2026-07-09事故后已回退成喂弧度：今天真机实测过的yaw误差(6.12度)，
        换算成弧度后PID输出仍会被int()截断成0——这是当前刻意维持的安全(但也
        意味着无效)状态，不是bug。如果这个断言开始失败，说明yaw_pid又被改成
        接收角度了，必须先看问题16确认失控根因已解决才能这么做。"""
        m = _make_mission()
        target = m.targets[0]  # 默认航点，跟pos一致，只有yaw有误差
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        vyaw_sent = sent[0][2]
        assert vyaw_sent == 0

    def test_small_yaw_error_still_zero(self):
        """误差很小(0.5度)时输出仍然是0，跟回退前的死区行为一致。"""
        m = _make_mission()
        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(0.05)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        assert sent[0][2] == 0
