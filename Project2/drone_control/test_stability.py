"""Unit tests for the stability refactor (Lpid D term, Mission height ramp,
closed-loop takeoff).

Run on the Raspberry Pi (where simple_pid, colorlog, pyserial, numpy, wiringpi
are installed):

    cd drone_control && python -m pytest test_stability.py -v

On a dev machine lacking the Raspberry-Pi-only `wiringpi` module, stub it before
import, e.g. at the top of a conftest.py:

    import sys, types; sys.modules.setdefault('wiringpi', types.ModuleType('wiringpi'))
"""
import pytest
import sys
import os

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
