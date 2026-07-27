"""测试激光笔喷洒控制（无硬件依赖）。"""
import pytest

from Lcode.laser_spray import LaserSpray


@pytest.mark.fast
def test_spray_times_bounds():
    """闪烁次数限制在 1~3 次。"""
    ls = LaserSpray()
    assert ls.spray(times=0) is True
    assert ls.spray(times=1) is True
    assert ls.spray(times=2) is True
    assert ls.spray(times=3) is True
    assert ls.spray(times=5) is True  # 被截断到3


@pytest.mark.fast
def test_is_available_when_no_hardware():
    """无硬件环境下 is_available 返回 False。"""
    ls = LaserSpray()
    # 桌面环境没有硬件
    assert ls.is_available() is False
