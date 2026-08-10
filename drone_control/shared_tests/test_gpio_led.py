"""GPIO LED测试的共享实现。

各子模块的 RGB LED 实现基本相同，但引脚映射可能不同。通过 pytest fixture
提供模块特定的预期引脚映射。

使用方式：在各子模块的 conftest.py 中定义：
    @pytest.fixture
    def expected_led_pins():
        return {'R': 6, 'G': 25, 'B': 24}  # 取决于模块

然后在 test_gpio_led.py 中导入：
    from shared_tests.test_gpio_led import TestSetRgbLed
"""
import pytest

from Lcode.gpio_led import set_rgb_led, _get_gpio


def _gpio_available():
    return _get_gpio() is not None


class TestSetRgbLed:
    def test_pin_map_matches_expected(self, expected_led_pins):
        """LED引脚映射应该跟各模块验证过的接线一致。"""
        from Lcode.gpio_led import LED_PINS
        assert LED_PINS == expected_led_pins

    def test_does_not_raise_and_returns_bool(self):
        """两边环境都要能跑：本机(Windows)没有Hobot.GPIO，降级返回False；
        板载环境有Hobot.GPIO时走真实GPIO分支返回True(2026-07-16板载实测确认
        真的点亮了LED，之后手动关闭)。不管哪边，都不应该抛异常。"""
        result = set_rgb_led('R')
        assert isinstance(result, bool)
        set_rgb_led('OFF')  # 不留灯亮着，两边环境都执行(本机是空操作)

    def test_matches_gpio_availability_on_this_machine(self):
        """结果应该跟这台机器上Hobot.GPIO是否真的可导入一致，不能反过来。"""
        expected = _gpio_available()
        result = set_rgb_led('R')
        set_rgb_led('OFF')
        assert result is expected

    def test_unsupported_color_returns_false_when_gpio_available(self):
        if not _gpio_available():
            pytest.skip("需要真实Hobot.GPIO才能测到颜色校验分支，本机跳过")
        assert set_rgb_led('PURPLE') is False
