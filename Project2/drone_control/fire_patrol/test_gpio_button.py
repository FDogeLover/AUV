import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.gpio_button import GpioButton, BUTTON_PIN_DEFAULT


class TestGpioButtonStart:
    def test_start_returns_false_when_gpio_unavailable(self):
        """本机(Windows)开发环境没有Hobot.GPIO，start()应该优雅降级返回False
        而不是抛异常(板载环境有Hobot.GPIO时会走真实GPIO分支)。"""
        btn = GpioButton()
        assert btn.start() is False

    def test_was_pressed_false_before_start(self):
        btn = GpioButton()
        assert btn.was_pressed() is False

    def test_default_pin_matches_reference_wiring(self):
        """默认引脚应该跟Desktop/GPIO测试/按键测试.ipynb验证过的接线(BCM17)一致。"""
        assert BUTTON_PIN_DEFAULT == 17

    def test_stop_before_start_does_not_raise(self):
        btn = GpioButton()
        btn.stop()  # 不应该抛异常
