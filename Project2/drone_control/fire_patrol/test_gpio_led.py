import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.gpio_led import set_rgb_led


class TestSetRgbLed:
    def test_returns_false_when_gpio_unavailable(self):
        """本机(Windows)开发环境没有Hobot.GPIO，应该优雅降级返回False而不是抛异常
        (板载环境有Hobot.GPIO时会走真实GPIO分支，见Lcode/gpio_led.py说明)。
        颜色合法性校验(比如传入不支持的颜色名)需要真实GPIO可用才会执行到，
        本机测不到那条分支，只能在板载环境手动验证。"""
        result = set_rgb_led('R')
        assert result is False
