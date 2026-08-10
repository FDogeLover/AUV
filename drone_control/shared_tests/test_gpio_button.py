"""GPIO按键测试的共享实现。

各子模块的 GpioButton 实现相同，但默认引脚可能不同(如 basic 用 BCM5，
fire_patrol/competition_2026 用 BCM17)。通过 pytest fixture 提供模块特定值。

使用方式：在各子模块的 conftest.py 中定义：
    @pytest.fixture
    def expected_gpio_pin():
        return 5  # 或 17，取决于模块

然后在 test_gpio_button.py 中导入：
    from shared_tests.test_gpio_button import TestGpioButtonStart
"""
import pytest


def _gpio_available():
    try:
        import Hobot.GPIO  # noqa: F401
    except ImportError:
        return False
    return True


class TestGpioButtonStart:
    def test_start_matches_gpio_availability_on_this_machine(self):
        """本机(Windows)没有Hobot.GPIO，start()应该优雅降级返回False；板载环境
        有Hobot.GPIO时走真实GPIO分支返回True。结果应该跟这台机器上Hobot.GPIO
        是否真的可导入一致，不能反过来，也不应该抛异常。"""
        from Lcode.gpio_button import GpioButton
        btn = GpioButton()
        try:
            result = btn.start()
            assert result is _gpio_available()
        finally:
            btn.stop()

    def test_was_pressed_false_before_start(self):
        from Lcode.gpio_button import GpioButton
        btn = GpioButton()
        assert btn.was_pressed() is False

    def test_default_pin_matches_reference_wiring(self, expected_gpio_pin):
        """默认引脚应该跟各模块验证过的接线一致。"""
        from Lcode.gpio_button import BUTTON_PIN_DEFAULT
        assert BUTTON_PIN_DEFAULT == expected_gpio_pin

    def test_stop_before_start_does_not_raise(self):
        from Lcode.gpio_button import GpioButton
        btn = GpioButton()
        btn.stop()  # 不应该抛异常
