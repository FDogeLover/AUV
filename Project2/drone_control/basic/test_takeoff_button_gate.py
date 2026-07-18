"""一键起飞按钮门禁测试：仅验证GPIO调用时序，不启动飞控或任务线程。"""
import inspect
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import Mission_GPT
from Mission_GPT import TAKEOFF_BUTTON_POLL_S, TAKEOFF_WARN_LED_DURATION_S, mission


def _make_mission():
    item = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    item._log_file = None
    return item


class FakeButton:
    available = True
    pressed_values = [False, True]
    last_instance = None

    def __init__(self):
        self.started = False
        self.stopped = False
        self._pressed_values = iter(self.pressed_values)
        type(self).last_instance = self

    def start(self):
        self.started = True
        return self.available

    def was_pressed(self):
        return next(self._pressed_values)

    def stop(self):
        self.stopped = True


def test_green_wait_then_red_two_seconds(monkeypatch):
    colors = []
    sleeps = []
    FakeButton.available = True
    FakeButton.pressed_values = [False, True]
    monkeypatch.setattr("Lcode.gpio_button.GpioButton", FakeButton)
    monkeypatch.setattr(
        "Lcode.gpio_led.set_rgb_led",
        lambda color: colors.append(color) or True,
    )
    monkeypatch.setattr(Mission_GPT.time, "sleep", sleeps.append)

    assert _make_mission()._wait_for_takeoff_button() is True
    assert colors == ["G", "R", "OFF"]
    assert sleeps == [TAKEOFF_BUTTON_POLL_S, TAKEOFF_WARN_LED_DURATION_S]
    assert TAKEOFF_WARN_LED_DURATION_S == 5.0
    assert FakeButton.last_instance.stopped is True


def test_button_unavailable_fails_closed_and_stops(monkeypatch):
    colors = []
    FakeButton.available = False
    FakeButton.pressed_values = []
    monkeypatch.setattr("Lcode.gpio_button.GpioButton", FakeButton)
    monkeypatch.setattr(
        "Lcode.gpio_led.set_rgb_led",
        lambda color: colors.append(color) or True,
    )

    assert _make_mission()._wait_for_takeoff_button() is False
    assert colors == []
    assert FakeButton.last_instance.stopped is True


def test_green_led_failure_fails_closed_and_cleans_up(monkeypatch):
    colors = []
    FakeButton.available = True
    FakeButton.pressed_values = [True]
    monkeypatch.setattr("Lcode.gpio_button.GpioButton", FakeButton)

    def set_led(color):
        colors.append(color)
        return color == "OFF"

    monkeypatch.setattr("Lcode.gpio_led.set_rgb_led", set_led)

    assert _make_mission()._wait_for_takeoff_button() is False
    assert colors == ["G", "OFF"]
    assert FakeButton.last_instance.stopped is True


def test_takeoff_does_not_repeat_warning_gate():
    assert "_blink_warning_led" not in inspect.getsource(mission.takeoff)
