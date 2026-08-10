"""basic 模块的 pytest 配置和共享测试 fixtures。"""
import os
import sys

# 确保 Lcode 模块可导入
_module_dir = os.path.dirname(__file__)
if _module_dir not in sys.path:
    sys.path.insert(0, _module_dir)

# 确保 drone_control 根目录在 sys.path 中，以便导入 shared_tests
_drone_control_root = os.path.dirname(_module_dir)
if _drone_control_root not in sys.path:
    sys.path.insert(0, _drone_control_root)

import pytest


@pytest.fixture
def expected_gpio_pin():
    """basic 模块使用 BCM5 作为按键默认引脚。"""
    return 5


@pytest.fixture
def expected_led_pins():
    """basic 模块的 LED 引脚映射(避免 UART7 引脚)。"""
    return {'R': 6, 'G': 25, 'B': 24}


@pytest.fixture
def mission_extra_kwargs():
    """basic 模块的 mission() 构造函数不需要额外参数。"""
    return {}
