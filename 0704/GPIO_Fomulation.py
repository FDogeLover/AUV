import pyrealsense2 as rs
import serial
import numpy as np
import time
from threading import Thread
import threading
import csv
import DataDeal265 as dd
import struct
import math
import OpenVision as ov
import imutils
from test_class_GPIO import *

Gpio = GpioCtrl()
try:
    Gpio.laser_set_1(1)
    Gpio.duoji_move_servo(90)  # 使用实例方法
    Gpio.led_set_red(2)
    Gpio.led_set_green(2)
    Gpio.led_set_blue(2)
    Gpio.laser_set_1(1)
finally:
    Gpio.release()  # 确保最后释放资源
