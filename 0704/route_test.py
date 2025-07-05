import serial
import time
from test_class_GPIO import *

# 串口参数，根据实际情况修改
SERIAL_PORT = "/dev/ttyUSB0"      # 串口设备名，比如COM3、/dev/ttyUSB0
BAUD_RATE = 9600              # 波特率
TIMEOUT = 1                     # 超时时间（秒）

LaserIndex1 = [2, 3, 4, 5, 6, 7]
LaserIndex2 = [10, 11, 12, 13, 14, 15]


LocationIndex1 = ["A1", "A2", "A3", "A4", "A5", "A6"]
LocationIndex2 = ["B1", "B2", "B3", "B4", "B5", "B6"]

# 内容点表
ItemIndex1 = ['2', '3', '4', '5', '6', '7']
ItemIndex2 = ['10', '11', '12', '13', '14', '15']

blue_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

Gpio = GpioCtrl()

for index  in range(15):
    print("当前节点为node{index}")
    if index in LaserIndex1:
        LaserShotFlag1 = 1
        data_str = LocationIndex1[index - 2] + " " + ItemIndex1[index - 2] + " "
        blue_port.write(data_str.encode('utf-8'))
        Gpio.laser_set_1(1)
    else:
        LaserShotFlag1 = 0
    if index in LaserIndex2:
        LaserShotFlag2 = 1
        data_str = LocationIndex2[index - 10] + " " + ItemIndex2[index - 10] + " "
        blue_port.write(data_str.encode('utf-8'))
        Gpio.laser_set_2(1)
    else:
        LaserShotFlag2 = 0
    if index == 8:
        CamearSpinFlag = 1
        Gpio.duoji_move_servo(180)
    time.sleep(3)
