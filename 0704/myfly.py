import pyrealsense2 as rs
import serial
import numpy as np
import time
from threading import Thread, Timer
import threading
import csv
import DataDeal as dd
import struct
import math
import OpenVision as ov
import imutils
from test_class_GPIO import *


class FlyController:
    def __init__(self,Gpio, route_file='router.txt', serial_port="/dev/ttyS0", baudrate=230400,
                 blue_port="/dev/ttyUSB0", blue_baudrate=9600):
        # 控制变量
        self.Gpio = Gpio
        self.CopterTakingOff = 1
        self.CopterLanding = 0
        self.SendTargetPos = 0
        self.routeStartFlag = False
        self.LaserArray = 0
        self.LaserDistance = 0
        self.FlightMode = 0

        # 激光与舵机标志
        self.LaserShotFlag1 = 0
        self.LaserShotFlag2 = 0
        self.CamearSpinFlag = 0

        # 路径点
        self.routeNodeIndex = 1
        self.routeList = []
        self.routeNodeNum = 0
        self.timer = None
        self.TargetPosition = [0.0, 0.0, 0.0]

        # 设备与通信
        self.pipe = None
        self.cfg = None
        self._265Ready = False
        self.GetOnceCmd = False
        self.CheckSum = 0

        # 激光点表
        self.LaserIndex1 = [2, 3, 4, 5, 6, 7]
        self.LaserIndex2 = [10, 11, 12, 13, 14, 15]

        # 坐标点表
        self.LocationIndex1 = ["A1", "A2", "A3", "A4", "A5", "A6"]
        self.LocationIndex2 = ["B1", "B2", "B3", "B4", "B5", "B6"]

        # 内容点表
        self.ItemIndex1 = ['2', '3', '4', '5', '6', '7']
        self.ItemIndex2 = ['10', '11', '12', '13', '14', '15']

        # 数据处理实例
        self.data_dealer = dd.DataDeal()

        # 路径文件加载
        self.load_route(route_file)

        # 串口
        self.port = serial.Serial(port=serial_port, baudrate=baudrate, stopbits=1, parity=serial.PARITY_NONE, timeout=1)
        self.blue_port = serial.Serial(port=blue_port, baudrate=blue_baudrate, stopbits=1, parity=serial.PARITY_NONE, timeout=20)

        # 数据缓冲区
        self.dataBuf = [0] * 65
        self.ifTakeOff = True
        self.count = 0

        # 通信线程
        self.thread_Serial = Thread(target=self.PortCom)
        self.thread_Serial.setDaemon(True)
        self.thread_Serial.start()

    def load_route(self, route_file):
        with open(route_file, newline='') as csvfile:
            routeCsv = csv.reader(csvfile)
            self.routeList = list(routeCsv)
        self.routeNodeNum = len(self.routeList)
        print("route nodes num is : " + str(self.routeNodeNum - 1))
        self.routeNodeIndex = 1

    def Router(self, name):
        if self.routeNodeIndex < self.routeNodeNum and self.routeStartFlag:
            self.TargetPosition[0] = float(self.routeList[self.routeNodeIndex][0])
            self.TargetPosition[1] = float(self.routeList[self.routeNodeIndex][1])
            self.TargetPosition[2] = float(self.routeList[self.routeNodeIndex][2])
            duration = float(self.routeList[self.routeNodeIndex][3])
            self.LaserArray = int(self.routeList[self.routeNodeIndex][4])
            self.LaserDistance = float(self.routeList[self.routeNodeIndex][5])
            self.FlightMode = int(self.routeList[self.routeNodeIndex][6])

            print("route node %d: x : %.1f , y : %.1f , z : %.1f , time : %.1f s ,Arrray : %d , Dis : %.1f , S : %d" % (
                self.routeNodeIndex, self.TargetPosition[0], self.TargetPosition[1], self.TargetPosition[2],
                duration, self.LaserArray, self.LaserDistance, self.FlightMode))

            self.SendTargetPos = 1
            self.SetFlag(self.routeNodeIndex)
            self.routeNodeIndex += 1
            self.timer = threading.Timer(duration, self.Router, ["Router"])
            self.timer.start()
        else:
            self.CopterTakingOff = 1
            self.CopterLanding = 1
            self.routeNodeIndex = 1
            print("Landing")
            if self.timer:
                self.timer.cancel()

    def SetFlag(self, index):
        if index in self.LaserIndex1:
            self.LaserShotFlag1 = 1
            data_str = self.LocationIndex1[index-2] + " " +self.ItemIndex1[index-2] + " "
            self.blue_port.write(data_str.encode('utf-8'))
            self.Gpio.laser_set_1(1)
        else:
            self.LaserShotFlag1 = 0
        if index in self.LaserIndex2:
            self.LaserShotFlag2 = 1
            data_str = self.LocationIndex2[index - 10] + " " + self.ItemIndex2[index - 10] + " "
            self.blue_port.write(data_str.encode('utf-8'))
            self.Gpio.laser_set_2(1)
        else:
            self.LaserShotFlag2 = 0
        if index == 8:
            self.CamearSpinFlag = 1
            self.Gpio.duoji_move_servo(180)

    def PortCom(self):
        while True:
            response = self.port.readline()
            if (response):
                self.port.flushInput()
                CMD = str(response)
                # 刷新265
                if (response == b'Start265\n' and self.GetOnceCmd == False):
                    print(response)
                    self.pipe = rs.pipeline()
                    self.cfg = rs.config()
                    self.cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
                    self.pipe.start(self.cfg)
                    self.SendTargetPos = 0
                    self.CopterLanding = 0
                    self._265Ready = True
                    self.GetOnceCmd = True
                    self.routeStartFlag = True
                elif (response == b'Departures\n' and self.CopterTakingOff == 1):
                    print(response)
                    print("Get!")
                    self.Router("first")
                    self.CopterTakingOff = 0
                elif (response == b'Refresh265\n'):
                    self._265Ready = False
                    self.GetOnceCmd = False
                    self.routeNodeIndex = 1
                    self.CopterTakingOff = 1
                    self.routeStartFlag = False
                    print(response)
                    print("ReStart!")
                    try:
                        if self.pipe:
                            self.pipe.stop()
                        time.sleep(1.0)
                    except Exception as e:
                        print("Error2:", e)
                time.sleep(0.02)

    def main_loop(self):
        try:
            while True:
                if self._265Ready:
                    try:
                        frames = self.pipe.wait_for_frames()
                        pose = frames.get_pose_frame()
                        if pose:
                            data = pose.get_pose_data()
                            self.dataBuf, pos_X, pos_Y, pos_Z, Euler = self.data_dealer.solve_data(data)
                            # 目标点下发
                            if self.SendTargetPos == 1:
                                posX = self.TargetPosition[0]
                                posY = self.TargetPosition[1]
                                posZ = self.TargetPosition[2]
                                self.dataBuf[43] = 0x20
                                posX_buf = struct.pack("f", posX)
                                self.dataBuf[44:48] = list(posX_buf)
                                posY_buf = struct.pack("f", posY)
                                self.dataBuf[48:52] = list(posY_buf)
                                posZ_buf = struct.pack("f", posZ)
                                self.dataBuf[52:56] = list(posZ_buf)
                                self.dataBuf[56] = self.LaserArray
                                Laser_Dis = struct.pack("f", self.LaserDistance)
                                self.dataBuf[57:61] = list(Laser_Dis)
                                self.dataBuf[61] = self.FlightMode
                            # 降落标志
                            if self.CopterLanding == 1:
                                self.dataBuf[62] = 0xA5
                            else:
                                self.dataBuf[62] = 0x00
                            # 校验和
                            self.CheckSum = 0
                            for i in range(0, 62):
                                self.CheckSum += self.dataBuf[i]
                            self.dataBuf[63] = 0xAA
                            self.dataBuf[64] = self.CheckSum & 0x00ff
                            # 输出状态
                            print("\rrpy_rad[0]:{:.2f},rpy_rad[1]:{:.2f},rpy_rad[2]:{:.2f} ,X:{:.2f},Y:{:.2f},Z:{:.2f} ".format(
                                Euler[0] * 57.3, Euler[1] * 57.3, Euler[2] * 57.3, pos_X, pos_Y, pos_Z), end=" ")
                            if self.ifTakeOff:
                                self.count += 1
                            if self.count == 100:
                                self.count = 0
                                self.ifTakeOff = False
                                self.dataBuf[0] = 0x55
                                self.dataBuf[1] = 0xAA
                                self.dataBuf[2] = 0x40
                                self.dataBuf[63] = 0xAA
                                self.dataBuf[64] = 0x00
                            self.port.write(bytearray(self.dataBuf))
                            self.CheckSum = 0
                    except Exception as e:
                        print("Frame exception:", e)
                        time.sleep(0.1)
                else:
                    self.dataBuf[0] = 0x55
                    self.dataBuf[1] = 0xAA
                    self.dataBuf[2] = 0xFF
                    self.dataBuf[63] = 0xAA
                    self.dataBuf[64] = 0x00
                    self.port.write(bytearray(self.dataBuf))
                    time.sleep(0.1)
        except Exception as e:
            print("some error:", e)
        finally:
            try:
                if self.pipe:
                    self.pipe.stop()
            except Exception as e:
                print("pipe stop error:", e)


if __name__ == '__main__':
    Gpio = GpioCtrl()
    fc = FlyController(Gpio)
    fc.main_loop()