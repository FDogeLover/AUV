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

CopterTakingOff = 1
TargetPosition = [0.0, 0.0, 0.0]


# 定时更新路径点
def Router(name):
    global timer
    global routeNodeIndex
    global SendTargetPos
    global CopterLanding
    global LaserArray
    global LaserDistance
    global FlightMode
    global CopterTakingOff
    global routeStartFlag
    if routeNodeIndex < routeNodeNum and routeStartFlag == True:
        TargetPosition[0] = float(routeList[routeNodeIndex][0])
        TargetPosition[1] = float(routeList[routeNodeIndex][1])
        TargetPosition[2] = float(routeList[routeNodeIndex][2])
        time = float(routeList[routeNodeIndex][3])
        LaserArray = int(routeList[routeNodeIndex][4])
        LaserDistance = float(routeList[routeNodeIndex][5])
        FlightMode = int(routeList[routeNodeIndex][6])

        print("route node %d: x : %.1f , y : %.1f , z : %.1f , time : %.1f s ,Arrray : %d , Dis : %.1f , S : %d" % (
            routeNodeIndex, TargetPosition[0], TargetPosition[1], TargetPosition[2], time, LaserArray, LaserDistance,
            FlightMode))
        SendTargetPos = 1
        routeNodeIndex = routeNodeIndex + 1
        timer = threading.Timer(time, Router, ["Router"])
        timer.start()
    else:
        # SendTargetPos = 0
        CopterTakingOff = 1
        CopterLanding = 1
        routeNodeIndex = 1
        print("Landing")
        timer.cancel()


# 字符串对比
def StrComparison(str1, str2):
    n = len(str1)
    res = []
    for x in str1:
        if x in str2:
            res.append(x)
    # print (n)
    return (n - len(res))


# 串口通信线程
def PortCom(port):
    global pipe
    global cfg
    global SendTargetPos
    global CopterLanding
    global CopterTakingOff
    global _265Ready
    global GetOnceCmd
    global routeNodeIndex
    global routeStartFlag
    while (True):
        response = port.readline()
        if (response):
            port.flushInput()
            #             CmdStr1 = str(b'Start265\n')
            #             CmdStr2 = str(b'Departures\n')
            #             CmdStr3 = str(b'Refresh265\n')
            #             CMD = str(response)
            # 刷新265
            if (response == b'Start265\n' and GetOnceCmd == False):
                print(response)
                # Declare RealSense pipeline, encapsulating the actual device and sensors
                pipe = rs.pipeline()
                #                 try:
                #                     pipe.stop()
                #                 except:
                #                     print("Error1")
                # Build config object and request pose data
                cfg = rs.config()
                cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
                # Start streaming with requested config
                pipe.start(cfg)
                dd.initData()
                SendTargetPos = 0
                CopterLanding = 0
                _265Ready = True
                GetOnceCmd = True
                routeStartFlag = True

            elif (response == b'Departures\n' and CopterTakingOff == 1):
                print(response)
                print("Get!")
                Router("first")
                CopterTakingOff = 0

            elif (response == b'Refresh265\n'):
                _265Ready = False
                GetOnceCmd = False
                routeNodeIndex = 1
                CopterTakingOff = 1
                routeStartFlag = False
                print(response)
                print("ReStart!")
                try:
                    pipe.stop()
                    time.sleep(1.0)
                except:
                    print("Error2")
            response = 0
        time.sleep(0.02)


if __name__ == '__main__':
    global routeNodeIndex
    global SendTargetPos
    global CopterLanding
    global LaserArray
    global LaserDistance
    global FlightMode
    global pipe
    global _265Ready
    global GetOnceCmd
    global CheckSum

    port = serial.Serial(port="/dev/ttyS0", baudrate=230400, stopbits=1, parity=serial.PARITY_NONE, timeout=20)

    # 串口通信线程
    thread_Serial = Thread(target=PortCom, args=(port,))
    thread_Serial.start()
    # 导入路径文件
    routeCsv = csv.reader(open('router.txt'))
    routeList = list(routeCsv)
    routeNodeNum = len(routeList)
    # 输出路径点个数
    print("route nodes num is : " + str(routeNodeNum - 1))
    routeNodeIndex = 1
    _265Ready = False
    GetOnceCmd = False
    CheckSum = 0
    dataBuf = [0] * 65
    # 自启动标志
    ifTakeOff = True
    count = 0

    try:
        while (True):
            if _265Ready:
                # Wait for the next set of frames from the camera
                frames = pipe.wait_for_frames()
                # Fetch pose frame
                pose = frames.get_pose_frame()
                if pose:
                    # Print some of the pose data to the terminal
                    data = pose.get_pose_data()
                    dataBuf, pos_X, pos_Y, pos_Z, Euler = dd.solveData(data)
                    if (SendTargetPos == 1):
                        posX = TargetPosition[0]
                        posY = TargetPosition[1]
                        posZ = TargetPosition[2]

                        dataBuf[43] = 0x20
                        posX_buf = struct.pack("f", posX)
                        dataBuf[44] = posX_buf[0]
                        dataBuf[45] = posX_buf[1]
                        dataBuf[46] = posX_buf[2]
                        dataBuf[47] = posX_buf[3]
                        posY_buf = struct.pack("f", posY)
                        dataBuf[48] = posY_buf[0]
                        dataBuf[49] = posY_buf[1]
                        dataBuf[50] = posY_buf[2]
                        dataBuf[51] = posY_buf[3]
                        posZ_buf = struct.pack("f", posZ)
                        dataBuf[52] = posZ_buf[0]
                        dataBuf[53] = posZ_buf[1]
                        dataBuf[54] = posZ_buf[2]
                        dataBuf[55] = posZ_buf[3]

                        dataBuf[56] = LaserArray
                        Laser_Dis = struct.pack("f", LaserDistance)
                        dataBuf[57] = Laser_Dis[0]
                        dataBuf[58] = Laser_Dis[1]
                        dataBuf[59] = Laser_Dis[2]
                        dataBuf[60] = Laser_Dis[3]
                        dataBuf[61] = FlightMode

                    if CopterLanding == 1:
                        dataBuf[62] = 0xA5
                    else:
                        dataBuf[62] = 0x00

                    CheckSum = 0  # 每帧都要归零
                    for i in range(0, 62):
                        CheckSum = CheckSum + dataBuf[i]

                    dataBuf[63] = 0xAA
                    dataBuf[64] = CheckSum & 0x00ff

                    # 输出 pitch 、 roll 、 yaw ,x , y , z
                    print("\rrpy_rad[0]:{:.2f},rpy_rad[1]:{:.2f},rpy_rad[2]:{:.2f} ,X:{:.2f},Y:{:.2f},Z:{:.2f} ".format(
                        Euler[0] * 57.3, Euler[1] * 57.3, Euler[2] * 57.3, pos_X, pos_Y, pos_Z), end=" ")
                    # 自动判断,time.time()单位为秒
                    #                     if (ifTakeOff and time.time() - initTime >= 20):
                    #                         ifTakeOff = False
                    #                         dataBuf[0] = 0x55
                    #                         dataBuf[1] = 0xAA
                    #                         dataBuf[2] = 0x20
                    #                         dataBuf[63] = 0xAA
                    #                         dataBuf[64] = 0x00
                    #                         setGPIO()
                    # 在中途开启识别线程
                    if ifTakeOff == True:
                        count = count + 1
                    if count == 200:
                        count = 0
                        ifTakeOff = False
                        dataBuf[0] = 0x55
                        dataBuf[1] = 0xAA
                        dataBuf[2] = 0x40
                        dataBuf[63] = 0xAA
                        dataBuf[64] = 0x00
                    port.write(dataBuf)
                    # 添加自启动代码
                    CheckSum = 0
            #                     pipe.stop()
            else:
                dataBuf[0] = 0x55
                dataBuf[1] = 0xAA
                dataBuf[2] = 0xFF
                dataBuf[63] = 0xAA
                dataBuf[64] = 0x00
                port.write(dataBuf)
                time.sleep(0.1)
    finally:
        print("some erro")
#         pipe.stop()