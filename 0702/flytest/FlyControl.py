import pyrealsense2 as rs
import serial
import time
import csv
import struct
from DataDeal import DataDeal
import threading
from test_class_GPIO import *
# from test_class_visual import *
# from test_class_file import *


class FlyControl:

    def __init__(self):
        self.data_deal = DataDeal()
        self.routeNodeIndex = 1
        self.SendTargetPos = 0
        self.CopterLanding = 0
        self.LaserArray = 0
        self.LaserDistance = 0
        self.FlightMode = 0
        self.CopterTakingOff = 1
        self.routeStartFlag = False
        self.pipe = None
        self.cfg = None
        self.copterLanding = False
        self._265Ready = False
        self.GetOnceCmd = False
        self.routeNodeNum = 0
        self.TargetPosition = [0.0, 0.0, 0.0]
        self.CheckSum = 0
        self.routeCsv = None
        self.routeList = None
        self.dataBuf = [0] * 65
        self.ifTakeOff = True
        self.count = 0
        self.pos_X = 0.
        self.pos_Y = 0.
        self.pos_Z = 0.
        self.Euler = 0.

        self.port = None
        self.route_road_permit = True
        self.timer = None

        self.route_once = True
        self.route_block_index = [3, 5, 6, 8, 9, 11, 14, 15, 16, 18, 19, 20, 23, 24, 25, 27, 28, 29]
        self.duoji_rotate_index = [14, 15, 16, 18, 19, 20]
        self.duoji_rotate_d = True

        self.route_index_add = True
        self.turn_b_c = 0
        self.gpio = GpioCtrl()
        self.vis_deal = VisualDeal()
        self.file = FileHandler('data.txt')

        self.fly_mode = 1
        self.router_mode2_break_index = None
        self.turn_laser = True
        self.turn_duoji = True
        self.router_list = None
        pass

    def init(self):
        try:
            self.port = serial.Serial(port="/dev/ttyAMA0", baudrate=230400, stopbits=1, parity=serial.PARITY_NONE,
                                      timeout=1000)
            print("串口1已打开")
            if self.fly_mode == 1:
                self.routeCsv = csv.reader(open('router.txt'))
                self.routeList = list(self.routeCsv)
                self.routeNodeNum = len(self.routeList)
            elif self.fly_mode == 2:
                if self.router_list is not None:
                    self.routeList = self.router_list
                    self.routeNodeNum = len(self.routeList)
                else:
                    print("router_list is None")
            # 输出路径点个数
            print("route nodes num is : " + str(self.routeNodeNum - 1))
        except Exception as e:
            print("init error: ", e)

    def init_Gpio(self):
        self.gpio.init_gpio()
        self.gpio.camera_spin('left')
        self.gpio.init_key()

    def set_fly_mode(self, fly_mode):
        self.fly_mode = fly_mode

    def set_router_mode2_break_index(self, index):
        self.router_mode2_break_index = index

    def set_turn_laser(self, is_turn):
        self.turn_laser = is_turn

    def set_turn_duoji(self, is_turn):
        self.turn_duoji = is_turn

    def set_router_list(self, router_list):
        self.router_list = router_list

    def set_route_block_index(self, list_block):
        self.route_block_index = list_block

    def key_mode(self):
        key_fly_mode = self.gpio.key_mode()
        if key_fly_mode == 2:
            return 1
        elif key_fly_mode == 1:
            return 2

    def key_fly(self):
        self.gpio.key_fly()

    # 定时更新路径点
    def Router(self):
        if self.route_once:
            self.route_once = False
        elif self.route_road_permit:
            if self.route_index_add:
                self.routeNodeIndex = self.routeNodeIndex + 1
            else:
                self.route_index_add = True
        if self.fly_mode == 1:
            if self.routeNodeIndex in self.route_block_index:
                self.route_road_permit = False
                self.route_index_add = False
                print("Router 停止")
                self.route_block_index.remove(self.routeNodeIndex)
        elif self.fly_mode == 2:
            if self.routeNodeIndex == self.router_mode2_break_index:
                self.route_road_permit = False
                self.route_index_add = False
                print("Router 停止")
                self.router_mode2_break_index = -1
        # timer = None
        if self.route_road_permit:
            if self.routeNodeIndex < self.routeNodeNum and self.routeStartFlag == True:
                self.TargetPosition[0] = float(self.routeList[self.routeNodeIndex][0])
                self.TargetPosition[1] = float(self.routeList[self.routeNodeIndex][1])
                self.TargetPosition[2] = float(self.routeList[self.routeNodeIndex][2])
                time = float(self.routeList[self.routeNodeIndex][3])
                self.LaserArray = int(self.routeList[self.routeNodeIndex][4])
                self.LaserDistance = float(self.routeList[self.routeNodeIndex][5])
                self.FlightMode = int(self.routeList[self.routeNodeIndex][6])

                print(
                    "route node %d: x : %.1f , y : %.1f , z : %.1f , time : %.1f s ,Arrray : %d , Dis : %.1f , S : %d" % (
                        self.routeNodeIndex, self.TargetPosition[0], self.TargetPosition[1], self.TargetPosition[2],
                        time, self.LaserArray, self.LaserDistance, self.FlightMode))
                self.SendTargetPos = 1
                # self.routeNodeIndex = self.routeNodeIndex + 1
                self.timer = threading.Timer(time, self.Router)
                self.timer.start()
            else:
                # SendTargetPos = 0
                self.CopterTakingOff = 1
                self.CopterLanding = 1
                self.routeNodeIndex = 1
                print("Landing")
                if self.timer is not None:
                    self.timer.cancel()
        else:
            self.timer = threading.Timer(0.2, self.Router)
            self.timer.start()

    def StrComparison(self, str1, str2):
        n = len(str1)
        res = []
        for x in str1:
            if x in str2:
                res.append(x)
        # print (n)
        return (n - len(res))

    def PortCom(self):

        while (True):
            #         size=port.inWaiting()
            #         if(size!=0):
            response = self.port.readline()
            if (response != None):
                self.port.flushInput()
                CmdStr1 = str(b'Start265\n')
                CmdStr2 = str(b'Departures\n')
                CmdStr3 = str(b'Refresh265\n')
                CMD = str(response)
                # 刷新265
                if ((self.StrComparison(CMD, CmdStr1) <= 1) and self.GetOnceCmd == False):
                    print(self.StrComparison(CMD, CmdStr1), response, CMD)
                    # Declare RealSense pipeline, encapsulating the actual device and sensors
                    self.pipe = rs.pipeline()
                    #                 try:
                    #                     pipe.stop()
                    #                 except:
                    #                     print("Error1")
                    # Build config object and request pose data
                    self.cfg = rs.config()
                    self.cfg.enable_stream(rs.stream.pose, rs.format.any, framerate=200)
                    # Start streaming with requested config
                    self.pipe.start(self.cfg)
                    # self.data_deal.initData()
                    self.SendTargetPos = 0
                    self.CopterLanding = 0
                    self._265Ready = True
                    self.GetOnceCmd = True
                    self.routeStartFlag = True

                elif ((self.StrComparison(CMD, CmdStr2) <= 1) and self.CopterTakingOff == 1):
                    print(self.StrComparison(CMD, CmdStr2), response, CMD)
                    print("Get!")
                    self.Router()
                    self.CopterTakingOff = 0

                elif (self.StrComparison(CMD, CmdStr3) <= 1):
                    self._265Ready = False
                    self.GetOnceCmd = False
                    self.routeNodeIndex = 1
                    self.CopterTakingOff = 1
                    self.routeStartFlag = False
                    print("ReStart!")
                    print(self.StrComparison(CMD, CmdStr3), response, CMD)
                    try:
                        # self.pipe.stop()
                        time.sleep(1.0)
                    except:
                        print("Error2")
                response = 0
                CMD = 0
            time.sleep(0.02)

    def xuanting(self, time_xuanting: float, mode=1):
        self.route_road_permit = False
        position_x = self.pos_X
        position_y = self.pos_Y
        position_z = self.pos_Z
        self.TargetPosition[0] = position_x
        self.TargetPosition[1] = position_y
        self.TargetPosition[2] = position_z
        time.sleep(time_xuanting)
        if mode == 1:
            self.route_road_permit = False
        if mode == 2:
            self.route_road_permit = True

    def move(self, x, y, z, yaw, time_move, mode=2):
        self.TargetPosition[0] = x
        self.TargetPosition[1] = y
        self.TargetPosition[2] = z
        self.LaserDistance = yaw
        time.sleep(time_move)
        if mode == 1:
            self.route_road_permit = True
        if mode == 2:
            self.route_road_permit = False

    def deal(self, fly_mode=1):
        if fly_mode == 1:
            self.file.open_write()
            time.sleep(0.2)
            self.file.file_close()
            try:
                while True:
                    if self.routeNodeIndex == 22 and self.duoji_rotate_d == True:
#                         self.gpio.duoji_set(2)
                        self.gpio.camera_spin('right')
                        self.duoji_rotate_d = False
                        self.turn_b_c += 1
                        time.sleep(0.3)
                    if not self.route_road_permit:
                        index_bc_laser = self.turn_b_c % 2
#                         if index_bc_laser == 0:
#                             self.gpio.laser_set_1(0.5)
#                         elif index_bc_laser == 1:
#                             self.gpio.laser_set_2(0.5)
#                         time.sleep(0.1)
                        num_detect = 1
                        if index_bc_laser == 0:
                            self.gpio.laser_set_1(0.5)
                        elif index_bc_laser == 1:
                            self.gpio.laser_set_2(0.5)
                        if num_detect != -1:
                            # print("Value: ", num_detect)
                            # send message and write in file
                            if self.routeNodeIndex in self.duoji_rotate_index:
                                str_pos = Containers.dict_route_index_pos[self.routeNodeIndex][index_bc_laser]
                                print("str_pos: ", str_pos)
                            else:
                                str_pos = Containers.dict_route_index_pos[self.routeNodeIndex]
                                print("str_pos: ", str_pos)
                            # self.file.write_open(str_pos, num_detect)
                            # print("已写入数据")
                            # send message
                            # self.state_info_send(1, str_pos, num_detect)
                            pass
                        if self.routeNodeIndex in self.duoji_rotate_index:
                            self.duoji_rotate_index.remove(self.routeNodeIndex)
                            self.turn_b_c += 1
                            if self.turn_b_c % 2 == 0:
#                                 self.gpio.duoji_set(3)
#                                 self.gpio.laser_set_1(0.5)
                                self.gpio.camera_spin('left')
                            elif self.turn_b_c % 2 == 1:
                                self.gpio.camera_spin('right')
#                                 self.gpio.duoji_set(2)
#                                 self.gpio.laser_set_2(0.5)
                            time.sleep(0.1)
                            # self.gpio.laser_set_2(0.5)
                            num_detect = 1
                            if self.turn_b_c % 2 == 0:
#                                 self.gpio.duoji_set(3)
                                self.gpio.laser_set_1(0.5)
                            elif self.turn_b_c % 2 == 1:
#                                 self.gpio.duoji_set(2)
                                self.gpio.laser_set_2(0.5)
                            if num_detect != -1:
                                # print("Value: ", num_detect)
                                # send message and write in file
                                str_pos = Containers.dict_route_index_pos[self.routeNodeIndex][self.turn_b_c % 2]
                                print("str_pos: ", str_pos)
                                # self.file.write_open(str_pos, num_detect)
                                # print("已写入数据")
                                # send message
                                # self.state_info_send(1, str_pos, num_detect)
                                pass
                        self.route_road_permit = True
                        if self.routeNodeIndex == self.routeNodeNum - 1:
                            break
                    time.sleep(0.2)
            except Exception as e:
                print("error: ", e)
        #                 self.file.close()
        #                 print("文件已关闭")

        elif fly_mode == 2:
            while True:
                if not self.route_road_permit:
                    time.sleep(0.3)
                    if self.turn_duoji:
#                         self.gpio.duoji_set(2)
                        self.gpio.camera_spin('right')
                    else:
#                         self.gpio.duoji_set(3)
                        self.gpio.camera_spin('left')
                    time.sleep(0.3)
                    num_detect = 1
                    if num_detect != -1:
                        # print("Value: ", num_detect)
                        # self.state_info_send(3, 'A1', num_detect)
                        if self.turn_laser:
                            self.gpio.laser_set_2(0.6)
                        else:
                            self.gpio.laser_set_1(0.6)
                    self.route_road_permit = True
                    break
                time.sleep(0.2)
            pass

    def position_send(self):
        try:
            while (True):
                if self._265Ready:
                    # Wait for the next set of frames from the camera
                    frames = self.pipe.wait_for_frames()
                    # Fetch pose frame
                    pose = frames.get_pose_frame()
                    if pose:
                        # Print some of the pose data to the terminal
                        data = pose.get_pose_data()
                        self.dataBuf, self.pos_X, self.pos_Y, self.pos_Z, self.Euler = self.data_deal.solve_data(data)
                        if (self.SendTargetPos == 1):
                            self.posX = self.TargetPosition[0]
                            self.posY = self.TargetPosition[1]
                            self.posZ = self.TargetPosition[2]

                            self.dataBuf[43] = 0x20
                            posX_buf = struct.pack("f", self.posX)
                            self.dataBuf[44] = posX_buf[0]
                            self.dataBuf[45] = posX_buf[1]
                            self.dataBuf[46] = posX_buf[2]
                            self.dataBuf[47] = posX_buf[3]
                            posY_buf = struct.pack("f", self.posY)
                            self.dataBuf[48] = posY_buf[0]
                            self.dataBuf[49] = posY_buf[1]
                            self.dataBuf[50] = posY_buf[2]
                            self.dataBuf[51] = posY_buf[3]
                            posZ_buf = struct.pack("f", self.posZ)
                            self.dataBuf[52] = posZ_buf[0]
                            self.dataBuf[53] = posZ_buf[1]
                            self.dataBuf[54] = posZ_buf[2]
                            self.dataBuf[55] = posZ_buf[3]

                            self.dataBuf[56] = self.LaserArray
                            Laser_Dis = struct.pack("f", self.LaserDistance)
                            self.dataBuf[57] = Laser_Dis[0]
                            self.dataBuf[58] = Laser_Dis[1]
                            self.dataBuf[59] = Laser_Dis[2]
                            self.dataBuf[60] = Laser_Dis[3]
                            self.dataBuf[61] = self.FlightMode

                        if self.CopterLanding == 1:
                            self.dataBuf[62] = 0xA5
                        else:
                            self.dataBuf[62] = 0x00

                        for i in range(0, 62):
                            self.CheckSum = self.CheckSum + self.dataBuf[i]

                        self.dataBuf[63] = 0xAA
                        self.dataBuf[64] = self.CheckSum & 0x00ff

                        # 输出 pitch 、 roll 、 yaw ,x , y , z
                        print(
                            "\rrpy_rad[0]:{:.2f},rpy_rad[1]:{:.2f},rpy_rad[2]:{:.2f} ,X:{:.2f},Y:{:.2f},Z:{:.2f} ".format(
                                self.Euler[0] * 57.3, self.Euler[1] * 57.3, self.Euler[2] * 57.3,
                                self.pos_X, self.pos_Y, self.pos_Z), end=" ")
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
                        if self.ifTakeOff == True:
                            self.count = self.count + 1
                        if self.count == 200:
                            self.count = 0
                            self.ifTakeOff = False
                            self.dataBuf[0] = 0x55
                            self.dataBuf[1] = 0xAA
                            self.dataBuf[2] = 0x40
                            self.dataBuf[63] = 0xAA
                            self.dataBuf[64] = 0x00
                        self.port.write(self.dataBuf)
                        # 添加自启动代码
                        self.CheckSum = 0
                #                     pipe.stop()
                else:

                    self.dataBuf[0] = 0x55
                    self.dataBuf[1] = 0xAA
                    self.dataBuf[2] = 0xFF
                    self.dataBuf[63] = 0xAA
                    self.dataBuf[64] = 0x00
                    self.port.write(self.dataBuf)
                    time.sleep(0.1)
        finally:
            print("some erro")
        pass

    pass
