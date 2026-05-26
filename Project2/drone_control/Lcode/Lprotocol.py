import serial
import threading
from typing import List
from Lcode.Logger import logger
import time
from Lcode.global_variable import lock,task_start_sign,fc_last_rx_time
DEBUG=False
class Serial_fc(object):
    def __init__(self,port,baudrate):
        self.ser=serial.Serial(port=port,baudrate=baudrate,timeout=0.05)
        self.fclisten_running=False
        self.t265send_running=False
        self.cmdsend_running=False
        self.rate=460800
        self.startbyte=b'\xAA'
        self.endbyte=0xFF
        self._last_laser_height_cm = 0.0  # 最后已知激光高度(cm)
    def port_open(self):
        if not self.ser.is_open:
            self.ser.open()
            logger.info("目前飞控串口状态：%s",self.ser.is_open)
    def listen_start(self,rxbuffer:List[int]):
        self.fclisten_running=True
        listen_thread=threading.Thread(target=Serial_fc.listen_fc,args=(self,rxbuffer))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("飞控串口监听线程启动")
    def listen_end(self):
        self.fclisten_running=False
        logger.info("飞控串口监听线程关闭")
    def listen_fc(self,rxbuffer:List[int]):
        global fc_last_rx_time
        while self.fclisten_running ==True:
            byte_data = self.ser.read()
            if byte_data == self.startbyte:
                # 读取接下来的10个字节数据
                # 新帧格式: AA | task_sta | x_h x_l | y_h y_l | h_b0 b1 b2 b3 | CK FF
                recv = self.ser.read(10)
                if len(recv) < 10:
                    continue
                # 判断数据是否符合通信协议，即以0xFF结尾
                if recv[9] == self.endbyte:
                    intergral_x = ((recv[1] << 8) | recv[2])-0x4000
                    intergral_y = ((recv[3] << 8) | recv[4])-0x4000
                    # 激光测距高度(cm)，4字节小端序
                    laser_height_cm = (recv[5]) | (recv[6] << 8) | (recv[7] << 16) | (recv[8] << 24)
                    with lock:
                        rxbuffer.clear()
                        rxbuffer.append(recv[0])
                        rxbuffer.append(intergral_x)
                        rxbuffer.append(intergral_y)
                        rxbuffer.append(laser_height_cm)
                    # 缓存激光高度供外部查询（有效值 > 10cm）
                    if laser_height_cm > 50:
                        with lock:
                            self._last_laser_height_cm = float(laser_height_cm) / 100.0
                    fc_last_rx_time.value = time.time()
                    if recv[0]==0x05:
                        task_start_sign.value=True
                    else:
                        task_start_sign.value=False
                    if DEBUG :
                        logger.info(rxbuffer)
            time.sleep(0.05)
    def _send_t265_loop(self, t265_obj, freq):
        """独立线程：发送 T265 速度+偏航帧 (0x01)"""
        sleep_time = 1.0 / freq
        while self.t265send_running:
            if t265_obj is not None and t265_obj.is_running():
                vx, vy, _ = t265_obj.get_velocity()
                vx_cm = int(vx * 100)
                vy_cm = int(vy * 100)
                yaw_x100 = t265_obj.get_yaw_deg_x100()
                # 帧格式: AA 01 vx_h vx_l vy_h vy_l yaw_h yaw_l CK FF
                t265_frame = [0xAA, 0x01,
                             (vx_cm >> 8) & 0xFF, vx_cm & 0xFF,
                             (vy_cm >> 8) & 0xFF, vy_cm & 0xFF,
                             (yaw_x100 >> 8) & 0xFF, yaw_x100 & 0xFF]
                ck = 0
                for b in t265_frame[1:]:
                    ck ^= b
                t265_frame.append(ck)
                t265_frame.append(0xFF)
                self.ser.write(bytes(t265_frame))
            time.sleep(sleep_time)

    def _send_command_loop(self, comlist, freq):
        """独立线程：发送指令帧 (0x02)"""
        sleep_time = 1.0 / freq
        while self.cmdsend_running:
            with lock:
                values = list(comlist)
            # CK = XOR(02 ... sp_side)，位于 FF 前
            ck = 0
            for b in values[1:-2]:
                ck ^= b
            values[-2] = ck
            self.ser.write(bytes(values))
            time.sleep(sleep_time)

    def send_start(self, comlist=None, t265_obj=None, vel_freq=100, cmd_freq=50):
        self.t265send_running = True
        self.cmdsend_running = True

        if t265_obj is not None:
            t265_thread = threading.Thread(
                target=self._send_t265_loop, args=(t265_obj, vel_freq))
            t265_thread.daemon = True
            t265_thread.start()

        if comlist is not None:
            cmd_thread = threading.Thread(
                target=self._send_command_loop, args=(comlist, cmd_freq))
            cmd_thread.daemon = True
            cmd_thread.start()

        parts = []
        if t265_obj is not None:
            parts.append("速度帧 %dHz" % vel_freq)
        if comlist is not None:
            parts.append("指令帧 %dHz" % cmd_freq)
        logger.info("飞控串口发送线程启动（%s）", " + ".join(parts))

    def send_end(self):
        self.t265send_running = False
        self.cmdsend_running = False
        logger.info("飞控串口发送线程关闭")

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("飞控串口已关闭")

class Serial_dmz(object):
    def __init__(self,port,baudrate):
        self.ser=serial.Serial(port=port,baudrate=baudrate,timeout=0.05)
        self.dmzlisten_running=False
        self.dmzsend_running=False
        self.rate=115200
    def port_open(self):
        if not self.ser.is_open:
            self.ser.open()
            logger.info("目前地面站串口状态：%s",self.ser.is_open)
    def listen_start(self,rxbuffer:List[tuple]):
        self.dmzlisten_running=True
        listen_thread=threading.Thread(target=Serial_dmz.listen_dmz,args=(self,rxbuffer))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("地面站串口监听线程启动")
    def listen_end(self):
        self.dmzlisten_running=False
        logger.info("地面站串口监听线程关闭")
    def listen_dmz(self, rxbuffer: List[tuple]):
        while self.dmzlisten_running == True:
            byte_data = self.ser.read()
            if byte_data == b'\xAA':
                recv = self.ser.read(7)

                if len(recv) < 7:
                    continue

                if recv[6] == 0xFF:
                    with lock:
                        rxbuffer.clear()
                        for i in range(3):
                            a_val = recv[i * 2] - 0xA0
                            b_val = recv[i * 2 + 1] - 0xB0

                            a_str = f"A{a_val}"
                            b_str = f"B{b_val}"
                            rxbuffer.append((a_str, b_str))

                        if DEBUG:
                            logger.info("地面站反传禁飞区坐标: %s", rxbuffer)

            time.sleep(0.05)
    def send_dmz(self,comlist:List[int]):
        while self.dmzsend_running==True:
            self.ser.write(bytes(comlist))  # 单次发送整帧
            time.sleep(0.1)
    def send_start(self,comlist:List[int]):
        self.dmzsend_running=True
        dmzsend_thread=threading.Thread(target=Serial_dmz.send_dmz,args=(self,comlist))
        dmzsend_thread.daemon=True
        dmzsend_thread.start()
        logger.info("地面站串口发送线程启动")
    def send_end(self):
        self.dmzsend_running=False
        logger.info("地面站串口发送线程关闭")
    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("地面站串口已关闭")
