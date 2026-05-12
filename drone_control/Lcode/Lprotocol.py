import serial
import threading
import pickle
import socket
from typing import List
from Lcode.Logger import logger
import time
from Lcode.global_variable import lock,task_start_sign
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
        while self.fclisten_running ==True:
            byte_data = self.ser.read() 
            if byte_data == self.startbyte:
                # 读取接下来的6个字节数据
                recv = self.ser.read(6) #recv[0]是任务模式，recv[1]和recv[2]是x积分值，recv[3]和recv[4]是y积分值，recv[5]是帧尾
                # 判断数据是否符合通信协议，即以0xFF结尾
                if recv[5] == self.endbyte:
                    intergral_x = ((recv[1] << 8) | recv[2])-0x4000
                    intergral_y = ((recv[3] << 8) | recv[4])-0x4000
                    rxbuffer.clear()
                    rxbuffer.append(recv[0])
                    rxbuffer.append(intergral_x)
                    rxbuffer.append(intergral_y)
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
                # 帧格式: AA 01 vx_h vx_l vy_h vy_l yaw_h yaw_l FF
                t265_frame = [0xAA, 0x01,
                             (vx_cm >> 8) & 0xFF, vx_cm & 0xFF,
                             (vy_cm >> 8) & 0xFF, vy_cm & 0xFF,
                             (yaw_x100 >> 8) & 0xFF, yaw_x100 & 0xFF,
                             0xFF]
                self.ser.write(bytes(t265_frame))
            time.sleep(sleep_time)

    def _send_command_loop(self, comlist, freq):
        """独立线程：发送指令帧 (0x02)"""
        sleep_time = 1.0 / freq
        while self.cmdsend_running:
            with lock:
                values = list(comlist)
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
    def listen_dmz(self,rxbuffer:List[tuple]):
        """
        监听地面站反传信息
        通信协议：AA + A1 + B1 + A2 + B2 + A3 + B3 + FF
        其中A值范围1-9，B值范围1-7
        解析后存入rxbuffer，格式为[('A9','B1'), ('A10','B2'), ('A11','B3')]
        """
        while self.dmzlisten_running ==True:
            byte_data = self.ser.read() 
            if byte_data == b'\xAA': 
                # 读取接下来的8个字节数据 
                recv = self.ser.read(8) # recv[0]=AA, recv[1]=A1, recv[2]=B1, recv[3]=A2, recv[4]=B2, recv[5]=A3, recv[6]=B3, recv[7]=FF
                
                # 判断数据是否符合通信协议，即以0xFF结尾和以0xAA开头
                if recv[7] == 0xFF:
                    with lock:
                        rxbuffer.clear()
                        # 解析三个禁飞区坐标
                        for i in range(3):
                            a_val = recv[1 + i*2]-0xA0      # A值 (1-9)减去0xA0得到1-9的整数
                            b_val = recv[2 + i*2]-0xB0      # B值 (1-7)减去0xB0得到1-7的整数
                            a_str = f"A{a_val}"
                            b_str = f"B{b_val}"
                            rxbuffer.append((a_str, b_str))
                        if DEBUG:
                            logger.info("地面站反传禁飞区坐标: %s", rxbuffer)
            time.sleep(0.05)
    def send_dmz(self,comlist:List[int]):
        while self.dmzsend_running==True:
            self.ser.write(bytes(comlist))  # 单次发送整帧
            time.sleep(0.01)
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


class Serial_gpio(object):
    def __init__(self,port,baudrate):
        self.ser=serial.Serial(port=port,baudrate=baudrate,timeout=0.05)
        self.gpiosend_running=False
        self.gpiolisten_running=False
        self.rate=460800
    def port_open(self):
        if not self.ser.is_open:
            self.ser.open()
            logger.info("目前gpio串口状态：%s",self.ser.is_open)
    def send_gpio(self,comlist:List[int]):
        while self.gpiosend_running==True:
            self.ser.write(bytes(comlist))  # 单次发送整帧
            time.sleep(0.02)
    def send_start(self,comlist:List[int]):
        self.gpiosend_running=True
        gpiosend_thread=threading.Thread(target=Serial_gpio.send_gpio,args=(self,comlist))
        gpiosend_thread.daemon=True
        gpiosend_thread.start()
        logger.info("gpio串口发送线程启动")
    def send_end(self):
        self.gpiosend_running=False
        logger.info("gpio串口发送线程关闭")
    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("GPIO串口已关闭")
    def listen_start(self,rxbuffer:List[int]):
        self.gpiolisten_running=True
        listen_thread=threading.Thread(target=Serial_gpio.listen_gpio,args=(self,rxbuffer))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("gpio串口监听线程启动")
    def listen_end(self):
        self.gpiolisten_running=False
        logger.info("gpio串口监听线程关闭")
    def listen_gpio(self,rxbuffer:List[int]):
        while self.gpiolisten_running ==True:
            byte_data = self.ser.read() 
            if byte_data == b'\xAA':
                # 读取接下来的四个字节数据
                recv = self.ser.read(5)
                # 判断数据是否符合通信协议，即以0xFF结尾
                if recv[4] == 0xFF:
                    with lock:
                        rxbuffer.clear()
                        for i in range(0,4):
                            rxbuffer.append(recv[i])
                        if task_start_sign.value ==False:
                            logger.info(rxbuffer)
            time.sleep(0.05)
class udp_terminal(object):
    def __init__(self):
        self.udp_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_listen_running=False
        self.udp_send_running=False
        self.takeoff_sign=False
        self.task_hjm=False
        self.task_number=0
    def listen_start(self,IP,PORT):
        self.udp_listen_running=True
        self.udp_socket.bind((IP,PORT))
        listen_thread=threading.Thread(target=udp_terminal.listen_udp,args=(self,))
        listen_thread.daemon=True
        listen_thread.start()
        logger.info("udp监听线程启动")
    def listen_end(self):
        self.udp_listen_running=False
        logger.info("udp监听线程关闭")
    def listen_udp(self):
        received_data=[]
        state=0
        while self.udp_listen_running==True:
            data, client_address = self.udp_socket.recvfrom(1024)
            #logger.info("接收到的数据是%s",data)
            try:
                realdata=pickle.loads(data)
            except Exception:
                logger.warning("udp_terminal: pickle���л�ʧ��")
                continue
            if realdata[0]==170 and realdata[3]==255:
                if realdata[1]==160 and realdata[2]==160:
                    self.task_number=1
                elif realdata[1]==160 and realdata[2]==161:
                    self.task_number=2
                elif realdata[1]==192 and realdata[2]==192:
                    self.task_hjm=True
            time.sleep(0.02)
    """ def tasksort(self):
        self.task_list.sort(key=lambda x:x[0],reverse=True)
        logger.info("任务列表排序后:%s",self.task_list) """
    
    def send_start(self,IP,PORT,senddata):
        self.udp_send_running=True
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # 设置允许发送广播数据
        send_thread=threading.Thread(target=udp_terminal.send_udp,args=(self,IP,PORT,senddata))
        send_thread.daemon=True
        send_thread.start()
        logger.info("udp发送线程启动")
    def send_udp(self,IP,PORT,senddata):
        while self.udp_send_running==True:
            changedata=pickle.dumps(senddata)
            self.udp_socket.sendto(changedata,(IP,PORT))
            time.sleep(0.05)