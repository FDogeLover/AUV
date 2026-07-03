"""
串口通信模块 — 仅飞控 (Serial_fc)，无地面站

帧协议:
  AA 01: T265 速度帧 (vx, vy, yaw)  @100Hz
  AA 02: 指令帧 (task_sta, com_x/y/z/yaw)  @50Hz
  AA..FF: 飞控下行遥测 (mission_stage, 姿态, 激光高度)
"""
import serial
import threading
import time
from typing import List
from Lcode.Logger import logger
from Lcode.global_variable import lock, fc_last_rx_time


class Serial_fc(object):
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)
        self.fclisten_running = False
        self.t265send_running = False
        self.cmdsend_running = False
        self._last_laser_height_cm = 0.0  # 最后已知激光高度 (m)

    def listen_start(self, rxbuffer: List[int]):
        self.fclisten_running = True
        t = threading.Thread(target=Serial_fc.listen_fc, args=(self, rxbuffer))
        t.daemon = True
        t.start()
        logger.info("飞控串口监听线程启动")

    def listen_end(self):
        self.fclisten_running = False
        logger.info("飞控串口监听线程关闭")

    def listen_fc(self, rxbuffer: List[int]):
        """接收下行帧: AA mission_stage rol_h/l pit_h/l yaw_h/l state x_int_h/l y_int_h/l laser_h(4B) CK FF"""
        while self.fclisten_running:
            byte_data = self.ser.read()
            if byte_data == b'\xAA':
                recv = self.ser.read(10)
                if len(recv) < 10:
                    continue
                if recv[9] == 0xFF:
                    integral_x = ((recv[1] << 8) | recv[2]) - 0x4000
                    integral_y = ((recv[3] << 8) | recv[4]) - 0x4000
                    laser_height_cm = (recv[5]) | (recv[6] << 8) | (recv[7] << 16) | (recv[8] << 24)
                    with lock:
                        rxbuffer.clear()
                        rxbuffer.append(recv[0])
                        rxbuffer.append(integral_x)
                        rxbuffer.append(integral_y)
                        rxbuffer.append(laser_height_cm)
                    if laser_height_cm > 50:
                        with lock:
                            self._last_laser_height_cm = float(laser_height_cm) / 100.0
                    fc_last_rx_time.value = time.time()
            time.sleep(0.05)

    def _send_t265_loop(self, t265_obj, freq):
        """独立线程：发送 T265 速度帧 (AA 01)"""
        sleep_time = 1.0 / freq
        while self.t265send_running:
            if t265_obj is not None and t265_obj.is_running():
                vx, vy, _ = t265_obj.get_velocity()
                vx_cm = int(vx * 100)
                vy_cm = int(vy * 100)
                yaw_x100 = t265_obj.get_yaw_deg_x100()
                frame = [0xAA, 0x01,
                         (vx_cm >> 8) & 0xFF, vx_cm & 0xFF,
                         (vy_cm >> 8) & 0xFF, vy_cm & 0xFF,
                         (yaw_x100 >> 8) & 0xFF, yaw_x100 & 0xFF]
                ck = 0
                for b in frame[1:]:
                    ck ^= b
                frame.append(ck)
                frame.append(0xFF)
                self.ser.write(bytes(frame))
            time.sleep(sleep_time)

    def _send_command_loop(self, comlist, freq):
        """独立线程：发送指令帧 (AA 02)"""
        sleep_time = 1.0 / freq
        while self.cmdsend_running:
            with lock:
                values = list(comlist)
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
            t = threading.Thread(target=self._send_t265_loop, args=(t265_obj, vel_freq))
            t.daemon = True
            t.start()

        if comlist is not None:
            t = threading.Thread(target=self._send_command_loop, args=(comlist, cmd_freq))
            t.daemon = True
            t.start()

        parts = []
        if t265_obj is not None:
            parts.append("速度帧 %dHz" % vel_freq)
        if comlist is not None:
            parts.append("指令帧 %dHz" % cmd_freq)
        logger.info("飞控串口发送线程启动（%s）" % " + ".join(parts))

    def send_end(self):
        self.t265send_running = False
        self.cmdsend_running = False
        logger.info("飞控串口发送线程关闭")

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("飞控串口已关闭")
