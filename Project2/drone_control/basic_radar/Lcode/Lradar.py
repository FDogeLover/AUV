"""N10P 激光雷达（镭神智能）串口驱动。

协议参考：《镭神智能_N10 Plus_通讯协议_V1.0.0_20220926.pdf》
帧格式（108字节，大端）：
  Byte0-1   帧头 A5 5A
  Byte2     Length，整帧长度(含帧头到校验)，正常输出固定 0x6C=108
  Byte3-4   Speed_H/L，一个码盘周期时间(us)，转速Hz = 1e6/(值*24)  [注：协议原文按24点/圈换算]
  Byte5-6   Start_angle_H/L，起始角度*100
  Byte7-102 32组点(每组3字节：距离H/L(mm,大端) + 强度)
  Byte103-104 预留位
  Byte105-106 Stop_angle_H/L，结束角度*100
  Byte107   校验：byte0..byte106 字节和 & 0xFF
"""
import serial
import threading
import time
from Lcode.Logger import logger
from Lcode.global_variable import lock

FRAME_LEN = 108        # 整帧长度（含帧头）
BODY_LEN = FRAME_LEN - 3  # 去掉 A5 5A Length 之后剩余字节数
POINTS_PER_FRAME = 32


class Serial_radar(object):
    def __init__(self, port, baudrate=460800):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1.0, write_timeout=1.0)
        self.listen_running = False
        # 最近一圈的点云缓存：{angle_deg(0~360, float): (distance_mm, intensity)}
        # 用字典按角度覆盖，避免转速抖动导致列表无限增长
        self._scan = {}
        self._last_frame_time = 0.0

    def port_open(self):
        if not self.ser.is_open:
            self.ser.open()
            logger.info("雷达串口状态：%s", self.ser.is_open)

    def listen_start(self):
        self.listen_running = True
        t = threading.Thread(target=self._listen_loop)
        t.daemon = True
        t.start()
        logger.info("雷达串口监听线程启动")

    def listen_end(self):
        self.listen_running = False
        logger.info("雷达串口监听线程关闭")

    def _listen_loop(self):
        while self.listen_running:
            b = self.ser.read(1)
            if len(b) == 0 or b[0] != 0xA5:
                continue
            b2 = self.ser.read(1)
            if len(b2) == 0 or b2[0] != 0x5A:
                continue
            length_byte = self.ser.read(1)
            if len(length_byte) == 0:
                continue
            length = length_byte[0]
            if length != FRAME_LEN:
                # 长度异常，跳过这帧，不强行按固定长度硬读，避免和下一帧错位
                continue
            body = self.ser.read(BODY_LEN)
            if len(body) < BODY_LEN:
                continue

            header = bytes([0xA5, 0x5A, length])
            checksum_calc = (sum(header) + sum(body[:-1])) & 0xFF
            checksum_recv = body[-1]
            if checksum_calc != checksum_recv:
                continue

            self._parse_frame(body)
            self._last_frame_time = time.time()

    def _parse_frame(self, body):
        # body 是去掉 A5 5A Length 后的 105 字节(对应协议 Byte3~Byte107)：
        # body[0:2]=speed  body[2:4]=start_angle  body[4:100]=32点(3字节/点)
        # body[100:102]=预留位  body[102:104]=stop_angle  body[104]=crc
        start_angle = ((body[2] << 8) | body[3]) / 100.0
        stop_angle = ((body[102] << 8) | body[103]) / 100.0

        span = stop_angle - start_angle
        if span < 0:
            span += 360.0
        step = span / (POINTS_PER_FRAME - 1) if POINTS_PER_FRAME > 1 else 0.0

        points = []
        offset = 4  # 跳过 speed(2) + start_angle(2)
        for i in range(POINTS_PER_FRAME):
            dist_h = body[offset]
            dist_l = body[offset + 1]
            intensity = body[offset + 2]
            distance_mm = (dist_h << 8) | dist_l
            angle = (start_angle + step * i) % 360.0
            points.append((angle, distance_mm, intensity))
            offset += 3

        with lock:
            for angle, distance_mm, intensity in points:
                self._scan[round(angle)] = (distance_mm, intensity)

    def get_scan(self):
        """返回最近一圈的点云快照：{angle_deg(int,0~359): (distance_mm, intensity)}"""
        with lock:
            return dict(self._scan)

    def get_nearest(self, min_intensity=0):
        """返回当前缓存点云里距离最近的点 (angle_deg, distance_mm)，无有效点返回 None。"""
        with lock:
            scan = list(self._scan.items())
        best = None
        for angle, (distance_mm, intensity) in scan:
            if distance_mm <= 0 or intensity < min_intensity:
                continue
            if best is None or distance_mm < best[1]:
                best = (angle, distance_mm)
        return best

    def is_alive(self, timeout=1.0):
        return (time.time() - self._last_frame_time) < timeout

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            logger.info("雷达串口已关闭")
