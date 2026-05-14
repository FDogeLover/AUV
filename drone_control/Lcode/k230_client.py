"""
Pi ↔ K230 双向串口通信客户端
=============================
协议帧 (二进制):
  Pi→K230: AA 10 grid_idx           — 启动检测
  K230→Pi: AA 20 grid_idx cls cnt total conf  — 检测结果
  Pi→K230: AA 11 grid_idx           — ACK确认

物理层: /dev/ttyS3, 115200 baud, 8N1
"""

import serial
import threading
import time
from Lcode.Logger import logger

FRAME_HEAD = 0xAA
CMD_START  = 0x10
CMD_ACK    = 0x11
CMD_RESULT = 0x20
RESULT_LEN = 7  # AA + CMD + grid_idx + cls + cnt + total + conf


class K230Client:
    """Pi端 K230 通信客户端"""

    def __init__(self, port="/dev/ttyS3", baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)
        self.result = None          # (grid_idx, cls, cnt, total, conf)
        self.result_event = threading.Event()
        self._running = True
        self._listen_thread = threading.Thread(target=self._listen, daemon=True)
        self._listen_thread.start()
        logger.info(f"K230串口已连接: {port} @ {baudrate}")

    def _listen(self):
        """守护线程: 接收 K230 的 RESULT 帧"""
        while self._running:
            try:
                b = self.ser.read(1)
                if not b or b[0] != FRAME_HEAD:
                    continue
                buf = self.ser.read(RESULT_LEN - 1)
                if len(buf) < RESULT_LEN - 1:
                    continue
                cmd = buf[0]
                if cmd == CMD_RESULT:
                    grid_idx = buf[1]
                    cls_id = buf[2]
                    best_cnt = buf[3]
                    total_dets = buf[4]
                    avg_conf = buf[5]
                    self.result = (grid_idx, cls_id, best_cnt, total_dets, avg_conf)
                    self.result_event.set()
            except Exception:
                time.sleep(0.01)

    def send_start(self, grid_idx):
        """发送 START 帧: AA 10 grid_idx"""
        self.result = None
        self.result_event.clear()
        frame = bytes([FRAME_HEAD, CMD_START, grid_idx & 0xFF])
        self.ser.write(frame)
        logger.info(f"K230 START → grid_idx={grid_idx}")

    def send_ack(self, grid_idx):
        """发送 ACK 帧: AA 11 grid_idx"""
        frame = bytes([FRAME_HEAD, CMD_ACK, grid_idx & 0xFF])
        self.ser.write(frame)
        logger.info(f"K230 ACK   → grid_idx={grid_idx}")

    def poll_result(self):
        """非阻塞轮询: 有结果返回 (grid, cls, cnt, total, conf)，无结果返回 None"""
        if self.result_event.wait(timeout=0):
            self.result_event.clear()
            return self.result
        return None

    def close(self):
        """关闭串口和线程"""
        self._running = False
        if self.ser.is_open:
            self.ser.close()
        logger.info("K230串口已关闭")
