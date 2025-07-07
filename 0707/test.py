import threading
import time
from qrcode_test import *


class Test:  # 类名建议大写
    def __init__(self, qr):
        self.route_road_permit = True
        self.routeNodeIndex = 1
        self.route_once = True
        self.route_index_add = True
        self.route_block_index = [3, 4, 5, 6, 7]
        self.routeStartFlag = True
        self.block_flag = True
        self.routeNodeNum = 15
        self.LaserIndex1 = [2, 3, 4, 5, 6, 7]
        self.LaserIndex2 = [10, 11, 12, 13, 14, 15]
        self.lock = threading.Lock()  # 添加锁

    def SetFlag(self, timeout=5):
        while True:
            with self.lock:  # 加锁
                if not self.route_road_permit:
                    print("SetFlag start")
                    time.sleep(0.3)
                    if self.routeNodeIndex in self.LaserIndex1:
                        qr_value, qr_position = self.qr.qrcode_deal(timeout)
                        print("QR Code")
                        print(qr_value, qr_position)
                    if self.routeNodeIndex in self.LaserIndex2:
                        qr_value, qr_position = self.qr.qrcode_deal(timeout)
                        print("QR Code")
                        print(qr_value, qr_position)
                    time.sleep(0.3)
                    self.route_road_permit = True
            time.sleep(0.1)  # 避免空转消耗CPU

    def Router(self):
        while True:
            with self.lock:  # 加锁
                if self.route_once:
                    self.route_once = False
                elif self.route_road_permit:
                    if self.route_index_add:
                        self.routeNodeIndex += 1
                    else:
                        self.route_index_add = True

                if self.block_flag and self.routeNodeIndex in self.route_block_index:
                    self.route_road_permit = False
                    self.route_index_add = False
                    print("Router 停止")
                    try:
                        self.route_block_index.remove(self.routeNodeIndex)
                    except ValueError:
                        pass

                if self.route_road_permit and self.routeNodeIndex < self.routeNodeNum and self.routeStartFlag:
                    print("Router 运行 - 当前节点：")
                    print(self.routeNodeIndex)

            time.sleep(0.1)  # 避免空转消耗CPU


if __name__ == "__main__":
    source_dict = SourceDict()
    qr = QrcodeDeal(source_dict)
    te = Test(qr)

    # 创建线程时不要加括号
    test_thread = threading.Thread(target=te.SetFlag, daemon=True)
    router_thread = threading.Thread(target=te.Router, daemon=True)

    test_thread.start()
    router_thread.start()

    try:
        while True:  # 主线程保持运行
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序终止")