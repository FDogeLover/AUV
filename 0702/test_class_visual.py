from test_container import Containers

import cv2
from pyzbar.pyzbar import decode
import pyzbar
import time

class VisualDeal():
    def __init__(self):
        self.container = Containers()
        pass
    def get_frame(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)
#         time.sleep(0.2)
        ret, image = cap.read()
        if ret:
            cap.release()
            print("摄像头已关闭")
            frame = image[100:380, 140:500]
#             time.sleep(0.1)
            return frame
        else:
            print("未获取到图像数据")
            cap.release()
            print("摄像头已关闭")
            time.sleep(0.1)
            return None

    def qrcode_detect(self, image):
        qr_det = cv2.QRCodeDetector()
        codeinfo, points, straight_qrcode = qr_det.detectAndDecode(image)
        if codeinfo:
            print("解码成功:", codeinfo)
            return codeinfo
        else:
            print("未检测到二维码或解码失败")
            return None

    def visual_deal(self, times_detect=7):
        num_detected = 0
        while True:
            if num_detected >= times_detect:
                current_time = time.strftime("%Y-%m-%d-%H-%M-%S")
                filename = f"image_{current_time}.jpg"
                cv2.imwrite(filename, image)
                return -1
                # break
            image = self.get_frame()
            if image is not None:
                str_code = self.qrcode_detect(image)
                if str_code is not None:
                    if str_code in Containers.list_key_str:
                        return Containers.dict_qrcode[str_code]
                    else:
                        num_detected += 1
                else:
                    num_detected += 1
            else:
                num_detected += 1

    def visual_deal_mode_2(self):
        # num_detected = 0
        while True:
            image = self.get_frame()
            if image is not None:
                str_code = self.qrcode_detect(image)
                if str_code is not None:
                    if str_code in Containers.list_key_str:
                        return Containers.dict_qrcode[str_code]
                    else:
                        time.sleep(0.1)
                else:
                    time.sleep(0.1)
            else:
                time.sleep(0.1)


    pass