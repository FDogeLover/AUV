from source_dict import SourceDict
import cv2
import time
import threading


class QrcodeDeal:
    def __init__(self, source_dict):
        self.source_dict = source_dict
        self.cap = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.detected_value = None
        self.detected_position = None

    def get_frame(self):
        if not self.cap:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("无法打开摄像头")
                return None

            # 设置摄像头参数
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)

        ret, image = self.cap.read()
        if ret:
            with self.frame_lock:
                self.latest_frame = image
            return image
        return None

    def camera_thread(self):
        while self.running:
            self.get_frame()
            time.sleep(0.01)  # 控制帧率

    def qrcode_detect(self, image):
        qr_det = cv2.QRCodeDetector()
        codeinfo, _, _ = qr_det.detectAndDecode(image)
        return codeinfo if codeinfo else None

    def qrcode_deal(self, timeout=5):
        try:
            # 启动摄像头线程
            cam_thread = threading.Thread(target=self.camera_thread)
            cam_thread.daemon = True
            cam_thread.start()

            start_time = time.time()
            window_shown = False

            while self.running:
                # 检查超时
                if time.time() - start_time > timeout:
                    print("未检测到二维码，自动退出")
                    break

                # 获取最新帧
                with self.frame_lock:
                    if self.latest_frame is None:
                        continue
                    frame = self.latest_frame.copy()

                # 显示图像
                if not window_shown:
                    cv2.namedWindow('QR Code Scanner', cv2.WINDOW_NORMAL)
                    window_shown = True
                cv2.imshow('QR Code Scanner', frame)

                # 必须调用waitKey
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("用户手动退出")
                    break

                # 二维码检测
                str_code = self.qrcode_detect(frame)
                if str_code and str_code in self.source_dict.list_key_str:
                    self.detected_value = self.source_dict.dict_qrcode[str_code]
                    self.detected_position = self.source_dict.dict_route_index_pos[self.detected_value]
                    print("识别成功")
                    # print(self.detected_value, self.detected_position)
                    break

        except Exception as e:
            print("程序出错")
        finally:
            self.running = False
            if self.cap and self.cap.isOpened():
                self.cap.release()
            if window_shown:
                cv2.destroyAllWindows()
            if 'cam_thread' in locals() and cam_thread.is_alive():
                cam_thread.join()

        return self.detected_value, self.detected_position


# if __name__ == '__main__':
#     source_dict = SourceDict()
#     qr = QrcodeDeal(source_dict)
#     qr_thread = threading.Thread(target=qr.qrcode_deal, args=(20,))  # 注意这里的逗号
#     qr_thread.start()
    # qr_thread.join()  # 等待线程结束

    # if qr.detected_value is not None:
    #     print("最终结果")
    #     print(qr.detected_value, qr.detected_position)
    # else:
    #     print("未能识别有效二维码")