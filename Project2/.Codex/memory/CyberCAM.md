# CyberCAM 开发参考

> 基于嘉楠 K230 (RISC-V 双核 1.6GHz + 800MHz) 的全能 AI 相机
> 官方 Wiki：https://wiki.01studio.cc/docs/category/cybercam%E4%BB%8B%E7%BB%8D
> GitHub：https://github.com/01studio-lab
> 遇到 CyberCAM 开发问题，先查官方 Wiki 上述链接。

---

## 一、基础信息

### SSH 连接
```bash
ssh CyberCAM
# 用户: pi | 密码: (已配免密)
# 主机: 192.168.137.65
```

### 系统
- 核桃派 Linux (K230 SDK)，内核 6.6.36
- CPU: RISC-V 64 位双核
- RAM: 1GB LPDDR4
- 存储: MicroSD (最大 512G)

### 硬件接口速查
| 接口 | 说明 |
|---|---|
| UART2 | IO11-TX2, IO12-RX2, 3.3V 电平 |
| GPIO | 2.54mm x 12P 排针 |
| USB | USB 2.0 HOST x1 |
| CSI | 板载 GC2093 + 1x 扩展 CSI (2lane) |
| WiFi6 + BT5.0 | 板载天线 |

---

## 二、摄像头

### 板载摄像头
- 型号: GC2093，1080P@60fps，70° FOV，镜头可旋转对焦
- Python API:
```python
from modules import Sensor, Display, IDE

cap = Sensor.Sensor(640, 480)  # 默认 1920×1080
ret, img = cap.read()           # 返回 (ret, img)
Display.show(img)               # 显示到屏幕
IDE.show(img)                   # 显示到 IDE 调试窗口
```

### USB 摄像头
- 已识别设备: `icspring camera` (UVC 1.00, idVendor=32e6, idProduct=9211)
- 设备节点: **`/dev/video5`**（MJPG + YUYV 格式）
- `v4l2-ctl -d /dev/video5 --info` 查看具体设备信息
- **已知坑**：UVC Gadget 服务 (`cybercam-usb.service`) 会占用 UVC 驱动，导致 `/dev/video5` 报 `Device or resource busy`。需要先停用：
  ```bash
  sudo systemctl stop cybercam-usb.service
  sudo systemctl disable cybercam-usb.service
  ```
- OpenCV 打开方式：
  ```python
  cap = cv2.VideoCapture("/dev/video5", cv2.CAP_V4L2)
  cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  ```
- USB 摄像头 + 屏幕显示实测可达 **30 FPS**

---

## 三、OpenCV

- 版本: **OpenCV 4.9.0**（"满血版"）
- 完整 OpenCV Python API 可用
- 文档参考: https://docs.opencv.org/4.9.0/

---

## 四、AI 视觉 (KPU / NPU)

### NPU 规格
- 算力: 6 TOPS (INT8)
- 支持: INT8 / INT16 / FP16 / BFP16
- 模型编译: nncase (ONNX / TFLite → .kmodel)

### 内置检测 API
```python
# 人脸检测
detector = kpu.FACE_DETECT(model_path, anchors_path, (width, height))
results = detector.run(img, reliability_threshold=0.6, nms_threshold=0.7)
# result: x, y, w, h, reliability, left_eye, right_eye, nose, left_mouth, right_mouth

# YOLO11 检测/分类（需先编译模型）
yolo = kpu.YOLO11(model_path)
results = yolo.detect(img, conf_threshold=0.5)
```

### 官方示例
- [人脸检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/face_detection)
- [人体检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/person_detection)
- [人体关键点检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/person_keypoint)
- [跌倒检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/falldown_detection)
- [手掌检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/hand_detection)
- [手掌关键点检测/分类](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/hand_keypoint_det)
- [YOLO11 检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/yolo11_det)
- [YOLO11 分类](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/yolo11_cls)
- [在线模型训练](https://wiki.01studio.cc/docs/cybercam/machine_vision/train)

---

## 五、UART 串口通讯（对接飞控）

### 启用
```bash
sudo set-device enable uart2   # 启用 UART2
# 然后重启生效
```

### 引脚
- TX2: IO11
- RX2: IO12
- 电平: 3.3V（交叉接线：TX ↔ RX，GND 共地）
- ⚠️ 线材中红色线是 5V，**不要**接到 3.3V 设备供电

### Python 代码
```python
import serial

com = serial.Serial("/dev/ttyS2", 115200)  # 或 460800/921600
com.write(b'Hello')

count = com.inWaiting()
recv = com.read(count)
com.flushInput()
```

---

## 六、其他外围

### GPIO
```python
from modules import Pin
pin = Pin(Pin.IO0, Pin.OUT)
pin.value(1)  # 高电平
```
教程: [GPIO Python库](https://wiki.01studio.cc/docs/cybercam/basic_examples/gpio_python)

### PWM
```python
from modules import PWM
pwm = PWM(Pin.IO0, 50, 50)  # 频率 50Hz, 占空比 50%
pwm.duty(75)  # 改占空比
```
教程: [PWM（补光灯）](https://wiki.01studio.cc/docs/cybercam/basic_examples/pwm_light)

### IMU (QMI8658A)
板载三轴加速度 + 三轴陀螺仪，可用于姿态估计

### 屏幕 (Display 库)
2.4寸 640×480 电容触摸屏，可拆卸。注意屏幕物理方向是 **竖屏 480×640**。

```python
import sys
sys.path.insert(0, "/usr/lib/walnutpi/k230_libdisplay/py_lib")
import Display

Display.init()
print(f"屏幕: {Display.get_width()}x{Display.get_height()}")
# → 480x640 (竖屏)

# 显示图像（接受 BGR888 / OpenCV 原生格式，零转换）
Display.show(img)

# 程序退出前调用
Display.flush()
```

- **BGR888** 格式是快速路径（直接 memcpy，零转换）
- 输入尺寸不匹配时自动 resize
- 显示 USB 摄像头实时画面实测 **30 FPS**

---

## 七、二维码扫描 (QR Code)

使用 OpenCV 内置的 `QRCodeDetector`，无需安装额外库。

### 性能
- 每帧解码耗时约 **210ms** (RISC-V CPU, 640×480)
- 建议**降频解码**（每秒 1 次），视频全速显示不受影响

### 示例
```python
import cv2

cap = cv2.VideoCapture("/dev/video5", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

qr = cv2.QRCodeDetector()
ret, frame = cap.read()
data, bbox, _ = qr.detectAndDecode(frame)
if data:
    print(f"QR: {data}")
```

完整参考脚本: `k230/test_qr_scan.py`

---

## 八、系统工具

### 启用/禁用外设
```bash
sudo set-device enable uart2   # 启用 UART2
sudo set-device enable csi0    # 切换到扩展 CSI0 接口
sudo set-device list            # 查看当前外设状态
# 切换后需重启
```

### 烧录系统
- rar 压缩包解压成 .img 文件
- 用烧录工具写入 MicroSD 卡
- 下载链接见官方 Wiki 下载页面

---

## 八、与无人机项目的集成方案

当前用途：**K230 作为视觉协处理器**，通过 UART 将视觉检测结果发送给飞控（或经树莓派中转）。

架构选项：
1. **CyberCAM → UART → 飞控 STM32** （直接直连，延迟最低）
2. **CyberCAM → UART → 树莓派 RDK X5 → UART → 飞控** （树莓派汇总后再发）

推荐：视觉伺服指令（目标 dx/dy 等）走方案 1，树莓派专注 T265 导航融合和状态机。
