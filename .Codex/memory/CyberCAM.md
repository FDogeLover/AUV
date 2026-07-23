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
- CPU: RISC-V 64 位双核 (C908 @ 1.6GHz + C908 @ 800MHz)
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

### 已知坑
- **UVC Gadget**：`cybercam-usb.service` 会占用 UVC 驱动，导致 USB 摄像头报 `Device or resource busy`。
  ```bash
  sudo systemctl stop cybercam-usb.service
  sudo systemctl disable cybercam-usb.service
  ```

---

## 二、项目文件结构

### 本地 (Project2/CyberCAM/)
```
CyberCAM/
├── servo/                          ← 视觉伺服程序
│   ├── qr_visual_servo.py          #   QR 视觉伺服（主程序）
│   ├── test_qr_scan.py             #   QR 扫码测试
│   └── test_usb_cam.py             #   USB 摄像头测试
├── animal_detect/                  ← 动物检测（YOLOv8n）
│   ├── animal_detect_visual.py
│   ├── animal_detect_yolov8n.py
│   ├── dataset_capture.py
│   └── detector_base.py
├── data/                           ← 配置文件（空，放本地参考）
└── logs/                           ← 日志文件（空，放本地参考）
```

### CyberCAM 端 (~/FJJ/)
```
~/FJJ/
├── servo/qr_visual_servo.py         # 视觉伺服（实际运行）
├── data/qr_mapping.txt              # 货架映射表
└── logs/qr_servo.log                # 运行日志
```

SSH 后台运行：
```bash
ssh CyberCAM 'cd ~/FJJ && nohup python3 -u servo/qr_visual_servo.py > logs/qr_servo.log 2>&1 &'
ssh CyberCAM 'pkill -f qr_visual_servo'    # 停止
```

---

## 三、摄像头

### 板载摄像头 (GC2093)
- 1080P@60fps，70° FOV，镜头可旋转对焦
- Python API（使用专用 Sensor 模块）：
```python
from k230_sensor import Sensor
cap = Sensor(640, 480)      # 默认 1920×1080
ret, img = cap.read()        # 返回 BGR 格式
```
- 设备节点：`/dev/video1`（vvcam ISP 管线）

### USB 摄像头 (icspring camera)
- 型号：`icspring camera` (UVC 1.00, idVendor=32e6, idProduct=9211)
- 设备节点：**`/dev/video5`**（MJPG + YUYV）
- 实测性能：
  - 帧捕获 `read()`：**30 FPS**（33ms/帧）
  - QR 解码 `detectAndDecode()`：**~5 FPS**（210ms/帧，CPU 瓶颈）
  - 光流跟踪 `calcOpticalFlowPyrLK`：**~30 FPS**（5ms/帧）
- 打开方式：
```python
cap = cv2.VideoCapture("/dev/video5", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

---

## 四、OpenCV

- 版本: **OpenCV 4.10.0**（"满血版"）
- 完整 OpenCV Python API 可用
- 文档参考: https://docs.opencv.org/4.9.0/
- 内置 `QRCodeDetector`，无需额外装库

---

## 五、屏幕 (Display 库)

2.4寸 640×480 电容触摸屏，物理方向 **竖屏 480×640**。函数式 API（非类）。

```python
import sys
sys.path.insert(0, "/usr/lib/walnutpi/k230_libdisplay/py_lib")
import Display

Display.init()
print(f"屏幕: {Display.get_width()}x{Display.get_height()}")  # → 480x640

# 显示图像（接受 BGR888 / OpenCV 原生格式，零转换）
Display.show(img)

# 程序退出前调用
Display.flush()
```

- **BGR888** 是快速路径（直接 memcpy，零转换）
- 输入尺寸不匹配时自动 resize
- USB 摄像头实时画面实测 **30 FPS**

注意：与 LVGL 桌面 GUI (`cybercam-desktop.service`) 冲突，运行前需先停用：
```bash
sudo systemctl stop cybercam-desktop
```

---

## 六、AI 视觉 (KPU / NPU)

### NPU 规格
- 算力: 6 TOPS (INT8)
- 支持: INT8 / INT16 / FP16 / BFP16
- 模型编译: nncase (ONNX / TFLite → .kmodel)

### 内置检测 API
```python
# 人脸检测
detector = kpu.FACE_DETECT(model_path, anchors_path, (width, height))
results = detector.run(img, reliability_threshold=0.6, nms_threshold=0.7)

# YOLO11 检测/分类（需先编译模型）
yolo = kpu.YOLO11(model_path)
results = yolo.detect(img, conf_threshold=0.5)
```

### 官方示例
- [人脸检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/face_detection)
- [YOLO11 检测](https://wiki.01studio.cc/docs/cybercam/machine_vision/ai_vision/yolo11_det)
- [在线模型训练](https://wiki.01studio.cc/docs/cybercam/machine_vision/train)

---

## 七、UART 串口通讯（对接飞控）

### 启用
```bash
sudo set-device enable uart2   # 启用 UART2
# 重启生效
```

### 引脚
- TX2: IO11, RX2: IO12, 电平: 3.3V
- 交叉接线：TX ↔ RX，GND 共地
- ⚠️ 红色线是 5V，**不要**接到 3.3V 设备供电

### 代码
```python
import serial
com = serial.Serial("/dev/ttyS2", 115200)  # 或 460800/921600
com.write(b'data')
recv = com.read(com.inWaiting())
com.flushInput()
```

---

## 八、二维码扫码 (QR Code)

使用 OpenCV 内置 `QRCodeDetector`，**约 210ms/次**（~5 FPS）。

### 降频扫码（test_qr_scan.py）
每秒只解码 1 次，视频全速显示（30 FPS）。适合纯扫码场景。

### 视觉伺服（qr_visual_servo.py）
状态机架构，适合飞控场景：

```
SEARCH → 首次 detectAndDecode → 得货架号 + 四角坐标
   ↓
TRACK  → 提取 60 个 Shi-Tomasi 特征点 → 光流跟踪中位数偏移
   ↓       每 30 帧 refine 一次（detectAndDecode 校正漂移）
   ↓       丢失 >30 帧 → 回到 SEARCH
```

| 阶段 | 耗时 | 输出 |
|---|---|---|
| SEARCH (首次 decode) | ~210ms | 货架号 + QR 四角 |
| TRACK (光流跟踪) | ~5ms/帧 | `dx`, `dy`, `area` |
| refine (每30帧) | ~210ms | 校正漂移 |

跟踪方式：不在 QR 四角上做光流（易丢），而是在 QR 区域内提取 60 个 Shi-Tomasi 特征点，计算整体中位数偏移。

视觉伺服输出：
| 数据 | 含义 | 用途 |
|---|---|---|
| `dx_px` | QR 中心距画面中心水平偏移 | yaw/侧向速度 |
| `dy_px` | QR 中心距画面中心垂直偏移 | 高度控制 |
| `area` | QR 四边形面积 | 距离估计 |
| `shelf` | 货架编号 (1~24) | 目标身份 |

完整参考：`CyberCAM/servo/qr_visual_servo.py`

---

## 九、系统工具

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

## 十、与无人机项目的集成方案

当前用途：**CyberCAM 作为视觉协处理器**，通过 UART 将视觉检测结果发送给飞控。

架构选项：
1. **CyberCAM → UART → 飞控 STM32**（直连，延迟最低，推荐）
2. **CyberCAM → UART → 树莓派 RDK X5 → UART → 飞控**（树莓派汇总）

推荐方案 1：视觉伺服指令（目标 dx/dy 等）走 UART 直连飞控，树莓派专注 T265 导航融合和状态机。
