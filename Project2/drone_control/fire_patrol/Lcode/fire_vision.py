"""下视摄像头红色火源检测。见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md
"覆盖巡逻路径"/"APPROACH"一节。

2026-07-16确认：下视摄像头硬件是IMX219 CSI摄像头(ubuntu-pi/RDK X5, /dev/video10)，
不是标准USB摄像头——官方文档明确警告不能直接`cv2.VideoCapture(device)`("当前 RDK
驱动在通用 OpenCV/V4L2 取流路径中可能超时")，必须通过板载专用封装取流。改用
`Lcode/rdk_imx219_jupyter_preview.VisionSystem`（该模块内部持有自己的后台采集
线程+V4L2 MMAP专用C生产程序，从ubuntu-pi板子`Desktop/IMX219/`复制进本仓库纳入
版本控制，避免依赖仓库外未跟踪路径——那个目录还有一份不完整的旧版本，见
project_imx219_camera_bringup记忆，不要弄混）。

`detect_fire()`/`SmoothedFireDetector`是纯函数/纯逻辑，跟摄像头取流方式无关，
不受这次硬件方案调整影响。
"""
import os
import threading
import time
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np

from Lcode.Logger import logger
from Lcode.rdk_imx219_jupyter_preview import VisionSystem

IMX219_DEVICE = "/dev/video10"  # VSE缩放输出节点，官方文档推荐的实时预览/取流节点
IMX219_WIDTH = 960
IMX219_HEIGHT = 540
# 2026-07-16台架测试用值(文档"综合推荐"档)，画面偏暗噪点重是已知问题
# (见project_imx219_camera_bringup记忆)，用户决定先实际飞行看效果，不理想再调
IMX219_EXPOSURE = 2645
IMX219_VERTICAL_BLANKING = 2446

# 火源面积范围：灯罩高度不超过10cm、俯视为近似圆形光斑。上限用于排除大面积
# 反光/其他红色物体误触发(见设计文档审查发现1"误触发风险不可逆")，具体像素
# 数值需现场标定(取决于飞行高度/摄像头视场角)，这里给经验初始值。
# 2026-07-16台架测试(bench_test_snapshot.png/2)实测：室内测试距离下红色光晕连通域
# 面积298832/129534px，原50000的上限在室内可达到的任何测试距离下都会被撑爆——
# 按距离平方反比外推，要降到50000以内大约需要再拉远1.6倍距离，室内空间达不到。
# 上限本意是挡"占满大半画面"级别的明显异常反光，不是精确卡目标尺寸，因此放宽到
# 200000(能通过这两次台架数据，仍能挡住293万像素总画幅的~40%以上占比这种极端情况)。
# 真实18dm飞行高度下光源占比会小得多，这个上限届时基本不会被触及，留待真机测试验证。
MIN_FIRE_AREA_PX = 200
MAX_FIRE_AREA_PX = 200000

# 圆度过滤：4*pi*面积/周长^2，完美圆形=1.0，越扁/越不规则越接近0。真实火源
# 灯罩俯视是近似圆形光斑，用这个过滤能排除面积恰好落在范围内、但形状是矩形/
# 长条形的干扰物(比如小车车身反光)——2026-07-16真机测试实测触发了一次误检测，
# 检测到的目标后来确认是"起点边上小车启动的红色矩形"，不是真实火源，面积上限
# 挡不住(矩形恰好在合理面积范围内)，需要额外靠形状区分。
# 2026-07-17从0.5上调到0.7：真机测试复现了另一种绕过方式——下视摄像头视场里当时
# 有一条固定遮挡物(疑似脚架腿，已现场挪开)把一块红色矩形地标切成两个连通域，
# 完整矩形圆度0.326被正确挡住，但被切碎后较小的那块碎片圆度0.639混过了0.5的
# 阈值，触发误判(详见test_data/fire_patrol_20260717离线复算)。0.7能挡住这类
# "形状不规则但圆度介于0.5~0.7之间"的碎片，离唯一一次确认过的真实火源命中
# (圆度0.790，见known_issues问题28)还留了约0.09的余量，但样本量仍然只有
# 1真1假两个数据点，不是很有统计把握——遮挡物本身已挪开是更根本的修复，
# 这个阈值调整是双重保险，不是唯一防线。
MIN_CIRCULARITY = 0.7

# 红色HSV阈值，色相环绕0/180两段取并集。经验初始值，现场需按实际LED光源标定
# (参照circle_pole真机标定红杆子的经验：偏暗/偏亮光源饱和度差异很大)。
HSV_RANGES_RED = [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))]


def detect_fire(frame_bgr, min_area: int = MIN_FIRE_AREA_PX,
                 max_area: int = MAX_FIRE_AREA_PX,
                 min_circularity: float = MIN_CIRCULARITY) -> Optional[Tuple[float, float]]:
    """在BGR图像里找面积在[min_area, max_area]范围内、形状接近圆形(圆度>=
    min_circularity)的最大红色连通域，返回(dx_px, dy_px) = 质心像素坐标 - 画面
    中心，没找到时返回None。"""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    height, width = frame_bgr.shape[:2]

    mask = None
    for lower, upper in HSV_RANGES_RED:
        m = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = m if mask is None else (mask | m)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_area = 0
    best_cx, best_cy = None, None
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter <= 0:
            continue
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < min_circularity:
            continue
        if area > best_area:
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue
            best_area = area
            best_cx = moments["m10"] / moments["m00"]
            best_cy = moments["m01"] / moments["m00"]

    if best_cx is None:
        return None
    return best_cx - width / 2.0, best_cy - height / 2.0


class SmoothedFireDetector:
    """对detect_fire()逐帧结果做滑动平均，减少单帧噪声导致APPROACH阶段误修正
    (见设计文档"独立的、更保守的伺服增益"一节)。任意一帧丢失目标(None)时清空
    窗口重新积累，不能用陈旧偏移量继续参与平均。"""

    def __init__(self, window: int = 5):
        self.window = window
        self._buf = deque(maxlen=window)

    def update(self, detection: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        if detection is None:
            self._buf.clear()
            return None
        self._buf.append(detection)
        if len(self._buf) < self.window:
            return None
        avg_dx = sum(d[0] for d in self._buf) / len(self._buf)
        avg_dy = sum(d[1] for d in self._buf) / len(self._buf)
        return avg_dx, avg_dy


class FireVision:
    """后台线程持续从IMX219取帧+红色检测+滑动平均，主循环每tick只读`latest()`
    共享的最新结果，不阻塞30ms主循环(风格与Serial_fc.listen_fc()一致)。

    底层`VisionSystem`已经有自己的后台采集线程和帧缓存，`get_frame()`是非阻塞的
    (立即返回缓存的最新帧或None)——本类自己的`_loop()`只是按固定节奏轮询它、
    跑detect_fire()，不需要也不应该假设`get_frame()`会阻塞到下一帧就绪
    (那是cv2.VideoCapture.read()的行为，VisionSystem不是这样)。

    摄像头打不开时`start()`返回False，`latest()`永远返回全None——PATROL态"检测到
    火情"条件永远不满足，等同于纯巡逻场景，不会crash也不会阻塞主循环(见设计文档
    审查发现2"检测线程健康检查缺失"，此处只保证不crash，心跳超时告警留待真机
    测试阶段)。`VisionSystem.open()`打不开摄像头时会抛`RuntimeError`(不是像
    `cv2.VideoCapture.isOpened()`那样返回False)，这里用try/except吞掉转换成
    同样的"返回False"约定，维持对`main.py`调用方透明。
    """

    def __init__(self, device: str = IMX219_DEVICE, smooth_window: int = 5,
                 debug_dir: str = None):
        self.device = device
        self._lock = threading.Lock()
        self._latest = {"dx_px": None, "dy_px": None, "t": 0.0}
        self._latest_frame = None  # 最新一帧原始BGR画面，供save_snapshot()存档用
        self._running = False
        self._vision = None
        self._smoother = SmoothedFireDetector(window=smooth_window)
        # 2026-07-16新增：火情触发时存一张现场画面方便事后调试分析。不像
        # circle_pole/Lcode/pole_vision.py那样持续存debug帧(那边默认要env var
        # 显式开启)——fire_patrol因为fire_triggered全程只触发一次，存图频率
        # 天然很低，默认就开(不需要环境变量才启用)，目录仍可用环境变量覆盖。
        self.debug_dir = debug_dir or os.getenv("DRONE_FIRE_DEBUG_DIR", "fire_debug")

    def start(self) -> bool:
        self._vision = VisionSystem(
            device=self.device,
            raw_width=IMX219_WIDTH,
            raw_height=IMX219_HEIGHT,
            display_width=IMX219_WIDTH,
            display_height=IMX219_HEIGHT,
            exposure=IMX219_EXPOSURE,
            analogue_gain=0,
            vertical_blanking=IMX219_VERTICAL_BLANKING,
            blue_gain=0.95,
            green_gain=1.06,
            red_gain=1.00,
            saturation=1.08,
            enable_bright_detection=False,  # 用自己的detect_fire()，不用模块自带的亮点检测
            capture_thread_daemon=True,     # 跟随主进程退出，不强制要求显式release()
            auto_open=False,
        )
        try:
            self._vision.open()
        except Exception as e:
            logger.error(f"下视摄像头(IMX219, {self.device})打不开: {e}，火情检测禁用")
            self._vision = None
            return False
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        return True

    def stop(self):
        self._running = False
        if self._vision is not None:
            self._vision.release()

    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    def save_snapshot(self, reason: str = "fire_triggered") -> Optional[str]:
        """把最新一帧原始画面存到self.debug_dir，文件名带时间戳+reason，
        方便事后对照当时到底看到了什么(比如误检测复盘)。没有可用帧或写入失败
        时返回None，不抛异常——调用方(火情触发时)不应该因为存图失败被打断。"""
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
        if frame is None:
            logger.warning("save_snapshot: 当前没有可用帧，跳过存图")
            return None
        try:
            os.makedirs(self.debug_dir, exist_ok=True)
            path = os.path.join(
                self.debug_dir,
                f"{time.strftime('%Y%m%d_%H%M%S')}_{reason}.jpg",
            )
            cv2.imwrite(path, frame)
            logger.info(f"save_snapshot: 已存图 {path}")
            return path
        except Exception as e:
            logger.error(f"save_snapshot 失败: {e}")
            return None

    def _loop(self):
        while self._running:
            frame = self._vision.get_frame()
            if frame is not None:
                raw = detect_fire(frame)
                smoothed = self._smoother.update(raw)
                with self._lock:
                    self._latest_frame = frame
                    if smoothed is None:
                        self._latest = {"dx_px": None, "dy_px": None, "t": time.time()}
                    else:
                        self._latest = {"dx_px": smoothed[0], "dy_px": smoothed[1], "t": time.time()}
            time.sleep(0.03)
