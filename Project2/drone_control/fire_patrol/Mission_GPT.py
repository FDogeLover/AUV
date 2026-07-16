"""
任务状态机 — 基本飞行 (无 K230 / 地面站 / 覆盖规划)

状态机:  IDLE → TAKEOFF → NAVIGATE → LAND → END
控制周期: 30ms
安全保护: FC 超时 2s / T265 丢失急停
"""
import threading
import time
import json
import math
import os
import sys
from collections import deque
from typing import List, Optional
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock, fc_last_rx_time
from t265 import t265_class

# ---------- 常量 ----------
DRY_RUN = os.getenv("DRONE_DRY_RUN", "0") == "1"  # 桌面测试: 不解锁飞控，电机不会转
put_height = 100
VEL_SCALE = 0.7
posthreshold_xy = 0.15
posthreshold_z = 0.20
arrival_confirm_need = 15
arrival_hold_s = 1.5   # 到达判定满足后，在原地强制停留观察的时长（阶跃响应测试用）
arrival_timeout_max = 5.0 + arrival_hold_s
T265_CONFIDENCE_MIN = 2       # 定点所需最低追踪置信度 (0=失败,1=低,2=中,3=高)
T265_CONFIDENCE_WAIT_S = 8.0  # 等待置信度达标的超时时间
FLIGHT_LOG_INTERVAL = 0.05
RAMP_STEP = 1.5
TAKEOFF_CONFIRM_NEED = 10
TAKEOFF_TIMEOUT_S = 15.0
TAKEOFF_LIFTOFF_CM = 35.0  # 一键起飞只负责盲飞离地这一小段，其余交给navigate()的x/y PID+高度ramp爬升到真正目标高度
                            # 不能设太低：2026-07-06实测15cm时T265/激光近地面定位质量下降，起飞confirm超时+机体水平旋转
LAND_CONFIRM_TIMEOUT_S = 25.0  # 降落触发后最多等待多久确认unlock_sta==0(已上锁)，超时也强制退出，避免卡死
                                # 2026-07-09从10.0改为25.0：1.0m高度门槛测试发现10秒内unlock_sta/motor_pwm_mask
                                # 全程未变但用户确认物理已自动降落成功，怀疑是超时定太短、真实上锁发生在
                                # 断开串口之后——调长验证这个假设，同时避免后续测试的"超时"结果继续有歧义
LAND_UNLOCK_CONFIRM_COUNT = 5  # 降落确认去抖：要连续读到N次unlock_sta==0才真正确认已上锁，不是单次就退出。
                                # 2026-07-09真机观察到疑似假阳性——终端打印"已上锁"退出，但用户确认电机实际
                                # 未停转/没有真正降落；飞行日志显示确认发生的那一刻之前unlock_sta全程是1，
                                # 说明原逻辑单次读到0就退出，容易被单帧通信噪声/校验巧合触发误判
LASER_HEIGHT_MAX_M = 10.0  # 激光高度覆盖Z轴前的合理性上限：2026-07-10真机测试(basic_radar)发现降落末尾
                            # 激光传感器偶发返回类似0xFFFFFFFF的错误码，除以100后变成约4.29e7米的垃圾值，
                            # 原逻辑只判断laser_h>0.05、没有上限，会把这个垃圾值当真实高度写进pos[2]。
                            # 10m远超室内飞行实际高度(实测未超过1.4m)，只用来挡掉这种量级的错误码。
ARRIVAL_VEL_THRESH = 0.05  # 到达判定除了位置阈值外，还要求T265速度模长小于此值(m/s)，避免带着残余速度就触发land()盲降
ARRIVAL_VEL_WINDOW = 5  # 到达判定用的速度取最近N帧均值而非单帧瞬时值，平滑T265速度噪声尖峰
                         # (2026-07-07实测: 单帧瞬时速度噪声可达0.07m/s，用瞬时值+连续N次达标会导致到达确认永远凑不齐、超时强制跳过)
ARRIVAL_CONFIRM_RATIO = 0.6  # 到达确认改用滑动窗口比例制而非严格连续帧数：旧逻辑下任意一帧不达标就把
                              # 计数器清零重来，2026-07-08矩形路径测试实测达标帧占比只有30-40%，几乎不可能
                              # 连续凑够arrival_confirm_need帧，导致大多数航点靠超时兜底而非真正确认到达
                              # (2026-07-08复测: 0.8比例下仍有部分航点(占比26-34%)无法确认，下调到0.6)

# ---------- fire_patrol 新增常量 ----------
FIRE_VISION_STALE_S = 0.5       # 火情检测结果超过此值未更新，视为摄像头/线程故障，不阻断飞行
APPROACH_DEADBAND_PX = 20       # 像素偏移小于此值不修正，防止中心附近来回抖
APPROACH_GAIN = 0.15            # APPROACH独立的小增益，明显小于navigate()跨格移动用的PID增益
APPROACH_MAX_STEP_CMPS = 8      # APPROACH阶段单次修正的速度上限(cm/s)，远小于navigate()的40，避免大幅晃动
APPROACH_TIMEOUT_S = 10.0       # 视觉伺服对准超时兜底，超时不强求完全居中，直接进入CONFIRM_WARN
APPROACH_CENTERED_DIST_M = 0.3  # 像素偏移换算的水平距离小于此值才算"对准"，对应赛题
                                  # 发挥部分(1)"接近火源水平距离<=5dm"，留量到3dm量级
HOVER_DROP_ALTITUDE_CM = 10.0 * 10  # 悬停抛投高度=10dm=100cm
HOVER_DROP_DURATION_S = 3.0     # 赛题写死的固定悬停时长，与arrival_hold_s(navigate()到达确认用)
                                  # 完全独立，不能混用——见设计文档"HOVER_DROP"一节
HOVER_HOLD_MAX_STEP_CMPS = APPROACH_MAX_STEP_CMPS  # HOVER_DROP水平位置闭环锁定的输出限幅，
                                  # 复用APPROACH的保守值(8cm/s)而非navigate()跨格移动的40——
                                  # 抛投前悬停应该只做小幅漂移修正，不该有大动作
PIXEL_TO_METER_AT_CRUISE = 0.0015  # 像素偏移->水平距离粗略换算系数，现场需按实际摄像头
                                     # FOV/高度标定，这里给占位初始值

# 2026-07-09新增：yaw方向验证专用——问题16事故后yaw修正回路已回退，但符号问题
# 尚未定位。这里加一个非闭环、短时、固定输出的yaw脉冲测试，绕开PID，直接验证
# 凌霄IMU执行vyaw的真实物理方向，跟转盘验证过的T265测量方向做对比。
# 默认关闭，不会影响正常飞行；显式设DRONE_YAW_TEST_BURST=1才会触发。
YAW_TEST_BURST_ENABLED = os.getenv("DRONE_YAW_TEST_BURST", "0") == "1"
YAW_TEST_BURST_VALUE = int(os.getenv("DRONE_YAW_TEST_BURST_VALUE", "-8"))
YAW_TEST_BURST_DURATION_S = float(os.getenv("DRONE_YAW_TEST_BURST_DURATION_S", "1.5"))


def arrival_window_confirmed(window, need, ratio):
    """window: 最近若干帧"位置+速度是否同时达标"的布尔值(deque)。
    窗口填满(len>=need)且达标帧占比>=ratio才算确认到达——替代旧的"严格连续N帧"
    逻辑，单帧噪声不会让已经积累的进度清零(见 ARRIVAL_CONFIRM_RATIO 常量注释)。"""
    return len(window) >= need and (sum(window) / len(window)) >= ratio


def laser_height_valid(laser_h):
    """激光高度是否合理，可以用来覆盖pos[2]/land_pos[2]。见 LASER_HEIGHT_MAX_M 注释：
    2026-07-10真机测试(basic_radar)捕获到传感器错误码(约0xFFFFFFFF/100)未被过滤污染日志的真实案例。"""
    return 0.05 < laser_h <= LASER_HEIGHT_MAX_M


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int],
                 realsense_obj: Optional[t265_class] = None,
                 serial_fc_ref=None):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.serial_fc_ref = serial_fc_ref

        # 状态机
        self.state = "IDLE"

        # 控制
        self.task_running = False
        self.t265_ok = False
        self.realsense = realsense_obj

        # PID
        self.x_pid = PID(0, 0)
        self.y_pid = PID(0, 0)
        self.yaw_pid = PID(1, 0)

        # 航点
        self.targets = self.load_waypoints()
        self.target_index = 0
        self.emergency_stop = False

        # fire_patrol: 火情检测/接近/悬停抛投状态
        self.nav_mode = "PATROL"  # PATROL / APPROACH / CONFIRM_WARN / HOVER_DROP
        self.fire_triggered = False  # 全程只响应一次火情检测（见设计文档）
        self.saved_target_index_before_fire = None
        self._approach_start_time = None
        self.fire_vision = None  # main.py 注入 FireVision 实例
        self.serial_ground = None  # main.py 注入 Serial_ground 实例
        self._hover_drop_start_time = None
        self._hover_hold_pos = None  # HOVER_DROP闭环锁定的水平目标点，见_do_hover_drop()

        # 到达判断
        self._arrival_window = deque(maxlen=arrival_confirm_need)
        self.arrival_start_time = 0.0
        self.arrival_confirmed_time: Optional[float] = None
        self.last_target_index = -1
        self._vel_window = deque(maxlen=ARRIVAL_VEL_WINDOW)

        # 高度 ramp
        self._ramp_z_cm = 0.0

        # 飞行数据日志
        self._log_file = None
        self._last_log_time = 0.0

        # yaw方向测试(问题16)
        self._yaw_burst_done = False

    def load_waypoints(self):
        try:
            with open('router.txt', 'r', encoding="utf-8") as f:
                waypoints = []
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and not line.startswith('x'):
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                x = float(parts[0].strip())
                                y = float(parts[1].strip())
                                z = float(parts[2].strip())
                                waypoints.append([x, y, z])
                            except ValueError:
                                logger.warning(f"无效航点: {line}")
                if waypoints:
                    logger.info(f"加载 {len(waypoints)} 个航点")
                    return waypoints
        except FileNotFoundError:
            logger.warning("router.txt 不存在，使用默认航点")
        except Exception as e:
            logger.warning(f"读取 router.txt 失败: {e}，使用默认航点")

        default = [[0.0, 0.0, put_height/100],
                   [0.5, 0.0, put_height/100],
                   [0.5, 0.5, put_height/100],
                   [0.0, 0.5, put_height/100]]
        return default

    # ================= 启动 =================
    def start(self):
        if DRY_RUN:
            logger.warning("=" * 40)
            logger.warning("DRY_RUN 模式已启用 — 飞控不会解锁，电机不会转")
            logger.warning("=" * 40)

        if self.realsense and self.realsense.start():
            self.realsense.autoset()
            self.t265_ok = True
            logger.info("T265 OK")

            # 等待追踪置信度稳定，避免T265刚连上、置信度还没起来就进入定点模式
            t_wait_start = time.time()
            confidence = self.realsense.get_tracking_confidence()
            while confidence < T265_CONFIDENCE_MIN and time.time() - t_wait_start < T265_CONFIDENCE_WAIT_S:
                time.sleep(0.2)
                confidence = self.realsense.get_tracking_confidence()

            if confidence < T265_CONFIDENCE_MIN:
                logger.error(f"T265 置信度 {T265_CONFIDENCE_WAIT_S:.0f} 秒内仍偏低(confidence={confidence})，定点可能不稳定")
                confirm = input(f"T265置信度过低(confidence={confidence})，输入 YES 强制起飞，其他任意键取消任务: ")
                if confirm.strip() != "YES":
                    logger.error("任务已取消（T265置信度未确认）")
                    return
                logger.warning("已人工确认，强制以低置信度T265数据起飞")
            else:
                logger.info(f"T265 追踪置信度已稳定 (confidence={confidence})")
        else:
            logger.error("T265 FAILED — 无水平位置反馈，仅高度模式起飞有失控风险")
            confirm = input("T265 未连接，输入 YES 强制以仅高度模式起飞，其他任意键取消任务: ")
            if confirm.strip() != "YES":
                logger.error("任务已取消（T265 未确认）")
                return
            logger.warning("已人工确认，强制以仅高度模式起飞")

        logger.info(f"任务启动, {len(self.targets)} 个航点")

        self.task_running = True
        self.state = "TAKEOFF"

        try:
            path = os.path.dirname(os.path.realpath(sys.argv[0]))
            self._log_file = open(path + "/flight_data.jsonl", "a")
            self._log_file.write(json.dumps({"event": "task_start"}) + "\n")
            self._log_file.flush()
        except Exception:
            pass

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
    def loop(self):
        while self.task_running:

            if self.emergency_stop:
                self.stop_all()
                continue

            # FC 串口超时
            if fc_last_rx_time.value > 0 and time.time() - fc_last_rx_time.value > 2.0:
                logger.error("飞控串口超时 2s，紧急降落")
                self.emergency_stop = True
                continue

            # T265 存活
            if self.t265_ok and self.realsense and not self.realsense.is_running():
                logger.error("T265 已停止，紧急降落")
                self.emergency_stop = True
                continue

            # 获取位置
            pos = [0.0, 0.0, 0.0]
            yaw = 0.0
            if self.realsense:
                try:
                    pos = list(self.realsense.get_position())
                    yaw = self.realsense.get_orientation()[2]
                except Exception:
                    logger.error("T265 读取失败")
                    time.sleep(0.03)
                    continue

            # 激光高度覆盖 Z
            if self.serial_fc_ref is not None:
                with lock:
                    laser_h = self.serial_fc_ref._last_laser_height_cm
                if laser_height_valid(laser_h):
                    pos[2] = laser_h

            # 状态机
            if self.state == "TAKEOFF":
                self.takeoff()
            elif self.state == "NAVIGATE":
                self.navigate(pos, yaw)
            elif self.state == "LAND":
                self.land()
            elif self.state == "END":
                self.stop_all()

            time.sleep(0.03)

    # ================= 起飞 =================
    def takeoff(self):
        if DRY_RUN:
            logger.warning("takeoff: DRY_RUN 模式，不发送解锁指令，电机不会转")
        else:
            logger.info("takeoff: started")

        target_h_cm = TAKEOFF_LIFTOFF_CM  # 一键起飞只爬升到离地高度，真正目标高度交给 navigate() 闭环爬升

        with lock:
            self.se_fc[5] = int(target_h_cm)  # com_z：一键起飞目标高度，必须在 task_sta 触发前写入，
            self.se_fc[2] = 0 if DRY_RUN else 1  # 否则飞控读到的是 se_fc 初始默认值(120cm)而非本次航点高度

        confirm_count = 0
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start

            yaw = 0.0
            vyaw = 0
            if self.t265_ok and self.realsense:
                try:
                    yaw = self.realsense.get_orientation()[2]
                    # 2026-07-09临时回退问题16的角度修复：真机验证发现yaw修正回路一旦真正
                    # 输出非零指令会导致yaw持续发散(疑似固件/协议层符号约定与Python假设不一致，
                    # 形成正反馈而非负反馈)，触发一次~90°失控需人工介入。喂弧度让PID输出重新
                    # 恒近似为0，回到"修正回路事实上不生效"的已知安全状态，直到符号问题查清。
                    vyaw = int(self.limit(self.yaw_pid.get_pid(yaw) * VEL_SCALE, 30))
                    with lock:
                        self.se_fc[6] = vyaw + sp_side
                except Exception:
                    pass

            with lock:
                laser_m = self.serial_fc_ref._last_laser_height_cm if self.serial_fc_ref else 0.0
            laser_cm = laser_m * 100.0

            # 排查起飞离地阶段yaw自稳是否正确（2026-07-06 15cm离地测试观察到水平旋转，
            # 怀疑本函数的vyaw符号跟navigate()相反）：记录T265原始yaw、飞控自己融合的yaw(re_fc[3])、
            # 算出来的修正指令，两路yaw互相交叉验证谁的读数有问题
            with lock:
                fc_yaw_deg = self.re_fc[3] / 100.0 if len(self.re_fc) > 3 else 0.0
            if self._log_file:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(time.time(), 3),
                        "state": "TAKEOFF",
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "fc_yaw_deg": round(fc_yaw_deg, 2),
                        "vyaw": vyaw,
                        "laser_cm": round(laser_cm, 1),
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass

            if laser_cm > 5.0 and abs(laser_cm - target_h_cm) <= 10.0:
                confirm_count += 1
            else:
                confirm_count = 0

            if confirm_count >= TAKEOFF_CONFIRM_NEED:
                logger.info(f"takeoff: 高度确认 {laser_cm:.0f} cm")
                break

            if elapsed >= TAKEOFF_TIMEOUT_S:
                logger.warning("takeoff: 超时，强制切换")
                break

            time.sleep(0.03)

        self._ramp_z_cm = target_h_cm
        self.state = "NAVIGATE"

    # ================= 导航 =================
    def navigate(self, pos, yaw):
        if self.nav_mode == "APPROACH":
            self._do_approach()
            return
        if self.nav_mode == "CONFIRM_WARN":
            self._do_confirm_warn()
            return
        if self.nav_mode == "HOVER_DROP":
            self._do_hover_drop(pos)
            return

        # PATROL态：持续检查火情检测结果，触发APPROACH
        if self.nav_mode == "PATROL" and self.fire_vision is not None:
            latest = self.fire_vision.latest()
            now = time.time()
            if (latest.get("dx_px") is not None
                    and now - latest.get("t", 0) < FIRE_VISION_STALE_S):
                if self.maybe_trigger_approach((latest["dx_px"], latest["dy_px"])):
                    return

        if self.target_index >= len(self.targets):
            logger.info("全部航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]
        target_z = int(target[2] * 100)

        confidence = self.realsense.get_tracking_confidence() if (self.t265_ok and self.realsense) else 0

        if confidence == 0 and self.t265_ok:
            logger.warning("T265 追踪丢失，悬停等待")
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            # 2026-07-09从basic_radar/补同步(2026-07-08已在那边修复)：这里原本直接return
            # 会跳过日志写入，导致T265追踪丢失期间完全没有数据记录。
            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "target_idx": self.target_index,
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                        "vx": 0, "vy": 0, "vyaw": 0,
                        "t265_yaw_deg": round(math.degrees(yaw), 2),
                        "height_setpoint_cm": round(self._ramp_z_cm, 1),
                        "t265_confidence_lost": True,
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now
            return

        if self.t265_ok and self.realsense:
            self.x_pid.set_target(target[0])
            self.y_pid.set_target(target[1])
            self.yaw_pid.set_target(0)
            vx = self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE
            vy = self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE
            # 2026-07-09临时回退问题16的角度修复，理由同上(navigate()同一个符号问题)
            vyaw = self.yaw_pid.get_pid(yaw) * VEL_SCALE
            vx = int(self.limit(vx, 40))
            vy = int(self.limit(vy, 40))
            vyaw = int(self.limit(vyaw, 30))
        else:
            vx, vy, vyaw = 0, 0, 0

        self._step_ramp_z(target_z)
        self.set_speed(vx, vy, -vyaw, int(self._ramp_z_cm))

        # T265 速度（到达检测的速度门槛 + 后面日志/终端输出共用，避免重复取值）
        if self.t265_ok and self.realsense:
            tv = self.realsense.get_velocity()
        else:
            tv = (0.0, 0.0, 0.0)

        # 到达检测
        if self.t265_ok and self.realsense:
            xy_thresh = 0.10 if confidence >= 3 else (posthreshold_xy if confidence == 2 else 0.30)
            dx = abs(pos[0] - target[0])
            dy = abs(pos[1] - target[1])
            dz = abs(pos[2] - target[2])
            # 速度用最近N帧均值而非瞬时值，平滑T265速度噪声尖峰(见 ARRIVAL_VEL_WINDOW 注释)
            self._vel_window.append((tv[0], tv[1]))
            avg_vx = sum(v[0] for v in self._vel_window) / len(self._vel_window)
            avg_vy = sum(v[1] for v in self._vel_window) / len(self._vel_window)
            speed = math.hypot(avg_vx, avg_vy)

            if self.target_index != self.last_target_index:
                self.last_target_index = self.target_index
                self._arrival_window.clear()
                self.arrival_confirmed_time = None
                self.arrival_start_time = time.time()

            if dx > 0.3:
                self.x_pid.reset()
            if dy > 0.3:
                self.y_pid.reset()

            frame_ok = dx < xy_thresh and dy < xy_thresh and dz < posthreshold_z and speed < ARRIVAL_VEL_THRESH
            self._arrival_window.append(frame_ok)

            if arrival_window_confirmed(self._arrival_window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO):
                if self.arrival_confirmed_time is None:
                    self.arrival_confirmed_time = time.time()
                    logger.info(f"到达航点 {self.target_index}，停留 {arrival_hold_s:.0f}s 观察")
                elif time.time() - self.arrival_confirmed_time >= arrival_hold_s:
                    logger.info(f"航点 {self.target_index} 停留完成")
                    self._on_arrival(target)
            else:
                self.arrival_confirmed_time = None

            if time.time() - self.arrival_start_time >= arrival_timeout_max:
                logger.warning(f"航点 {self.target_index} 超时，强制跳过")
                self.target_index += 1

        # 光流融合速度（帧1 of1_dx/dy，用于跟 T265 速度交叉对比）
        # + roll/pitch（帧1 已回传，用于排查高度控制异常是否跟倾角同步，见 CLAUDE.md 已知问题6）
        # + 光流质量/状态（排查异常是否由光流信号本身变差导致）
        with lock:
            of1_dx = self.re_fc[9] if len(self.re_fc) > 9 else 0
            of1_dy = self.re_fc[10] if len(self.re_fc) > 10 else 0
            roll_deg = self.re_fc[1] / 100.0 if len(self.re_fc) > 1 else 0.0
            pitch_deg = self.re_fc[2] / 100.0 if len(self.re_fc) > 2 else 0.0
            fc_yaw_deg = self.re_fc[3] / 100.0 if len(self.re_fc) > 3 else 0.0
            of_quality = self.re_fc[11] if len(self.re_fc) > 11 else 0
            of_link_sta = self.re_fc[12] if len(self.re_fc) > 12 else 0
            of_work_sta = self.re_fc[13] if len(self.re_fc) > 13 else 0

        # 日志
        now = time.time()
        if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
            try:
                self._log_file.write(json.dumps({
                    "t": round(now, 3),
                    "state": self.state,
                    "target_idx": self.target_index,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "vx": vx, "vy": vy, "vyaw": vyaw,
                    "t265_yaw_deg": round(math.degrees(yaw), 2),
                    "fc_yaw_deg": round(fc_yaw_deg, 2),
                    "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                    "of1_vel_cms": [of1_dx, of1_dy],
                    "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                    "height_setpoint_cm": round(self._ramp_z_cm, 1),
                    "of_status": [of_quality, of_link_sta, of_work_sta],
                }) + "\n")
                self._log_file.flush()
            except Exception:
                pass
            self._last_log_time = now

        # 终端输出
        if self.t265_ok and self.realsense:
            t265_str = f"| t265v=({tv[0]:+.2f},{tv[1]:+.2f}) | of1=({of1_dx:+d},{of1_dy:+d})"
        else:
            t265_str = ""
        t265_str += f" | att=({roll_deg:+.1f},{pitch_deg:+.1f})"
        print(
            f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
            f"| tgt=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) "
            f"| v=({vx:>3},{vy:>3}) "
            f"| send=({self.se_fc[3]:>3},{self.se_fc[4]:>3},{self.se_fc[5]:>3})"
            f"{t265_str}",
            end="", flush=True
        )

    # ================= 到达处理 =================
    def _on_arrival(self, target):
        if YAW_TEST_BURST_ENABLED and self.target_index == 0 and not self._yaw_burst_done:
            self._yaw_burst_done = True
            self._do_yaw_test_burst()
        if self.target_index == len(self.targets) - 2:
            pass  # 到达倒数第二个航点 (原 rgb_led 逻辑已移除)
        self.target_index += 1

    # ================= fire_patrol: 火情检测触发 =================
    def maybe_trigger_approach(self, detection):
        """detection为None或已经触发过(fire_triggered=True)时不触发。
        触发时保存当前target_index用于HOVER_DROP完成后续飞。"""
        if detection is None or self.fire_triggered:
            return False
        self.saved_target_index_before_fire = self.target_index
        self.fire_triggered = True
        self.nav_mode = "APPROACH"
        self._approach_start_time = time.time()
        logger.info(f"检测到火情，悬停进入APPROACH（保存航点索引{self.target_index}）")
        return True

    def is_approach_centered(self, dx_px, dy_px):
        """像素偏移换算成水平距离，小于APPROACH_CENTERED_DIST_M才算对准正下方。"""
        dist_m = math.hypot(dx_px, dy_px) * PIXEL_TO_METER_AT_CRUISE
        return dist_m < APPROACH_CENTERED_DIST_M

    def approach_timed_out(self):
        if self._approach_start_time is None:
            return False
        return time.time() - self._approach_start_time >= APPROACH_TIMEOUT_S

    def finish_hover_drop_and_resume(self):
        """HOVER_DROP完成后恢复PATROL，从保存的target_index继续（不重置为0，
        不退回触发点），见设计文档"续飞逻辑"。

        必须重置到达检测状态（arrival_start_time/_arrival_window/arrival_confirmed_time），
        否则arrival_start_time还停留在火情触发前的旧值，APPROACH/CONFIRM_WARN/HOVER_DROP
        整个过程都会被navigate()的超时判断计入"航点等待时长"，恢复PATROL后几乎立刻
        触发"航点超时，强制跳过"，导致续飞航点被立即跳过。
        同时把last_target_index同步为恢复后的target_index：navigate()靠
        target_index != last_target_index判断"是否换了新航点"才会做上述重置，
        如果这里不同步，之后即使真正换到下一个航点，由于凑巧数值相等也会漏判。"""
        self.target_index = self.saved_target_index_before_fire
        self.last_target_index = self.target_index
        self.nav_mode = "PATROL"
        self._approach_start_time = None
        self._arrival_window.clear()
        self.arrival_confirmed_time = None
        self.arrival_start_time = time.time()
        # 清空HOVER_DROP闭环锁定期间累积的PID积分项，避免带着旧误差历史进入
        # 恢复后的巡航航段(navigate()的PATROL分支会在下一tick重新set_target，
        # 但reset()清掉积分项更干净，不留隐性偏置)
        self.x_pid.reset()
        self.y_pid.reset()
        self._hover_hold_pos = None

        # 2026-07-16真机测试发现：警示LED点亮后(_do_confirm_warn里的warn_led())
        # 从没有任何地方关掉，导致降落时灯还亮着——set_rgb_led()是状态型接口，
        # 点亮后不会自动熄灭，调用方必须自己在合适时机关掉。LED的作用是示警
        # "正在处理这次火情"，续飞后不该继续亮着占用状态，这里关掉。
        try:
            from Lcode.gpio_led import set_rgb_led
            set_rgb_led('OFF')
        except Exception as e:
            logger.error(f"set_rgb_led('OFF') 调用失败: {e}")

    def _do_approach(self):
        """悬停对准正下方：独立小增益+死区+符号预验证的伺服修正，超时兜底进CONFIRM_WARN。
        见设计文档"APPROACH（视觉伺服对准正下方）"一节。"""
        if self.fire_vision is None:
            self.nav_mode = "CONFIRM_WARN"
            return
        latest = self.fire_vision.latest()
        dx_px, dy_px = latest.get("dx_px"), latest.get("dy_px")

        if dx_px is not None and dy_px is not None:
            if self.is_approach_centered(dx_px, dy_px):
                logger.info("APPROACH: 已对准正下方")
                self.nav_mode = "CONFIRM_WARN"
                return
            # 死区：像素偏移小于阈值不修正
            # 符号约定(2026-07-16物理确认下视摄像头安装朝向后推导，非猜测)：
            #   画面上边=+y、右边=+x(无镜像安装)
            #   dx_px>0(目标在画面右侧) → 目标物理上在+x方向 → 需要+x速度靠近 → vx与dx_px同号
            #   dy_px>0(目标在画面下方) → 画面"下"对应-y(上边才是+y) → 目标物理上在-y方向
            #     → 需要-y速度靠近 → vy与dy_px反号，不能直接用dy_px*增益(那样会正反馈发散，
            #     跟这个项目里yaw方向出过的同类问题一样)
            vx = 0 if abs(dx_px) < APPROACH_DEADBAND_PX else self.limit(
                dx_px * APPROACH_GAIN, APPROACH_MAX_STEP_CMPS)
            vy = 0 if abs(dy_px) < APPROACH_DEADBAND_PX else self.limit(
                -dy_px * APPROACH_GAIN, APPROACH_MAX_STEP_CMPS)
            self.set_speed(int(vx), int(vy), 0, int(self._ramp_z_cm))
        else:
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))

        if self.approach_timed_out():
            logger.warning("APPROACH: 对准超时，按当前位置继续")
            self.nav_mode = "CONFIRM_WARN"

    def _do_confirm_warn(self):
        """点亮警示LED后立即进入HOVER_DROP。"""
        from Lcode.actuators import warn_led
        try:
            warn_led()
        except Exception as e:
            logger.error(f"warn_led() 调用失败: {e}")
        self.nav_mode = "HOVER_DROP"
        self._hover_drop_start_time = None
        self._hover_hold_pos = None  # 触发_do_hover_drop()第一次调用时锁定当前位置

    def _do_hover_drop(self, pos):
        """降到10dm悬停HOVER_DROP_DURATION_S后抛投+广播坐标，见设计文档"HOVER_DROP"一节。
        水平位置用x_pid/y_pid闭环锁定在进入本状态那一刻的坐标，不能像之前那样只发
        vx=vy=0的开环零速度指令——3秒悬停期间任何风扰/残余速度都会导致漂移，
        影响抛投精度和最终广播坐标的准确性(用户2026-07-16反馈：悬停期间需要闭环
        控制保证精度，不能开环)。"""
        if self._hover_hold_pos is None:
            self._hover_hold_pos = (pos[0], pos[1])
            self.x_pid.set_target(pos[0])
            self.y_pid.set_target(pos[1])
            self.x_pid.reset()
            self.y_pid.reset()
            logger.info(f"HOVER_DROP: 锁定水平位置 ({pos[0]:.2f}, {pos[1]:.2f})")

        self._step_ramp_z(HOVER_DROP_ALTITUDE_CM)

        vx = self.x_pid.get_pid(pos[0]) * 100 * VEL_SCALE
        vy = self.y_pid.get_pid(pos[1]) * 100 * VEL_SCALE
        vx = int(self.limit(vx, HOVER_HOLD_MAX_STEP_CMPS))
        vy = int(self.limit(vy, HOVER_HOLD_MAX_STEP_CMPS))
        self.set_speed(vx, vy, 0, int(self._ramp_z_cm))

        # setpoint_ok: 内部软件ramp是否已收敛到目标值（跟原逻辑一样）。
        # measured_ok: pos[2](loop()里已被激光高度覆盖成米制真实测量值，见laser_height_valid
        # 分支)是否也接近目标高度。只信setpoint会在真实高度还没跟上（比如飞控/凌霄IMU响应滞后、
        # 或悬停中被风扰导致实际高度偏离）时提前开始计时，抛投时实际高度不在10dm。
        # 阈值10cm：比posthreshold_z(0.20m=20cm，navigate()到达检测用的量级)更严格，因为
        # HOVER_DROP要求精确到10dm这个赛题写死的高度用于抛投，容忍度不宜比普通到达检测更松；
        # 但也不需要严格到cm级——激光高度本身有噪声，10cm对应赛题100cm目标的10%相对误差，
        # 参考takeoff()里laser_cm的10cm确认容差保持一致。
        setpoint_ok = abs(self._ramp_z_cm - HOVER_DROP_ALTITUDE_CM) <= 2.0
        measured_ok = abs(pos[2] * 100 - HOVER_DROP_ALTITUDE_CM) <= 10.0
        if not (setpoint_ok and measured_ok):
            return  # 还没真正降到位，不开始计时

        if self._hover_drop_start_time is None:
            self._hover_drop_start_time = time.time()
            logger.info(f"HOVER_DROP: 到达{HOVER_DROP_ALTITUDE_CM:.0f}cm，悬停{HOVER_DROP_DURATION_S:.0f}s")
            return

        if time.time() - self._hover_drop_start_time < HOVER_DROP_DURATION_S:
            return

        from Lcode.actuators import drop_bag
        try:
            drop_bag()
        except Exception as e:
            logger.error(f"drop_bag() 调用失败: {e}")
        if self.serial_ground is not None:
            try:
                self.serial_ground.send_fire(int(pos[0] * 100), int(pos[1] * 100))
            except Exception as e:
                logger.error(f"serial_ground.send_fire() 调用失败: {e}")
        logger.info("HOVER_DROP: 抛投+坐标广播完成，恢复PATROL续飞")
        self.finish_hover_drop_and_resume()

    def _do_yaw_test_burst(self):
        """非闭环yaw方向验证：直接发固定vyaw一小段时间后归零，不经过yaw_pid。
        阻塞调用线程(navigate()所在的主循环线程)，但发送线程独立运行在另一个
        线程，se_fc当前值会持续以100Hz/50Hz发出，不受阻塞影响，是安全的。"""
        if not (self.t265_ok and self.realsense):
            logger.warning("[YAW测试] T265不可用，跳过")
            return
        yaw0 = math.degrees(self.realsense.get_orientation()[2])
        logger.warning(
            f"[YAW测试] 脉冲前yaw={yaw0:.1f}° 发送固定vyaw={YAW_TEST_BURST_VALUE}"
            f"持续{YAW_TEST_BURST_DURATION_S:.1f}秒(非闭环，不经过yaw_pid)"
        )
        t_start = time.time()
        with lock:
            self.se_fc[6] = YAW_TEST_BURST_VALUE + sp_side
        while time.time() - t_start < YAW_TEST_BURST_DURATION_S:
            time.sleep(0.05)
        with lock:
            self.se_fc[6] = 0 + sp_side
        yaw1 = math.degrees(self.realsense.get_orientation()[2])
        logger.warning(
            f"[YAW测试] 脉冲后yaw={yaw1:.1f}° 变化={yaw1 - yaw0:+.1f}°"
            f"（vyaw为负，若变化为负=方向符合固件'逆时针为正'约定；若变化为正=方向相反，疑似正反馈根因）"
        )

    # ================= 降落 =================
    def land(self):
        logger.info("降落")
        with lock:
            self.se_fc[2] = 0

        # 不能一触发就关串口退出：凌霄IMU定点悬停依赖Pi持续喂T265速度参考(CMD 0x33)，
        # 串口一关这个参考直接断流，而OneKey_Land()的物理下降通常要持续数秒。
        # 这里继续跑主循环(保持串口/T265速度帧不断)，轮询真实解锁状态(unlock_sta)，
        # 确认真的上锁了(或超时兜底)才真正进入END关闭退出。
        #
        # 2026-07-08修复：这个循环原本只轮询unlock_sta，没有主动清零速度指令——
        # se_fc[3]/[4]/[6]会停留在navigate()最后一次set_speed()的值上，被发送线程原样
        # 重复发送，且没有PID再持续修正，真机测试观察到一键降落无响应时飞机会明显
        # 偏离原位置。这里持续调用set_speed(0,0,0,ramp)清零水平速度、保持高度，
        # 避免过期指令导致失控漂移。
        # 2026-07-08修复：land()原本从触发到确认/超时全程不写任何飞行日志，导致
        # 降落物理下降过程完全没有位置数据(真机测试想验证"降落时有没有额外偏移"
        # 但发现日志是空的)。这里跟takeoff()一样，自己在循环里直接采样T265(不依赖
        # loop()调用时传入的旧值，那个值在整个等待期间不会更新)。
        t_start = time.time()
        unlock_confirm_count = 0
        while True:
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))

            if self.t265_ok and self.realsense:
                try:
                    land_pos = list(self.realsense.get_position())
                    land_yaw = self.realsense.get_orientation()[2]
                    land_tv = self.realsense.get_velocity()
                except Exception:
                    land_pos, land_yaw, land_tv = [0.0, 0.0, 0.0], 0.0, (0.0, 0.0, 0.0)
            else:
                land_pos, land_yaw, land_tv = [0.0, 0.0, 0.0], 0.0, (0.0, 0.0, 0.0)

            # 激光高度覆盖Z：跟 loop()/takeoff() 一样，T265自身Z轴未标定不是真实高度，
            # 这里如果继续用原始T265 Z，降落阶段记录的"高度"会是假数据，没法验证物理降落过程。
            with lock:
                laser_h = self.serial_fc_ref._last_laser_height_cm if self.serial_fc_ref else 0.0
            if laser_height_valid(laser_h):
                land_pos[2] = laser_h

            with lock:
                unlock_sta = self.re_fc[5] if len(self.re_fc) > 5 else 0

            # 电机PWM非零位掩码(帧2新增字段)：诊断unlock_sta是否假阳性
            # (问题7 2026-07-08：unlock_sta读到0但用户确认电机实际未停转)
            motor_pwm_mask = None
            motor_pwm_mask_t = None
            if self.serial_fc_ref is not None:
                with lock:
                    motor_pwm_mask = self.serial_fc_ref.debug_data.get("motor_pwm_mask")
                    motor_pwm_mask_t = self.serial_fc_ref.debug_data.get("motor_pwm_mask_t")

            now = time.time()
            if self._log_file and now - self._last_log_time >= FLIGHT_LOG_INTERVAL:
                try:
                    self._log_file.write(json.dumps({
                        "t": round(now, 3),
                        "state": self.state,
                        "pos": [round(land_pos[0], 4), round(land_pos[1], 4), round(land_pos[2], 4)],
                        "t265_yaw_deg": round(math.degrees(land_yaw), 2),
                        "t265_vel": [round(land_tv[0], 4), round(land_tv[1], 4)],
                        "unlock_sta": unlock_sta,
                        "motor_pwm_mask": motor_pwm_mask,
                        "motor_pwm_mask_t": motor_pwm_mask_t,
                    }) + "\n")
                    self._log_file.flush()
                except Exception:
                    pass
                self._last_log_time = now

            # 2026-07-10修复：只看unlock_sta的去抖仍会假阳性(问题7)——矩形路径基线测试
            # (basic_radar)复现了unlock_sta连续读到0、去抖满足，但motor_pwm_mask全程
            # 非零(电机仍在出PWM)的矛盾场景，用户确认那次是人工接管才降落的。这里要求
            # unlock_sta==0同时motor_pwm_mask==0才计入确认；motor_pwm_mask为None(诊断
            # 数据不可用)时不阻塞，退化成只看unlock_sta。
            motor_pwm_ok = motor_pwm_mask is None or motor_pwm_mask == 0
            if unlock_sta == 0 and motor_pwm_ok:
                unlock_confirm_count += 1
                if unlock_confirm_count >= LAND_UNLOCK_CONFIRM_COUNT:
                    logger.info("降落确认：已上锁")
                    break
            else:
                unlock_confirm_count = 0
            if time.time() - t_start >= LAND_CONFIRM_TIMEOUT_S:
                logger.warning("降落确认超时，强制退出")
                break
            time.sleep(0.03)

        self.state = "END"

    # ================= 停止 =================
    def stop_all(self):
        logger.info("任务结束")
        try:
            if self._log_file:
                self._log_file.close()
        except Exception:
            pass
        with lock:
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[6] = sp_side
            self.se_fc[7] = 101
        if self.realsense:
            self.realsense.stop()
        self.task_running = False

    # ================= 控制接口 =================
    def set_speed(self, x, y, yaw, z):
        with lock:
            self.se_fc[3] = x + sp_side
            self.se_fc[4] = y + sp_side
            self.se_fc[5] = z
            self.se_fc[6] = yaw + sp_side

    # ================= 工具 =================
    def limit(self, v, max_v=0.3):
        return max(min(v, max_v), -max_v)

    def _step_ramp_z(self, target_z_cm: float):
        if self._ramp_z_cm < target_z_cm - RAMP_STEP:
            self._ramp_z_cm += RAMP_STEP
        elif self._ramp_z_cm > target_z_cm + RAMP_STEP:
            self._ramp_z_cm -= RAMP_STEP
        else:
            self._ramp_z_cm = target_z_cm

    # ================= 急停 =================
    def emergency(self):
        logger.warning("紧急停止触发！")
        self.emergency_stop = True
