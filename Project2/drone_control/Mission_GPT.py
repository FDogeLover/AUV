import threading
import time
import json
import os
import sys
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock, fc_last_rx_time
from Lcode.k230_client import K230Client
from Lcode.coverage_planner import CoveragePlanner
from t265 import t265_class
from Lcode.rgb_led import rgb_led

put_height = 100
fly_height = 100
VEL_SCALE = 0.7  # XY/Yaw 速度缩放系数 (1.0=原速, 0.7=七成)
posthreshold_xy = 0.15  # XY 到达阈值（米），置信度高时使用，低置信度时动态增大
posthreshold_z = 0.20   # Z 到达阈值（米）
arrival_confirm_need = 15  # XY 连续确认到达次数 (~450ms)
arrival_timeout_max = 5.0  # 单航点超时（秒）
FLIGHT_LOG_INTERVAL = 1.0  # 飞行数据记录间隔（秒）
RAMP_STEP = 1.5        # cm per frame at 30 ms cycle ≈ 50 cm/s climb/descent rate
TAKEOFF_CONFIRM_NEED = 10     # consecutive frames within ±10 cm of target
TAKEOFF_TIMEOUT_S    = 15.0   # force transition to NAVIGATE after this
T265_CONFIDENCE_MIN = 2       # 定点所需最低追踪置信度 (0=失败,1=低,2=中,3=高)
T265_CONFIDENCE_WAIT_S = 8.0  # 等待置信度达标的超时时间


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int], re_dmz: List[int], se_dmz: List[int],
                 realsense_obj=None, k230_client=None, serial_fc_ref=None):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.re_dmz = re_dmz
        self.se_dmz = se_dmz
        self.k230 = k230_client
        self.serial_fc_ref = serial_fc_ref  # 用于查询激光高度

        # 状态机
        self.state = "IDLE"

        # 控制
        self.task_running = False
        self.t265_ok = False
        self.realsense = realsense_obj

        # PID控制器（常驻）
        self.x_pid = PID(0, 0)
        self.y_pid = PID(0, 0)
        self.yaw_pid = PID(1, 0)

        # 当前目标
        self.current_target = None

        # 航点
        self.targets = self.load_waypoints()
        
        self.target_index = 0
        self.emergency_stop = False

        # 到达判断状态（进入航点时自动重置）
        self.arrival_confirm_count = 0
        self.arrival_start_time = 0.0
        self.last_target_index = -1

        # Height ramp state (cm); steps toward target each navigate() frame
        self._ramp_z_cm = 0.0

        # 飞行数据日志
        self._log_file = None
        self._last_log_time = 0.0

    def load_waypoints(self):
        """从router.txt文件加载航点"""
        try:
            with open('router.txt', 'r') as f:      
                waypoints = []
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and not line.startswith('x'):
                        # 格式: x,y,z  (逗号分隔)
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                x = float(parts[0].strip())
                                y = float(parts[1].strip())
                                z = float(parts[2].strip()) 
                                waypoints.append([x, y, z])  
                            except ValueError:
                                logger.warning(f"无效的航点数据: {line}")
                if waypoints:
                    logger.info(f"从router.txt加载了{len(waypoints)}个航点")
                    return waypoints
                else:
                    logger.warning("router.txt中没有有效的航点数据，使用默认航点")
        except FileNotFoundError:
            logger.warning("router.txt文件不存在，使用默认航点")
        except Exception as e:
            logger.warning(f"读取router.txt时出错: {str(e)}，使用默认航点")

        # 默认航点（第一个航点为起飞点：原地爬升到目标高度）
        default_waypoints = [[0.0, 0.0,put_height/100], [0.5, 0.0,put_height/100], [0.5, 0.5,put_height/100], [0.0, 0.5,put_height/100]]
        logger.info(f"使用默认航点: {default_waypoints}")
        return default_waypoints

    def _gs_data_valid(self):
        """校验地面站数据：3个有效禁飞区坐标 (A1~A9, B1~B7)"""
        if len(self.re_dmz) < 3:
            return False
        for a_str, b_str in self.re_dmz:
            try:
                a = int(a_str[1:])
                b = int(b_str[1:])
                if not (1 <= a <= 9 and 1 <= b <= 7):
                    return False
            except (ValueError, IndexError):
                return False
        return True

    # ================= 启动 =================
    def start(self):

        if self.realsense.start():
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

        # === 动态路径生成（优先用地面站禁飞区，失败则保留 load_waypoints 结果） ===
        gs_ok = False
        logger.info("等待地面站禁飞区数据...")
        for _ in range(50):  # 5s 超时
            with lock:
                gs_ok = self._gs_data_valid()
            if gs_ok:
                break
            time.sleep(0.1)

        if gs_ok:
            with lock:
                forbidden = list(self.re_dmz)
            logger.info(f"地面站禁飞区: {forbidden}")
            planner = CoveragePlanner(forbidden)
            name, full_path, steps = planner.plan()
            self.targets = []
            for xy in full_path:
                rx, ry = planner.xy_to_real(xy)
                self.targets.append([rx, ry, fly_height / 100])
            self.target_index = 0
            self.last_target_index = -1
            logger.info(f"动态路径: 策略={name}, {len(self.targets)}航点")
        else:
            logger.warning("地面站数据不可用，使用 router.txt 静态路径")

        self.task_running = True
        self.state = "TAKEOFF"
        # 打开飞行数据日志
        try:
            path = os.path.dirname(os.path.realpath(sys.argv[0]))
            log_file = open(path + "/flight_data.jsonl", "a")
            log_file.write(json.dumps({"event": "task_start"}) + "\n")
            log_file.flush()
            self._log_file = log_file
        except Exception:
            pass

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
    def loop(self):
        while self.task_running:

            if self.emergency_stop:
                self.stop_all()
                continue

            # 串口超时检测：超过2秒无飞控回传数据则急停
            if fc_last_rx_time.value > 0 and time.time() - fc_last_rx_time.value > 2.0:
                logger.error("飞控串口超时无回传，触发紧急降落")
                self.emergency_stop = True
                continue

            # T265存活检测：数据采集线程已停止则急停
            if self.t265_ok and not self.realsense.is_running():
                logger.error("T265数据采集已停止，触发紧急降落")
                self.emergency_stop = True
                continue

            # 获取定位
            try:
                pos = self.realsense.get_position()
                yaw = self.realsense.get_orientation()[2]
            except Exception:
                logger.error("T265 ERROR")
                continue

            # P1a: 用飞控回传的激光测距高度覆盖 Z 轴（替代 T265 伪 Z 数据）
            if self.serial_fc_ref is not None:
                # 注意: _last_laser_height_cm 需要在 listen_fc 的 lock 中写入，保持一致
                with lock:
                    laser_h = self.serial_fc_ref._last_laser_height_cm
                if laser_h > 0.05:  # 有效值 > 5cm
                    pos[2] = laser_h

            # 状态机调度
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
        logger.info("takeoff: started")

        with lock:
            self.se_fc[2] = 1  # trigger FC: unlock + mode switch

        target_h_cm = float(self.targets[0][2] * 100)
        confirm_count = 0
        t_start = time.time()

        while True:
            elapsed = time.time() - t_start

            # Yaw stabilization during climb
            if self.t265_ok:
                try:
                    yaw = self.realsense.get_orientation()[2]
                    vyaw = int(self.limit(self.yaw_pid.get_pid(yaw) * VEL_SCALE, 30))
                    with lock:
                        self.se_fc[6] = vyaw + sp_side
                except Exception:
                    pass

            # Height confirmation (note: _last_laser_height_cm is in metres)
            with lock:
                laser_m = self.serial_fc_ref._last_laser_height_cm \
                          if self.serial_fc_ref else 0.0
            laser_cm = laser_m * 100.0

            if laser_cm > 5.0 and abs(laser_cm - target_h_cm) <= 10.0:
                confirm_count += 1
            else:
                confirm_count = 0

            if confirm_count >= TAKEOFF_CONFIRM_NEED:
                logger.info(f"takeoff: height confirmed {laser_cm:.0f} cm")
                break

            if elapsed >= TAKEOFF_TIMEOUT_S:
                logger.warning("takeoff: timeout, proceeding anyway")
                break

            time.sleep(0.03)

        # Seed ramp at first waypoint height so navigate() starts smooth
        self._ramp_z_cm = target_h_cm
        self.state = "NAVIGATE"
    
    def navigate(self, pos, yaw):
        if self.target_index >= len(self.targets):
            logger.info("所有航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]

        # Z轴: 直接传航点高度（米→厘米），FC自主控高
        target_z = int(target[2] * 100)

        # 每帧更新地面站: idx=实时进度, cls/cnt=哨兵值(FF/0)
        with lock:
            self.se_dmz[1] = self.target_index & 0xFF
            if self.se_dmz[2] != 0xFF or self.se_dmz[3] != 0:
                self.se_dmz[2] = 0xFF
                self.se_dmz[3] = 0

        # XY/Yaw: PID计算速度（检测期间也运行，维持悬停）
        confidence = self.realsense.get_tracking_confidence() if self.t265_ok else 0

        # P1c: 置信度==0 → 悬停不飞
        if confidence == 0:
            logger.warning("T265追踪完全丢失，悬停等待恢复")
            # Hold current ramp height during hover; avoid a Z jump on lost tracking
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            return
        self.x_pid.set_target(target[0])
        self.y_pid.set_target(target[1])
        self.yaw_pid.set_target(0)
        vx = self.x_pid.get_pid(pos[0])
        vy = self.y_pid.get_pid(pos[1])
        vyaw = self.yaw_pid.get_pid(yaw)

        vx *= 100 * VEL_SCALE
        vy *= 100 * VEL_SCALE
        vx = int(self.limit(vx, 40))
        vy = int(self.limit(vy, 40))
        vyaw = int(self.limit(vyaw * VEL_SCALE, 30))
        # Smooth height: step the ramp toward target_z, send ramped value (no jumps)
        self._step_ramp_z(target_z)
        self.set_speed(vx, vy, -vyaw, int(self._ramp_z_cm))

        # === 到达判断 ===
        # P0b: 动态阈值 - 按追踪置信度自适应调整XY阈值
        if confidence >= 3:
            xy_thresh = 0.10   # 高置信度: 严格阈值
        elif confidence == 2:
            xy_thresh = posthreshold_xy  # 中置信度: 默认
        else:
            xy_thresh = 0.30   # 低置信度(1): 宽松阈值，避免噪声误判
        dx = abs(pos[0] - target[0])
        dy = abs(pos[1] - target[1])
        dz = abs(pos[2] - target[2])

        if self.target_index != self.last_target_index:
            self.last_target_index = self.target_index
            self.arrival_confirm_count = 0
            self.arrival_start_time = time.time()

        if dx > 0.3:
            self.x_pid.reset()
        if dy > 0.3:
            self.y_pid.reset()

        xy_ok = dx < xy_thresh and dy < xy_thresh
        z_ok = dz < posthreshold_z

        if xy_ok and z_ok:
            self.arrival_confirm_count += 1
            if self.arrival_confirm_count >= arrival_confirm_need:
                logger.info(f"到达航点 {self.target_index}")
                self._on_arrival(target)
        else:
            self.arrival_confirm_count = 0

        if time.time() - self.arrival_start_time >= arrival_timeout_max:
            logger.warning(f"航点 {self.target_index} 超时，强制跳过")
            self.target_index += 1

        # T265 速度（日志和终端输出共用，避免重复取值）
        if self.t265_ok:
            t265v = self.realsense.get_velocity()
        else:
            t265v = (0.0, 0.0, 0.0)

        # 光流融合速度（帧1 of1_dx/dy，用于跟 T265 速度交叉对比）
        with lock:
            of1_dx = self.re_fc[9] if len(self.re_fc) > 9 else 0
            of1_dy = self.re_fc[10] if len(self.re_fc) > 10 else 0

        # === 飞行数据日志（每秒记录一次） ===
        now = time.time()
        if now - self._last_log_time >= FLIGHT_LOG_INTERVAL and self._log_file is not None:
            try:
                record = {
                    "t": round(now, 3),
                    "state": self.state,
                    "target_idx": self.target_index,
                    "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                    "target": [round(target[0], 4), round(target[1], 4), round(target[2], 4)],
                    "vx": vx, "vy": vy, "vyaw": vyaw,
                    "t265_vel": [round(t265v[0], 4), round(t265v[1], 4)],
                    "of1_vel_cms": [of1_dx, of1_dy],
                }
                self._log_file.write(json.dumps(record) + "\n")
                self._log_file.flush()
            except Exception:
                pass
            self._last_log_time = now

        if self.t265_ok:
            t265_str = f"| t265v=({t265v[0]:+.2f},{t265v[1]:+.2f}) | of1=({of1_dx:+d},{of1_dy:+d})"
        else:
            t265_str = ""

        print(
            f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
            f"| tgt=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) "
            f"| v=({vx:>3},{vy:>3}) "
            f"| send=({self.se_fc[3]:>3},{self.se_fc[4]:>3},{self.se_fc[5]:>3})"
            f"{t265_str}",
            end="",
            flush=True
        )

    # ================= 到达处理 =================
    def _on_arrival(self, target):
        """到达航点后直接前往下一航点（检测已关闭）"""
        if self.target_index == len(self.targets) - 2:
            rgb_led('R', 1)
        self.target_index += 1

    # ================= 降落 =================
    def land(self):
        logger.info("降落")

        with lock:
            self.se_fc[2] = 0

        self.state = "END"

    # ================= 停止 =================
    def stop_all(self):
        logger.info("任务结束")

        try:
            self._log_file.close()
        except Exception:
            pass

        with lock:
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[6] = sp_side
            self.se_fc[7] = 101

        self.realsense.stop()
        if self.k230:
            self.k230.close()
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
        """Step the ramped Z command one frame toward target_z_cm (no jumps)."""
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
