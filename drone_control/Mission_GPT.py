import threading
import time
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock
from t265 import t265_class
import math

put_height = 100
fly_height = 100
posthreshold_xy = 0.15  # XY 到达阈值（米）
posthreshold_z = 0.20   # Z 到达阈值（米）
arrival_confirm_need = 5  # XY 连续确认到达次数
arrival_timeout_max = 10.0  # 单航点超时（秒）

realsense = t265_class()


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int], re_dmz: List[int], se_dmz: List[int], realsense_obj=None):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.re_dmz = re_dmz
        self.se_dmz = se_dmz

        # 状态机
        self.state = "IDLE"

        # 控制
        self.task_running = False
        self.t265_ok = False
        self.realsense = realsense_obj or realsense

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
        self.detect_flag = False

        # 到达判断状态（进入航点时自动重置）
        self.arrival_confirm_count = 0
        self.arrival_start_time = 0.0
        self.last_target_index = -1

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

    # ================= 启动 =================
    def start(self):

        if self.realsense.start():
            self.realsense.autoset()
            self.t265_ok = True
            logger.info("T265 OK")
        else:
            logger.error("T265 FAILED")

        self.task_running = True
        # self.state = "TAKEOFF"

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
    def loop(self):
        while self.task_running:

            if self.emergency_stop:
                self.stop_all()
                continue

            # 获取定位
            try:
                pos = self.realsense.get_position()
                yaw = self.realsense.get_orientation()[2]
            except Exception:
                logger.error("T265 ERROR")
                continue

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
        logger.info("起飞")

        lock.acquire()
        self.se_fc[2] = 1  # 触发飞控任务（解锁→模式切换→进入指令接收）
        lock.release()

        time.sleep(1)  # 等待飞控完成 unlock + mode switch
        self.state = "NAVIGATE"
    
    def navigate(self, pos, yaw):

        if self.target_index >= len(self.targets):
            logger.info("所有航点完成")
            self.state = "LAND"
            return

        target = self.targets[self.target_index]

        # Z轴: 直接传航点高度（米→厘米），FC自主控高，避免双级位置环震荡
        target_z = int(target[2] * 100)

        # XY/Yaw: PID计算速度
        self.x_pid.set_target(target[0])
        self.y_pid.set_target(target[1])
        # Yaw target: bearing to next waypoint
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        yaw_target = math.atan2(dy, dx)
        self.yaw_pid.set_target(yaw_target)
        vx = self.x_pid.get_pid(pos[0])
        vy = self.y_pid.get_pid(pos[1])
        vyaw = self.yaw_pid.get_pid(yaw)

        # === 转 cm ===
        vx *= 100
        vy *= 100

        # === 限幅 ===
        vx = int(self.limit(vx, 40))
        vy = int(self.limit(vy, 40))
        vyaw = int(self.limit(vyaw, 30)) 
        # === 发送 ===
        self.set_speed(vx, vy, -vyaw, target_z)

        # === 到达判断（米）===
        dx = abs(pos[0] - target[0])
        dy = abs(pos[1] - target[1])
        dz = abs(pos[2] - target[2])

        # 航点切换时重置所有状态（计数器 + PID 积分）
        if self.target_index != self.last_target_index:
            self.last_target_index = self.target_index
            self.arrival_confirm_count = 0
            self.arrival_start_time = time.time()
            self.x_pid.reset()
            self.y_pid.reset()
            self.yaw_pid.reset()

        # 远距离时清零积分防 windup（近距才启用 I 项消除静差）
        if dx > 0.3:
            self.x_pid.reset()
        if dy > 0.3:
            self.y_pid.reset()

        # XY 到达条件（连续确认）
        xy_ok = dx < posthreshold_xy and dy < posthreshold_xy
        # Z 到达条件（独立放松阈值，单次判定）
        z_ok = dz < posthreshold_z

        if xy_ok and z_ok:
            self.arrival_confirm_count += 1
            if self.arrival_confirm_count >= arrival_confirm_need:
                logger.info(f"到达航点 {self.target_index}")
                self.target_index += 1
        else:
            # 任一轴漂出阈值则重置确认计数
            self.arrival_confirm_count = 0

        # 超时强制跳过
        if time.time() - self.arrival_start_time >= arrival_timeout_max:
            logger.warning(f"航点 {self.target_index} 超时，强制跳过")
            self.target_index += 1

        # 获取 T265 实时速度用于打印
        if self.t265_ok:
            t265v = self.realsense.get_velocity()
            t265_str = f"| t265v=({t265v[0]:+.2f},{t265v[1]:+.2f})"
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
    
    # ================= 降落 =================
    def land(self):
        logger.info("降落")

        lock.acquire()
        self.se_fc[2] = 0
        lock.release()

        self.state = "END"

    # ================= 停止 =================
    def stop_all(self):
        logger.info("任务结束")

        lock.acquire()
        self.se_fc[3] = sp_side
        self.se_fc[4] = sp_side
        self.se_fc[6] = sp_side
        self.se_fc[7] = 101
        lock.release()

        self.realsense.stop()
        self.task_running = False

    # ================= 控制接口 =================
    def set_speed(self, x, y, yaw, z):
        lock.acquire()
        self.se_fc[3] = x + sp_side
        self.se_fc[4] = y + sp_side
        self.se_fc[5] = z
        self.se_fc[6] = yaw + sp_side
        lock.release()
        
    # ================= 工具 =================
    def limit(self, v, max_v=0.3):
        return max(min(v, max_v), -max_v)

    # ================= 急停 =================
    def emergency(self):
        logger.warning("紧急停止触发！")
        self.emergency_stop = True

    def detect_loop(self):
        """检测循环"""
        if self.detect_flag:
            return True
        else:
            return False

        
