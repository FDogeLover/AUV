import threading
import time
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock
from Lcode.coverage_planner import CoveragePlanner
from t265 import t265_class

put_height = 100
fly_height = 100
posthreshold = 0.03  # 米

realsense = t265_class()


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int], re_dmz: List[int], se_dmz: List[int]):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.re_dmz = re_dmz
        self.se_dmz = se_dmz

        # 状态机
        self.state = "IDLE"

        # 控制
        self.task_running = False
        self.t265_ok = False

        # PID控制器（常驻）
        self.x_pid = PID(0, 0)
        self.y_pid = PID(0, 0)
        self.yaw_pid = PID(1, 0)

        # 当前目标
        self.current_target = None

        # 航点
        # self.targets = self.calculate_waypoints_fromdmz()
        self.targets = self.load_waypoints()
        
        self.target_index = 0
        self.emergency_stop = False
        self.detect_flag = False

    def calculate_waypoints_fromdmz(self):
        """从地面站反传信息计算航点并加载"""
        planner = CoveragePlanner(self.re_dmz)
        planner.plan()
        planner.save_router_txt("router.txt")
        return self.load_waypoints()
        
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

        # 默认航点
        default_waypoints = [[0.5, 0.0,put_height/100], [0.5, 0.5,put_height/100], [0.0, 0.5,put_height/100], [0.0, 0.0,put_height/100]]
        logger.info(f"使用默认航点: {default_waypoints}")
        return default_waypoints

    # ================= 启动 =================
    def start(self):

        if realsense.start():
            realsense.autoset()
            self.t265_ok = True
            logger.info("T265 OK")
        else:
            logger.error("T265 FAILED")

        self.task_running = True
        self.state = "TAKEOFF"

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
    def loop(self):
        while self.task_running:

            if self.emergency_stop:
                self.stop_all()
                continue

            # 获取定位
            try:
                pos = realsense.get_position()
                yaw = realsense.get_orientation()[2]
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
        self.se_fc[1] = 1
        self.se_fc[4] = fly_height
        lock.release()

        time.sleep(2)
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
        self.yaw_pid.set_target(0)
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
        
        print(
            f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
            f"| tgt=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) "
            f"| v=({vx:>3},{vy:>3}) "
            f"| send=({self.se_fc[2]:>3},{self.se_fc[3]:>3},{self.se_fc[4]:>3})",
            end="",
            flush=True
        )
        if dx < posthreshold and dy < posthreshold and dz < posthreshold:
            logger.info(f"到达航点 {self.target_index}")
            self.target_index += 1
    
    # ================= 降落 =================
    def land(self):
        logger.info("降落")

        lock.acquire()
        self.se_fc[1] = 0
        lock.release()

        self.state = "END"

    # ================= 停止 =================
    def stop_all(self):
        logger.info("任务结束")

        lock.acquire()
        self.se_fc[2] = sp_side
        self.se_fc[3] = sp_side
        self.se_fc[5] = sp_side
        self.se_fc[6] = 101
        lock.release()

        realsense.stop()
        self.task_running = False

    # ================= 控制接口 =================
    def set_speed(self, x, y, yaw, z):
        lock.acquire()
        self.se_fc[2] = x + sp_side
        self.se_fc[3] = y + sp_side
        self.se_fc[4] = z
        self.se_fc[5] = yaw + sp_side
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

        
