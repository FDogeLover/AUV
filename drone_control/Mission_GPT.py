import threading
import time
from typing import List
from Lcode.Lpid import PID
from Lcode.Logger import logger
from Lcode.global_variable import sp_side, lock, fc_last_rx_time
from Lcode.k230_client import K230Client
from Lcode.coverage_planner import CoveragePlanner
from t265 import t265_class

put_height = 100
fly_height = 100
VEL_SCALE = 0.7  # XY/Yaw 速度缩放系数 (1.0=原速, 0.7=七成)
posthreshold_xy = 0.15  # XY 到达阈值（米）
posthreshold_z = 0.20   # Z 到达阈值（米）
arrival_confirm_need = 5  # XY 连续确认到达次数
arrival_timeout_max = 10.0  # 单航点超时（秒）
ANIMAL_LABELS = ["tiger", "wolf", "monkey", "peacock", "elephant"]

realsense = t265_class()


class mission:

    def __init__(self, re_fc: List[int], se_fc: List[int], re_dmz: List[int], se_dmz: List[int],
                 realsense_obj=None, k230_client=None):
        self.re_fc = re_fc
        self.se_fc = se_fc
        self.re_dmz = re_dmz
        self.se_dmz = se_dmz
        self.k230 = k230_client

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

        # 到达判断状态（进入航点时自动重置）
        self.arrival_confirm_count = 0
        self.arrival_start_time = 0.0
        self.last_target_index = -1

        # === K230 检测相关 ===
        self.detected_grids = set()          # 已检测的格子 (ix, iy)
        self.detecting = False               # 是否在检测阶段
        self.detect_start_time = 0.0         # 检测开始时刻
        self.detect_triggered = False        # 是否已发 START
        self.detect_result = None            # poll_result 缓存
        self.detect_grid = None              # 当前检测的 (ix, iy)
        self.detect_round = 1                # 轮次 1/2
        self.grid_results = {}               # (ix,iy) → cls_id

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
        else:
            logger.error("T265 FAILED")

        # === 动态路径生成（优先用地面站禁飞区，失败则保留 load_waypoints 结果） ===
        gs_ok = False
        logger.info("等待地面站禁飞区数据...")
        for _ in range(50):  # 5s 超时
            lock.acquire()
            gs_ok = self._gs_data_valid()
            lock.release()
            if gs_ok:
                break
            time.sleep(0.1)

        if gs_ok:
            lock.acquire()
            forbidden = list(self.re_dmz)
            lock.release()
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

        # 预标记起飞格子为已检测（起点不检测）
        pos = self.realsense.get_position()
        ix = int(round(-pos[0] * 2))
        iy = int(round( pos[1] * 2))
        self.detected_grids.add((ix, iy))
        logger.info(f"起飞格({ix},{iy}) 预标记已检测")

        threading.Thread(target=self.loop, daemon=True).start()

    # ================= 主循环 =================
    def loop(self):
        while self.task_running:

            if self.emergency_stop:
                self.stop_all()
                continue

            # 串口超时检测：超过2秒无飞控回传数据则急停
            if fc_last_rx_time > 0 and time.time() - fc_last_rx_time > 2.0:
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

        # Z轴: 直接传航点高度（米→厘米），FC自主控高
        target_z = int(target[2] * 100)

        # 每帧更新地面站进度序号（AA/FF/cls/cnt 由 _detect_accept 一次性写入后保持）
        lock.acquire()
        self.se_dmz[1] = self.target_index & 0xFF
        lock.release()

        # XY/Yaw: PID计算速度（检测期间也运行，维持悬停）
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
        self.set_speed(vx, vy, -vyaw, target_z)

        # === 检测阶段：推进状态机，跳过到达判断 ===
        if self.detecting:
            self._handle_detection()
            return

        # === 到达判断 ===
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

        xy_ok = dx < posthreshold_xy and dy < posthreshold_xy
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

    # ================= 到达处理 =================
    def _grid_from_real(self, rx, ry):
        """实际坐标 → 内部格子 (ix, iy)，超出9x7返回 None"""
        ix = int(round(-rx * 2))
        iy = int(round( ry * 2))
        if 0 <= ix < 9 and 0 <= iy < 7:
            return (ix, iy)
        return None

    def _on_arrival(self, target):
        """到达航点时判断：已检测→跳过，未检测→进入检测阶段"""
        grid = self._grid_from_real(target[0], target[1])
        if grid is None:
            logger.info("非格子航点，直接跳过")
            self.target_index += 1
            return

        if grid in self.detected_grids:
            logger.info(f"格子{grid} 已检测，飞越")
            self.target_index += 1
            return

        if self.k230 is None:
            logger.info(f"格子{grid} 无K230，标记跳过")
            self.detected_grids.add(grid)
            self.target_index += 1
            return

        logger.info(f"格子{grid} 开始检测")
        self.detecting = True
        self.detect_grid = grid
        self.detect_round = 1
        self.detect_start_time = time.time()
        self.detect_triggered = False
        self.detect_result = None

    # ================= 检测状态机 =================
    def _handle_detection(self):
        """每30Hz调用，推进检测流程"""
        elapsed = time.time() - self.detect_start_time

        # 阶段1: 等机身稳定0.5s → 发送 START
        if not self.detect_triggered:
            if elapsed > 0.5:
                gidx = self.detect_grid[1] * 9 + self.detect_grid[0]
                self.k230.send_start(gidx)
                self.detect_triggered = True
                self.detect_start_time = time.time()
            return

        # 阶段2: 等待 K230 结果（超时5s）
        if self.detect_result is None:
            if elapsed > 5.0:
                logger.warning(f"格子{self.detect_grid} K230超时，跳过")
                self._detect_accept(0xFF)  # 标记为无动物
                return
            result = self.k230.poll_result()
            if result:
                self.detect_result = result
            return

        # 阶段3: 判决
        _, cls_id, best_cnt, total_dets, avg_conf = self.detect_result
        ok = self._evaluate_detection(cls_id, best_cnt, total_dets, avg_conf)
        if ok:
            self._detect_accept(cls_id, best_cnt)
        elif self.detect_round == 1:
            logger.info(f"格子{self.detect_grid} 结果不稳，第2轮复测")
            self.detect_round = 2
            gidx = self.detect_grid[1] * 9 + self.detect_grid[0]
            self.k230.send_start(gidx)
            self.detect_triggered = True
            self.detect_result = None
            self.detect_start_time = time.time()
        else:
            logger.info(f"格子{self.detect_grid} 2轮仍不稳，强制接受")
            self._detect_accept(cls_id, best_cnt)

    def _evaluate_detection(self, cls_id, best_cnt, total_dets, avg_conf):
        """判决：占比≥70% 且 置信≥50% 则接受；无动物也接受"""
        if cls_id == 0xFF or best_cnt == 0:
            return True
        dominance = best_cnt / max(total_dets, 1)
        confidence = avg_conf / 100.0
        label = ANIMAL_LABELS[cls_id] if cls_id < 5 else "?"
        logger.info(f"  判决: {label} cnt={best_cnt}/{total_dets} "
                    f"占比={dominance:.0%} 置信={confidence:.0%}")
        return dominance >= 0.7 and confidence >= 0.5

    def _detect_accept(self, cls_id, best_cnt=0):
        """接受检测结果，标记格子，发ACK + 地面站结果，推进航点"""
        count = int(round(best_cnt / 30)) if cls_id != 0xFF else 0
        label = ANIMAL_LABELS[cls_id] if cls_id < 5 else "无"
        logger.info(f"格子{self.detect_grid} 确认: {label} x{count}")
        self.detected_grids.add(self.detect_grid)
        self.grid_results[self.detect_grid] = (cls_id, count)

        # 点亮地面站帧: AA idx cls cnt FF — 之后持续广播直到下个检测覆盖
        lock.acquire()
        self.se_dmz[0] = 0xAA
        self.se_dmz[1] = self.target_index & 0xFF
        self.se_dmz[2] = cls_id if cls_id < 5 else 0xFF
        self.se_dmz[3] = max(count, 1)
        self.se_dmz[4] = 0xFF
        lock.release()

        if self.k230:
            gidx = self.detect_grid[1] * 9 + self.detect_grid[0]
            self.k230.send_ack(gidx)
        self.detecting = False
        self.detect_triggered = False
        self.detect_result = None
        self.detect_grid = None
        self.detect_round = 1
        self.target_index += 1
    
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
        if self.k230:
            self.k230.close()
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
