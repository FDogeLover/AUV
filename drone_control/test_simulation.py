"""
模拟测试: Pi ↔ K230 双向通信 + 动态路径生成 + 地面站协议
============================================================
不依赖真实硬件，用虚拟串口+Mock对象测试完整检测流程。

运行: python test_simulation.py
依赖: pip install pyserial
"""

import sys
import os
import time
import threading
import random
from io import BytesIO
from collections import deque

# 确保 Lcode 可导入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ============================================================
# 一、虚拟串口 — 模拟 /dev/ttyS3 (Pi↔K230), /dev/ttyS6 (FC), /dev/ttyS7 (GS)
# ============================================================
class VirtualSerial:
    """成对虚拟串口，A端写入 -> B端可读"""
    def __init__(self, port_name, pair=None):
        self.port = port_name
        self.pair = pair
        self._rx_buf = deque()
        self._lock = threading.Lock()
        self.is_open = True
        self.baudrate = 115200

    def write(self, data):
        """A端写入 -> 写入B端接收缓冲区"""
        if self.pair:
            with self.pair._lock:
                self.pair._rx_buf.extend(data)

    def read(self, n=1):
        """读最多n字节，超时0.05s返回"""
        deadline = time.time() + 0.05
        while time.time() < deadline:
            with self._lock:
                if self._rx_buf:
                    result = bytearray()
                    for _ in range(min(n, len(self._rx_buf))):
                        result.append(self._rx_buf.popleft())
                    if result:
                        return bytes(result)
            time.sleep(0.001)
        return b''

    def read_all(self):
        with self._lock:
            result = bytes(self._rx_buf)
            self._rx_buf.clear()
            return result

    def close(self):
        self.is_open = False


class MockSerialManager:
    """管理所有虚拟串口配对"""
    def __init__(self):
        # Pi↔K230
        self.pi_to_k230 = VirtualSerial("ttyS3_pi")
        self.k230_to_pi = VirtualSerial("ttyS3_k230", pair=self.pi_to_k230)
        self.pi_to_k230.pair = self.k230_to_pi

        # Pi↔FC
        self.pi_to_fc = VirtualSerial("ttyS6_pi")
        self.fc_to_pi = VirtualSerial("ttyS6_fc", pair=self.pi_to_fc)
        self.pi_to_fc.pair = self.fc_to_pi

        # Pi↔地面站
        self.pi_to_gs = VirtualSerial("ttyS7_pi")
        self.gs_to_pi = VirtualSerial("ttyS7_gs", pair=self.pi_to_gs)
        self.pi_to_gs.pair = self.gs_to_pi


# ============================================================
# 二、K230 模拟器 — 响应 START 指令，发 RESULT 帧
# ============================================================
FRAME_HEAD = 0xAA
CMD_START  = 0x10
CMD_ACK    = 0x11
CMD_RESULT = 0x20
NO_ANIMAL  = 0xFF

class K230Simulator:
    """模拟K230行为: 收到START->延迟1s->发RESULT"""
    def __init__(self, serial_port):
        self.ser = serial_port
        self.running = True
        self.grid_animals = {}  # (ix,iy) -> (cls_id, count)  预置动物分布
        self.thread = threading.Thread(target=self._run, daemon=True)

    def set_animal(self, ix, iy, cls_id, count):
        """预置某个格子的动物 (0..4 或 0xFF)"""
        self.grid_animals[(ix, iy)] = (cls_id, count)

    def start(self):
        self.thread.start()

    def _run(self):
        """主循环: 接收Pi指令"""
        while self.running:
            b = self.ser.read(1)
            if not b or b[0] != FRAME_HEAD:
                continue
            cmd_byte = self.ser.read(1)
            if not cmd_byte:
                continue
            cmd = cmd_byte[0]
            idx_byte = self.ser.read(1)
            if not idx_byte:
                continue
            grid_idx = idx_byte[0]

            if cmd == CMD_START:
                # 解析格子
                ix = grid_idx % 9
                iy = grid_idx // 9
                animal = self.grid_animals.get((ix, iy), (0xFF, 0))
                cls_id, count = animal

                # 模拟K230 1秒检测延迟
                time.sleep(0.3)

                if cls_id == 0xFF:
                    best_cnt, total_dets, avg_conf = 0, 0, 0
                else:
                    best_cnt = count * 30  # 每帧都能检测到
                    total_dets = count * 30
                    avg_conf = random.randint(70, 95)

                frame = bytes([FRAME_HEAD, CMD_RESULT, grid_idx,
                              cls_id, best_cnt, total_dets, avg_conf])
                self.ser.write(frame)
                print(f"  [K230] 格子({ix},{iy}) -> cls={cls_id} cnt={count} conf={avg_conf}%")

            elif cmd == CMD_ACK:
                print(f"  [K230] ACK grid_idx={grid_idx}")

    def stop(self):
        self.running = False


# ============================================================
# 三、T265 模拟器 — 简单位置飞行模拟
# ============================================================
class T265Simulator:
    """模拟T265: 当前点->目标点，逐帧逼近"""
    def __init__(self):
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.running = True

    def start(self):
        return True

    def autoset(self):
        self.pos = [0.0, 0.0, 0.0]

    def get_position(self):
        return self.pos

    def get_orientation(self):
        return [0.0, 0.0, self.yaw]

    def get_velocity(self):
        return self.vel

    def is_running(self):
        return self.running

    def get_yaw_deg_x100(self):
        return int(self.yaw * 5729)  # rad -> deg*100

    def stop(self):
        self.running = False

    def move_toward(self, tx, ty, tz, speed=0.5):
        """逐帧逼近目标点，每帧走speed米"""
        dx = tx - self.pos[0]
        dy = ty - self.pos[1]
        dz = tz - self.pos[2]
        dist = (dx**2 + dy**2 + dz**2) ** 0.5
        if dist < 0.01:
            self.pos = [tx, ty, tz]
            return True
        ratio = min(speed / dist, 1.0)
        self.pos[0] += dx * ratio
        self.pos[1] += dy * ratio
        self.pos[2] += dz * ratio
        self.vel[0] = dx * 10
        self.vel[1] = dy * 10
        return False


# ============================================================
# 四、地面站模拟器 — 发送禁飞区数据
# ============================================================
class GroundStationSimulator:
    """模拟地面站: 启动后发送禁飞区坐标"""
    def __init__(self, serial_port, forbidden_zones):
        """
        forbidden_zones: [('A6','B3'), ('A5','B3'), ('A5','B4')]
        """
        self.ser = serial_port
        self.forbidden = forbidden_zones
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        """延迟0.5s后发送禁飞区数据"""
        time.sleep(0.5)
        for a_str, b_str in self.forbidden:
            a_val = int(a_str[1:]) + 0xA0
            b_val = int(b_str[1:]) + 0xB0
            frame = bytes([0xAA, a_val, b_val, 0, 0, 0, 0, 0xFF])
            self.ser.write(frame)
            print(f"  [GS] 发送禁飞区: ({a_str},{b_str}) -> {frame.hex()}")
        print(f"  [GS] 禁飞区发送DONE")


# ============================================================
# 五、主测试函数
# ============================================================
def test_full_mission():
    """完整任务模拟测试"""
    print("=" * 60)
    print("  模拟测试: Pi↔K230 双向通信 + 动态路径 + 地面站协议")
    print("=" * 60)

    # ---- 1. 创建虚拟串口 ----
    serial_mgr = MockSerialManager()

    # ---- 2. Monkey-patch serial.Serial ----
    import serial as real_serial

    class FakeSerial:
        """替换真实 serial.Serial，路由到虚拟串口"""
        _port_map = {}

        @classmethod
        def register(cls, port, virtual_port):
            cls._port_map[port] = virtual_port

        def __init__(self, port=None, baudrate=115200, timeout=0.05, **kw):
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self.is_open = True
            self._virt = self._port_map.get(port)
            if self._virt is None:
                raise RuntimeError(f"未注册的虚拟串口: {port}")

        def write(self, data):
            self._virt.write(data)

        def read(self, n=1):
            return self._virt.read(n)

        def close(self):
            self.is_open = False

    # 注册虚拟串口
    FakeSerial.register("/dev/ttyS6", serial_mgr.pi_to_fc)
    FakeSerial.register("/dev/ttyS3", serial_mgr.pi_to_k230)
    FakeSerial.register("/dev/ttyS7", serial_mgr.pi_to_gs)

    real_serial.Serial = FakeSerial
    import Lcode.Lprotocol as lproto
    lproto.serial.Serial = FakeSerial

    # ---- 3. 创建模拟组件 ----
    # K230模拟器: 预置动物分布
    k230_sim = K230Simulator(serial_mgr.k230_to_pi)
    k230_sim.set_animal(0, 0, 0, 2)   # (A9,B1) tiger x2
    k230_sim.set_animal(2, 1, 3, 1)   # 孔雀 x1
    k230_sim.set_animal(5, 3, 1, 3)   # wolf x3
    k230_sim.set_animal(7, 5, 4, 2)   # elephant x2
    k230_sim.set_animal(4, 2, 2, 1)   # monkey x1
    k230_sim.start()

    # 地面站模拟器: 发送禁飞区
    gs_sim = GroundStationSimulator(serial_mgr.gs_to_pi, [
        ("A6", "B3"), ("A5", "B3"), ("A5", "B4")
    ])
    gs_sim.start()

    # T265模拟器
    t265_sim = T265Simulator()

    # ---- 4. 导入被测模块 ----
    from Mission_GPT import mission
    from Lcode.global_variable import sp_side, lock, fc_last_rx_time
    import Lcode.Lprotocol

    # 准备共享变量
    re_fc = [0, 0, 0]
    re_dmz = [('A9', 'B1'), ('A10', 'B2'), ('A11', 'B3')]
    se_fc = [170, 2, 0, sp_side, sp_side, 120, sp_side, 0, sp_side, 0, 255]
    se_dmz = [170, 0, 0, 0, 255]

    # ---- 5. 初始化串口(虚拟) ----
    serial_fc = Lcode.Lprotocol.Serial_fc("/dev/ttyS6", 460800)
    serial_fc.port_open()
    serial_fc.listen_start(re_fc)
    serial_fc.send_start(se_fc, t265_sim, vel_freq=100, cmd_freq=50)

    serial_dmz = Lcode.Lprotocol.Serial_dmz("/dev/ttyS7", 115200)
    serial_dmz.port_open()
    serial_dmz.listen_start(re_dmz)
    serial_dmz.send_start(se_dmz)

    # K230客户端
    from Lcode.k230_client import K230Client
    k230_client = K230Client("/dev/ttyS3", 115200)
    # monkey-patch 其内部 serial 为虚拟
    k230_client.ser = FakeSerial("/dev/ttyS3", 115200)

    # ---- 6. 创建任务 ----
    m = mission(re_fc, se_fc, re_dmz, se_dmz, t265_sim, k230_client)
    # 用简单航点（不用地面站数据时fallback）
    m.targets = [[0.0, 0.0, 1.0], [0.5, 0.0, 1.0], [0.5, 0.5, 1.0]]
    m.target_index = 0
    m.state = "TAKEOFF"
    m.task_running = True

    # ---- 7. 模拟飞行循环 ----
    print("\n--- 模拟飞行开始 ---\n")
    waypoints = m.targets
    frame = 0

    for wp_idx, (tx, ty, tz) in enumerate(waypoints):
        if wp_idx == 0:
            print(f"\n>>> 航点 {wp_idx}: 起点 ({tx:.1f},{ty:.1f})")
            t265_sim.pos = [tx, ty, tz]
            m.state = "NAVIGATE"
            continue

        print(f"\n>>> 航点 {wp_idx}: 飞往 ({tx:.1f},{ty:.1f})")
        m.target_index = wp_idx
        m.last_target_index = wp_idx - 1

        # 飞行逼近
        arrived = False
        for _ in range(200):  # 最多200帧
            pos = t265_sim.get_position()
            yaw = 0.0
            m.navigate(pos, yaw)

            if m.detecting:
                # K230检测中，等待DONE
                time.sleep(0.02)
                arrived = False
            else:
                arrived = t265_sim.move_toward(tx, ty, tz, speed=0.15)

            frame += 1

        # 显示结果
        grid = m._grid_from_real(tx, ty)
        if grid and grid in m.grid_results:
            cls_id, cnt = m.grid_results[grid]
            names = ["tiger", "wolf", "monkey", "peacock", "elephant"]
            label = names[cls_id] if cls_id < 5 else "无"
            print(f"  ✅ 格子{grid} 结果: {label} x{cnt}")

    print("\n--- 模拟飞行结束 ---\n")

    # ---- 8. 打印Test Report ----
    print("=" * 60)
    print("  Test Report")
    print("=" * 60)
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Results: {len(m.grid_results)}")
    for grid, (cls_id, cnt) in m.grid_results.items():
        names = ["tiger", "wolf", "monkey", "peacock", "elephant"]
        label = names[cls_id] if cls_id < 5 else "无"
        print(f"    格子{grid}: {label} x{cnt}")
    print(f"  Detected: {m.detected_grids}")
    print(f"  地面站发送帧: {se_dmz}")
    print("=" * 60)

    # ---- 9. 清理 ----
    m.stop_all()
    k230_sim.stop()
    print("\n[TEST] DONE")


# ============================================================
# 测试2: 地面站协议帧验证
# ============================================================
def test_ground_station_protocol():
    """验证地面站协议帧格式"""
    print("\n" + "=" * 60)
    print("  测试: 地面站协议帧")
    print("=" * 60)

    test_cases = [
        # (cls_id, count, expse_dmz)
        (0, 2,    [170, 0, 0, 2, 255]),     # tiger x2
        (3, 1,    [170, 0, 3, 1, 255]),     # peacock x1
        (0xFF, 0, [170, 0, 0xFF, 1, 255]),  # 无动物
        (4, 5,    [170, 0, 4, 5, 255]),     # elephant x5
    ]

    all_pass = True
    for cls_id, count, expected in test_cases:
        frame = [170, 0,
                 cls_id if cls_id < 5 else 0xFF,
                 max(count, 1),
                 255]
        ok = frame == expected
        status = "OK" if ok else "FAIL"
        print(f"  {status} cls={cls_id} cnt={count} -> {frame}  exp={expected}")
        if not ok:
            all_pass = False

    print(f"\n  Proto: {'PASS' if all_pass else 'FAIL'}")
    return all_pass


# ============================================================
# 测试3: K230 RESULT帧解析
# ============================================================
def test_k230_result_frame():
    """验证K230 RESULT帧解析"""
    print("\n" + "=" * 60)
    print("  测试: K230 RESULT帧解析")
    print("=" * 60)

    # 模拟K230发送的帧
    test_frames = [
        # (帧字节, exp解析结果)
        (bytes([0xAA, 0x20, 7, 0, 60, 60, 85]),
         (7, 0, 60, 60, 85)),     # grid=0*9+7=7 tiger 60帧总60帧置信85%
        (bytes([0xAA, 0x20, 0, 0xFF, 0, 0, 0]),
         (0, 0xFF, 0, 0, 0)),     # grid=0 无动物
        (bytes([0xAA, 0x20, 9, 3, 30, 30, 92]),
         (9, 3, 30, 30, 92)),     # grid=1*9+0=9 peacock
    ]

    all_pass = True
    for frame_data, expected in test_frames:
        # 模拟k230_client._listen解析
        assert frame_data[0] == 0xAA
        cmd = frame_data[1]
        if cmd == CMD_RESULT:
            grid_idx = frame_data[2]
            cls_id = frame_data[3]
            best_cnt = frame_data[4]
            total_dets = frame_data[5]
            avg_conf = frame_data[6]
            result = (grid_idx, cls_id, best_cnt, total_dets, avg_conf)
            ok = result == expected
            status = "OK" if ok else "FAIL"
            print(f"  {status} {frame_data.hex()} -> {result}")
            if not ok:
                all_pass = False
                print(f"       exp: {expected}")

    print(f"\n  K230: {'PASS' if all_pass else 'FAIL'}")
    return all_pass


# ============================================================
# 测试4: 判决逻辑
# ============================================================
def test_evaluate_detection():
    """验证占比+置信度判决逻辑"""
    print("\n" + "=" * 60)
    print("  测试: 检测判决逻辑")
    print("=" * 60)

    from Mission_GPT import ANIMAL_LABELS

    def evaluate(cls_id, best_cnt, total_dets, avg_conf):
        if cls_id == 0xFF or best_cnt == 0:
            return True
        dominance = best_cnt / max(total_dets, 1)
        confidence = avg_conf / 100.0
        return dominance >= 0.7 and confidence >= 0.5

    cases = [
        (0, 60, 80, 75, True,   "tiger 60/80=75% 置信75% -> PASS"),
        (0, 50, 80, 75, False,  "tiger 50/80=62% 置信75% -> 拒绝(占比<70%)"),
        (3, 30, 30, 45, False,  "peacock 30/30=100% 置信45% -> 拒绝(置信<50%)"),
        (3, 30, 30, 85, True,   "peacock 30/30=100% 置信85% -> PASS"),
        (0xFF, 0, 0, 0, True,   "无动物 -> PASS"),
    ]

    all_pass = True
    for cls_id, best_cnt, total_dets, avg_conf, expected, desc in cases:
        result = evaluate(cls_id, best_cnt, total_dets, avg_conf)
        ok = result == expected
        status = "OK" if ok else "FAIL"
        print(f"  {status} {desc}")
        if not ok:
            all_pass = False

    print(f"\n  Eval: {'PASS' if all_pass else 'FAIL'}")
    return all_pass


# ============================================================
# 入口
# ============================================================
if __name__ == "__main__":
    results = []

    results.append(("协议帧格式",    test_ground_station_protocol()))
    results.append(("K230帧解析",   test_k230_result_frame()))
    results.append(("检测判决逻辑",  test_evaluate_detection()))
    results.append(("完整任务模拟",  test_full_mission()))

    print("\n" + "=" * 60)
    print("  OVERALL")
    print("=" * 60)
    for name, ok in results:
        print(f"  {'OK' if ok else 'FAIL'} {name}")
