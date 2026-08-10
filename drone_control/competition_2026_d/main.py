"""
2026 D题竞赛 — 主入口

运行:
  python main.py
  python main.py --config config.json
  python main.py --start-timeout 60

统一自启动流程:
  1. 白色预检灯 → 投放舵机锁定 → 蓝牙链路启动
  2. Cyber Camera 双向通信预检
  3. 飞控串口打开（仅发送指令帧，不创建 T265）
  4. 等待人工拔插 T265
  5. 广播 UAV_READY，等待小车 CAR_START（task_mode=1 或 2）
  6. ACK 确认后构造 T265 → 启动对应任务状态机
  7. 保持主线程存活，实时上报遥测
"""
import os
import sys

# ---------- Lcode 共享层（与 basic/ 同级引用方式一致） ----------
BASIC_DIR = os.path.join(os.path.dirname(__file__), os.pardir, "basic")
if os.path.abspath(BASIC_DIR) not in sys.path:
    sys.path.insert(0, os.path.abspath(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.global_variable import sp_side  # noqa: E402

# ---------- 模块核心组件（供测试和外部脚本导入） ----------
from .auto_start import main as auto_start_main  # noqa: E402
from .task1_flight import Task1FlightMission  # noqa: E402
from .task2_flight import Task2FlightMission  # noqa: E402
from .task1_mission import Task1Config  # noqa: E402
from .task2_mission import Task2Config  # noqa: E402
from .payload_servo import build_payload_actuator  # noqa: E402
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


def main(argv=None):
    """委托给 auto_start 统一入口。"""
    logger.info("=" * 40)
    logger.info("competition_2026_d — 2026 D题竞赛控制器")
    logger.info("=" * 40)
    auto_start_main(argv)


if __name__ == "__main__":
    main()
