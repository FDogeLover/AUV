"""任务一固定路径专项实飞入口（不使用视觉、不等待小车通信、不投放）。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import replace
from pathlib import Path

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.global_variable import sp_side  # noqa: E402
from main import wait_for_start_button  # noqa: E402
from t265 import t265_class  # noqa: E402

from .payload_servo import build_payload_actuator  # noqa: E402
from .task1_flight import Task1FlightMission  # noqa: E402
from .task1_mission import Task1Config  # noqa: E402


class NullVisionReader:
    def latest(self, _now=None, _max_age_s=0.15):
        return None


def _load_task_config(data: dict) -> Task1Config:
    task = data["task1"]
    return Task1Config(
        cruise_height_m=float(task["cruise_height_m"]),
        follow_height_m=float(task["follow_height_m"]),
        drop_height_m=float(task["drop_height_m"]),
        final_height_m=float(task["final_height_m"]),
        hold_duration_s=float(task["hover_duration_s"]),
        intercept_speed_m_s=float(task["intercept_speed_m_s"]),
        car_speed_m_s=float(task["car_speed_m_s"]),
        car_speed_scale=float(task.get("car_speed_scale", 1.0)),
        return_speed_m_s=float(task["return_speed_m_s"]),
        point_arrival_radius_m=float(task["path_capture_radius_m"]),
        hold_position_max_speed_m_s=float(
            task.get("hold_position_max_speed_m_s", 0.15)
        ),
        hold_stable_speed_m_s=float(
            task.get("hold_stable_speed_m_s", 0.12)
        ),
        path_lookahead_m=float(task.get("path_lookahead_m", 0.20)),
        path_cross_track_kp=float(task.get("path_cross_track_kp", 0.80)),
        path_max_correction_m_s=float(
            task.get("path_max_correction_m_s", 0.08)
        ),
        path_max_speed_m_s=float(task.get("path_max_speed_m_s", 0.20)),
        path_max_accel_m_s2=float(
            task.get("path_max_accel_m_s2", 0.30)
        ),
        vision_min_quality=int(task["vision_min_quality"]),
        vision_confirm_frames=int(task["vision_confirm_frames"]),
        drop_max_error_m=float(task["drop_max_error_m"]),
        drop_descent_speed_m_s=float(task["drop_descent_speed_m_s"]),
        drop_time_margin_s=float(task["drop_time_margin_s"]),
        release_timeout_s=float(task["release_timeout_s"]),
    )


def _stop_mission_safely(mission_obj, timeout_s: float = 0.8) -> None:
    if mission_obj is None or not mission_obj.task_running:
        return
    mission_obj.emergency()
    deadline = time.monotonic() + timeout_s
    while mission_obj.task_running and time.monotonic() < deadline:
        time.sleep(0.03)
    if mission_obj.task_running:
        mission_obj.stop_all()


def build_path_test_config(
    base,
    *,
    height_m: float,
    path_speed_m_s: float,
    intercept_speed_m_s: float,
    return_speed_m_s: float,
):
    """保持正式路径算法，只关闭视觉等待、下降与投放。"""
    return replace(
        base,
        cruise_height_m=height_m,
        follow_height_m=height_m,
        car_speed_m_s=path_speed_m_s,
        car_speed_scale=1.0,
        intercept_speed_m_s=intercept_speed_m_s,
        return_speed_m_s=return_speed_m_s,
        acquire_timeout_s=0.0,
    )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    parser.add_argument("--height", type=float, default=1.5)
    parser.add_argument("--path-speed", type=float, default=0.13)
    parser.add_argument("--intercept-speed", type=float, default=0.38)
    parser.add_argument("--return-speed", type=float, default=0.35)
    parser.add_argument(
        "--height-source",
        choices=("laser", "t265"),
        default="laser",
    )
    args = parser.parse_args(argv)

    if not 1.0 <= args.height <= 1.6:
        parser.error("--height建议范围为1.0~1.6 m")
    if not 0.05 <= args.path_speed <= 0.20:
        parser.error("--path-speed安全范围为0.05~0.20 m/s")
    if not 0.10 <= args.intercept_speed <= 0.40:
        parser.error("--intercept-speed安全范围为0.10~0.40 m/s")
    if not 0.10 <= args.return_speed <= 0.40:
        parser.error("--return-speed安全范围为0.10~0.40 m/s")

    data = json.loads(args.config.read_text(encoding="utf-8"))
    task_config = build_path_test_config(
        _load_task_config(data),
        height_m=args.height,
        path_speed_m_s=args.path_speed,
        intercept_speed_m_s=args.intercept_speed,
        return_speed_m_s=args.return_speed,
    )
    payload_cfg = data.get("payload_servo", {})
    actuator, payload_hardware = build_payload_actuator(
        release_hold_s=float(payload_cfg.get("release_hold_s", 1.0))
    )
    if not payload_hardware.ready:
        raise RuntimeError("舵机未能锁定到180°，禁止路径测试")

    serial_fc = None
    mission_obj = None
    try:
        logger.warning(
            "固定路径专项：不启用视觉、不等待小车通信、不执行投放；"
            f"高度={args.height:.2f}m，路径速度={args.path_speed:.2f}m/s"
        )
        if not wait_for_start_button():
            logger.error("本地按键门禁失败，程序退出")
            return

        realsense = t265_class()
        re_fc = [0] * 14
        se_fc = [
            170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255
        ]
        port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(
            se_fc, realsense, vel_freq=100, cmd_freq=50
        )
        mission_obj = Task1FlightMission(
            re_fc,
            se_fc,
            realsense,
            serial_fc,
            vision_reader=NullVisionReader(),
            payload_actuator=actuator,
            task_config=task_config,
            car_speed_provider=None,
            height_source=args.height_source,
        )
        mission_obj.start()
        if not mission_obj.task_running:
            raise RuntimeError("T265或起飞预检失败，路径测试未启动")
        while mission_obj.task_running:
            time.sleep(0.10)
    except KeyboardInterrupt:
        logger.warning("用户中断路径测试")
        _stop_mission_safely(mission_obj)
    except Exception:
        _stop_mission_safely(mission_obj)
        raise
    finally:
        _stop_mission_safely(mission_obj)
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()


if __name__ == "__main__":
    main()
