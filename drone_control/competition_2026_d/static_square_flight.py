"""1.5m静态蓝方块XY闭环专项实飞入口。"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path

# basic仍按其原有方式导入顶层Lcode/t265；只在本专用入口补充搜索路径。
BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

# 必须在导入Mission_GPT（其模块常量在导入时读取环境变量）之前冻结末点行为。
os.environ["DRONE_ARRIVAL_HOLD_S"] = "0"
os.environ["DRONE_FINAL_WAYPOINT_HOLD_S"] = "0"

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.global_variable import sp_side  # noqa: E402
from Mission_GPT import mission  # noqa: E402
from main import wait_for_start_button  # noqa: E402
from t265 import t265_class  # noqa: E402

from .static_square_servo import StaticSquareServo, StaticSquareServoConfig  # noqa: E402
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


class ServoTelemetryLogger:
    """控制线程只入队；后台线程以最高10Hz批量写JSONL，不调用fsync。"""

    def __init__(self, path: Path, interval_s: float = 0.10) -> None:
        self.path = path
        self.interval_s = interval_s
        self._queue: queue.Queue = queue.Queue(maxsize=512)
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_submit = 0.0
        self._last_mode: str | None = None
        self.dropped = 0

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._running.set()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="servo-log")
        self._thread.start()

    def submit(self, now: float, snapshot) -> None:
        event = snapshot.mode != self._last_mode or snapshot.faulted or snapshot.finished
        if not event and now - self._last_submit < self.interval_s:
            return
        self._last_submit = now
        self._last_mode = snapshot.mode
        item = {"monotonic": round(now, 6), **asdict(snapshot)}
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _worker(self) -> None:
        with self.path.open("a", encoding="utf-8", buffering=8192) as handle:
            while self._running.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                handle.write(json.dumps(item, ensure_ascii=False) + "\n")


class StaticSquareMission(mission):
    def __init__(
        self, re_fc, se_fc, realsense_obj, serial_fc_ref, servo, telemetry,
        flight_config,
    ):
        super().__init__(
            re_fc, se_fc, realsense_obj, serial_fc_ref,
            horizontal_velocity_provider=servo,
        )
        self.servo = servo
        self.telemetry = telemetry
        self.flight_config = flight_config
        self.targets = [
            [0.0, 0.0, float(flight_config["test_height_m"])],
            [0.0, 0.0, float(flight_config["final_waypoint_height_m"])],
        ]
        self._visual_phase = "APPROACH_1P5M"
        self._test_waypoint_reached = False
        self._stable_since: float | None = None
        self._stable_wait_start: float | None = None

    def navigate(self, pos, yaw):
        now = time.monotonic()
        if self.target_index == 0 and not self.servo.finished:
            velocity = self.realsense.get_velocity() if self.realsense else (0.0, 0.0, 0.0)
            confidence = self.realsense.get_tracking_confidence() if self.realsense else 0
            test_height = float(self.flight_config["test_height_m"])
            height_tolerance = float(self.flight_config["stable_height_tolerance_m"])
            stable = (
                self._test_waypoint_reached
                and test_height - height_tolerance <= pos[2] <= test_height + height_tolerance
                and math.hypot(velocity[0], velocity[1])
                < float(self.flight_config["stable_horizontal_speed_m_s"])
                and confidence >= int(self.flight_config["stable_min_confidence"])
            )
            if stable:
                if self._stable_since is None:
                    self._stable_since = now
                    self._visual_phase = "STABILIZING"
                elif (
                    not self.servo.active
                    and now - self._stable_since
                    >= float(self.flight_config["stable_duration_s"])
                ):
                    self.servo.arm(now, (pos[0], pos[1]))
                    self._visual_phase = "VISUAL_SERVO"
                    logger.info("1.5m稳定1.0s，蓝方块XY视觉接管开始（最长15s）")
            else:
                self._stable_since = None
                if self._test_waypoint_reached:
                    self._visual_phase = "WAIT_STABLE"
            if (
                self._test_waypoint_reached
                and not self.servo.active
                and self._stable_wait_start is not None
                and now - self._stable_wait_start
                >= float(self.flight_config["stable_wait_timeout_s"])
            ):
                logger.error("1.5m稳定条件5秒内未满足，取消视觉接管并进入安全下降")
                self.servo.abort_before_arm("stabilization_timeout")

        if self.target_index == 0 and self.servo.finished:
            reason = "visual_fault" if self.servo.faulted else "visual_complete"
            logger.warning(f"视觉阶段结束: {self.servo.reason}; 交回T265并进入0.15m末航点")
            self.horizontal_velocity_provider = None
            self.x_pid.reset()
            self.y_pid.reset()
            self._visual_phase = "FAULT_LOCKED" if self.servo.faulted else "COMPLETE"
            target = self.targets[0]
            distance = math.hypot(pos[0] - target[0], pos[1] - target[1])
            super()._advance_waypoint(reason, pos, target, distance)

        if self.target_index == 0 and self._test_waypoint_reached:
            # 专用视觉阶段自己管理持续时间；阻止basic普通航点窗口每30ms重复
            # 触发“停留完成/超时”日志和推进请求。
            self._arrival_window.clear()
            self.arrival_confirmed_time = None
            self.arrival_start_time = time.time()
        super().navigate(pos, yaw)
        self.telemetry.submit(now, self.servo.snapshot())

    def _advance_waypoint(self, reason, pos, target, arrival_distance):
        if self.target_index == 0:
            self._test_waypoint_reached = True
            if self._stable_wait_start is None:
                self._stable_wait_start = time.monotonic()
            self.arrival_start_time = time.time()
            self.arrival_confirmed_time = None
            if self.servo.active:
                return
            if not self.servo.finished:
                self._visual_phase = "WAIT_STABLE"
                return
        super()._advance_waypoint(reason, pos, target, arrival_distance)


def _load_config(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    square = data["static_square_test"]
    cybercam = data["cybercam"]
    test_height = float(square["test_height_m"])
    height_tolerance = float(square["stable_height_tolerance_m"])
    config = StaticSquareServoConfig(
        image_cx_px=float(square["image_center_px"][0]),
        image_cy_px=float(square["image_center_px"][1]),
        focal_x_px=float(square["focal_px"][0]),
        focal_y_px=float(square["focal_px"][1]),
        min_quality_full=int(square["min_quality"]),
        min_quality_partial=int(square["partial_min_quality"]),
        max_observation_age_s=float(square["lost_timeout_s"]),
        confirm_frames=int(square["confirm_frames"]),
        full_max_speed_m_s=float(square["max_speed_m_s"]),
        partial_max_speed_m_s=float(square["partial_max_speed_m_s"]),
        max_accel_m_s2=float(square["max_accel_m_s2"]),
        max_jerk_m_s3=float(square["max_jerk_m_s3"]),
        deadband_m=float(square["position_deadband_m"]),
        centered_hold_s=float(square["centered_hold_s"]),
        max_duration_s=float(square["max_duration_s"]),
        soft_radius_m=float(square["soft_radius_m"]),
        hard_radius_m=float(square["hard_radius_m"]),
        min_height_m=test_height - height_tolerance,
        max_height_m=test_height + height_tolerance,
        low_confidence_grace_s=float(square["low_confidence_grace_s"]),
        t265_jump_window_s=float(square["t265_jump_window_s"]),
        t265_jump_m=float(square["t265_jump_m"]),
        max_measurement_innovation_m=float(square["measurement_innovation_m"]),
    )
    return data, cybercam, square, config


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-visual-servo", action="store_true")
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    parser.add_argument("--telemetry", type=Path, default=Path("static_square_servo.jsonl"))
    args = parser.parse_args(argv)
    if not args.enable_visual_servo:
        parser.error("安全门禁：必须显式传入 --enable-visual-servo")

    _, cybercam_cfg, square_cfg, servo_cfg = _load_config(args.config)
    reader = CyberCamReader(
        port=str(cybercam_cfg["port"]),
        baudrate=int(cybercam_cfg["baudrate"]),
    )
    if not reader.start():
        raise RuntimeError("VS1串口启动失败，禁止起飞")
    deadline = time.monotonic() + 3.0
    while reader.stats()["accepted_frames"] == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    if reader.stats()["accepted_frames"] == 0:
        reader.close()
        raise RuntimeError("3秒内未收到任何VS1帧，禁止起飞")

    telemetry = ServoTelemetryLogger(args.telemetry)
    telemetry.start()
    serial_fc = None
    mission_obj = None
    try:
        if not wait_for_start_button():
            logger.error("一键起飞门禁失败，程序退出；飞控不会解锁")
            return
        re_fc = [0] * 14
        se_fc = [170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255]
        realsense = t265_class()
        port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)
        servo = StaticSquareServo(reader, servo_cfg)
        mission_obj = StaticSquareMission(
            re_fc, se_fc, realsense, serial_fc, servo, telemetry, square_cfg
        )
        mission_obj.start()
        while mission_obj.task_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("用户中断")
        if mission_obj is not None:
            mission_obj.emergency()
            if not mission_obj.task_running:
                mission_obj.stop_all()
    finally:
        telemetry.close()
        reader.close()
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()


if __name__ == "__main__":
    main()
