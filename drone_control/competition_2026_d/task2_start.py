"""任务二正式一键启动入口。

启动顺序与任务一一致，区别仅在于：
1. CAR_START 的 task_mode 必须为 2；
2. 构造的是 Task2FlightMission，C 点后激活 T265 坐标伴飞与动态降落。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import zlib
from pathlib import Path

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.gpio_led import set_rgb_led  # noqa: E402
from Lcode.global_variable import fc_last_rx_time, sp_side  # noqa: E402
from t265 import t265_class  # noqa: E402

from shared.competition_2026_d_protocol import (  # noqa: E402
    Device,
    Flag,
    MessageType,
    pack_payload,
    unpack_payload,
)

from .Lcode.air_ground_link import AirGroundLink, LinkConfig  # noqa: E402
from .control.formation_controller import FormationConfig  # noqa: E402
from .dynamic_landing import LandingConfig  # noqa: E402
from .payload_servo import build_payload_actuator  # noqa: E402
from .task1_start import (  # noqa: E402
    READY_BT,
    READY_FC_LINK,
    READY_PAYLOAD_LOCKED,
    READY_VISION,
    Task1StartGate,
    _stop_mission_safely,
    _wait_for_fc,
    _wait_for_vision,
)
from .task2_flight import Task2FlightMission  # noqa: E402
from .task2_mission import Task2Config  # noqa: E402
from .task2_telemetry import (  # noqa: E402
    Task2TelemetryPublisher,
    Task2TelemetrySample,
)
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


TASK2_MODE = 2
TASK2_MASK = 1 << 1


class Task2StartGate(Task1StartGate):
    """只接受来自小车、任务模式为2、session非0的CAR_START。"""

    def wait(self, timeout_s=None):
        started = self.clock()
        next_ready = 0.0
        next_diagnostic = started + 3.0
        while not self._start_event.is_set():
            now = self.clock()
            if timeout_s is not None and now - started >= timeout_s:
                return None
            if now >= next_ready:
                next_ready = now + self.ready_interval_s
                self.link.publish(
                    MessageType.UAV_READY,
                    pack_payload(
                        MessageType.UAV_READY,
                        (
                            TASK2_MASK,
                            self.ready_bits,
                            self.config_hash,
                        ),
                    ),
                    session_id=0,
                    dest=Device.CAR,
                )
            if now >= next_diagnostic:
                next_diagnostic = now + 3.0
                stats = self.link.stats
                logger.info(
                    "蓝牙接收诊断: "
                    f"bytes={stats.rx_bytes}, "
                    f"valid_frames={stats.rx_frames}, "
                    f"parser_rejected={stats.rx_rejected}, "
                    f"wrong_dest={stats.rx_wrong_dest}, "
                    f"start_rejected={self.rejected_start_frames}, "
                    f"state_rejected={self.rejected_state_frames}, "
                    f"position_rejected={self.rejected_position_frames}, "
                    f"last_type={stats.last_frame_type}, "
                    f"last_src={stats.last_frame_source}, "
                    f"last_dst={stats.last_frame_dest}, "
                    f"last_hex={stats.last_rx_hex or '-'}"
                )
            self._start_event.wait(0.05)
        return self.session_id

    def _handle_start(self, frame):
        required = int(Flag.ACK_REQUIRED | Flag.EVENT)
        if int(frame.flags) & required != required:
            self.rejected_start_frames += 1
            if int(frame.flags) & int(Flag.ACK_REQUIRED):
                self.link.acknowledge(frame, result=1)
            logger.warning(
                "拒绝CAR_START: flags=0x%02X，必须包含0x%02X",
                int(frame.flags),
                required,
            )
            return
        try:
            task_mode, car_config_hash = unpack_payload(
                MessageType.CAR_START, frame.payload
            )
        except ValueError:
            self.rejected_start_frames += 1
            self.link.acknowledge(frame, result=1)
            logger.warning(
                "拒绝CAR_START: payload长度或格式错误，实际%d字节",
                len(frame.payload),
            )
            return
        if task_mode != TASK2_MODE or int(frame.session_id) == 0:
            self.rejected_start_frames += 1
            self.link.acknowledge(frame, result=1)
            logger.warning(
                "拒绝CAR_START: task_mode=%d, session_id=%d",
                task_mode,
                int(frame.session_id),
            )
            return
        with self._lock:
            if self._session_id is not None:
                result = 0 if self._session_id == int(frame.session_id) else 1
                self.link.acknowledge(frame, result=result)
                return
            self._session_id = int(frame.session_id)
            self._car_config_hash = int(car_config_hash)
            self._car_position = None
            self._last_car_position_seq = None
        self.link.acknowledge(frame, result=0)
        self._start_event.set()


def _load_task_config(data: dict) -> Task2Config:
    task = data["task2"]
    return Task2Config(
        cruise_height_m=float(task["cruise_height_m"]),
        follow_height_m=float(task["follow_height_m"]),
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
        path_cross_track_kp=float(
            task.get("path_cross_track_kp", 0.80)
        ),
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
        t265_min_confidence=int(task.get("t265_min_confidence", 2)),
        retakeoff_height_m=float(task.get("retakeoff_height_m", 1.50)),
        abort_climb_height_m=float(task.get("abort_climb_height_m", 1.50)),
        activate_tracker_timeout_s=float(
            task.get("activate_tracker_timeout_s", 3.0)
        ),
        require_car_position_at_c=bool(
            task.get("require_car_position_at_c", True)
        ),
    )


def _load_landing_config(data: dict) -> LandingConfig:
    landing = data.get("landing", {})
    terminal = data.get("terminal_landing", {})
    touchdown = data.get("touchdown", {})
    return LandingConfig(
        mid_height_m=float(landing.get("mid_height_m", 0.80)),
        low_height_m=float(landing.get("low_height_m", 0.35)),
        visual_min_height_m=float(landing.get("visual_min_height_m", 0.16)),
        contact_height_m=float(landing.get("contact_height_m", 0.10)),
        descend_high_m_s=float(landing.get("descend_high_m_s", 0.25)),
        descend_mid_m_s=float(landing.get("descend_mid_m_s", 0.16)),
        descend_low_m_s=float(landing.get("descend_low_m_s", 0.08)),
        reacquire_climb_m_s=float(landing.get("reacquire_climb_m_s", 0.12)),
        terminal_max_s=float(terminal.get("max_duration_s", 0.50)),
        terminal_max_drop_m=float(terminal.get("max_drop_m", 0.10)),
        terminal_max_error_m=float(
            terminal.get("max_position_error_m", 0.05)
        ),
        terminal_max_relative_speed_m_s=float(
            terminal.get("max_relative_speed_m_s", 0.08)
        ),
        terminal_max_uncertainty_m=float(
            terminal.get("max_uncertainty_m", 0.06)
        ),
        touchdown_height_margin_m=float(
            touchdown.get("height_margin_m", 0.03)
        ),
        touchdown_max_vz_m_s=float(
            touchdown.get("max_vertical_speed_m_s", 0.08)
        ),
        touchdown_max_relative_speed_m_s=float(
            touchdown.get("max_relative_speed_m_s", 0.10)
        ),
        touchdown_max_tilt_deg=float(touchdown.get("max_tilt_deg", 8.0)),
        touchdown_hold_s=float(touchdown.get("hold_s", 0.40)),
        deck_ride_s=float(landing.get("deck_ride_s", 5.0)),
        max_terminal_retries=int(terminal.get("max_retries", 1)),
    )


def _load_formation_config(data: dict) -> FormationConfig:
    form = data.get("formation", {})
    return FormationConfig(
        kp=float(form.get("kp", 0.75)),
        kd=float(form.get("kd", 0.22)),
        max_speed_m_s=float(form.get("max_speed_m_s", 0.40)),
        max_accel_m_s2=float(form.get("max_accel_m_s2", 0.45)),
        max_jerk_m_s3=float(form.get("max_jerk_m_s3", 1.2)),
    )


def _build_telemetry_sample(mission_obj) -> Task2TelemetrySample | None:
    return Task2TelemetrySample(
        phase=mission_obj.director.phase,
        base_state=str(mission_obj.state),
        position_xyz_m=tuple(
            float(v) for v in mission_obj.last_world_position
        ),
        landing_state=mission_obj.last_landing_state,
        mission_success=mission_obj.director.mission_success,
    )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    parser.add_argument(
        "--start-timeout",
        type=float,
        default=None,
        help="等待CAR_START的秒数；默认无限等待",
    )
    args = parser.parse_args(argv)

    raw_config = args.config.read_bytes()
    data = json.loads(raw_config.decode("utf-8"))
    config_hash = zlib.crc32(raw_config) & 0xFFFFFFFF
    task_config = _load_task_config(data)
    landing_config = _load_landing_config(data)
    formation_config = _load_formation_config(data)
    bluetooth = data["bluetooth"]
    cybercam = data["cybercam"]
    square = data["static_square_test"]
    payload_cfg = data.get("payload_servo", {})
    dry_run = os.getenv("DRONE_DRY_RUN", "0").strip().lower() in {
        "1",
        "true",
    }

    actuator, payload_hardware = build_payload_actuator(
        release_hold_s=float(payload_cfg.get("release_hold_s", 1.0))
    )
    if not payload_hardware.ready:
        raise RuntimeError("投放舵机未能锁定到180°，禁止进入等待启动状态")

    link = AirGroundLink(
        LinkConfig(
            port=str(bluetooth["port"]),
            baudrate=int(bluetooth["baudrate"]),
            write_timeout_s=float(
                bluetooth.get("write_timeout_s", 0.20)
            ),
            ack_timeout_s=float(bluetooth["ack_timeout_s"]),
            max_retries=int(bluetooth["max_retries"]),
            max_consecutive_tx_errors=int(
                bluetooth.get("max_consecutive_tx_errors", 3)
            ),
        )
    )
    if not link.start():
        raise RuntimeError("蓝牙链路启动失败")

    reader = None
    serial_fc = None
    mission_obj = None
    telemetry = None
    telemetry_finished = False
    warning_led_active = False
    try:
        reader = CyberCamReader(
            port=str(cybercam["port"]),
            baudrate=int(cybercam["baudrate"]),
        )
        vision_started = reader.start()
        vision_ready = (
            vision_started and _wait_for_vision(reader)
            if not dry_run
            else False
        )
        if not dry_run and not vision_ready:
            raise RuntimeError("Cyber Camera双向通信预检失败，禁止起飞")
        if vision_ready:
            logger.info("Cyber Camera VS1与PING/PONG预检通过")
        else:
            logger.warning(
                "DRONE_DRY_RUN桌面通信测试：跳过Cyber Camera双向预检"
            )

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
        if not _wait_for_fc():
            raise RuntimeError("飞控串口3秒内未收到遥测，禁止进入等待启动状态")

        start_gate = Task2StartGate(
            link,
            config_hash=config_hash,
            ready_bits=(
                READY_BT
                | READY_PAYLOAD_LOCKED
                | READY_VISION
                | READY_FC_LINK
            ),
            state_max_age_s=float(
                bluetooth.get("car_state_max_age_s", 0.30)
            ),
            state_max_speed_m_s=float(
                bluetooth.get("car_speed_max_m_s", 0.30)
            ),
            state_max_component_m_s=float(
                bluetooth.get("car_component_max_m_s", 0.40)
            ),
        )

        def _car_position_provider():
            pos = start_gate.latest_car_position()
            return pos.position_xy_m if pos else None

        mission_obj = Task2FlightMission(
            re_fc,
            se_fc,
            realsense,
            serial_fc,
            vision_reader=reader,
            payload_actuator=actuator,
            task_config=task_config,
            formation_config=formation_config,
            landing_config=landing_config,
            vision_config={
                "max_age_s": float(cybercam["max_age_s"]),
                "min_quality": int(task_config.vision_min_quality),
                "image_center_px": tuple(square["image_center_px"]),
                "focal_px": tuple(square["focal_px"]),
            },
            car_speed_provider=start_gate.car_speed,
            car_position_provider=_car_position_provider,
            car_velocity_provider=start_gate.car_velocity,
        )

        if not set_rgb_led("G"):
            raise RuntimeError("等待启动绿色状态灯点亮失败")
        warning_led_active = True
        logger.info(
            "绿灯：蓝牙、视觉、飞控和舵机均已就绪；等待小车按键发送CAR_START。"
            "T265尚未初始化；任务二将在C点激活T265坐标伴飞与动态降落"
        )
        session_id = start_gate.wait(args.start_timeout)
        if session_id is None:
            logger.error("等待CAR_START超时，任务取消")
            return
        logger.info(
            f"收到任务二CAR_START，session={session_id}，"
            f"car_config_hash=0x{start_gate.car_config_hash:08X}；"
            "解除T265拔插阻塞并开始初始化"
        )
        if not set_rgb_led("R"):
            raise RuntimeError("收到CAR_START后红色起飞警示灯点亮失败")
        mission_obj.preflight_warning_started_at = time.monotonic()
        telemetry_hz = max(
            1.0, float(bluetooth.get("uav_state_hz", 10.0))
        )
        telemetry = Task2TelemetryPublisher(
            link,
            session_id=session_id,
            state_interval_s=1.0 / telemetry_hz,
        )
        if not link.is_running:
            raise RuntimeError("蓝牙链路已在起飞预检前中断，取消任务")
        mission_obj.start()
        if not mission_obj.task_running:
            raise RuntimeError("T265或起飞预检失败，任务未启动")
        if not link.is_running:
            raise RuntimeError("蓝牙链路在起飞预检期间中断，紧急停止任务")
        warning_led_active = False
        while mission_obj.task_running:
            sample = _build_telemetry_sample(mission_obj)
            if sample is not None:
                telemetry.update(sample)
            time.sleep(0.03)
        telemetry.finish(
            mission_success=mission_obj.director.mission_success,
            faulted=bool(mission_obj.emergency_stop),
        )
        telemetry_finished = True
        link.wait_pending(0.65)
    except KeyboardInterrupt:
        logger.warning("用户中断任务二启动程序")
        _stop_mission_safely(mission_obj)
    except Exception:
        _stop_mission_safely(mission_obj)
        raise
    finally:
        _stop_mission_safely(mission_obj)
        if telemetry is not None and not telemetry_finished:
            mission_success = bool(
                mission_obj is not None
                and mission_obj.director.mission_success
            )
            telemetry.finish(
                mission_success=mission_success,
                faulted=True,
            )
            link.wait_pending(0.65)
        if warning_led_active:
            try:
                set_rgb_led("OFF")
            except Exception:
                pass
        if reader is not None:
            reader.close()
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()
        link.close()


if __name__ == "__main__":
    main()
