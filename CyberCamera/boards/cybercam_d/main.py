"""Cyber Camera D题独立调试入口。

默认把VS1输出到stdout；上板时用--serial指定连接Pi的UART。预览仅用于调试，
协议发送不依赖显示窗口。
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2

try:
    from .camera_backend import create_capture
    from .detector import PlatformDetector
    from .display_backend import create_display
    from .protocol import BAUDRATE, encode
except ImportError:  # 允许直接python main.py运行
    from camera_backend import create_capture
    from detector import PlatformDetector
    from display_backend import create_display
    from protocol import BAUDRATE, encode


def _stream_id() -> int:
    value = (time.monotonic_ns() ^ id(object())) & 0xFFFF
    return value or 1


def run(
    camera=0,
    serial_port: str | None = None,
    preview: bool = False,
    backend: str = "auto",
    width: int = 640,
    height: int = 480,
    hmirror: bool = False,
    vflip: bool = False,
    display_mode: str = "off",
    display_fps: float = 10.0,
    display_rotation: int = 0,
) -> int:
    capture = create_capture(
        backend, camera, width, height, hmirror=hmirror, vflip=vflip
    )
    if not capture.isOpened():
        raise RuntimeError(f"无法打开摄像头: backend={backend}, source={camera}")
    display = create_display(display_mode, display_rotation)
    output = None
    if serial_port:
        import serial
        output = serial.Serial(serial_port, BAUDRATE, timeout=0, write_timeout=0.03)
    detector = PlatformDetector()
    stream_id = _stream_id()
    seq = 0
    next_display = 0.0
    try:
        while True:
            ok, frame = capture.read()
            capture_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            if not ok:
                result = None
                packet = encode(stream_id, seq, capture_ms, False, 0, 0, 0, 0, 0, 0, 0)
            else:
                result = detector.detect(frame)
                packet = encode(
                    stream_id, seq, capture_ms, result.found, result.cx, result.cy,
                    result.outer_px, result.inner_px, result.angle_cdeg,
                    result.quality, result.flags,
                )
            if output is None:
                sys.stdout.buffer.write(packet)
                sys.stdout.buffer.flush()
            else:
                output.write(packet)
            seq = (seq + 1) & 0xFFFFFFFF
            if display.enabled and ok and time.monotonic() >= next_display:
                annotated = frame.copy()
                if result and result.found:
                    cv2.circle(annotated, (result.cx, result.cy), 6, (0, 0, 255), 2)
                    cv2.putText(
                        annotated, f"FOUND Q={result.quality} flags=0x{result.flags:02X}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
                    )
                else:
                    cv2.putText(
                        annotated, "NO TARGET", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 180, 255), 2,
                    )
                if not display.show(annotated):
                    break
                next_display = time.monotonic() + 1.0 / max(1.0, display_fps)
    finally:
        capture.release()
        display.close()
        if output is not None:
            output.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="0", help="摄像头索引或视频路径")
    parser.add_argument("--backend", choices=("auto", "csi", "opencv"), default="csi")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--hmirror", action="store_true")
    parser.add_argument("--vflip", action="store_true")
    parser.add_argument("--serial", default=None, help="连接Pi的UART设备")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--display", choices=("off", "builtin", "opencv"), default="off")
    parser.add_argument("--display-fps", type=float, default=10.0)
    parser.add_argument("--display-rotation", type=int, choices=(0, 90, 180, 270), default=0)
    args = parser.parse_args()
    camera = int(args.camera) if str(args.camera).isdigit() else args.camera
    display_mode = "opencv" if args.preview and args.display == "off" else args.display
    return run(
        camera, args.serial, args.preview, args.backend,
        args.width, args.height, args.hmirror, args.vflip,
        display_mode, args.display_fps, args.display_rotation,
    )


if __name__ == "__main__":
    raise SystemExit(main())
