"""RDK板端VS1串口最小检查；不打开飞控、不解锁。"""

from __future__ import annotations

import argparse
import json
import time

from .vision.cybercam_reader import CyberCamReader


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyS7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--require-frames", type=int, default=1)
    args = parser.parse_args(argv)
    reader = CyberCamReader(args.port, args.baudrate)
    if not reader.start():
        print(json.dumps({"ok": False, "reason": "open_failed"}))
        return 2
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            time.sleep(0.02)
        stats = reader.stats()
        latest = reader.latest(time.monotonic(), max(1.0, args.duration + 1.0))
        result = {
            "ok": stats["accepted_frames"] >= args.require_frames,
            **stats,
            "latest": None if latest is None else {
                "stream_id": latest.stream_id,
                "seq": latest.seq,
                "cx": latest.cx,
                "cy": latest.cy,
                "quality": latest.quality,
                "flags": latest.flags,
            },
        }
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result["ok"] else 1
    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
