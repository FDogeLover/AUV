"""2026 competition-preparation entry point.

Examples::

    python competition_main.py --phase scout --dry-plan
    python competition_main.py --phase execute --points P2,P5 --dry-plan
    python competition_main.py --phase scout
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from Lcode.competition_plan import (
    CompetitionPlanError,
    load_competition_config,
    plan_mission,
)
from Lcode.video_source import VideoSourceError, load_video_catalog


DEFAULT_CONFIG = Path(__file__).with_name("competition_config.json")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="2026 competition mission runner")
    parser.add_argument("--phase", choices=("scout", "execute"), required=True)
    parser.add_argument(
        "--points",
        default="",
        help="Comma-separated point ids for execute phase, for example P2,P5",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--dry-plan",
        action="store_true",
        help="Print the generated mission without initializing flight hardware",
    )
    return parser


def parse_point_ids(raw: str) -> tuple[str, ...]:
    return tuple(part.strip() for part in raw.split(",") if part.strip())


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_competition_config(args.config)
        planned = plan_mission(config, args.phase, parse_point_ids(args.points))
        video_catalog = load_video_catalog(args.config)
    except (CompetitionPlanError, VideoSourceError) as exc:
        print(f"Mission planning failed: {exc}")
        return 2

    preview = planned.as_dict()
    preview["video"] = video_catalog.as_dict()
    print(json.dumps(preview, ensure_ascii=False, indent=2))
    if args.dry_plan:
        return 0

    # Importing the hardware entry point is intentionally delayed so route
    # planning can be tested on a development PC without RealSense/GPIO.
    from main import main as run_basic_flight

    run_basic_flight(
        targets=[list(point) for point in planned.waypoints],
        waypoint_holds=list(planned.hold_s),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

