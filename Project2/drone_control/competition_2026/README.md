# competition_2026

This directory is the competition-preparation branch derived from `basic`.
The original `basic` directory remains the stable flight baseline.

The first milestone implements two configurable mission phases:

- `scout`: visit every configured observation point and return home;
- `execute`: revisit selected key points in the requested order and return home.

Preview a route without flight hardware:

```powershell
python competition_main.py --phase scout --dry-plan
python competition_main.py --phase execute --points P2,P5 --dry-plan
```

Remove `--dry-plan` only after verifying `competition_config.json`, coordinate
orientation, flight boundaries, height, point order, and emergency procedures.
The included point coordinates are conservative placeholders for bench and
small-area testing, not an official competition field definition.

Planned extension layers are real-time video, per-point snapshots, ground-side
selection, and an optional car executor. These layers should consume point ids
and task actions without changing the stable flight-control protocol.

