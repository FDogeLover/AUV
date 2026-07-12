import json
import sys

files = [
    "test_data_20260709/flight_data_landing_0p1.jsonl",
    "test_data_20260709/flight_data_landing_0p2.jsonl",
    "test_data_20260709/flight_data_landing_0p5.jsonl",
    "test_data_20260709/flight_data_landing_1p0_timeout25s.jsonl",
    "test_data_20260709/flight_data_rect_replay.jsonl",
    "test_data_20260710/flight_data_frame2timestamp_verify_success.jsonl",
    "test_data_20260710/flight_data_land_cmd_sent_f_fix_verify_0p3m.jsonl",
    "test_data_20260710/flight_data_land_pwm_fix_regression.jsonl",
    "test_data_20260712/flight_data_yawtest_kp03_landanomaly.jsonl",
]

for path in files:
    try:
        with open(path) as f:
            lines = [json.loads(l) for l in f if l.strip()]
    except FileNotFoundError:
        print(f"{path}: NOT FOUND")
        continue

    nav_records = [d for d in lines if d.get("state") == "NAVIGATE" and "pos" in d]
    land_records = [d for d in lines if d.get("state") == "LAND" and "pos" in d]
    if not nav_records or not land_records:
        print(f"{path}: missing NAVIGATE or LAND records (nav={len(nav_records)}, land={len(land_records)})")
        continue

    last_nav = nav_records[-1]
    target = last_nav.get("target")
    last_nav_z = last_nav["pos"][2]
    setpoint = last_nav.get("height_setpoint_cm")

    land_t0 = land_records[0]["t"]
    # height trend over first 3s of LAND
    early_land = [d for d in land_records if d["t"] - land_t0 <= 3.0]
    z0 = land_records[0]["pos"][2]
    z_early_max = max(d["pos"][2] for d in early_land)
    z_early_min = min(d["pos"][2] for d in early_land)
    climbed = z_early_max - z0

    print(f"{path}")
    print(f"  last NAVIGATE: pos.z={last_nav_z:.3f} target={target} height_setpoint_cm={setpoint}")
    print(f"  LAND start z={z0:.3f}, first-3s range=[{z_early_min:.3f},{z_early_max:.3f}], "
          f"climbed_from_start={climbed:+.3f}")
    print()
