import json
import sys

files = [
    "test_data_20260710/flight_data_land_pwm_fix_regression.jsonl",
    "test_data_20260710/flight_data_frame2timestamp_verify_success.jsonl",
    "test_data_20260712/flight_data_yawtest_kp03_landanomaly.jsonl",
]

for path in files:
    with open(path) as f:
        lines = [json.loads(l) for l in f if l.strip()]
    nav = [d for d in lines if d.get("state") == "NAVIGATE" and "pos" in d]
    print(f"=== {path} (last 15 NAVIGATE records) ===")
    for d in nav[-15:]:
        print(f"  t={d['t']:.3f} z={d['pos'][2]:.3f} setpoint_cm={d.get('height_setpoint_cm')} "
              f"target_z={d.get('target',[None]*3)[2]} vy={d.get('vy')}")
    print()
