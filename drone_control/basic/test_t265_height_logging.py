from Mission_GPT import mission


class FakeT265:
    def get_raw_position(self):
        return (0.1, 0.2, 1.23456)

    def get_position(self):
        return (0.0, 0.0, 1.2)

    def get_tracking_confidence(self):
        return 3


class FakeSerial:
    _last_laser_height_cm = 1.18


def test_height_source_log_fields_preserve_t265_and_laser_separately(monkeypatch, tmp_path):
    router = tmp_path / "router.txt"
    router.write_text("0,0,1\n", encoding="utf-8")
    monkeypatch.setenv("DRONE_ROUTER_FILE", str(router))
    m = mission([0] * 14, [0] * 11, FakeT265(), FakeSerial())

    fields = m._height_source_log_fields()

    assert fields == {
        "laser_height_m": 1.18,
        "t265_raw_z_m": 1.23456,
        "t265_filtered_z_m": 1.2,
        "t265_confidence": 3,
    }
