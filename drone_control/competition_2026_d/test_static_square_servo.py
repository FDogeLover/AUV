import unittest

from .static_square_servo import StaticSquareServo, StaticSquareServoConfig
from .vision.platform_observation import FeatureFlag, PlatformObservation


class FakeReader:
    def __init__(self):
        self.observation = None
        self.running = True

    def latest(self, now, max_age_s):
        if self.observation is None or self.observation.age_s(now) > max_age_s:
            return None
        return self.observation

    def is_running(self):
        return self.running


def context(now, x=0.0, y=0.0, z=1.5, confidence=3):
    return {
        "now_monotonic": now,
        "position_m": (x, y, z),
        "velocity_m_s": (0.0, 0.0, 0.0),
        "t265_confidence": confidence,
    }


class StaticSquareServoSafetyTest(unittest.TestCase):
    def test_confirmed_left_target_commands_positive_x_then_stale_zero(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader)
        servo.arm(0.0, (0.0, 0.0))
        for seq, now in enumerate((0.03, 0.06, 0.09), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 220, 240, 80, 0, 0, 90,
                int(FeatureFlag.SURROGATE_SQUARE), now,
            )
            servo(context(now))
        self.assertGreater(servo.snapshot().command_m_s[0], 0.0)
        self.assertAlmostEqual(servo.snapshot().command_m_s[1], 0.0, places=6)
        decision = servo(context(0.30))
        self.assertEqual((decision["vx_cms"], decision["vy_cms"]), (0, 0))
        self.assertEqual(servo.snapshot().mode, "LOST")

    def test_partial_target_uses_coarse_speed_limit(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader)
        servo.arm(1.0, (0.0, 0.0))
        reader.observation = PlatformObservation(
            1, 1, 1, True, 100, 240, 0, 0, 0, 70,
            int(FeatureFlag.SURROGATE_SQUARE | FeatureFlag.PARTIAL), 1.03,
        )
        decision = servo(context(1.03))
        self.assertEqual(servo.snapshot().mode, "PARTIAL_COARSE")
        self.assertGreaterEqual(decision["vx_cms"], 0)
        self.assertLessEqual((decision["vx_cms"] ** 2 + decision["vy_cms"] ** 2) ** 0.5, 5.0)

    def test_hard_geofence_fault_is_latched(self):
        servo = StaticSquareServo(FakeReader())
        servo.arm(1.0, (0.0, 0.0))
        decision = servo(context(1.03, x=0.61))
        self.assertTrue(decision["fault"])
        self.assertEqual(decision["reason"], "hard_geofence")
        with self.assertRaises(RuntimeError):
            servo.arm(2.0, (0.0, 0.0))

    def test_soft_geofence_removes_outward_component(self):
        servo = StaticSquareServo(FakeReader())
        servo.arm(1.0, (0.0, 0.0))
        command = servo._apply_soft_geofence((0.08, 0.02), (0.55, 0.0, 1.5))
        self.assertAlmostEqual(command[0], 0.0, places=9)
        self.assertAlmostEqual(command[1], 0.02, places=9)

    def test_timeout_and_height_or_t265_faults(self):
        servo = StaticSquareServo(FakeReader(), StaticSquareServoConfig(max_duration_s=0.1))
        servo.arm(1.0, (0.0, 0.0))
        decision = servo(context(1.11))
        self.assertFalse(decision["fault"])
        self.assertEqual(servo.snapshot().mode, "EXITING")

        for kwargs, reason in (
            ({"z": 1.7}, "height_out_of_range"),
            ({"confidence": 0}, "t265_confidence_zero"),
        ):
            guarded = StaticSquareServo(FakeReader())
            guarded.arm(2.0, (0.0, 0.0))
            fault = guarded(context(2.03, **kwargs))
            self.assertTrue(fault["fault"])
            self.assertEqual(fault["reason"], reason)

    def test_reader_thread_stop_faults_same_tick(self):
        reader = FakeReader()
        reader.running = False
        servo = StaticSquareServo(reader)
        servo.arm(1.0, (0.0, 0.0))
        decision = servo(context(1.03))
        self.assertTrue(decision["fault"])
        self.assertEqual(decision["reason"], "cybercam_reader_stopped")


if __name__ == "__main__":
    unittest.main()
