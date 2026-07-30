from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    Frame,
    MessageType,
    pack_payload,
)

from drone_control.competition_2026_d.task1_start import Task1StartGate


class FakeLink:
    def __init__(self):
        self.callbacks = []
        self.published = []

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def publish(self, message_type, payload, **kwargs):
        self.published.append((message_type, payload, kwargs))
        return len(self.published)

    def feed(self, frame):
        for callback in self.callbacks:
            callback(frame)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def car_start(*, mode=1, session=10, flags=None):
    return Frame(
        message_type=MessageType.CAR_START,
        flags=int(
            Flag.ACK_REQUIRED | Flag.EVENT if flags is None else flags
        ),
        source=Device.CAR,
        dest=Device.UAV,
        session_id=session,
        seq=1,
        sender_ms=1,
        payload=pack_payload(MessageType.CAR_START, (mode, 1234)),
    )


def test_gate_rejects_wrong_mode_and_accepts_task1_once():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(mode=2))
    assert gate.session_id is None
    link.feed(car_start(session=20))
    link.feed(car_start(session=21))
    assert gate.session_id == 20
    assert gate.car_config_hash == 1234


def test_gate_requires_event_and_ack_flags():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(flags=Flag.EVENT))
    assert gate.session_id is None
    assert gate.rejected_start_frames == 1


def test_car_speed_requires_bound_session_and_fresh_data():
    clock = Clock()
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99, clock=clock)
    link.feed(car_start(session=20))
    state = Frame(
        message_type=MessageType.CAR_STATE,
        flags=Flag.NONE,
        source=Device.CAR,
        dest=Device.UAV,
        session_id=20,
        seq=2,
        sender_ms=2,
        payload=pack_payload(
            MessageType.CAR_STATE, (1, 100, 130, 0, 0)
        ),
    )
    link.feed(state)
    assert gate.car_speed() == 0.13
    clock.now = 0.6
    assert gate.car_speed() is None
