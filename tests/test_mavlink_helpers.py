from __future__ import annotations

import time

from legacy_flow_bridge.gps_denied.mavlink_helpers import wait_for_gps_home


class FakeMsg:
    def __init__(self, msg_type: str, **fields):
        self._msg_type = msg_type
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        return self._msg_type


class FakeMav:
    def __init__(self):
        self.status_texts: list[tuple[int, bytes]] = []

    def statustext_send(self, severity: int, text: bytes) -> None:
        self.status_texts.append((severity, text))


class FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self, messages):
        self._messages = list(messages)
        self.mav = FakeMav()

    def recv_match(self, *args, **kwargs):
        if self._messages:
            return self._messages.pop(0)
        time.sleep(min(float(kwargs.get("timeout", 0.0) or 0.0), 0.01))
        return None

    def close(self) -> None:
        pass


def test_wait_for_gps_home_rejects_home_position_without_raw_gps_fix():
    master = FakeMaster(
        [
            FakeMsg(
                "HOME_POSITION",
                latitude=129715987,
                longitude=775945627,
                altitude=900000,
            ),
            FakeMsg(
                "GPS_RAW_INT",
                fix_type=1,
                satellites_visible=0,
                lat=0,
                lon=0,
                alt=0,
            ),
        ]
    )

    fix = wait_for_gps_home(master, timeout_s=0.03, min_fix_type=3, min_sats=6)

    assert fix is None


def test_wait_for_gps_home_accepts_good_raw_gps_fix():
    master = FakeMaster(
        [
            FakeMsg(
                "HOME_POSITION",
                latitude=129715987,
                longitude=775945627,
                altitude=900000,
            ),
            FakeMsg(
                "GPS_RAW_INT",
                fix_type=3,
                satellites_visible=10,
                lat=129715987,
                lon=775945627,
                alt=900000,
            ),
        ]
    )

    fix = wait_for_gps_home(master, timeout_s=0.5, min_fix_type=3, min_sats=6)

    assert fix is not None
    assert fix.source == "GPS_RAW_INT"
    assert fix.fix_type == 3
    assert fix.satellites_visible == 10
