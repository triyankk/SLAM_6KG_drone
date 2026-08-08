from optflow_slam.im10a_config import (
    LIO_OUTPUT_MASK,
    _nearest_rate,
    _write_register,
)


class FakePort:
    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.flushes = 0

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        self.flushes += 1


def test_register_write_uses_official_normal_serial_packet(monkeypatch) -> None:
    monkeypatch.setattr("optflow_slam.im10a_config.time.sleep", lambda _: None)
    port = FakePort()

    _write_register(port, 0x02, LIO_OUTPUT_MASK)

    assert port.writes == [bytes((0xFF, 0xAA, 0x02, 0x07, 0x00))]
    assert port.flushes == 1


def test_restore_rate_uses_nearest_supported_profile() -> None:
    assert _nearest_rate(9.8) == 10.0
    assert _nearest_rate(196.0) == 200.0
    assert _nearest_rate(None) == 10.0
