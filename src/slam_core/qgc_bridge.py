import os

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil


class QgcUdpBridge:
    def __init__(self, forward_host: str, forward_port: int, bind_host: str, bind_port: int):
        self.out = mavutil.mavlink_connection(
            f"udpout:{forward_host}:{forward_port}",
            source_system=255,
            source_component=0,
        )
        self.inp = mavutil.mavlink_connection(
            f"udpin:{bind_host}:{bind_port}",
            source_system=255,
            source_component=0,
        )
        self.forward_host = forward_host
        self.forward_port = forward_port
        self.bind_host = bind_host
        self.bind_port = bind_port

    def forward_downlink(self, msg) -> None:
        try:
            self.out.mav.send(msg)
        except Exception:  # noqa: BLE001
            pass

    def forward_uplink_to_cube(self, cube_master) -> int:
        forwarded = 0
        while True:
            try:
                msg = self.inp.recv_match(blocking=False)
            except Exception:  # noqa: BLE001
                return forwarded
            if msg is None:
                return forwarded
            try:
                cube_master.mav.send(msg)
                forwarded += 1
            except Exception:  # noqa: BLE001
                continue

    def close(self) -> None:
        for link in (self.out, self.inp):
            if hasattr(link, "close"):
                try:
                    link.close()
                except Exception:  # noqa: BLE001
                    pass
