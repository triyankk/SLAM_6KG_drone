"""Small MAVLink UDP router for QGC/MK15 visibility.

The Cube serial link is owned by the SLAM bridge, so QGC cannot also grab that
same port. This helper forwards Cube downlink to UDP and relays QGC uplink back
to the Cube through the bridge-owned connection.
"""

import os

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil


class QgcUdpBridge:
    def __init__(self, forward_host: str, forward_port: int, bind_host: str, bind_port: int):
        # The configured output is normally localhost for bench work. If that is
        # local, add a LAN broadcast too so an MK15/QGC tablet on the same
        # network can still see STATUSTEXT and telemetry without hard-coding IP.
        self.out_links = [
            mavutil.mavlink_connection(
                f"udpout:{forward_host}:{forward_port}",
                source_system=255,
                source_component=0,
            )
        ]
        if forward_host in {"127.0.0.1", "localhost"}:
            self.out_links.append(
                mavutil.mavlink_connection(
                    f"udpbcast:255.255.255.255:{forward_port}",
                    source_system=255,
                    source_component=0,
                )
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
        for link in self.out_links:
            try:
                link.mav.send(msg)
            except Exception:  # noqa: BLE001
                pass

    def forward_uplink_to_cube(self, cube_master) -> int:
        # Read from both the dedicated input socket and any output sockets that
        # can receive replies. This keeps QGC parameter requests and mode
        # changes flowing back through the same Cube connection.
        forwarded = 0
        for link in (self.inp, *self.out_links):
            while True:
                try:
                    msg = link.recv_match(blocking=False)
                except Exception:  # noqa: BLE001
                    break
                if msg is None:
                    break
                try:
                    cube_master.mav.send(msg)
                    forwarded += 1
                except Exception:  # noqa: BLE001
                    continue
        return forwarded

    def close(self) -> None:
        for link in (*self.out_links, self.inp):
            if hasattr(link, "close"):
                try:
                    link.close()
                except Exception:  # noqa: BLE001
                    pass
