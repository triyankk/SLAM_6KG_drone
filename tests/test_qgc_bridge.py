from intellisense_slam.fc_config import FlightControllerTelemetry, drain_fc_telemetry


class FakeMsg:
    def __init__(self, msg_type):
        self.msg_type = msg_type

    def get_type(self):
        return self.msg_type


class FakeMaster:
    def __init__(self, messages):
        self.messages = list(messages)

    def recv_match(self, *args, **kwargs):
        if not self.messages:
            return None
        return self.messages.pop(0)


class FakeQgcBridge:
    def __init__(self):
        self.downlink = []

    def forward_downlink(self, msg):
        self.downlink.append(msg)


def test_drain_fc_telemetry_forwards_downlink_to_qgc():
    msg = FakeMsg("HEARTBEAT")
    master = FakeMaster([msg])
    bridge = FakeQgcBridge()

    drain_fc_telemetry(master, FlightControllerTelemetry(), bridge)

    assert bridge.downlink == [msg]
