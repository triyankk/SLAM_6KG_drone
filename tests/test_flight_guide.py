from optflow_slam.flight_guide import FlightShadowGuide


class FakeClock:
    def __init__(self) -> None:
        self.now_ns = 1_000_000_000

    def __call__(self) -> int:
        return self.now_ns

    def advance(self, seconds: float) -> None:
        self.now_ns += round(seconds * 1.0e9)


def heartbeat(
    guide: FlightShadowGuide,
    clock: FakeClock,
    *,
    armed: bool,
    mode: str = "LOITER",
) -> None:
    guide.observe_cube(
        "HEARTBEAT",
        {"base_mode": 128 if armed else 0},
        mode_name=mode,
        now_ns=clock(),
    )


def refresh_flight_state(
    guide: FlightShadowGuide,
    clock: FakeClock,
    *,
    north_m: float = 0.0,
    east_m: float = 0.0,
    altitude_m: float = 1.5,
    horizontal_speed_mps: float = 0.0,
    yaw_rad: float = 0.0,
) -> None:
    now_ns = clock()
    guide.observe_cube(
        "LOCAL_POSITION_NED",
        {
            "x": north_m,
            "y": east_m,
            "z": -altitude_m,
            "vx": horizontal_speed_mps,
            "vy": 0.0,
            "vz": 0.0,
        },
        now_ns=now_ns,
    )
    guide.observe_cube(
        "ATTITUDE", {"yaw": yaw_rad}, now_ns=now_ns
    )
    guide.observe_cube(
        "RC_CHANNELS",
        {
            "chan1_raw": 1500,
            "chan2_raw": 1500,
            "chan3_raw": 1500,
            "chan4_raw": 1500,
        },
        now_ns=now_ns,
    )
    guide.observe_cube(
        "DISTANCE_SENSOR",
        {
            "orientation": 25,
            "current_distance": round(altitude_m * 100),
            "min_distance": 8,
            "max_distance": 3000,
        },
        now_ns=now_ns,
    )


def hold_for(
    guide: FlightShadowGuide,
    clock: FakeClock,
    seconds: int,
    *,
    north_m: float,
) -> None:
    for _ in range(seconds):
        clock.advance(1.0)
        refresh_flight_state(guide, clock, north_m=north_m)


def test_qgc_guide_completes_manual_out_and_back_sequence() -> None:
    clock = FakeClock()
    guide = FlightShadowGuide(clock_ns=clock)

    guide.set_pipeline_ready(True, now_ns=clock())
    heartbeat(guide, clock, armed=True)
    refresh_flight_state(guide, clock)
    assert guide.phase == "initial_hold"

    hold_for(guide, clock, 10, north_m=0.0)
    assert guide.phase == "outbound"

    refresh_flight_state(guide, clock, north_m=0.46)
    assert guide.phase == "hold_out"
    hold_for(guide, clock, 5, north_m=0.46)
    assert guide.phase == "return"

    refresh_flight_state(guide, clock, north_m=0.05)
    assert guide.phase == "final_hold"
    hold_for(guide, clock, 10, north_m=0.05)
    assert guide.phase == "land"

    heartbeat(guide, clock, armed=False)
    assert guide.phase == "complete"
    report = guide.report()
    assert report["mode"] == "qgc_advisory_only"
    assert not report["movement_commands_sent"]
    assert report["final_status"]["phase"] == "complete"


def test_initial_hold_requires_ten_continuous_seconds_in_altitude_band() -> None:
    clock = FakeClock()
    guide = FlightShadowGuide(clock_ns=clock)
    guide.set_pipeline_ready(True, now_ns=clock())
    heartbeat(guide, clock, armed=True)
    refresh_flight_state(guide, clock)

    hold_for(guide, clock, 6, north_m=0.0)
    refresh_flight_state(guide, clock, altitude_m=0.7)
    clock.advance(1.0)
    refresh_flight_state(guide, clock)
    hold_for(guide, clock, 9, north_m=0.0)
    assert guide.phase == "initial_hold"

    hold_for(guide, clock, 1, north_m=0.0)
    assert guide.phase == "outbound"


def test_initial_hold_waits_for_fresh_rc_input_and_loiter() -> None:
    clock = FakeClock()
    guide = FlightShadowGuide(clock_ns=clock)
    guide.set_pipeline_ready(True, now_ns=clock())
    heartbeat(guide, clock, armed=True, mode="ALT_HOLD")
    guide.observe_cube(
        "DISTANCE_SENSOR",
        {
            "orientation": 25,
            "current_distance": 150,
            "min_distance": 8,
            "max_distance": 3000,
        },
        now_ns=clock(),
    )
    assert guide.phase == "climb"

    heartbeat(guide, clock, armed=True, mode="LOITER")
    guide.observe_cube(
        "LOCAL_POSITION_NED",
        {"x": 0, "y": 0, "z": -1.5, "vx": 0, "vy": 0, "vz": 0},
        now_ns=clock(),
    )
    assert guide.phase == "climb"

    refresh_flight_state(guide, clock)
    assert guide.phase == "initial_hold"


def test_every_qgc_prompt_fits_single_statustext_packet() -> None:
    for text in FlightShadowGuide.PHASE_TEXT.values():
        assert len(text.encode("ascii")) <= 50
