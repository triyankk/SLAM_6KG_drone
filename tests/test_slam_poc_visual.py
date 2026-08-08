from __future__ import annotations

import numpy as np

from optflow_slam.slam_poc_visual import SlamPocState


class FakeClock:
    def __init__(self) -> None:
        self.now_ns = 0

    def __call__(self) -> int:
        return self.now_ns

    def advance(self, seconds: float) -> None:
        self.now_ns += int(seconds * 1.0e9)


def make_motion_ready(state: SlamPocState) -> None:
    state.update_diagnostics(
        {
            "publishing": True,
            "synchronized": True,
            "imu": {"connected": True, "rate_hz": 200.0},
            "lidar": {"connected": True, "rate_hz": 5.0},
        }
    )
    for _ in range(5):
        state.update_odometry((0.0, 0.0, 0.0))
    for frame in range(31):
        state.update_rgbd(
            {
                "tracking": True,
                "frames": frame + 1,
                "tracked_frames": frame + 1,
                "tracking_success_ratio": 1.0,
                "gyro_prior_coverage_ratio": 1.0,
                "position_local_flu_m": [0.0, 0.0, 0.0],
            }
        )


def test_poc_report_passes_only_with_live_motion_evidence() -> None:
    state = SlamPocState("proof")
    state.update_diagnostics(
        {
            "publishing": True,
            "synchronized": True,
            "imu": {"connected": True, "rate_hz": 200.0},
            "lidar": {"connected": True, "rate_hz": 5.0},
        }
    )
    for position in np.linspace(0.0, 0.20, 6):
        state.update_odometry((position, 0.0, 0.0))
    for frame in range(31):
        state.update_rgbd(
            {
                "tracking": frame > 0,
                "frames": frame + 1,
                "tracked_frames": frame,
                "tracking_success_ratio": frame / max(1, frame),
                "gyro_prior_coverage_ratio": frame / max(1, frame),
                "position_local_flu_m": [frame * 0.007, 0.0, 0.0],
                "path_length_m": frame * 0.007,
                "map_keyframes": 4,
                "map_points": 600,
            }
        )
    state.update_odometry((0.0, 0.0, 0.0))
    state.update_rgbd(
        {
            "tracking": True,
            "frames": 32,
            "tracked_frames": 31,
            "tracking_success_ratio": 1.0,
            "gyro_prior_coverage_ratio": 1.0,
            "position_local_flu_m": [0.0, 0.0, 0.0],
            "path_length_m": 0.42,
            "map_keyframes": 5,
            "map_points": 600,
        }
    )
    state.update_rgbd_map(
        np.zeros((600, 3), dtype=np.float32),
        np.zeros((600, 3), dtype=np.uint8),
    )

    report = state.report()

    assert report["result"] == "pass"
    assert report["pose_sent_to_cube"] is False
    assert all(report["gates"].values())


def test_map_snapshot_is_bounded_binary_payload() -> None:
    state = SlamPocState("proof")
    state.update_rgbd_map(
        np.array(((1.0, -2.0, 3.0),), dtype=np.float32),
        np.array(((10, 20, 30),), dtype=np.uint8),
    )

    payload = state.map_snapshot()

    assert payload["sequence"] == 1
    assert payload["point_count"] == 1
    assert payload["encoding"] == "int16_le_base64"


def test_motion_guide_advances_using_both_trajectories() -> None:
    clock = FakeClock()
    state = SlamPocState("proof", clock_ns=clock)
    make_motion_ready(state)

    guide = state.snapshot()["guide"]
    assert guide["phase"] == "settle"
    assert guide["sequence"] == 1

    clock.advance(3.1)
    guide = state.snapshot()["guide"]
    assert guide["phase"] == "outbound"
    assert guide["sequence"] == 2
    rtl_shadow = state.snapshot()["rtl_shadow"]
    assert rtl_shadow["state"] == "recording_outbound"
    assert rtl_shadow["launch_captured"] is True

    state.update_odometry((0.35, 0.0, 0.20))
    state.update_rgbd(
        {
            "tracking": True,
            "frames": 32,
            "tracking_success_ratio": 1.0,
            "gyro_prior_coverage_ratio": 1.0,
            "position_local_flu_m": [0.35, 0.0, 0.20],
        }
    )
    guide = state.snapshot()["guide"]
    assert guide["phase"] == "outbound"
    assert guide["vertical_warning"] is True

    state.update_odometry((0.31, 0.0, 0.0))
    state.update_rgbd(
        {
            "tracking": True,
            "frames": 33,
            "tracking_success_ratio": 1.0,
            "gyro_prior_coverage_ratio": 1.0,
            "position_local_flu_m": [0.31, 0.0, 0.0],
        }
    )
    guide = state.snapshot()["guide"]
    assert guide["phase"] == "hold_out"
    assert guide["sequence"] == 3

    clock.advance(3.1)
    guide = state.snapshot()["guide"]
    assert guide["phase"] == "return"
    assert guide["sequence"] == 4
    assert state.snapshot()["rtl_shadow"]["state"] == "returning"

    clock.advance(0.2)
    state.update_odometry((0.05, 0.0, 0.0))
    state.update_rgbd(
        {
            "tracking": True,
            "frames": 34,
            "tracking_success_ratio": 1.0,
            "gyro_prior_coverage_ratio": 1.0,
            "position_local_flu_m": [0.05, 0.0, 0.0],
        }
    )
    guide = state.snapshot()["guide"]
    assert guide["phase"] == "final_hold"
    assert guide["sequence"] == 5

    clock.advance(3.1)
    guide = state.snapshot()["guide"]
    assert guide["phase"] == "complete"
    assert guide["sequence"] == 6
    assert guide["progress"] == 1.0
    rtl_shadow = state.snapshot()["rtl_shadow"]
    assert rtl_shadow["state"] == "arrived"
    assert rtl_shadow["velocity_sent_to_cube"] is False


def test_motion_guide_resets_hold_timer_when_aircraft_moves() -> None:
    clock = FakeClock()
    state = SlamPocState("proof", clock_ns=clock)
    make_motion_ready(state)
    assert state.snapshot()["guide"]["phase"] == "settle"

    clock.advance(2.5)
    state.update_odometry((0.06, 0.0, 0.0))
    state.update_rgbd(
        {
            "tracking": True,
            "frames": 32,
            "tracking_success_ratio": 1.0,
            "gyro_prior_coverage_ratio": 1.0,
            "position_local_flu_m": [0.06, 0.0, 0.0],
        }
    )
    guide = state.snapshot()["guide"]
    assert guide["phase"] == "settle"
    assert guide["hold_remaining_s"] == 3.0

    clock.advance(2.9)
    assert state.snapshot()["guide"]["phase"] == "settle"
    clock.advance(0.2)
    assert state.snapshot()["guide"]["phase"] == "outbound"


def test_flight_shadow_allows_arm_and_completes_after_disarm_tail() -> None:
    clock = FakeClock()
    state = SlamPocState(
        "flight",
        clock_ns=clock,
        allow_armed=True,
        guide_enabled=False,
    )
    make_motion_ready(state)

    state.update_cube("HEARTBEAT", {"base_mode": 128})
    assert not state.should_stop()
    assert state.snapshot()["stage"] == "armed_flight_shadow"
    assert state.snapshot()["guide"]["phase"] == "flight_shadow"
    assert not state.flight_complete()

    clock.advance(1.0)
    state.update_cube("HEARTBEAT", {"base_mode": 0})
    assert state.flight_lifecycle()["completed_arm_cycle"]
    assert not state.flight_complete(post_disarm_s=3.0)

    clock.advance(3.1)
    assert state.flight_complete(post_disarm_s=3.0)


def test_poc_report_prefers_time_aligned_rtl_agreement() -> None:
    clock = FakeClock()
    state = SlamPocState("proof", clock_ns=clock)
    make_motion_ready(state)
    assert state.snapshot()["guide"]["phase"] == "settle"
    clock.advance(3.1)
    assert state.snapshot()["guide"]["phase"] == "outbound"

    frame = 31
    for position_m in (0.10, 0.20, 0.31):
        frame += 1
        clock.advance(0.2)
        state.update_odometry((position_m, 0.0, 0.0))
        state.update_rgbd(
            {
                "host_monotonic_ns": clock(),
                "tracking": True,
                "frames": frame,
                "tracking_success_ratio": 1.0,
                "gyro_prior_coverage_ratio": 1.0,
                "position_local_flu_m": [position_m, 0.0, 0.0],
                "map_points": 600,
            }
        )
        state.snapshot()
    assert state.snapshot()["guide"]["phase"] == "hold_out"
    clock.advance(3.1)
    assert state.snapshot()["guide"]["phase"] == "return"

    for position_m in (0.26, 0.20, 0.15, 0.10):
        frame += 1
        clock.advance(0.2)
        state.update_odometry((position_m, 0.0, 0.0))
        state.update_rgbd(
            {
                "host_monotonic_ns": clock(),
                "tracking": True,
                "frames": frame,
                "tracking_success_ratio": 1.0,
                "gyro_prior_coverage_ratio": 1.0,
                "position_local_flu_m": [position_m, 0.0, 0.0],
                "map_points": 600,
            }
        )
        state.snapshot()
    assert state.snapshot()["guide"]["phase"] == "final_hold"
    clock.advance(3.1)
    assert state.snapshot()["guide"]["phase"] == "complete"
    state.update_rgbd_map(
        np.zeros((600, 3), dtype=np.float32),
        np.zeros((600, 3), dtype=np.uint8),
    )

    frame += 1
    state.update_rgbd(
        {
            "host_monotonic_ns": clock(),
            "tracking": True,
            "frames": frame,
            "tracking_success_ratio": 1.0,
            "gyro_prior_coverage_ratio": 1.0,
            "position_local_flu_m": [-0.60, 0.0, 0.0],
            "map_points": 600,
        }
    )
    report = state.report()

    assert report["local_return_shadow"]["result"] == "shadow_pass"
    assert report["metrics"]["peak_trajectory_agreement"] is False
    assert report["metrics"]["trajectory_agreement_source"] == (
        "rtl_shadow_time_aligned"
    )
    assert report["result"] == "pass"
