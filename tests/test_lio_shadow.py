import inspect
import json
from pathlib import Path
from threading import Event
import time
from types import SimpleNamespace

import optflow_slam.lio_shadow as lio_shadow
from optflow_slam.config import load_config
from optflow_slam.flight_guide import FlightShadowGuide
from optflow_slam.obstacles import ObstacleScan, UNKNOWN_DISTANCE_CM
from optflow_slam.slam_poc_visual import SlamPocState
import yaml


def test_shadow_runner_has_no_movement_or_mode_mavlink_transmit() -> None:
    source = inspect.getsource(lio_shadow.CubeReferenceReader)

    assert "recv_match" in source
    assert "message_types = list(self.MESSAGE_TYPES)" in source
    assert "MAV_CMD_SET_MESSAGE_INTERVAL" in source
    assert "connection.reset" not in source
    assert "getattr(connection, \"reset\"" in source
    assert "MAXIMUM_RECEIVE_SILENCE_S" in source
    assert "set_position_target" not in source
    assert "set_mode" not in source
    assert "send_horizontal_distance_sensors" in source
    assert "odometry_send" not in source
    assert "link_handler: Any | None = None" in source
    assert "STATUSTEXT" in lio_shadow.CubeReferenceReader.MESSAGE_TYPES


def test_shadow_recorder_separates_safety_and_sensor_callbacks() -> None:
    source = inspect.getsource(lio_shadow._run_shadow_locked)

    assert "MultiThreadedExecutor(num_threads=4)" in source
    assert "self.safety_callback_group" in source
    assert "self.imu_callback_group" in source
    assert "self.lidar_callback_group" in source


def test_navigation_cube_link_sends_only_latest_fresh_obstacle() -> None:
    config = load_config(
        Path(__file__).resolve().parents[1] / "config/system.yaml"
    )
    sent: list[tuple] = []
    rows: list[dict] = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            distance_sensor_send=lambda *args, **kwargs: sent.append(
                (args, kwargs)
            )
        )
    )
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_COMP_ID_OBSTACLE_AVOIDANCE=196,
            MAV_DISTANCE_SENSOR_LASER=0,
        )
    )
    reader = lio_shadow.CubeReferenceReader(
        config,
        SimpleNamespace(write=rows.append),
        Event(),
    )
    older_distances = [UNKNOWN_DISTANCE_CM] * 72
    older_distances[0] = 200
    latest_distances = list(older_distances)
    latest_distances[0] = 150
    reader.queue_obstacle_scan(
        ObstacleScan(
            source="lidar",
            monotonic_ns=time.monotonic_ns(),
            distances_cm=tuple(older_distances),
            increment_deg=5.0,
            min_distance_cm=30,
            max_distance_cm=800,
        )
    )
    reader.queue_obstacle_scan(
        ObstacleScan(
            source="lidar",
            monotonic_ns=time.monotonic_ns(),
            distances_cm=tuple(latest_distances),
            increment_deg=5.0,
            min_distance_cm=30,
            max_distance_cm=800,
        )
    )

    assert reader._send_pending_obstacle(connection, mavutil)
    assert not reader._send_pending_obstacle(connection, mavutil)
    assert len(sent) == 8
    assert sent[0][0][3] == 150
    assert sent[0][0][6] == 0
    assert [packet[0][6] for packet in sent] == list(range(8))
    assert all(packet[0][3] == 801 for packet in sent[1:])
    assert reader.obstacle_messages_sent == 1
    assert reader.obstacle_wire_packets_sent == 8
    assert not hasattr(connection.mav, "srcComponent")
    assert rows[-1]["data"]["wire_protocol_version"] is None
    assert rows[-1]["data"]["transport"] == "DISTANCE_SENSOR_8_FACE"
    source = inspect.getsource(
        lio_shadow.CubeReferenceReader._send_pending_obstacle
    )
    assert ".srcComponent =" not in source


def test_navigation_cube_link_requests_bounded_message_rates() -> None:
    config = load_config(
        Path(__file__).resolve().parents[1] / "config/system.yaml"
    )
    commands: list[tuple] = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            command_long_send=lambda *args: commands.append(args)
        )
    )
    constants = {
        name: index
        for index, (name, _rate_hz) in enumerate(
            lio_shadow.FLIGHT_MESSAGE_RATES_HZ,
            start=1,
        )
    }
    constants["MAV_CMD_SET_MESSAGE_INTERVAL"] = 511
    mavutil = SimpleNamespace(mavlink=SimpleNamespace(**constants))
    reader = lio_shadow.CubeReferenceReader(
        config,
        SimpleNamespace(write=lambda _row: None),
        Event(),
    )

    reader._request_flight_message_intervals(
        connection,
        mavutil,
        1,
        1,
    )

    assert len(commands) == len(lio_shadow.FLIGHT_MESSAGE_RATES_HZ)
    assert all(command[2] == 511 for command in commands)
    assert all(command[5] >= 100_000 for command in commands)
    assert reader.stream_request_sent


def test_navigation_obstacle_component_heartbeat_is_rate_limited(
    monkeypatch,
) -> None:
    config = load_config(
        Path(__file__).resolve().parents[1] / "config/system.yaml"
    )
    sent: list[tuple] = []
    reader = lio_shadow.CubeReferenceReader(
        config,
        SimpleNamespace(write=lambda _row: None),
        Event(),
    )
    reader._obstacle_mav = SimpleNamespace(
        heartbeat_send=lambda *args: sent.append(args)
    )
    now_ns = 1_000_000_000
    monkeypatch.setattr(lio_shadow.time, "monotonic_ns", lambda: now_ns)
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_TYPE_ONBOARD_CONTROLLER=18,
            MAV_AUTOPILOT_INVALID=8,
            MAV_STATE_ACTIVE=4,
        )
    )

    assert reader._send_obstacle_heartbeat(mavutil)
    assert not reader._send_obstacle_heartbeat(mavutil)
    assert len(sent) == 1
    assert sent[0] == (18, 8, 0, 0, 4)


def test_cube_connection_tune_plays_once_per_link_session() -> None:
    config = load_config(
        Path(__file__).resolve().parents[1] / "config/system.yaml"
    )
    rows = []
    tunes = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(play_tune_send=lambda *args: tunes.append(args))
    )
    reader = lio_shadow.CubeReferenceReader(
        config,
        SimpleNamespace(write=lambda row: rows.append(row)),
        Event(),
    )

    assert reader._send_connection_tune(connection, 1, 1)
    assert not reader._send_connection_tune(connection, 1, 1)
    reader._mark_link_disconnected("test disconnect")
    assert reader._send_connection_tune(connection, 1, 1)

    assert tunes == [
        (1, 1, lio_shadow.CUBE_CONNECTED_TUNE.encode("ascii")),
        (1, 1, lio_shadow.CUBE_CONNECTED_TUNE.encode("ascii")),
    ]
    assert reader.connection_tunes_sent == 2
    assert [row["type"] for row in rows] == [
        "CUBE_CONNECTION_TUNE",
        "CUBE_LINK_RECOVERY",
        "CUBE_CONNECTION_TUNE",
    ]


def test_navigation_runtime_honors_disabled_fast_lio_map_output(
    tmp_path: Path,
) -> None:
    config = load_config(
        Path(__file__).resolve().parents[1] / "config/system.yaml"
    )
    resolved = lio_shadow._resolved_fast_lio_config(config, tmp_path)
    payload = yaml.safe_load(resolved.read_text(encoding="ascii"))
    parameters = payload["/**"]["ros__parameters"]

    assert not parameters["publish"]["map_en"]
    assert not parameters["pcd_save"]["pcd_save_en"]


def test_flight_window_metrics_require_data_inside_arm_window(tmp_path) -> None:
    session = tmp_path / "session"
    session.mkdir()

    def write(name, rows):
        (session / name).write_text(
            "".join(json.dumps(row) + "\n" for row in rows),
            encoding="utf-8",
        )

    write(
        "lio_odometry.ndjson",
        [
            {"host_monotonic_ns": stamp, "position_m": [index * 0.01, 0, 0]}
            for index, stamp in enumerate(range(1_000, 2_001, 100))
        ],
    )
    write(
        "rgbd_odometry.ndjson",
        [
            {"host_monotonic_ns": stamp, "tracking": True}
            for stamp in range(1_000, 2_001, 100)
        ],
    )
    write(
        "obstacles.ndjson",
        [
            {
                "host_monotonic_ns": 1_500,
                "kind": "source",
                "source": source,
            }
            for source in ("depth_camera", "lidar")
        ],
    )
    write(
        "cube_reference.ndjson",
        [
            {
                "host_monotonic_ns": 1_500,
                "type": "OPTICAL_FLOW",
                "data": {"quality": 200},
            },
            {
                "host_monotonic_ns": 1_600,
                "type": "DISTANCE_SENSOR",
                "data": {"orientation": 25, "current_distance": 120},
            },
        ],
    )

    metrics = lio_shadow._flight_shadow_metrics(
        session,
        {"arm_monotonic_ns": 1_000, "disarm_monotonic_ns": 2_000},
    )

    assert metrics["lio_samples"] == 11
    assert metrics["rgbd_tracking_ratio"] == 1.0
    assert metrics["obstacle_source_counts"] == {
        "depth_camera": 1,
        "lidar": 1,
    }
    assert metrics["flow_quality_p10"] == 200
    assert metrics["range_minimum_m"] == 1.2


def test_isolated_shadow_motion_uses_fixed_measured_extrinsics() -> None:
    root = Path(__file__).resolve().parents[1]
    payload = yaml.safe_load(
        (root / "config/lio/fast_lio_jt16_shadow.yaml").read_text(
            encoding="ascii"
        )
    )

    assert not payload["/**"]["ros__parameters"]["mapping"][
        "extrinsic_est_en"
    ]
    assert payload["/**"]["ros__parameters"]["mapping"][
        "extrinsic_T"
    ] == [-0.08, 0.0, -0.01]


def test_flight_guide_is_sent_as_qgc_text_and_tunes_only(tmp_path) -> None:
    statustext = []
    tunes = []
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            statustext_send=lambda *args: statustext.append(args),
            play_tune_send=lambda *args: tunes.append(args),
        )
    )
    mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(
            MAV_SEVERITY_NOTICE=5,
            MAV_SEVERITY_WARNING=4,
        )
    )
    state = SlamPocState(
        "test", allow_armed=True, guide_enabled=False
    )
    state.ready_for_motion = lambda: True
    guide = FlightShadowGuide()
    output = lio_shadow.NdjsonWriter(tmp_path / "cube.ndjson")
    reader = lio_shadow.CubeReferenceReader(
        load_config(Path(__file__).resolve().parents[1] / "config/system.yaml"),
        output,
        Event(),
        visual_state=state,
        request_flight_streams=True,
        flight_guide=guide,
    )

    reader._update_and_send_flight_guide(
        connection, mavutil, 1, 1
    )
    guide.observe_cube(
        "HEARTBEAT",
        {"base_mode": 128},
        mode_name="LOITER",
    )
    guide.observe_cube(
        "LOCAL_POSITION_NED",
        {"x": 0, "y": 0, "z": -1.5, "vx": 0, "vy": 0, "vz": 0},
    )
    guide.observe_cube("ATTITUDE", {"yaw": 0})
    guide.observe_cube(
        "RC_CHANNELS",
        {
            "chan1_raw": 1500,
            "chan2_raw": 1500,
            "chan3_raw": 1500,
            "chan4_raw": 1500,
        },
    )
    guide.observe_cube(
        "DISTANCE_SENSOR",
        {
            "orientation": 25,
            "current_distance": 150,
            "min_distance": 8,
            "max_distance": 3000,
        },
    )
    reader._update_and_send_flight_guide(
        connection, mavutil, 1, 1
    )
    output.close()

    assert reader.guidance_messages_sent >= 2
    assert reader.guidance_tunes_sent == 1
    assert statustext
    assert tunes == [(1, 1, lio_shadow.FLIGHT_GUIDE_TUNE.encode("ascii"))]
    rows = [
        json.loads(line)
        for line in (tmp_path / "cube.ndjson").read_text().splitlines()
    ]
    assert all(row["type"] == "JETSON_GUIDANCE" for row in rows)
    assert all(row["data"]["statustext_sent"] for row in rows)
