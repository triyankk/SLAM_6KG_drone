from pathlib import Path

import yaml
from pymavlink import mavutil

from slam_core.bridge_config import load_bridge_config
from slam_core.fc_config import (
    BRIDGE_HEARTBEAT_PARAM,
    BRIDGE_SOURCE_SET_PARAM,
    BRIDGE_STATE_JETSON_BOOT,
    BRIDGE_STATE_PARAM,
    FlightControllerSetupConfig,
    build_fc_setup_parameters,
    current_gps_week_time,
    gps_input_timestamp_from_reference,
    publish_bridge_state,
    send_distance_sensor,
    send_fixed_gps_input,
    send_gps_input_from_pose,
    send_gps_input_from_fc_reference,
    send_obstacle_distance,
    send_body_velocity_nudge,
    send_companion_heartbeat,
)
from slam_core.bridge_config import GpsInputConfig
from slam_core.mavlink_bridge import CubeConnection, send_odometry
from slam_core.types import PoseSample


class FakeMav:
    def __init__(self):
        self.param_sets = []
        self.heartbeats = []
        self.distance_sensors = []
        self.obstacle_distances = []
        self.odometry = []
        self.gps_inputs = []
        self.local_setpoints = []

    def param_set_send(self, target_system, target_component, name, value, param_type):
        self.param_sets.append((target_system, target_component, name, value, param_type))

    def heartbeat_send(self, vehicle_type, autopilot, base_mode, custom_mode, system_status):
        self.heartbeats.append((vehicle_type, autopilot, base_mode, custom_mode, system_status))

    def distance_sensor_send(self, *args):
        self.distance_sensors.append(args)

    def obstacle_distance_send(self, *args):
        self.obstacle_distances.append(args)

    def odometry_send(self, *args):
        self.odometry.append(args)

    def gps_input_send(self, *args):
        self.gps_inputs.append(args)

    def set_position_target_local_ned_send(self, *args):
        self.local_setpoints.append(args)


class FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()


def test_slam_source_set_parameters_are_scoped():
    params = build_fc_setup_parameters(FlightControllerSetupConfig(slam_source_set=3, viso_type=3))

    assert params["EK3_SRC3_POSXY"] == 6.0
    assert params["EK3_SRC3_VELXY"] == 6.0
    assert params["EK3_SRC3_POSZ"] == 1.0
    assert params["EK3_SRC3_VELZ"] == 0.0
    assert params["EK3_SRC3_YAW"] == 1.0
    assert params["EK3_SRC1_POSXY"] == 3.0
    assert params["EK3_SRC1_VELXY"] == 3.0
    assert params["EK3_SRC1_POSZ"] == 1.0
    assert params["EK3_SRC1_VELZ"] == 0.0
    assert params["EK3_SRC1_YAW"] == 1.0
    assert "EK3_SRC2_POSXY" not in params
    assert params["AVOID_MARGIN"] == 2.0
    assert params["PRX1_TYPE"] == 2.0
    assert "GPS2_TYPE" not in params


def test_visual_odom_sources_are_scrubbed_when_viso_disabled():
    params = build_fc_setup_parameters(
        FlightControllerSetupConfig(
            slam_source_set=3,
            idle_source_set=1,
            select_source_set_on_stream=False,
            viso_type=0,
        )
    )

    assert params["VISO_TYPE"] == 0.0
    assert params["EK3_SRC1_POSXY"] == 3.0
    assert params["EK3_SRC1_VELXY"] == 3.0
    assert params["EK3_SRC3_POSXY"] == 3.0
    assert params["EK3_SRC3_VELXY"] == 3.0
    assert params["EK3_SRC3_POSZ"] == 1.0
    assert params["EK3_SRC3_YAW"] == 1.0


def test_gps2_mavlink_params_are_optional():
    params = build_fc_setup_parameters(
        FlightControllerSetupConfig(
            slam_source_set=3,
            select_source_set_on_stream=False,
            gps2_type=14,
            gps_auto_switch=1,
        )
    )

    assert params["GPS2_TYPE"] == 14.0
    assert params["GPS_AUTO_SWITCH"] == 1.0
    assert params["EK3_SRC3_POSXY"] == 3.0


def test_publish_bridge_state_updates_lua_relay_params():
    master = FakeMaster()

    publish_bridge_state(master, BRIDGE_STATE_JETSON_BOOT, 3)

    assert master.mav.param_sets == [
        (1, 1, BRIDGE_STATE_PARAM.encode("ascii"), 10.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        (1, 1, BRIDGE_SOURCE_SET_PARAM.encode("ascii"), 3.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        (
            1,
            1,
            BRIDGE_HEARTBEAT_PARAM.encode("ascii"),
            master.mav.param_sets[2][3],
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        ),
    ]


def test_companion_heartbeat_identifies_onboard_controller():
    master = FakeMaster()

    send_companion_heartbeat(master)

    assert master.mav.heartbeats == [
        (
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
    ]


def test_send_distance_sensor_uses_laser_centimeters():
    master = FakeMaster()

    send_distance_sensor(master, 1.75, sensor_id=20, max_distance_m=40.0)

    assert master.mav.distance_sensors
    msg = master.mav.distance_sensors[0]
    assert msg[1] == 2
    assert msg[2] == 4000
    assert msg[3] == 175
    assert msg[5] == 20


def test_send_obstacle_distance_uses_72_bins():
    master = FakeMaster()

    send_obstacle_distance(master, [1.0, 0.0, 2.5], max_distance_m=40.0)

    assert master.mav.obstacle_distances
    msg = master.mav.obstacle_distances[0]
    assert len(msg[2]) == 72
    assert msg[2][0] == 100
    assert msg[2][1] == 65535
    assert msg[2][2] == 250
    assert msg[3] == 5


def test_send_gps_input_targets_gps2_from_local_pose():
    master = FakeMaster()
    pose = PoseSample(
        timestamp_us=123,
        x_m=11.1,
        y_m=0.0,
        z_m=-2.0,
        qw=1.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        vx_m_s=0.2,
    )

    ok = send_gps_input_from_pose(
        master,
        pose,
        GpsInputConfig(enabled=True, gps_id=1, origin_lat_deg=10.0, origin_lon_deg=20.0, origin_alt_m=100.0),
    )

    assert ok
    msg = master.mav.gps_inputs[0]
    assert msg[1] == 1
    assert msg[3] > 0
    assert msg[4] > 0
    assert msg[5] == 3
    assert msg[7] == 200000000
    assert msg[8] == 102.0
    assert msg[11] == 0.2


def test_send_gps_input_can_mirror_real_gps_to_gps2():
    master = FakeMaster()
    gps_time_usec = 1_775_000_000_000_000
    gps_week, gps_week_ms = current_gps_week_time(gps_time_usec / 1e6)
    state = type(
        "State",
        (),
        {
            "gps_fix_type": 3,
            "gps_satellites": 12,
            "gps_lat": 123456789,
            "gps_lon": 987654321,
            "gps_alt_mm": 456000,
            "global_vx_cm_s": 20,
            "global_vy_cm_s": -10,
            "global_vz_cm_s": 5,
            "gps_time_usec": gps_time_usec,
        },
    )()

    ok = send_gps_input_from_fc_reference(master, state, GpsInputConfig(enabled=True, gps_id=1))

    assert ok
    msg = master.mav.gps_inputs[0]
    assert msg[0] == gps_time_usec
    assert msg[1] == 1
    assert msg[3] == gps_week_ms
    assert msg[4] == gps_week
    assert msg[5] == 3
    assert msg[6] == 123456789
    assert msg[7] == 987654321
    assert msg[8] == 456.0
    assert msg[11] == 0.2


def test_gps_input_timestamp_falls_back_when_fc_gps_time_is_boot_time():
    state = type("State", (), {"gps_time_usec": 2_000_000})()

    time_usec, gps_week, gps_week_ms = gps_input_timestamp_from_reference(state)

    assert time_usec > 1_000_000_000_000_000
    assert gps_week > 2400
    assert 0 <= gps_week_ms < 604800000


def test_send_fixed_gps_input_targets_gps2():
    master = FakeMaster()

    send_fixed_gps_input(
        master,
        GpsInputConfig(
            enabled=True,
            gps_id=1,
            fixed_fix=True,
            fixed_lat_deg=37.7749,
            fixed_lon_deg=-122.4194,
            fixed_alt_m=10.0,
        ),
    )

    msg = master.mav.gps_inputs[0]
    assert msg[1] == 1
    assert msg[3] > 0
    assert msg[4] > 0
    assert msg[5] == 3
    assert msg[6] == 377749000
    assert msg[7] == -1224194000
    assert msg[8] == 10.0


def test_current_gps_week_time_is_nonzero_for_modern_dates():
    week, week_ms = current_gps_week_time(1_775_000_000.0)

    assert week > 2400
    assert 0 <= week_ms < 604800000


def test_send_body_velocity_nudge_uses_body_frame_velocity_only():
    master = FakeMaster()

    send_body_velocity_nudge(master, -0.3, 0.1, -0.05)

    msg = master.mav.local_setpoints[0]
    assert msg[3] != 0
    assert msg[8] == -0.3
    assert msg[9] == 0.1
    assert msg[10] == -0.05


def test_send_odometry_has_mavlink2_message_available():
    master = FakeMaster()
    connection = CubeConnection(master=master, port="/dev/null", baud=115200)
    pose = PoseSample(
        timestamp_us=123,
        x_m=1.0,
        y_m=2.0,
        z_m=-1.0,
        qw=1.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
    )

    send_odometry(connection, pose)

    assert master.mav.odometry


def test_autostart_config_uses_gated_gps2_bridge_mode():
    config_path = Path(__file__).resolve().parents[1] / "config" / "autostart.yaml"
    text = config_path.read_text(encoding="utf-8")
    payload = yaml.safe_load(text)
    config = load_bridge_config(config_path)

    assert text.count("viso_type:") == 1
    assert payload["fc_setup"]["viso_type"] == 0
    assert payload["fc_setup"]["gps2_type"] == 14
    assert payload["fc_setup"]["select_source_set_on_stream"] is False
    assert payload["gps_input"]["enabled"] is True
    assert config.fc_setup.viso_type == 0
    assert config.fc_setup.gps2_type == 14
    assert config.gps_input.enabled is True
    assert config.obstacle.safety_distance_m == 2.0
