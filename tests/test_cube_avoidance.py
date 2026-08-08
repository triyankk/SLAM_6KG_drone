from pathlib import Path

from optflow_slam.config import load_config
from optflow_slam.cube_avoidance import (
    active_avoidance_requested,
    desired_avoidance_parameters,
)


ROOT = Path(__file__).resolve().parents[1]


def test_active_profile_enables_lidar_proximity_and_rc7() -> None:
    config = load_config(ROOT / "config" / "system.yaml")

    parameters = desired_avoidance_parameters(config)

    assert active_avoidance_requested(config)
    assert parameters["RC7_OPTION"] == 40
    assert parameters["PRX1_TYPE"] == 2
    assert parameters["AVOID_ENABLE"] == 7
    assert parameters["AVOID_MARGIN"] == 1.5
    assert parameters["AVOID_DIST_MAX"] == 1.5


def test_shadow_profile_keeps_rc7_assignment_but_disables_avoidance(
    tmp_path: Path,
) -> None:
    source = (ROOT / "config" / "system.yaml").read_text(encoding="ascii")
    source = source.replace(
        "obstacle_avoidance:\n  stage: active\n"
        "  mavlink_output_enabled: true",
        "obstacle_avoidance:\n  stage: shadow\n"
        "  mavlink_output_enabled: false",
    )
    config_path = tmp_path / "shadow.yaml"
    config_path.write_text(source, encoding="ascii")
    config = load_config(config_path)

    parameters = desired_avoidance_parameters(config)

    assert not active_avoidance_requested(config)
    assert parameters["RC7_OPTION"] == 40
    assert parameters["PRX1_TYPE"] == 0
    assert parameters["AVOID_ENABLE"] == 0
    assert "AVOID_MARGIN" not in parameters
