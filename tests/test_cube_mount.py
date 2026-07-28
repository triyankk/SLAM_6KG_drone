from pathlib import Path

from optflow_slam.config import load_config
from optflow_slam.cube_mount import desired_mount_parameters


ROOT = Path(__file__).resolve().parents[1]


def test_cube_mount_parameters_apply_same_cg_offset_to_all_imus() -> None:
    config = load_config(ROOT / "config" / "system.yaml")

    parameters = desired_mount_parameters(config)

    assert parameters["AHRS_ORIENTATION"] == 6
    for imu_index in (1, 2, 3):
        assert parameters[f"INS_POS{imu_index}_X"] == 0.08
        assert parameters[f"INS_POS{imu_index}_Y"] == 0.0
        assert parameters[f"INS_POS{imu_index}_Z"] == -0.08
