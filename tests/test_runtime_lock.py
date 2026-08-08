from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

from optflow_slam.runtime_lock import cube_mavlink_lock


def test_cube_lock_is_reentrant_in_one_process(tmp_path: Path) -> None:
    with cube_mavlink_lock("outer", lock_dir=tmp_path):
        with cube_mavlink_lock("inner", lock_dir=tmp_path):
            assert (tmp_path / "cube_mavlink.lock").is_file()


def test_cube_lock_rejects_a_second_process(tmp_path: Path) -> None:
    script = """
import sys
from pathlib import Path
from optflow_slam.runtime_lock import RuntimeLockError, cube_mavlink_lock
try:
    with cube_mavlink_lock("child", lock_dir=Path(sys.argv[1])):
        print("acquired")
except RuntimeLockError:
    print("blocked")
"""
    with cube_mavlink_lock("parent", lock_dir=tmp_path):
        result = subprocess.run(
            [sys.executable, "-c", script, str(tmp_path)],
            check=True,
            capture_output=True,
            text=True,
            env={
                **os.environ,
                "PYTHONPATH": str(
                    Path(__file__).resolve().parents[1] / "src"
                ),
            },
        )

    assert result.stdout.strip() == "blocked"
