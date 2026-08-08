from __future__ import annotations

import os
from pathlib import Path
import re

from optflow_slam.paths import (
    CALIBRATION_DIR,
    CONFIG_DIR,
    DATA_DIR,
    LOG_DIR,
    MAP_DIR,
    PROJECT_ROOT,
    RECORDING_DIR,
    ROS_WORKSPACE_DIR,
    RUNTIME_DIR,
    THIRD_PARTY_DIR,
    VISUALIZER_DIR,
)


IGNORED_DIRECTORIES = {
    ".pytest_cache",
    ".venv",
    "__pycache__",
    "build",
    "calibrations",
    "dist",
    "logs",
    "maps",
    "node_modules",
    "recordings",
    "runtime",
    "test-output",
    "third_party",
    "vendor",
}
TEXT_SUFFIXES = {
    "",
    ".css",
    ".html",
    ".js",
    ".json",
    ".md",
    ".mjs",
    ".py",
    ".sh",
    ".toml",
    ".yaml",
    ".yml",
}
OUTSIDE_WORKSPACE_PATH = re.compile(r"/home/[^/\s]+/vscode/")
LEGACY_PROJECT_NAME = re.compile(r"intellisense[_-]slam", re.IGNORECASE)


def project_files():
    for root, directories, filenames in os.walk(PROJECT_ROOT):
        directories[:] = [
            name
            for name in directories
            if name not in IGNORED_DIRECTORIES
        ]
        root_path = Path(root)
        for filename in filenames:
            path = root_path / filename
            if path.suffix.lower() in TEXT_SUFFIXES:
                yield path


def test_canonical_paths_stay_inside_project() -> None:
    for path in (
        CONFIG_DIR,
        DATA_DIR,
        CALIBRATION_DIR,
        LOG_DIR,
        MAP_DIR,
        RECORDING_DIR,
        RUNTIME_DIR,
        ROS_WORKSPACE_DIR,
        THIRD_PARTY_DIR,
        VISUALIZER_DIR,
    ):
        assert path.is_relative_to(PROJECT_ROOT)


def test_project_text_has_no_external_workspace_references() -> None:
    violations = []
    for path in project_files():
        text = path.read_text(encoding="utf-8", errors="ignore")
        if OUTSIDE_WORKSPACE_PATH.search(text):
            violations.append(f"{path.relative_to(PROJECT_ROOT)}: workspace")
        if LEGACY_PROJECT_NAME.search(text):
            violations.append(f"{path.relative_to(PROJECT_ROOT)}: legacy")

    assert not violations, "\n".join(violations)


def test_project_symlinks_do_not_escape_project() -> None:
    violations = []
    for root, directories, filenames in os.walk(PROJECT_ROOT):
        directories[:] = [
            name
            for name in directories
            if name not in IGNORED_DIRECTORIES
        ]
        for name in (*directories, *filenames):
            path = Path(root) / name
            if path.is_symlink() and not path.resolve().is_relative_to(
                PROJECT_ROOT
            ):
                violations.append(str(path.relative_to(PROJECT_ROOT)))

    assert not violations, "\n".join(violations)
