"""Small data models shared by the bring-up tools."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class Profile(str, Enum):
    FC_BENCH = "fc_bench"
    SLAM_BENCH = "slam_bench"
    NAVIGATION = "navigation"


@dataclass(frozen=True)
class ProbeResult:
    name: str
    available: bool
    detail: str
    metrics: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ReadinessReport:
    profile: Profile
    results: tuple[ProbeResult, ...]
    required_names: frozenset[str]

    @property
    def blockers(self) -> tuple[ProbeResult, ...]:
        return tuple(
            result
            for result in self.results
            if result.name in self.required_names and not result.available
        )

    @property
    def ready(self) -> bool:
        return not self.blockers

