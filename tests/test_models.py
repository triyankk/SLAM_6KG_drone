from optflow_slam.models import ProbeResult, Profile, ReadinessReport


def test_readiness_blocks_only_required_failures() -> None:
    report = ReadinessReport(
        profile=Profile.FC_BENCH,
        results=(
            ProbeResult("cube_hflow", True, "ok"),
            ProbeResult("depth_camera", False, "disconnected"),
        ),
        required_names=frozenset({"cube_hflow"}),
    )

    assert report.ready
    assert report.blockers == ()


def test_required_failure_blocks_profile() -> None:
    failure = ProbeResult("cube_hflow", False, "missing")
    report = ReadinessReport(
        profile=Profile.FC_BENCH,
        results=(failure,),
        required_names=frozenset({"cube_hflow"}),
    )

    assert not report.ready
    assert report.blockers == (failure,)

