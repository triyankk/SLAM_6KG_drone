from collections import deque
import time

from slam_core.lidar import LidarReader, LidarSnapshot, POINT_PACKET_LEN, extract_point_samples


def test_extract_point_samples_scales_distance_and_azimuth():
    packet = bytearray(POINT_PACKET_LEN)
    packet[0:2] = b"\xee\xff"
    packet[5] = 0
    packet[16:18] = (1234).to_bytes(2, "little")
    packet[18:20] = (250).to_bytes(2, "little")
    packet[20] = 99

    samples = extract_point_samples(bytes(packet))

    assert len(samples) == 16
    assert samples[0].azimuth_deg == 12.34
    assert samples[0].distance_m == 1.0
    assert samples[0].reflectivity == 99
    assert 0.9 < samples[0].horizontal_distance_m < 1.0
    assert samples[0].z_m < 0.0


def make_packet(distances_m):
    packet = bytearray(POINT_PACKET_LEN)
    packet[0:2] = b"\xee\xff"
    packet[5] = 0
    packet[16:18] = (1234).to_bytes(2, "little")
    for idx, distance_m in enumerate(distances_m[:16]):
        offset = 18 + idx * 3
        packet[offset : offset + 2] = int(distance_m / 0.004).to_bytes(2, "little")
        packet[offset + 2] = 50
    return bytes(packet)


def test_lidar_filter_rejects_single_channel_near_spike():
    reader = object.__new__(LidarReader)
    reader.sector_count = 72
    reader.min_valid_distance_m = 0.15
    reader.max_valid_distance_m = 40.0
    reader.min_points_per_sector = 3
    reader.recent_packet_distances_m = deque(maxlen=5)
    reader.snapshot = LidarSnapshot(sector_distances_m=[0.0] * 72)

    reader._on_point_packet(make_packet([0.20, 3.0, 3.1, 3.2] + [3.0] * 12))
    sector = int(12.34 / 360.0 * 72)

    assert reader.snapshot.min_distance_m == 0.2
    assert reader.snapshot.filtered_distance_m >= 3.0
    assert reader.snapshot.sector_distances_m[sector] > 2.8


def test_lidar_sector_requires_configured_point_count():
    reader = object.__new__(LidarReader)
    reader.sector_count = 72
    reader.min_valid_distance_m = 0.15
    reader.max_valid_distance_m = 40.0
    reader.min_points_per_sector = 3
    reader.recent_packet_distances_m = deque(maxlen=5)
    reader.snapshot = LidarSnapshot(sector_distances_m=[0.0] * 72)

    reader._on_point_packet(make_packet([0.5, 0.6] + [0.0] * 14))
    sector = int(12.34 / 360.0 * 72)

    assert reader.snapshot.sector_distances_m[sector] == 0.0


def test_lidar_sector_distance_replaced_by_fresh_farther_packet():
    reader = object.__new__(LidarReader)
    reader.sector_count = 72
    reader.min_valid_distance_m = 0.15
    reader.max_valid_distance_m = 40.0
    reader.min_points_per_sector = 3
    reader.recent_packet_distances_m = deque(maxlen=5)
    reader.snapshot = LidarSnapshot(sector_distances_m=[0.0] * 72)

    reader._on_point_packet(make_packet([0.5] * 16))
    sector = int(12.34 / 360.0 * 72)
    first_distance = reader.snapshot.sector_distances_m[sector]
    reader._on_point_packet(make_packet([2.0] * 16))

    assert first_distance < 0.6
    assert reader.snapshot.sector_distances_m[sector] > 1.9


def test_lidar_snapshot_filters_stale_sectors():
    now = time.time()
    snapshot = LidarSnapshot(
        sector_distances_m=[1.0, 2.0, 3.0],
        sector_updated_s=[now, now - 0.2, now - 2.0],
    )

    assert snapshot.fresh_sector_distances(max_age_s=0.5, now_s=now) == [1.0, 2.0, 0.0]
