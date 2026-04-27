from collections import deque

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
    reader.recent_packet_distances_m = deque(maxlen=5)
    reader.snapshot = LidarSnapshot(sector_distances_m=[0.0] * 72)

    reader._on_point_packet(make_packet([0.20, 3.0, 3.1, 3.2] + [3.0] * 12))

    assert reader.snapshot.min_distance_m == 0.2
    assert reader.snapshot.filtered_distance_m >= 3.0
