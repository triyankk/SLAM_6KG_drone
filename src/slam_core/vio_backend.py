import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import pyrealsense2 as rs

from .types import PoseSample


def _rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> tuple[float, float, float, float]:
    matrix = rotation_matrix
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        if (matrix[0, 0] > matrix[1, 1]) and (matrix[0, 0] > matrix[2, 2]):
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif matrix[1, 1] > matrix[2, 2]:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    return float(qw), float(qx), float(qy), float(qz)


@dataclass
class VioPoseSource:
    width: int = 640
    height: int = 480
    fps: int = 30
    max_features: int = 500
    min_features: int = 120
    min_tracked_points: int = 40
    max_speed_m_s: float = 5.0
    feature_border_ratio: float = 0.12
    stationary_flow_px: float = 0.35
    stationary_translation_m: float = 0.012
    close_range_flow_blend_m: float = 0.8
    external_height_m: float | None = None

    def __post_init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
        try:
            self.profile = self.pipeline.start(config)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"Failed to start RealSense pipeline: {exc}. Is another process using the camera?"
            ) from exc

        frames = self.pipeline.wait_for_frames(timeout_ms=2000)
        depth = frames.get_depth_frame()
        infrared = frames.get_infrared_frame(1)
        intrinsics = infrared.get_profile().as_video_stream_profile().get_intrinsics()
        self.camera_matrix = np.array(
            [[intrinsics.fx, 0.0, intrinsics.ppx], [0.0, intrinsics.fy, intrinsics.ppy], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )

        frame_ts_ms = frames.get_timestamp()
        monotonic_ns = time.time_ns()
        self.frame_to_monotonic_offset_ns = monotonic_ns - int(frame_ts_ms * 1e6)

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.feature_mask = self._build_feature_mask(self.height, self.width)
        self.prev_gray = self._preprocess_gray(np.asanyarray(infrared.get_data()))
        self.prev_depth = depth
        self.prev_pts = self._detect_points(self.prev_gray)

        self.pose = np.eye(4, dtype=np.float64)
        self.last_time = monotonic_ns / 1e9
        self.last_velocity = np.zeros(3, dtype=np.float64)
        self.last_output = self._sample_from_state(
            timestamp_us=int(monotonic_ns // 1000),
            tracking_state="warmup",
            feature_count=len(self.prev_pts),
        )

    def _detect_points(self, gray: np.ndarray) -> np.ndarray:
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_features,
            qualityLevel=0.02,
            minDistance=9,
            blockSize=7,
            mask=self.feature_mask,
        )
        if points is None:
            return np.empty((0, 1, 2), dtype=np.float32)
        return points

    def _build_feature_mask(self, height: int, width: int) -> np.ndarray:
        mask = np.zeros((height, width), dtype=np.uint8)
        margin_x = int(width * self.feature_border_ratio)
        margin_y = int(height * self.feature_border_ratio)
        mask[margin_y : height - margin_y, margin_x : width - margin_x] = 255
        return mask

    def _preprocess_gray(self, gray: np.ndarray) -> np.ndarray:
        processed = self.clahe.apply(gray)
        return cv2.GaussianBlur(processed, (5, 5), 0)

    def _get_3d_for_pts(self, points: np.ndarray, depth_frame: rs.frame):
        points3d = []
        for point in points.reshape(-1, 2):
            x = int(round(point[0]))
            y = int(round(point[1]))
            if x < 0 or y < 0 or x >= self.width or y >= self.height:
                points3d.append(None)
                continue
            depth_m = depth_frame.get_distance(x, y)
            if depth_m <= 0.001:
                points3d.append(None)
                continue
            pos_x = ((x - self.camera_matrix[0, 2]) * depth_m) / self.camera_matrix[0, 0]
            pos_y = ((y - self.camera_matrix[1, 2]) * depth_m) / self.camera_matrix[1, 1]
            pos_z = depth_m
            points3d.append((pos_x, pos_y, pos_z))
        return points3d

    def set_external_height_m(self, height_m: float | None) -> None:
        if height_m is None or height_m <= 0.0:
            self.external_height_m = None
            return
        self.external_height_m = float(height_m)

    def _quality(self, feature_count: int, tracked_count: int, inlier_count: int) -> int:
        if tracked_count <= 0:
            return 0
        feature_term = min(1.0, feature_count / max(float(self.min_features), 1.0))
        tracked_term = min(1.0, tracked_count / max(float(self.min_tracked_points), 1.0))
        inlier_term = min(1.0, inlier_count / max(float(self.min_tracked_points), 1.0))
        ratio_term = min(1.0, inlier_count / max(float(tracked_count), 1.0))
        score = 0.2 * feature_term + 0.3 * tracked_term + 0.3 * inlier_term + 0.2 * ratio_term
        return int(max(0, min(100, round(score * 100.0))))

    def _sample_from_state(
        self,
        timestamp_us: int,
        tracking_state: str,
        feature_count: int,
        tracked_feature_count: int = 0,
        inlier_count: int = 0,
        velocity: np.ndarray | None = None,
        pose_quality: int | None = None,
    ) -> PoseSample:
        rotation = self.pose[:3, :3]
        translation = self.pose[:3, 3]
        qw, qx, qy, qz = _rotation_matrix_to_quaternion(rotation)
        if velocity is None:
            velocity = self.last_velocity
        if pose_quality is None:
            pose_quality = self._quality(feature_count, tracked_feature_count, inlier_count)
        sample = PoseSample(
            timestamp_us=timestamp_us,
            x_m=float(translation[0]),
            y_m=float(translation[1]),
            z_m=float(translation[2]),
            qw=qw,
            qx=qx,
            qy=qy,
            qz=qz,
            vx_m_s=float(velocity[0]),
            vy_m_s=float(velocity[1]),
            vz_m_s=float(velocity[2]),
            pose_quality=pose_quality,
            tracking_state=tracking_state,
            feature_count=int(feature_count),
            tracked_feature_count=int(tracked_feature_count),
            inlier_count=int(inlier_count),
            source_name="vio",
        )
        self.last_output = sample
        return sample

    def _reset_tracking(self, gray: np.ndarray, depth_frame: rs.frame) -> int:
        self.prev_gray = gray
        self.prev_depth = depth_frame
        self.prev_pts = self._detect_points(gray)
        return int(len(self.prev_pts))

    def close(self) -> None:
        self.pipeline.stop()

    def reset_origin(self) -> None:
        self.pose = np.eye(4, dtype=np.float64)
        self.last_velocity = np.zeros(3, dtype=np.float64)

    def _flow_only_translation(
        self,
        flow_vectors: np.ndarray,
        height_m: float,
    ) -> np.ndarray:
        median_dx_px = float(np.median(flow_vectors[:, 0]))
        median_dy_px = float(np.median(flow_vectors[:, 1]))
        return np.array(
            [
                -median_dx_px * height_m / float(self.camera_matrix[0, 0]),
                -median_dy_px * height_m / float(self.camera_matrix[1, 1]),
                0.0,
            ],
            dtype=np.float64,
        )

    def sample(self) -> PoseSample:
        frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        depth = frames.get_depth_frame()
        infrared = frames.get_infrared_frame(1)
        gray = self._preprocess_gray(np.asanyarray(infrared.get_data()))
        frame_ts_ms = frames.get_timestamp()
        now_monotonic_ns = int(frame_ts_ms * 1e6) + int(self.frame_to_monotonic_offset_ns)
        now_s = now_monotonic_ns / 1e9
        timestamp_us = int(now_monotonic_ns // 1000)
        dt_s = max(1e-6, now_s - self.last_time)

        previous_feature_count = int(len(self.prev_pts))
        if previous_feature_count < self.min_features:
            feature_count = self._reset_tracking(gray, depth)
            self.last_velocity = np.zeros(3, dtype=np.float64)
            self.last_time = now_s
            return self._sample_from_state(
                timestamp_us=timestamp_us,
                tracking_state="warmup",
                feature_count=feature_count,
                pose_quality=min(30, self._quality(feature_count, 0, 0)),
            )

        tracked_points, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None)
        if tracked_points is None or status is None:
            feature_count = self._reset_tracking(gray, depth)
            self.last_velocity = np.zeros(3, dtype=np.float64)
            self.last_time = now_s
            return self._sample_from_state(
                timestamp_us=timestamp_us,
                tracking_state="optflow_fail",
                feature_count=feature_count,
                pose_quality=0,
            )

        valid_mask = status.flatten() == 1
        previous_points = self.prev_pts[valid_mask].reshape(-1, 2)
        current_points = tracked_points[valid_mask].reshape(-1, 2)
        tracked_count = int(len(previous_points))
        flow_vectors = current_points - previous_points if tracked_count > 0 else np.empty((0, 2), dtype=np.float32)
        median_flow_px = (
            float(np.median(np.linalg.norm(flow_vectors, axis=1)))
            if tracked_count > 0
            else 0.0
        )
        if tracked_count < self.min_tracked_points:
            feature_count = self._reset_tracking(gray, depth)
            self.last_velocity = np.zeros(3, dtype=np.float64)
            self.last_time = now_s
            return self._sample_from_state(
                timestamp_us=timestamp_us,
                tracking_state="low_tracks",
                feature_count=feature_count,
                tracked_feature_count=tracked_count,
                pose_quality=self._quality(feature_count, tracked_count, 0),
            )

        points3d = self._get_3d_for_pts(previous_points, self.prev_depth)
        object_points = []
        image_points = []
        for point3d, point2d in zip(points3d, current_points):
            if point3d is None:
                continue
            object_points.append(point3d)
            image_points.append((float(point2d[0]), float(point2d[1])))

        valid_depths = [point3d[2] for point3d in points3d if point3d is not None]
        fallback_height_m = self.external_height_m
        if fallback_height_m is None and valid_depths:
            fallback_height_m = float(np.median(valid_depths))

        if len(object_points) < self.min_tracked_points:
            if tracked_count >= self.min_tracked_points and fallback_height_m is not None and fallback_height_m > 0.0:
                planar_translation = self._flow_only_translation(flow_vectors, fallback_height_m)
                translation_step_m = float(np.linalg.norm(planar_translation[:2]))
                if median_flow_px <= self.stationary_flow_px and translation_step_m <= self.stationary_translation_m:
                    feature_count = self._reset_tracking(gray, depth)
                    self.last_velocity = self.last_velocity * 0.25
                    self.last_time = now_s
                    return self._sample_from_state(
                        timestamp_us=timestamp_us,
                        tracking_state="ok_hold",
                        feature_count=feature_count,
                        tracked_feature_count=tracked_count,
                        velocity=np.zeros(3, dtype=np.float64),
                        pose_quality=self._quality(feature_count, tracked_count, 0),
                    )

                transform_inverse = np.eye(4, dtype=np.float64)
                transform_inverse[:3, 3] = planar_translation
                previous_translation = self.pose[:3, 3].copy()
                self.pose = self.pose.dot(transform_inverse)
                candidate_translation = self.pose[:3, 3]
                raw_velocity = (candidate_translation - previous_translation) / dt_s
                self.last_velocity = 0.7 * self.last_velocity + 0.3 * raw_velocity
                feature_count = self._reset_tracking(gray, depth)
                self.last_time = now_s
                return self._sample_from_state(
                    timestamp_us=timestamp_us,
                    tracking_state="ok_flow2d",
                    feature_count=feature_count,
                    tracked_feature_count=tracked_count,
                    velocity=self.last_velocity,
                    pose_quality=self._quality(feature_count, tracked_count, 0),
                )

            feature_count = self._reset_tracking(gray, depth)
            self.last_velocity = np.zeros(3, dtype=np.float64)
            self.last_time = now_s
            return self._sample_from_state(
                timestamp_us=timestamp_us,
                tracking_state="low_depth_support",
                feature_count=feature_count,
                tracked_feature_count=tracked_count,
                pose_quality=self._quality(feature_count, tracked_count, 0),
            )

        object_points_np = np.array(object_points, dtype=np.float32)
        image_points_np = np.array(image_points, dtype=np.float32)
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points_np,
            image_points_np,
            self.camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=8.0,
            iterationsCount=100,
            confidence=0.99,
        )
        inlier_count = 0 if inliers is None else int(len(inliers))
        if not success or inlier_count < self.min_tracked_points:
            feature_count = self._reset_tracking(gray, depth)
            self.last_velocity = np.zeros(3, dtype=np.float64)
            self.last_time = now_s
            return self._sample_from_state(
                timestamp_us=timestamp_us,
                tracking_state="pnp_reject",
                feature_count=feature_count,
                tracked_feature_count=tracked_count,
                inlier_count=inlier_count,
                pose_quality=self._quality(feature_count, tracked_count, inlier_count),
            )

        rotation_step, _ = cv2.Rodrigues(rvec)
        translation_step = tvec.reshape(3)

        inverse_rotation = rotation_step.T
        inverse_translation = -rotation_step.T.dot(translation_step)
        median_depth_m = float(np.median(object_points_np[:, 2])) if len(object_points_np) > 0 else 0.0
        if tracked_count > 0 and median_depth_m > 0.0:
            median_dx_px = float(np.median(flow_vectors[:, 0]))
            median_dy_px = float(np.median(flow_vectors[:, 1]))
            flow_translation = np.array(
                [
                    -median_dx_px * median_depth_m / float(self.camera_matrix[0, 0]),
                    -median_dy_px * median_depth_m / float(self.camera_matrix[1, 1]),
                    inverse_translation[2],
                ],
                dtype=np.float64,
            )
            flow_blend = 0.7 if median_depth_m <= self.close_range_flow_blend_m else 0.35
            inverse_translation = (1.0 - flow_blend) * inverse_translation + flow_blend * flow_translation

        transform_inverse = np.eye(4, dtype=np.float64)
        transform_inverse[:3, :3] = inverse_rotation
        transform_inverse[:3, 3] = inverse_translation
        new_pose = self.pose.dot(transform_inverse)

        previous_translation = self.pose[:3, 3].copy()
        candidate_translation = new_pose[:3, 3]
        translation_jump = float(np.linalg.norm(candidate_translation - previous_translation))
        max_jump_m = self.max_speed_m_s * dt_s * 2.0
        if translation_jump > max_jump_m:
            feature_count = self._reset_tracking(gray, depth)
            self.last_velocity = np.zeros(3, dtype=np.float64)
            self.last_time = now_s
            return self._sample_from_state(
                timestamp_us=timestamp_us,
                tracking_state="jump_reject",
                feature_count=feature_count,
                tracked_feature_count=tracked_count,
                inlier_count=inlier_count,
                pose_quality=self._quality(feature_count, tracked_count, inlier_count),
            )

        if median_flow_px <= self.stationary_flow_px and translation_jump <= self.stationary_translation_m:
            feature_count = self._reset_tracking(gray, depth)
            self.last_velocity = self.last_velocity * 0.25
            self.last_time = now_s
            return self._sample_from_state(
                timestamp_us=timestamp_us,
                tracking_state="ok_hold",
                feature_count=feature_count,
                tracked_feature_count=tracked_count,
                inlier_count=inlier_count,
                velocity=np.zeros(3, dtype=np.float64),
                pose_quality=self._quality(feature_count, tracked_count, inlier_count),
            )

        self.pose = new_pose
        raw_velocity = (candidate_translation - previous_translation) / dt_s
        self.last_velocity = 0.6 * self.last_velocity + 0.4 * raw_velocity

        feature_count = self._reset_tracking(gray, depth)
        self.last_time = now_s
        return self._sample_from_state(
            timestamp_us=timestamp_us,
            tracking_state="ok",
            feature_count=feature_count,
            tracked_feature_count=tracked_count,
            inlier_count=inlier_count,
            velocity=self.last_velocity,
            pose_quality=self._quality(feature_count, tracked_count, inlier_count),
        )

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
