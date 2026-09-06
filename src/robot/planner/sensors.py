from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
from scipy.ndimage import convolve
from sensor_msgs.msg import Imu, LaserScan

from .grid import Cell, LocalGrid


@dataclass(frozen=True)
class ScanPoint:
    x_m: float
    y_m: float
    range_m: float
    angle_rad: float


@dataclass(frozen=True)
class WallDistances:
    front: float = math.inf
    left: float = math.inf
    right: float = math.inf


@dataclass(frozen=True)
class FrontWallEstimate:
    distance_m: float = math.inf
    slope: float = 0.0
    point_count: int = 0
    valid: bool = False


@dataclass(frozen=True)
class SideWallEstimate:
    distance_m: float = math.inf
    slope: float = 0.0
    point_count: int = 0
    valid: bool = False


@dataclass(frozen=True)
class ImuReading:
    yaw_rad: float | None
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    yaw_rate: float


@dataclass(frozen=True)
class SensorFrame:
    local_grid: LocalGrid
    points: tuple[ScanPoint, ...]
    wall_distances: WallDistances
    front_wall: FrontWallEstimate = FrontWallEstimate()
    rear_wall: FrontWallEstimate = FrontWallEstimate()
    left_wall: SideWallEstimate = SideWallEstimate()
    right_wall: SideWallEstimate = SideWallEstimate()
    imu: ImuReading | None = None


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float | None:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        return None
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def imu_from_msg(msg: Imu) -> ImuReading:
    yaw = yaw_from_quat(
        float(msg.orientation.x),
        float(msg.orientation.y),
        float(msg.orientation.z),
        float(msg.orientation.w),
    )
    yaw_rate = float(msg.angular_velocity.z)
    if not math.isfinite(yaw_rate):
        yaw_rate = 0.0
    return ImuReading(
        yaw_rad=yaw,
        accel_x_g=float(msg.linear_acceleration.x),
        accel_y_g=float(msg.linear_acceleration.y),
        accel_z_g=float(msg.linear_acceleration.z),
        yaw_rate=yaw_rate,
    )


def scan_points_from_msg(
    msg: LaserScan,
    max_range_m: float = 2.2,
    min_range_m: float = 0.05,
    stride: int = 1,
) -> tuple[ScanPoint, ...]:
    msg_min = max(float(getattr(msg, "range_min", min_range_m)), min_range_m)
    msg_max = min(float(getattr(msg, "range_max", max_range_m)), max_range_m)
    stride = max(1, stride)
    ranges = np.asarray(msg.ranges, dtype=np.float64)[::stride]
    valid = np.isfinite(ranges) & (ranges >= msg_min) & (ranges <= msg_max)
    angles = float(msg.angle_min) + np.flatnonzero(valid) * stride * float(msg.angle_increment)
    ranges = ranges[valid]
    x_m = ranges * np.cos(angles)
    y_m = ranges * np.sin(angles)
    return tuple(
        ScanPoint(x, y, distance, angle)
        for x, y, distance, angle in zip(x_m.tolist(), y_m.tolist(), ranges.tolist(), angles.tolist())
    )


def wall_distances_from_points(points: Iterable[ScanPoint]) -> WallDistances:
    front = math.inf
    left = math.inf
    right = math.inf
    for point in points:
        deg = math.degrees(point.angle_rad)
        if abs(deg) <= 28:
            front = min(front, point.range_m)
        elif 45 <= deg <= 120:
            left = min(left, point.range_m)
        elif -120 <= deg <= -45:
            right = min(right, point.range_m)
    return WallDistances(front=front, left=left, right=right)


def front_wall_from_points(
    points: Iterable[ScanPoint],
    min_x_m: float = 0.18,
    max_abs_y_m: float = 0.55,
    inlier_margin_m: float = 0.08,
    min_points: int = 5,
) -> FrontWallEstimate:
    candidates = [
        point
        for point in points
        if point.x_m >= min_x_m and abs(point.y_m) <= max_abs_y_m
    ]
    if len(candidates) < min_points:
        return FrontWallEstimate()

    xs = sorted(point.x_m for point in candidates)
    median_x = xs[len(xs) // 2]
    inliers = [
        point
        for point in candidates
        if abs(point.x_m - median_x) <= inlier_margin_m
    ]
    if len(inliers) < min_points:
        return FrontWallEstimate()

    mean_y = sum(point.y_m for point in inliers) / len(inliers)
    mean_x = sum(point.x_m for point in inliers) / len(inliers)
    var_y = sum((point.y_m - mean_y) ** 2 for point in inliers)
    if var_y <= 1e-6:
        slope = 0.0
    else:
        cov_xy = sum((point.y_m - mean_y) * (point.x_m - mean_x) for point in inliers)
        slope = cov_xy / var_y
    distance = mean_x - slope * mean_y
    if not math.isfinite(distance) or distance < min_x_m:
        return FrontWallEstimate()
    return FrontWallEstimate(
        distance_m=distance,
        slope=slope,
        point_count=len(inliers),
        valid=True,
    )


def rear_wall_from_points(points: Iterable[ScanPoint]) -> FrontWallEstimate:
    candidates = [
        point for point in points
        if point.x_m < -0.08 and abs(point.y_m) < min(0.45, -0.8 * point.x_m)
    ]
    if len(candidates) < 7:
        return FrontWallEstimate()
    distance = float(np.median([-point.x_m for point in candidates]))
    inliers = [point for point in candidates if abs(-point.x_m - distance) < 0.035]
    if len(inliers) < 7 or max(point.y_m for point in inliers) - min(point.y_m for point in inliers) < 0.16:
        return FrontWallEstimate()
    return FrontWallEstimate(distance_m=distance, point_count=len(inliers), valid=True)


def side_wall_from_points(
    points: Iterable[ScanPoint],
    side: str,
    min_abs_y_m: float = 0.18,
    max_abs_x_m: float = 0.75,
    inlier_margin_m: float = 0.08,
    min_points: int = 5,
) -> SideWallEstimate:
    want_left = side == "left"
    candidates = [
        point
        for point in points
        if abs(point.x_m) <= max_abs_x_m
        and ((point.y_m >= min_abs_y_m) if want_left else (point.y_m <= -min_abs_y_m))
    ]
    if len(candidates) < min_points:
        return SideWallEstimate()

    ys = sorted(point.y_m for point in candidates)
    median_y = ys[len(ys) // 2]
    inliers = [
        point
        for point in candidates
        if abs(point.y_m - median_y) <= inlier_margin_m
    ]
    if len(inliers) < min_points:
        return SideWallEstimate()

    mean_x = sum(point.x_m for point in inliers) / len(inliers)
    mean_y = sum(point.y_m for point in inliers) / len(inliers)
    var_x = sum((point.x_m - mean_x) ** 2 for point in inliers)
    if var_x <= 1e-6:
        slope = 0.0
    else:
        cov_xy = sum((point.x_m - mean_x) * (point.y_m - mean_y) for point in inliers)
        slope = cov_xy / var_x
    intercept = mean_y - slope * mean_x
    distance = abs(intercept)
    if not math.isfinite(distance) or distance < min_abs_y_m:
        return SideWallEstimate()
    if want_left and intercept <= 0.0:
        return SideWallEstimate()
    if not want_left and intercept >= 0.0:
        return SideWallEstimate()
    return SideWallEstimate(
        distance_m=distance,
        slope=slope,
        point_count=len(inliers),
        valid=True,
    )


def local_grid_from_scan(
    msg: LaserScan,
    size_m: float = 1.5,
    resolution_m: float = 0.05,
    max_range_m: float = 2.2,
    wall_margin_m: float = 0.12,
    stride: int = 1,
) -> SensorFrame:
    points = scan_points_from_msg(
        msg,
        max_range_m=max_range_m,
        stride=stride,
    )
    walls = wall_distances_from_points(points)
    front_wall = front_wall_from_points(points)
    rear_wall = rear_wall_from_points(points)
    left_wall = side_wall_from_points(points, "left")
    right_wall = side_wall_from_points(points, "right")
    local = LocalGrid(size_m=size_m, resolution_m=resolution_m)
    _ = wall_margin_m
    local.mark_free_rays(
        (point.x_m, point.y_m)
        for point in points
        if abs(point.x_m) <= size_m * 0.5 and abs(point.y_m) <= size_m * 0.5
    )
    endpoints = _supported_endpoints(local, points, size_m)
    for x_m, y_m, endpoint_value in endpoints:
        local.mark_scan_points(((x_m, y_m),), endpoint_value)
    mark_wall_runs(local, points, front_wall, left_wall, right_wall)
    return SensorFrame(
        local_grid=local,
        points=points,
        wall_distances=walls,
        front_wall=front_wall,
        rear_wall=rear_wall,
        left_wall=left_wall,
        right_wall=right_wall,
    )


def _supported_endpoints(
    local: LocalGrid,
    points: tuple[ScanPoint, ...],
    size_m: float,
) -> tuple[tuple[float, float, Cell], ...]:
    if not points:
        return ()
    xy = np.asarray([(point.x_m, point.y_m) for point in points], dtype=np.float64)
    xy = xy[np.all(np.abs(xy) <= size_m * 0.5, axis=1)]
    rows = local.origin_cell - np.rint(xy[:, 0] / local.resolution_m).astype(np.intp)
    cols = local.origin_cell + np.rint(xy[:, 1] / local.resolution_m).astype(np.intp)
    inside = (rows >= 0) & (rows < local.size_cells) & (cols >= 0) & (cols < local.size_cells)
    indices = rows[inside] * local.size_cells + cols[inside]
    xy = xy[inside]
    counts = np.bincount(indices, minlength=local.cells.size)
    support = np.asarray(convolve(
        counts.reshape(local.cells.shape), np.ones((3, 3), dtype=np.int64), mode="constant",
    ), dtype=np.int64).ravel()
    supported = (counts > 0) & (support >= 2)
    x_sum = np.bincount(indices, weights=xy[:, 0], minlength=local.cells.size)
    y_sum = np.bincount(indices, weights=xy[:, 1], minlength=local.cells.size)
    return tuple(
        (x, y, Cell.UNKNOWN_OBSTRUCTION)
        for x, y in zip((x_sum[supported] / counts[supported]).tolist(),
                        (y_sum[supported] / counts[supported]).tolist())
    )


def mark_wall_runs(
    local: LocalGrid,
    points: tuple[ScanPoint, ...],
    front_wall: FrontWallEstimate,
    left_wall: SideWallEstimate,
    right_wall: SideWallEstimate,
) -> None:
    if not points or not (front_wall.valid or left_wall.valid or right_wall.valid):
        return
    x = np.asarray([p.x_m for p in points], dtype=np.float64)
    y = np.asarray([p.y_m for p in points], dtype=np.float64)
    wall_points = np.zeros(len(points), dtype=np.bool_)
    for valid, matches in (
        (front_wall.valid, (x >= 0.0) & (np.abs(x - (front_wall.distance_m + front_wall.slope * y)) <= 0.10)),
        (left_wall.valid, (y >= 0.0) & (np.abs(y - (left_wall.distance_m + left_wall.slope * x)) <= 0.10)),
        (right_wall.valid, (y <= 0.0) & (np.abs(y - (-right_wall.distance_m + right_wall.slope * x)) <= 0.10)),
    ):
        if not valid:
            continue
        indices = np.flatnonzero(matches)
        if len(indices) < 6:
            continue
        starts = np.r_[0, np.flatnonzero(np.diff(indices) > 3) + 1]
        ends = np.r_[starts[1:], len(indices)] - 1
        counts = ends - starts + 1
        span = np.hypot(x[indices[ends]] - x[indices[starts]],
                        y[indices[ends]] - y[indices[starts]])
        accepted = (counts >= 6) & (span >= 0.22)
        wall_points[indices[np.repeat(accepted, counts)]] = True
    local.mark_scan_points(zip(x[wall_points].tolist(), y[wall_points].tolist()), Cell.WALL)
