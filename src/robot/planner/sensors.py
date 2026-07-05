from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable

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
    points = []
    msg_min = max(float(getattr(msg, "range_min", min_range_m)), min_range_m)
    msg_max = min(float(getattr(msg, "range_max", max_range_m)), max_range_m)
    stride = max(1, stride)
    for i in range(0, len(msg.ranges), stride):
        rng = float(msg.ranges[i])
        if not math.isfinite(rng) or rng < msg_min or rng > msg_max:
            continue
        angle = float(msg.angle_min) + i * float(msg.angle_increment)
        points.append(
            ScanPoint(
                x_m=rng * math.cos(angle),
                y_m=rng * math.sin(angle),
                range_m=rng,
                angle_rad=angle,
            )
        )
    return tuple(points)


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
    left_wall = side_wall_from_points(points, "left")
    right_wall = side_wall_from_points(points, "right")
    local = LocalGrid(size_m=size_m, resolution_m=resolution_m)
    _ = wall_margin_m
    endpoints = [
        (point.x_m, point.y_m, Cell.UNKNOWN_OBSTRUCTION)
        for point in points
        if abs(point.x_m) <= size_m * 0.5 and abs(point.y_m) <= size_m * 0.5
    ]
    local.mark_lidar_points(endpoints)
    mark_wall_runs(local, points, front_wall, left_wall, right_wall)
    return SensorFrame(
        local_grid=local,
        points=points,
        wall_distances=walls,
        front_wall=front_wall,
        left_wall=left_wall,
        right_wall=right_wall,
    )


def mark_wall_runs(
    local: LocalGrid,
    points: tuple[ScanPoint, ...],
    front_wall: FrontWallEstimate,
    left_wall: SideWallEstimate,
    right_wall: SideWallEstimate,
) -> None:
    if front_wall.valid:
        mark_point_run(local, points, lambda p: p.x_m >= 0.0 and abs(p.x_m - (front_wall.distance_m + front_wall.slope * p.y_m)) <= 0.06)
    if left_wall.valid:
        mark_point_run(local, points, lambda p: p.y_m >= 0.0 and abs(p.y_m - (left_wall.distance_m + left_wall.slope * p.x_m)) <= 0.06)
    if right_wall.valid:
        mark_point_run(local, points, lambda p: p.y_m <= 0.0 and abs(p.y_m - (-right_wall.distance_m + right_wall.slope * p.x_m)) <= 0.06)


def mark_point_run(
    local: LocalGrid,
    points: tuple[ScanPoint, ...],
    matches_wall: Callable[[ScanPoint], bool],
    min_points: int = 6,
    min_span_m: float = 0.22,
) -> None:
    run: list[ScanPoint] = []
    for point in (*points, ScanPoint(math.inf, math.inf, math.inf, math.inf)):
        if matches_wall(point):
            run.append(point)
            continue
        if len(run) >= min_points and _run_span(run) >= min_span_m:
            local.mark_scan_points(((p.x_m, p.y_m) for p in run), Cell.WALL)
        run = []


def _run_span(points: list[ScanPoint]) -> float:
    first = points[0]
    last = points[-1]
    return math.hypot(last.x_m - first.x_m, last.y_m - first.y_m)
