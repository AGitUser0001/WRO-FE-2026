from __future__ import annotations

import math
from dataclasses import dataclass


def angle_diff(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


@dataclass
class PoseEstimate:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = math.pi / 2.0
    raw_yaw_rad: float | None = None
    yaw_bias_rad: float = 0.0
    confidence: float = 0.0
    alignment_score: float = 0.0
    front_wall_motion_m: float = 0.0
    front_wall_distance_m: float = math.inf
    front_wall_points: int = 0
    wall_pose_correction_x_m: float = 0.0
    wall_pose_correction_y_m: float = 0.0
    wall_pose_sources: int = 0
    wall_yaw_correction_rad: float = 0.0
    wall_yaw_sources: int = 0
