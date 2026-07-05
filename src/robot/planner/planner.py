from __future__ import annotations

import math
from dataclasses import dataclass

from .dstar_lite import DStarLite
from .grid import Direction, GridMap
from .graph_path import graph_waypoints
from .local_grid import LocalGrid
from .localize_types import PoseEstimate
from .trajectory import TrajectoryMixin


@dataclass(frozen=True)
class Plan:
    waypoints: tuple[tuple[float, float], ...]
    trajectory: tuple[tuple[float, float], ...]
    target: tuple[float, float] | None
    steering_angle_rad: float
    motion_direction: int
    reason: str


class GridPlanner(TrajectoryMixin):
    def __init__(
        self,
        grid: GridMap,
        safety_buffer_m: float = 0.15,
        lookahead_m: float = 1.2,
        wheelbase_m: float = 0.138,
        max_steering_angle_rad: float = 0.4188,
        robot_length_m: float = 0.25,
        robot_width_m: float = 0.15,
        trajectory_steps: int = 18,
        trajectory_length_m: float = 1.5,
        trajectory_step_m: float = 0.05,
        command_delay_s: float = 0.5,
        planning_speed_mps: float = 0.35,
    ):
        self.grid = grid
        self.safety_buffer_m = safety_buffer_m
        self.lookahead_m = lookahead_m
        self.wheelbase_m = max(wheelbase_m, 0.01)
        self.max_steering_angle_rad = max(max_steering_angle_rad, 0.01)
        self.robot_length_m = max(robot_length_m, 0.01)
        self.robot_width_m = max(robot_width_m, 0.01)
        self.trajectory_step_m = max(trajectory_step_m, 0.01)
        self.trajectory_length_m = max(trajectory_length_m, self.trajectory_step_m)
        self.trajectory_steps = max(2, trajectory_steps)
        self.command_delay_s = max(0.0, command_delay_s)
        self.planning_speed_mps = max(0.01, planning_speed_mps)
        self._traversable_cache: dict[tuple[int, int], bool] = {}
        self._footprint_local_cache: dict[float, tuple[tuple[float, float], ...]] = {}
        self._local_collision_grid: object | None = None
        self._local_collision_origin: tuple[float, float, float] | None = None
        self._last_steering_rad = 0.0
        self._last_motion_direction = 1
        self._last_variant_offset_m = 0.0
        self._last_trajectory: tuple[tuple[float, float], ...] = ()
        self._route_target_index: int | None = None
        self._dstar_state: DStarLite | None = None
        self._motion_blocks: list[tuple[float, float, int]] = []

    def plan(
        self,
        pose: PoseEstimate,
        motion_direction: int = 1,
        preferred_direction: Direction = Direction.UNKNOWN,
        local_grid: LocalGrid | None = None,
    ) -> Plan:
        self._traversable_cache.clear()
        self._decay_motion_blocks()
        self._local_collision_grid = local_grid
        self._local_collision_origin = (pose.x_m, pose.y_m, pose.yaw_rad) if local_grid is not None else None
        try:
            plan = self._plan_direction(pose, motion_direction, preferred_direction)
            self._last_steering_rad = plan.steering_angle_rad
            self._last_motion_direction = plan.motion_direction
            self._last_trajectory = plan.trajectory
            return plan
        finally:
            self._local_collision_grid = None
            self._local_collision_origin = None

    def note_stalled_trajectory(self, trajectory: tuple[tuple[float, float], ...]) -> None:
        if len(trajectory) < 6:
            return
        for x_m, y_m in trajectory[2:min(len(trajectory), 24):2]:
            self._motion_blocks.append((x_m, y_m, 40))
        if len(self._motion_blocks) > 96:
            self._motion_blocks = self._motion_blocks[-96:]

    def motion_block_points(self) -> tuple[tuple[float, float], ...]:
        return tuple((x_m, y_m) for x_m, y_m, _ttl in self._motion_blocks)

    def _decay_motion_blocks(self) -> None:
        self._motion_blocks = [(x_m, y_m, ttl - 1) for x_m, y_m, ttl in self._motion_blocks if ttl > 1]

    def _plan_direction(
        self,
        pose: PoseEstimate,
        motion_direction: int,
        preferred_direction: Direction,
    ) -> Plan:
        if self.grid.direction == Direction.UNKNOWN:
            return self._corridor_plan(pose, motion_direction, preferred_direction)
        return self._grid_plan(pose, motion_direction)

    def _corridor_plan(
        self,
        pose: PoseEstimate,
        motion_direction: int,
        preferred_direction: Direction,
    ) -> Plan:
        points = ((0.0, 1.0),)
        target = points[-1] if points else None
        trajectory, steering, first_direction = self._simulate_trajectory(pose, points, motion_direction)
        self._last_variant_offset_m = 0.0
        return Plan(
            waypoints=points,
            trajectory=trajectory,
            target=target,
            steering_angle_rad=steering,
            motion_direction=first_direction,
            reason="corridor",
        )

    def _grid_plan(self, pose: PoseEstimate, motion_direction: int) -> Plan:
        route_points = self._route_points(pose)
        base_waypoints = graph_waypoints(self, pose, route_points, self._route_block_points())
        escape = self._escape_waypoint(pose)
        if escape is not None:
            base_waypoints = (escape, *base_waypoints)
        base_target = base_waypoints[-1] if base_waypoints else None
        base_trajectory, base_steering, base_direction = self._simulate_trajectory(
            pose, base_waypoints, motion_direction,
        )
        base_plan = Plan(
            waypoints=base_waypoints,
            trajectory=base_trajectory,
            target=base_target,
            steering_angle_rad=base_steering,
            motion_direction=base_direction,
            reason="route",
        )
        self._last_variant_offset_m = 0.0
        return base_plan

    def _route_points(
        self,
        pose: PoseEstimate,
    ) -> tuple[tuple[float, float], ...]:
        centers = self._course_section_centers()
        self._update_route_target(centers, pose)
        idx = self._route_target_index or 0
        return (centers[idx],)

    def _route_block_points(self) -> tuple[tuple[float, float], ...]:
        centers = self._course_section_centers()
        if self._route_target_index is None or len(centers) < 4:
            return ()
        return (centers[(self._route_target_index + 2) % len(centers)],)

    def _escape_waypoint(self, pose: PoseEstimate) -> tuple[float, float] | None:
        blocks = [
            (x_m - pose.x_m, y_m - pose.y_m)
            for x_m, y_m in self.motion_block_points()
            if math.hypot(x_m - pose.x_m, y_m - pose.y_m) < 0.65
        ]
        if len(blocks) < 3:
            return None
        vx = sum(dx / max(math.hypot(dx, dy), 0.05) for dx, dy in blocks)
        vy = sum(dy / max(math.hypot(dx, dy), 0.05) for dx, dy in blocks)
        length = math.hypot(vx, vy)
        if length < 1e-4:
            return None
        away = (-vx / length, -vy / length)
        candidates = self._escape_candidates(pose, away)
        return min(candidates, key=lambda p: self._escape_cost(pose, p, blocks), default=None)

    def _escape_candidates(
        self,
        pose: PoseEstimate,
        away: tuple[float, float],
    ) -> tuple[tuple[float, float], ...]:
        out: list[tuple[float, float]] = []
        back = (-math.cos(pose.yaw_rad), -math.sin(pose.yaw_rad))
        blend_len = math.hypot(away[0] + back[0], away[1] + back[1])
        bases = [away, back]
        if blend_len > 1e-4:
            bases.append(((away[0] + back[0]) / blend_len, (away[1] + back[1]) / blend_len))
        angles = (0.0, 0.45, -0.45, 0.9, -0.9)
        seen: set[tuple[int, int]] = set()
        for base in bases:
            for dist in (0.28, 0.40, 0.55):
                for angle in angles:
                    ca = math.cos(angle); sa = math.sin(angle)
                    dx = base[0] * ca - base[1] * sa
                    dy = base[0] * sa + base[1] * ca
                    point = (pose.x_m + dx * dist, pose.y_m + dy * dist)
                    cell = self.grid.world_to_cell(point[0], point[1])
                    key = (round(point[0] / 0.05), round(point[1] / 0.05))
                    if cell is not None and key not in seen and self._is_traversable(cell[0], cell[1]):
                        seen.add(key)
                        out.append(point)
        return tuple(out)

    def _escape_cost(
        self,
        pose: PoseEstimate,
        point: tuple[float, float],
        blocks: list[tuple[float, float]],
    ) -> float:
        cx = math.cos(pose.yaw_rad); sy = math.sin(pose.yaw_rad)
        dx = point[0] - pose.x_m; dy = point[1] - pose.y_m
        clearance = min((math.hypot(point[0] - pose.x_m - bx, point[1] - pose.y_m - by) for bx, by in blocks), default=0.0)
        route = self._course_centerline()
        progress = self._nearest_route_progress(route, point[0], point[1])
        current = self._nearest_route_progress(route, pose.x_m, pose.y_m)
        signed = dx * cx + dy * sy
        front_blocks = sum(1 for bx, by in blocks if bx * cx + by * sy > 0.02)
        direction_weight = 1.25 if front_blocks >= max(2, len(blocks) // 2) else 0.25
        forward_penalty = 0.8 if front_blocks and signed > -0.03 else 0.0
        return -clearance * 2.0 - abs(progress - current) * 0.08 + signed * direction_weight + forward_penalty

    def _course_centerline(self) -> tuple[tuple[float, float], ...]:
        centers = self._course_section_centers()
        return (*centers, centers[0])

    def _course_section_centers(self) -> tuple[tuple[float, float], ...]:
        middle_x = -1.0 if self.grid.direction == Direction.LEFT else 1.0
        side_x = -2.0 if self.grid.direction == Direction.LEFT else 2.0
        return ((0.0, 0.0), (middle_x, 1.0), (side_x, 0.0), (middle_x, -1.0))

    def _update_route_target(self, centers: tuple[tuple[float, float], ...], pose: PoseEstimate) -> None:
        if not centers:
            self._route_target_index = None
            return
        if self._route_target_index is None:
            self._route_target_index = self._initial_route_target(centers, pose)
        for _ in range(len(centers)):
            target = centers[self._route_target_index]
            if math.hypot(target[0] - pose.x_m, target[1] - pose.y_m) >= 0.18:
                break
            self._route_target_index = (self._route_target_index + 1) % len(centers)

    def _initial_route_target(self, centers: tuple[tuple[float, float], ...], pose: PoseEstimate) -> int:
        route = (*centers, centers[0])
        probe = self._route_point_at(route, self._nearest_route_progress(route, pose.x_m, pose.y_m) + 0.75)
        return min(range(len(centers)), key=lambda i: math.hypot(centers[i][0] - probe[0], centers[i][1] - probe[1]))

    def _nearest_route_progress(self, route: tuple[tuple[float, float], ...], x_m: float, y_m: float) -> float:
        best_progress = 0.0
        best_dist = math.inf
        progress = 0.0
        for a, b in zip(route, route[1:]):
            vx, vy = b[0] - a[0], b[1] - a[1]
            seg_len = max(math.hypot(vx, vy), 1e-6)
            t = max(0.0, min(1.0, ((x_m - a[0]) * vx + (y_m - a[1]) * vy) / (seg_len * seg_len)))
            px, py = a[0] + vx * t, a[1] + vy * t
            dist = math.hypot(x_m - px, y_m - py)
            if dist < best_dist:
                best_dist = dist
                best_progress = progress + seg_len * t
            progress += seg_len
        return best_progress

    def _route_point_at(self, route: tuple[tuple[float, float], ...], progress: float) -> tuple[float, float]:
        total = self._route_length(route)
        progress %= max(total, 1e-6)
        for a, b in zip(route, route[1:]):
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            if progress <= seg_len:
                t = progress / max(seg_len, 1e-6)
                return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            progress -= seg_len
        return route[-1]

    @staticmethod
    def _route_length(route: tuple[tuple[float, float], ...]) -> float:
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(route, route[1:]))

    def _trajectory_route_progress(self, trajectory: tuple[tuple[float, float], ...]) -> float:
        route = self._course_centerline()
        total = max(self._route_length(route), 1e-6)
        start = self._nearest_route_progress(route, trajectory[0][0], trajectory[0][1])
        end = self._nearest_route_progress(route, trajectory[-1][0], trajectory[-1][1])
        delta = (end - start) % total
        return delta - total if delta > total * 0.5 else delta

    def _safe_guide_points(
        self,
        pose: PoseEstimate,
        points: tuple[tuple[float, float], ...],
        buffer_m: float | None = None,
    ) -> tuple[tuple[float, float], ...]:
        safe: list[tuple[float, float]] = []
        for idx, point in enumerate(points):
            if idx == 0:
                safe.append(point)
                continue
            cell = self.grid.world_to_cell(point[0], point[1])
            if cell is None:
                break
            traversable = self._is_traversable(cell[0], cell[1]) if buffer_m is None else self._cell_clear(cell[0], cell[1])
            if not traversable:
                break
            safe.append(point)
        return tuple(safe)

    def _pose_offset_point(
        self,
        pose: PoseEstimate,
        forward_m: float,
        left_m: float,
        motion_direction: int,
    ) -> tuple[float, float]:
        yaw = pose.yaw_rad if motion_direction >= 0 else pose.yaw_rad + math.pi
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return (
            pose.x_m + cy * forward_m - sy * left_m,
            pose.y_m + sy * forward_m + cy * left_m,
        )
