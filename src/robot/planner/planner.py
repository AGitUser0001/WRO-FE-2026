from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import cast

import numpy as np
from scipy.ndimage import distance_transform_edt, label, maximum_filter
from scipy.optimize import linear_sum_assignment

from .camera_color import ColorObstacle
from .config import DriverConfig
from .grid import Cell, Direction, GridMap, inscribed_obstacle_mask
from .graph_path import graph_waypoints
from .local_grid import LocalGrid
from .localize_types import PoseEstimate
from .obstacle_policy import TRACKED_OBSTACLE_ASSOCIATION_M

OBSTACLE_APPROACH_PROGRESS_M = 0.08
OBSTACLE_PASS_ENTRY_LEAD_M = 0.65
OBSTACLE_PASS_CLEAR_PROGRESS_M = 0.17
OBSTACLE_SLOT_ASSOCIATION_M = 0.22
COLOR_LIDAR_RANGE_ERROR_M = 0.30
PENDING_COLOR_MAX_AGE_S = 0.75
CORNER_HEADING_TOLERANCE_RAD = math.radians(24.0)
CORNER_PARK_LATERAL_TOLERANCE_M = 0.25
CORNER_ENTRY_RADIUS_M = 0.40
CORNER_MANEUVER_RADIUS_M = 0.45


@dataclass(frozen=True)
class Plan:
    waypoints: tuple[tuple[float, float], ...]
    trajectory: tuple[tuple[float, float], ...]
    target: tuple[float, float] | None
    steering_angle_rad: float
    motion_direction: int
    reason: str
    speed_scale: float = 1.0


@dataclass(frozen=True)
class TrackedObstacle:
    label: str
    actions: tuple[str, ...]
    anchor: tuple[float, float]
    deadline: float
    observations: int


@dataclass(frozen=True)
class PendingColor:
    obstacle: ColorObstacle
    estimate: tuple[float, float]
    last_seen: float
    observations: int


class GridPlanner:
    def __init__(
        self,
        grid: GridMap,
        wheelbase_m: float = 0.138,
        max_steering_angle_rad: float = DriverConfig.max_steering_angle_rad,
        robot_length_m: float = 0.22,
        robot_width_m: float = 0.15,
        trajectory_step_m: float = 0.05,
        target_laps: int = 3,
        actuation_delay_s: float = 0.15,
        decel_tau_s: float = 0.354,
    ):
        self.grid = grid
        self.wheelbase_m = max(wheelbase_m, 0.01)
        self.max_steering_angle_rad = max(max_steering_angle_rad, 0.01)
        self.robot_length_m = max(robot_length_m, 0.01)
        self.robot_width_m = max(robot_width_m, 0.01)
        self.trajectory_step_m = max(trajectory_step_m, 0.01)
        self.target_laps = max(1, target_laps)
        self.actuation_delay_s = max(0.0, actuation_delay_s)
        self.decel_tau_s = max(0.0, decel_tau_s)
        self._last_motion_direction = 1
        self._heading_motion_direction = 1
        self._route_entry_recovery = False
        self._heading_steering_rad = 0.0
        self._path_max_curvature = 0.0
        self._path_min_clearance = math.inf
        self._graph_search_connected = False
        self._route_step = 1
        self._route_target_index: int | None = None
        self._initial_route_target_index: int | None = None
        self._lap_start_pose: tuple[float, float, float] | None = None
        self._lap_previous_progress_pose: tuple[float, float] | None = None
        self._lap_departed_start = False
        self._lap_return_pending = False
        self._lap_visited_targets: set[int] = set()
        self.completed_laps = 0
        self._previous_local_obstacles: set[tuple[int, int]] = set()
        self._previous_obstacle_candidates: set[tuple[int, int]] = set()
        self._color_lidar_obstacles: set[tuple[int, int]] = set()
        self._confirmed_local_obstacles: set[tuple[int, int]] = set()
        self._semantic_obstacles: set[tuple[int, int]] = set()
        self._local_obstacle_score = np.zeros(self.grid.cells.shape, dtype=np.float32)
        self._route_static_cost_direction: Direction | None = None
        self._route_static_cost: np.ndarray | None = None
        self._wall_clearance_direction: Direction | None = None
        self._wall_clearance_m: np.ndarray | None = None
        self._wall_search_bounds: tuple[int, int, int, int] | None = None
        self._near_wall_direction: Direction | None = None
        self._near_wall_mask: np.ndarray | None = None
        self._tracked_obstacles: list[TrackedObstacle] = []
        self._handled_obstacle_anchors: list[tuple[float, float]] = []
        self._completed_u_turn_anchors: list[tuple[float, float]] = []
        self._colored_obstacle_anchors: list[tuple[float, float]] = []
        self._obstacle_program_anchor: tuple[float, float] | None = None
        self._obstacle_program_actions: tuple[str, ...] = ()
        self._obstacle_program_index = 0
        self._obstacle_program_approach_yaw = 0.0
        self._obstacle_program_gate_passed = False
        self._obstacle_program_turn_start_yaw: float | None = None
        self._obstacle_program_turn_last_yaw = 0.0
        self._obstacle_program_turn_radians = 0.0
        self._obstacle_program_turn_sign = 0
        self._obstacle_program_stage_target: tuple[float, float] | None = None
        self._obstacle_program_stage_direction = 0
        self._obstacle_program_stage_settling = False
        self._color_observation_status = "none"
        self._pending_camera_hold_until = 0.0
        self._pending_camera_since: dict[tuple[str, int], float] = {}
        self._pending_colors: list[PendingColor] = []
        self._uncolored_obstacle_anchor: tuple[float, float] | None = None
        self._uncolored_obstacle_started_at = 0.0
        self._uncolored_obstacle_debug = "none"
        self._uncolored_obstacle_hold = False
        self._uncolored_backup_start_pose: tuple[float, float, float] | None = None
        self._uncolored_backup_done = False
        self._actual_odometry_travel_m = 0.0
        self._corner_phase = 0
        self._corner_index: int | None = None
        self._corner_phase_odom_m = 0.0
        self._corner_phase_yaw_rad = 0.0
        self._corner_phase_time = 0.0
        self._corner_leg_m = 0.0
        self._corner_leg_yaw_rad = 0.0
        self._corner_speed_scale = 0.5
        self._corner_turn_sign = 1
        self._corner_first_direction = 1
        self._corner_recovery_direction = 1
        self._corner_recovery_resume_phase = 2
        self._corner_recovery_attempted_phase = 0
        self._parked_target_index: int | None = None
        self._lateral_maneuver_target: tuple[float, float] | None = None
        self._lateral_maneuver_phase = 0
        self._lateral_maneuver_arc_m = 0.0
        self._lateral_maneuver_phase_odom_m = 0.0
        self._lateral_maneuver_sign = 0
        self._lateral_maneuver_direction = 1
        self._lateral_maneuver_hold = False
        self._lateral_probe_target: tuple[float, float] | None = None
        self._lateral_probe_best_m = math.inf
        self._lateral_probe_odom_m = 0.0

    def plan(
        self,
        pose: PoseEstimate,
        local_grid: LocalGrid | None = None,
        planning_pose: PoseEstimate | None = None,
        delay_prefix: tuple[tuple[float, float], ...] = (),
        rear_wall_distance_m: float = math.inf,
        color_observations: tuple[ColorObstacle, ...] = (),
        observation_time: float | None = None,
        direction_hint: Direction = Direction.UNKNOWN,
    ) -> Plan:
        plan_started = time.perf_counter()
        self._actual_odometry_travel_m = pose.odometry_travel_m
        if self._lap_start_pose is None:
            self._lap_start_pose = (pose.x_m, pose.y_m, pose.yaw_rad)
        self._update_local_obstacle_support(pose, local_grid)
        if color_observations:
            observed_at = time.monotonic() if observation_time is None else observation_time
            occluded = (
                self._occluded_color_observations(color_observations)
                if len(color_observations) > 1
                else set()
            )
            assigned = (
                self._assign_color_observations(
                    pose, color_observations, occluded,
                )
                if len(color_observations) > 1
                else (None,)
            )
            for index, (obstacle, anchor) in enumerate(zip(color_observations, assigned)):
                if index in occluded:
                    continue
                self.observe_colored_obstacle(
                    pose,
                    obstacle,
                    observed_at,
                    direction_hint,
                    assigned_anchor=anchor,
                    association_resolved=len(color_observations) > 1,
                )
        self._associate_pending_colors(pose, time.monotonic())
        if self.grid.direction != Direction.UNKNOWN and self._route_target_index is None:
            centers = self._corner_centers()
            self._route_target_index = self._initial_route_target(centers, pose)
            self._initial_route_target_index = self._route_target_index
        self._ensure_obstacle_program(pose)
        self._update_lap_progress(
            pose,
            allow_completion=not self._uncolored_obstacle_hold,
        )
        obstacle_done = time.perf_counter()
        tracking_done = time.perf_counter()
        plan_pose = pose if planning_pose is None else planning_pose
        plan = self._obstacle_u_turn_plan(pose)
        if plan is None and self._obstacle_program_anchor is None:
            plan = self._corner_plan(pose, rear_wall_distance_m)
        if plan is None:
            plan = (
                self._corridor_plan(plan_pose)
                if self.grid.direction == Direction.UNKNOWN
                else self._grid_plan(plan_pose, pose)
            )
        route_done = time.perf_counter()
        self._profile_plan_ms = (
            (obstacle_done - plan_started) * 1000.0,
            (tracking_done - obstacle_done) * 1000.0,
            (route_done - tracking_done) * 1000.0,
        )
        if delay_prefix:
            trajectory = (*delay_prefix, *plan.trajectory[1:]) if plan.trajectory else delay_prefix
            plan = Plan(
                waypoints=plan.waypoints,
                trajectory=trajectory,
                target=plan.target,
                steering_angle_rad=plan.steering_angle_rad,
                motion_direction=plan.motion_direction,
                reason=plan.reason,
                speed_scale=plan.speed_scale,
            )
        if time.monotonic() < self._pending_camera_hold_until:
            plan = Plan(
                waypoints=plan.waypoints,
                trajectory=plan.trajectory,
                target=plan.target,
                steering_angle_rad=plan.steering_angle_rad,
                motion_direction=plan.motion_direction,
                reason="camera-color-wait",
                speed_scale=0.0,
            )
        self._last_motion_direction = plan.motion_direction
        return plan

    def _update_local_obstacle_support(self, pose: PoseEstimate, local: LocalGrid | None) -> None:
        started = time.perf_counter()
        current: set[tuple[int, int]] = set()
        color_support: set[tuple[int, int]] = set()
        color_eligible: set[tuple[int, int]] = set()
        if local is not None:
            dynamic = self._local_obstacle_mask(local, pose)
            components, count = cast(
                tuple[np.ndarray, int],
                label(
                    dynamic & (local.cells != int(Cell.WALL)),
                    structure=np.ones((3, 3), dtype=np.uint8),
                ),
            )
            sizes = np.bincount(components.ravel(), minlength=int(count) + 1)
            clustered = (components != 0) & (sizes[components] >= 3)
            clustered = inscribed_obstacle_mask(clustered)
            clustered |= dynamic & (local.cells == int(Cell.WALL))
            local_rows, local_cols = np.nonzero(dynamic)
            cy = math.cos(pose.yaw_rad)
            sy = math.sin(pose.yaw_rad)
            local_x = (
                local.origin_cell - local_rows.astype(np.float32)
            ) * local.resolution_m
            local_y = (
                local_cols.astype(np.float32) - local.origin_cell
            ) * local.resolution_m
            global_x = pose.x_m + local_x * cy - local_y * sy
            global_y = pose.y_m + local_x * sy + local_y * cy
            cols = np.rint(global_x / self.grid.resolution_m).astype(np.intp) + self.grid.origin_cell
            rows = self.grid.origin_cell - np.rint(global_y / self.grid.resolution_m).astype(np.intp)
            valid = (
                (rows >= 0) & (rows < self.grid.size_cells)
                & (cols >= 0) & (cols < self.grid.size_cells)
                & (global_y >= -1.5) & (global_y <= 1.5)
            )
            half_w = self.grid.corridor_width_m * 0.5
            if self.grid.direction == Direction.LEFT:
                x_min, x_max = -2.5, half_w
            elif self.grid.direction == Direction.RIGHT:
                x_min, x_max = -half_w, 2.5
            else:
                x_min, x_max = -half_w, half_w
            valid &= (global_x >= x_min) & (global_x <= x_max)
            if self.grid.direction != Direction.UNKNOWN:
                center_x = (x_min + x_max) * 0.5
                valid &= ~(
                    (global_x > center_x - 0.5) & (global_x < center_x + 0.5)
                    & (global_y > -0.5) & (global_y < 0.5)
                )
                valid &= self._obstacle_section_mask(global_x, global_y)
            if (
                self._near_wall_mask is None
                or self._near_wall_direction != self.grid.direction
            ):
                self._near_wall_mask = np.asarray(maximum_filter(
                    self.grid.cells == int(Cell.MAP_WALL),
                    size=7,
                    mode="constant",
                    cval=0,
                ), dtype=np.bool_)
                self._near_wall_direction = self.grid.direction
            near_wall = self._near_wall_mask
            valid_indices = np.asarray(np.nonzero(valid)[0], dtype=np.intp)
            valid_indices = valid_indices[
                ~near_wall[rows[valid_indices], cols[valid_indices]]
            ]
            flat = np.asarray(np.unique(
                rows[valid_indices] * self.grid.size_cells + cols[valid_indices]
            ), dtype=np.intp)
            color_support = {
                (int(index // self.grid.size_cells), int(index % self.grid.size_cells))
                for index in flat
            }
            if self.grid.direction == Direction.UNKNOWN:
                eligible = (
                    local.cells[local_rows[valid_indices], local_cols[valid_indices]]
                    != int(Cell.WALL)
                )
                shared_half_width = (
                    self.grid.corridor_width_m * 0.5
                    - 3.0 * self.grid.resolution_m
                )
                eligible &= np.abs(global_x[valid_indices]) <= shared_half_width
                eligible_indices = valid_indices[eligible]
            else:
                eligible_indices = valid_indices
            eligible_flat = np.asarray(np.unique(
                rows[eligible_indices] * self.grid.size_cells + cols[eligible_indices]
            ), dtype=np.intp)
            color_eligible = {
                (int(index // self.grid.size_cells), int(index % self.grid.size_cells))
                for index in eligible_flat
            }
            clustered_indices = valid_indices[
                clustered[local_rows[valid_indices], local_cols[valid_indices]]
            ]
            clustered_flat = np.asarray(np.unique(
                rows[clustered_indices] * self.grid.size_cells + cols[clustered_indices]
            ), dtype=np.intp)
            current = {
                (int(index // self.grid.size_cells), int(index % self.grid.size_cells))
                for index in clustered_flat
            }
            previous_candidates = self._previous_obstacle_candidates
            previous_mask = np.zeros_like(self.grid.cells, dtype=np.bool_)
            if previous_candidates:
                previous_rows, previous_cols = zip(*previous_candidates)
                previous_mask[np.asarray(previous_rows), np.asarray(previous_cols)] = True
            repeated = np.asarray(maximum_filter(
                previous_mask, size=3, mode="constant", cval=0,
            ), dtype=np.bool_)
            repeated_flat = flat[repeated.flat[flat]]
            current.update(
                (int(index // self.grid.size_cells), int(index % self.grid.size_cells))
                for index in repeated_flat
            )
        extraction_done = time.perf_counter()
        previous = self._previous_obstacle_candidates
        self._color_lidar_obstacles = current & color_eligible
        score = self._local_obstacle_score
        score *= 0.96
        if color_support and previous:
            rows, cols = zip(*color_support)
            rows_array = np.asarray(rows, dtype=np.intp)
            cols_array = np.asarray(cols, dtype=np.intp)
            previous_mask = np.zeros_like(score, dtype=np.bool_)
            previous_rows, previous_cols = zip(*previous)
            previous_mask[np.asarray(previous_rows), np.asarray(previous_cols)] = True
            matched = np.asarray(maximum_filter(
                previous_mask, size=3, mode="constant", cval=0,
            ), dtype=np.bool_)[rows_array, cols_array]
            matched_rows = rows_array[matched]
            matched_cols = cols_array[matched]
            score[matched_rows, matched_cols] = np.minimum(
                1.0, score[matched_rows, matched_cols] + 0.35,
            )
        previously_confirmed = np.zeros_like(score, dtype=np.bool_)
        if self._confirmed_local_obstacles:
            rows, cols = zip(*self._confirmed_local_obstacles)
            previously_confirmed[np.asarray(rows), np.asarray(cols)] = True
        confirmed_evidence = (score >= 0.50) | (
            previously_confirmed & (score >= 0.38)
        )
        persistent_mask = inscribed_obstacle_mask(confirmed_evidence)
        persistence_done = time.perf_counter()
        rows, cols = np.nonzero(persistent_mask)
        persistent = set(zip(rows.tolist(), cols.tolist()))
        self._confirmed_local_obstacles = persistent | self._semantic_obstacles
        self._previous_local_obstacles = current
        self._previous_obstacle_candidates = color_support
        self._profile_obstacle_ms = (
            (extraction_done - started) * 1000.0,
            (persistence_done - extraction_done) * 1000.0,
        )

    def _local_obstacle_clusters(
        self,
        local: LocalGrid,
        pose: PoseEstimate,
    ) -> tuple[tuple[int, int], ...]:
        dynamic = self._local_obstacle_mask(local, pose)
        components, count = cast(
            tuple[np.ndarray, int],
            label(
                dynamic & (local.cells != int(Cell.WALL)),
                structure=np.ones((3, 3), dtype=np.uint8),
            ),
        )
        sizes = np.bincount(components.ravel(), minlength=int(count) + 1)
        supported = (components != 0) & (sizes[components] >= 3)
        supported = inscribed_obstacle_mask(supported)
        supported |= dynamic & (local.cells == int(Cell.WALL))
        rows, cols = np.nonzero(supported)
        return tuple(zip(rows.tolist(), cols.tolist()))

    def _local_obstacle_mask(
        self,
        local: LocalGrid,
        pose: PoseEstimate,
    ) -> np.ndarray:
        obstacles = (
            (local.cells == int(Cell.UNKNOWN_OBSTRUCTION))
            | (local.cells == int(Cell.LIVE_OBSTACLE))
        )
        fitted_rows, fitted_cols = np.nonzero(local.cells == int(Cell.WALL))
        if not len(fitted_rows):
            return obstacles

        local_x = (local.origin_cell - fitted_rows) * local.resolution_m
        local_y = (fitted_cols - local.origin_cell) * local.resolution_m
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        global_x = pose.x_m + local_x * cy - local_y * sy
        global_y = pose.y_m + local_x * sy + local_y * cy
        cols = np.rint(global_x / self.grid.resolution_m).astype(np.intp) + self.grid.origin_cell
        rows = self.grid.origin_cell - np.rint(global_y / self.grid.resolution_m).astype(np.intp)
        valid = (
            (rows >= 0) & (rows < self.grid.size_cells)
            & (cols >= 0) & (cols < self.grid.size_cells)
        )
        if not bool(valid.any()):
            return obstacles

        if (
            self._near_wall_mask is None
            or self._near_wall_direction != self.grid.direction
        ):
            self._near_wall_mask = np.asarray(maximum_filter(
                self.grid.cells == int(Cell.MAP_WALL),
                size=7,
                mode="constant",
                cval=0,
            ), dtype=np.bool_)
            self._near_wall_direction = self.grid.direction
        unmatched = valid.copy()
        unmatched[valid] = ~self._near_wall_mask[rows[valid], cols[valid]]
        unmatched_mask = np.zeros_like(obstacles)
        unmatched_mask[fitted_rows[unmatched], fitted_cols[unmatched]] = True
        components, count = cast(
            tuple[np.ndarray, int],
            label(unmatched_mask, structure=np.ones((3, 3), dtype=np.uint8)),
        )
        for component in range(1, int(count) + 1):
            component_rows, component_cols = np.nonzero(components == component)
            center_row = float(component_rows.mean())
            center_col = float(component_cols.mean())
            nearest = int(np.argmin(
                np.square(component_rows - center_row)
                + np.square(component_cols - center_col)
            ))
            obstacles[component_rows[nearest], component_cols[nearest]] = True
        return obstacles

    def observe_colored_obstacle(
        self,
        pose: PoseEstimate,
        obstacle: ColorObstacle,
        now: float,
        direction_hint: Direction = Direction.UNKNOWN,
        assigned_anchor: tuple[float, float] | None = None,
        association_resolved: bool = False,
    ) -> None:
        forward_m = 0.163 + obstacle.depth_m
        right_m = obstacle.x_norm * 640.0 * obstacle.depth_m / 607.97784
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        estimate = (
            pose.x_m + cy * forward_m + sy * right_m,
            pose.y_m + sy * forward_m - cy * right_m,
        )
        effective_direction = self.grid.direction if self.grid.direction != Direction.UNKNOWN else direction_hint
        anchor = assigned_anchor
        if not association_resolved:
            anchor = self._nearest_supported_obstacle(
                pose,
                obstacle.label,
                expected_forward_m=forward_m,
                expected_right_m=right_m,
            )
        if anchor is None:
            pending_key = (obstacle.label, int(round(obstacle.x_norm * 4.0)))
            if self._matches_known_tracked_obstacle(
                pose, obstacle.label, forward_m, right_m,
            ):
                self._pending_camera_since.pop(pending_key, None)
                self._color_observation_status = "known-color"
                return
            if not self._inside_direction_bounds(estimate[0], estimate[1], effective_direction):
                self._color_observation_status = "camera-outside-map"
                return
            if self.completed_laps > 0:
                self._pending_camera_since.pop(pending_key, None)
                self._color_observation_status = "camera-known-course"
                return
            self._color_observation_status = "camera-pending"
            self._remember_pending_color(obstacle, estimate, now)
            if obstacle.depth_m <= 1.20:
                started = self._pending_camera_since.setdefault(pending_key, now)
                self._pending_camera_hold_until = max(
                    self._pending_camera_hold_until,
                    started + 0.50,
                )
            return
        self._color_observation_status = "lidar-anchor"
        pending_key = (obstacle.label, int(round(obstacle.x_norm * 4.0)))
        self._pending_camera_since.pop(pending_key, None)
        match_idx = self._matching_tracked_obstacle(obstacle.label, anchor)
        observations = 1
        matched_obstacle: TrackedObstacle | None = None
        old_anchor: tuple[float, float] | None = None
        if match_idx is not None:
            matched_obstacle = self._tracked_obstacles[match_idx]
            if matched_obstacle.observations >= 3 and matched_obstacle.label != obstacle.label:
                self._color_observation_status = "color-conflict"
                return
            observations = (
                matched_obstacle.observations + 1
                if matched_obstacle.label == obstacle.label
                else 1
            )
            old_anchor = matched_obstacle.anchor
            dx = anchor[0] - old_anchor[0]
            dy = anchor[1] - old_anchor[1]
            distance = math.hypot(dx, dy)
            scale = min(1.0, 0.025 / max(distance, 1.0e-6))
            anchor = (
                old_anchor[0] + dx * scale,
                old_anchor[1] + dy * scale,
            )
        tracked = TrackedObstacle(
            obstacle.label,
            obstacle.actions,
            anchor,
            math.inf if observations >= 2 else now + 0.75,
            observations,
        )
        if observations >= 3 and effective_direction != Direction.UNKNOWN:
            if old_anchor is not None:
                old_anchor_cell = self.grid.world_to_cell(*old_anchor)
                if old_anchor_cell is not None:
                    self._semantic_obstacles.discard(old_anchor_cell)
            anchor_cell = self.grid.world_to_cell(anchor[0], anchor[1])
            if anchor_cell is not None:
                self._semantic_obstacles.add(anchor_cell)
        if match_idx is None:
            self._tracked_obstacles.append(tracked)
        else:
            self._tracked_obstacles[match_idx] = tracked
        if (
            old_anchor is not None
            and self._obstacle_program_anchor is not None
            and math.dist(old_anchor, self._obstacle_program_anchor)
            <= TRACKED_OBSTACLE_ASSOCIATION_M
        ):
            self._obstacle_program_anchor = anchor
        if observations >= 3:
            self._remember_colored_anchor(anchor)
            if (
                self._uncolored_obstacle_anchor is not None
                and math.dist(anchor, self._uncolored_obstacle_anchor)
                <= TRACKED_OBSTACLE_ASSOCIATION_M
            ):
                self._uncolored_obstacle_anchor = None
                self._uncolored_obstacle_hold = False

    def _remember_pending_color(
        self,
        obstacle: ColorObstacle,
        estimate: tuple[float, float],
        now: float,
    ) -> None:
        self._pending_colors = [
            pending for pending in self._pending_colors
            if now - pending.last_seen <= PENDING_COLOR_MAX_AGE_S
        ]
        match = min(
            (
                (index, pending)
                for index, pending in enumerate(self._pending_colors)
                if pending.obstacle.label == obstacle.label
                and math.dist(pending.estimate, estimate) <= 0.30
            ),
            key=lambda item: math.dist(item[1].estimate, estimate),
            default=None,
        )
        if match is None:
            self._pending_colors.append(PendingColor(obstacle, estimate, now, 1))
            self._pending_colors = self._pending_colors[-12:]
            return
        index, pending = match
        weight = 1.0 / min(pending.observations + 1, 4)
        smoothed = (
            pending.estimate[0] + weight * (estimate[0] - pending.estimate[0]),
            pending.estimate[1] + weight * (estimate[1] - pending.estimate[1]),
        )
        self._pending_colors[index] = PendingColor(
            obstacle,
            smoothed,
            now,
            min(pending.observations + 1, 20),
        )

    def _associate_pending_colors(self, pose: PoseEstimate, now: float) -> None:
        if not self._pending_colors:
            return
        confirmed = np.asarray(
            [self.grid.cell_to_world(*cell) for cell in self._confirmed_local_obstacles],
            dtype=np.float32,
        ).reshape(-1, 2)

        def confirmed_anchor(anchor: tuple[float, float]) -> bool:
            return bool(
                len(confirmed)
                and float(np.min(
                    np.linalg.norm(confirmed - np.asarray(anchor), axis=1),
                )) <= 0.16
            )

        remaining: list[PendingColor] = []
        for pending in self._pending_colors:
            if now - pending.last_seen > PENDING_COLOR_MAX_AGE_S:
                continue
            if pending.observations < 2:
                remaining.append(pending)
                continue
            candidates: list[tuple[float, tuple[float, float]]] = []
            active_anchor = self._uncolored_obstacle_anchor
            if (
                active_anchor is not None
                and math.dist(active_anchor, pending.estimate) <= 0.40
                and confirmed_anchor(active_anchor)
            ):
                candidates.append((math.dist(active_anchor, pending.estimate), active_anchor))
            dx = pending.estimate[0] - pose.x_m
            dy = pending.estimate[1] - pose.y_m
            cy, sy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
            expected_forward = cy * dx + sy * dy
            expected_right = sy * dx - cy * dy
            candidates.extend(
                (score + math.dist(anchor, pending.estimate), anchor)
                for score, anchor in self._supported_obstacle_candidates(
                    pose, expected_forward, expected_right,
                )
                if math.dist(anchor, pending.estimate) <= 0.40
                and confirmed_anchor(anchor)
            )
            candidates = [
                candidate for candidate in candidates
                if not self._anchor_conflicts_with_color(
                    pending.obstacle.label, candidate[1],
                )
                and not any(
                    obstacle.label != pending.obstacle.label
                    and math.dist(obstacle.anchor, candidate[1]) <= 0.18
                    for obstacle in self._tracked_obstacles
                )
            ]
            if not candidates:
                remaining.append(pending)
                continue
            anchor = min(candidates, key=lambda candidate: candidate[0])[1]
            for _ in range(min(3, pending.observations)):
                self.observe_colored_obstacle(
                    pose,
                    pending.obstacle,
                    now,
                    assigned_anchor=anchor,
                    association_resolved=True,
                )
        self._pending_colors = remaining

    def _matches_known_tracked_obstacle(
        self,
        pose: PoseEstimate,
        label_name: str,
        expected_forward_m: float,
        expected_right_m: float,
    ) -> bool:
        return bool(self._known_tracked_obstacle_candidates(
            pose, label_name, expected_forward_m, expected_right_m,
        ))

    def _known_tracked_obstacle_candidates(
        self,
        pose: PoseEstimate,
        label_name: str,
        expected_forward_m: float,
        expected_right_m: float,
    ) -> tuple[tuple[float, tuple[float, float]], ...]:
        expected_range = math.hypot(expected_forward_m, expected_right_m)
        expected_bearing = math.atan2(expected_right_m, expected_forward_m)
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        candidates = []
        for obstacle in self._tracked_obstacles:
            if obstacle.label != label_name or obstacle.observations < 2:
                continue
            dx = obstacle.anchor[0] - pose.x_m
            dy = obstacle.anchor[1] - pose.y_m
            forward_m = cy * dx + sy * dy
            right_m = sy * dx - cy * dy
            bearing = math.atan2(right_m, forward_m)
            bearing_error = abs(math.atan2(
                math.sin(bearing - expected_bearing),
                math.cos(bearing - expected_bearing),
            ))
            if (
                forward_m >= 0.05
                and abs(math.hypot(dx, dy) - expected_range) <= 0.50
                and bearing_error <= math.radians(12.0)
            ):
                range_error = abs(math.hypot(dx, dy) - expected_range)
                candidates.append((
                    max(
                        0.0,
                        bearing_error / math.radians(12.0)
                        + range_error / 0.50
                        - 0.25,
                    ),
                    obstacle.anchor,
                ))
        return tuple(candidates)

    def _remember_colored_anchor(self, anchor: tuple[float, float]) -> None:
        for index, old_anchor in enumerate(self._colored_obstacle_anchors):
            if math.dist(anchor, old_anchor) <= 0.28:
                self._colored_obstacle_anchors[index] = anchor
                return
        self._colored_obstacle_anchors.append(anchor)

    def _matching_tracked_obstacle(
        self,
        label_name: str,
        anchor: tuple[float, float],
    ) -> int | None:
        nearest = (math.inf, -1)
        for idx, obstacle in enumerate(self._tracked_obstacles):
            distance = math.hypot(
                obstacle.anchor[0] - anchor[0], obstacle.anchor[1] - anchor[1],
            )
            if distance <= TRACKED_OBSTACLE_ASSOCIATION_M:
                nearest = min(nearest, (distance, idx))
        if nearest[1] >= 0:
            return nearest[1]

        placement = self._obstacle_placement_key(anchor)
        if placement is None:
            return None
        same_placement = (
            (math.dist(obstacle.anchor, anchor), idx)
            for idx, obstacle in enumerate(self._tracked_obstacles)
            if self._obstacle_placement_key(obstacle.anchor) == placement
        )
        match = min(same_placement, default=(math.inf, -1))
        return None if match[1] < 0 else match[1]

    def _obstacle_placement_key(
        self,
        anchor: tuple[float, float],
    ) -> tuple[int, int] | None:
        if self.grid.direction == Direction.UNKNOWN:
            return None
        half_w = self.grid.corridor_width_m * 0.5
        x_min, x_max = (
            (-2.5, half_w)
            if self.grid.direction == Direction.LEFT
            else (-half_w, 2.5)
        )
        center_x = (x_min + x_max) * 0.5
        candidates: list[tuple[float, int, int]] = []
        for column, along in enumerate((-0.5, 0.0, 0.5)):
            for radial in (0.875, 1.125):
                points = (
                    (center_x + along, radial),
                    (center_x + radial, -along),
                    (center_x - along, -radial),
                    (center_x - radial, along),
                )
                candidates.extend(
                    (math.dist(anchor, point), side, column)
                    for side, point in enumerate(points)
                )
        distance, side, column = min(candidates)
        if distance > OBSTACLE_SLOT_ASSOCIATION_M:
            return None
        return side, column

    def _anchor_conflicts_with_color(
        self,
        label_name: str,
        anchor: tuple[float, float],
    ) -> bool:
        return any(
            obstacle.label != label_name
            and obstacle.observations >= 3
            and math.dist(obstacle.anchor, anchor) <= 0.18
            for obstacle in self._tracked_obstacles
        )

    def expire_tracked_obstacles(self, now: float) -> None:
        self._tracked_obstacles = [
            obstacle for obstacle in self._tracked_obstacles
            if now <= obstacle.deadline
        ]

    def tracked_obstacle_status(self) -> str:
        return ",".join(
            f"{obstacle.label}:{'+'.join(obstacle.actions)}"
            for obstacle in self._tracked_obstacles
            if obstacle.observations >= 2
        ) or "none"

    def tracked_obstacle_debug(self) -> str:
        return ";".join(
            f"{obstacle.label}@{obstacle.anchor[0]:+.2f}:{obstacle.anchor[1]:+.2f}/"
            f"{'+'.join(obstacle.actions)}"
            for obstacle in self._tracked_obstacles
            if obstacle.observations >= 2
        ) or "none"

    def tracked_obstacle_display(
        self,
    ) -> tuple[tuple[str, tuple[float, float]], ...]:
        support = tuple(
            self.grid.cell_to_world(*cell)
            for cell in self._color_lidar_obstacles
        )
        available = set(range(len(support)))
        displayed: list[tuple[str, tuple[float, float]]] = []
        for obstacle in self._tracked_obstacles:
            if obstacle.observations < 2:
                continue
            nearest = min(
                (
                    (math.dist(obstacle.anchor, support[index]), index)
                    for index in available
                ),
                default=(math.inf, -1),
            )
            anchor = obstacle.anchor
            if nearest[0] <= TRACKED_OBSTACLE_ASSOCIATION_M:
                available.remove(nearest[1])
                anchor = support[nearest[1]]
            displayed.append((obstacle.label, anchor))
        return tuple(displayed)

    def color_observation_status(self) -> str:
        return self._color_observation_status

    def _nearest_supported_obstacle(
        self,
        pose: PoseEstimate,
        label_name: str,
        expected_forward_m: float,
        expected_right_m: float,
    ) -> tuple[float, float] | None:
        candidates = (
            candidate
            for candidate in (
                *self._known_tracked_obstacle_candidates(
                    pose, label_name, expected_forward_m, expected_right_m,
                ),
                *self._supported_obstacle_candidates(
                    pose, expected_forward_m, expected_right_m,
                ),
            )
            if not self._anchor_conflicts_with_color(label_name, candidate[1])
        )
        match = min(candidates, key=lambda candidate: candidate[0], default=None)
        return None if match is None else match[1]

    def _assign_color_observations(
        self,
        pose: PoseEstimate,
        observations: tuple[ColorObstacle, ...],
        occluded: set[int] | None = None,
    ) -> tuple[tuple[float, float] | None, ...]:
        if occluded is None:
            occluded = self._occluded_color_observations(observations)
        candidates_by_observation: list[tuple[tuple[float, tuple[float, float]], ...]] = []
        anchors: list[tuple[float, float]] = []
        for index, obstacle in enumerate(observations):
            forward_m = 0.163 + obstacle.depth_m
            right_m = obstacle.x_norm * 640.0 * obstacle.depth_m / 607.97784
            candidates = () if index in occluded else (
                *self._known_tracked_obstacle_candidates(
                    pose, obstacle.label, forward_m, right_m,
                ),
                *self._supported_obstacle_candidates(pose, forward_m, right_m),
            )
            candidates = tuple(
                candidate for candidate in candidates
                if not self._anchor_conflicts_with_color(obstacle.label, candidate[1])
            )
            candidates_by_observation.append(candidates)
            for _score, anchor in candidates:
                if anchor not in anchors:
                    anchors.append(anchor)
        if not anchors:
            return tuple(None for _ in observations)

        count = len(observations)
        costs = np.full((count, len(anchors) + count), 1.0e6, dtype=np.float64)
        costs[:, len(anchors):] = 1.6
        anchor_columns = {anchor: index for index, anchor in enumerate(anchors)}
        for row, candidates in enumerate(candidates_by_observation):
            for score, anchor in candidates:
                costs[row, anchor_columns[anchor]] = min(
                    costs[row, anchor_columns[anchor]], score,
                )
        assignments: list[tuple[float, float] | None] = [None] * count
        rows, cols = linear_sum_assignment(costs)
        for row, col in zip(rows.tolist(), cols.tolist()):
            if col < len(anchors) and costs[row, col] < 1.5:
                assignments[row] = anchors[col]
        return tuple(assignments)

    @staticmethod
    def _occluded_color_observations(
        observations: tuple[ColorObstacle, ...],
    ) -> set[int]:
        camera_geometry = tuple(
            (
                math.hypot(
                    0.163 + obstacle.depth_m,
                    obstacle.x_norm * 640.0 * obstacle.depth_m / 607.97784,
                ),
                math.atan2(
                    obstacle.x_norm * 640.0 * obstacle.depth_m / 607.97784,
                    0.163 + obstacle.depth_m,
                ),
            )
            for obstacle in observations
        )
        return {
            index
            for index, obstacle in enumerate(observations)
            for nearer_index, nearer in enumerate(observations)
            if nearer_index != index
            and nearer.label == obstacle.label
            and camera_geometry[nearer_index][0] + 0.12 < camera_geometry[index][0]
            and abs(math.atan2(
                math.sin(camera_geometry[nearer_index][1] - camera_geometry[index][1]),
                math.cos(camera_geometry[nearer_index][1] - camera_geometry[index][1]),
            )) <= math.radians(8.0)
        }

    def _supported_obstacle_candidates(
        self,
        pose: PoseEstimate,
        expected_forward_m: float,
        expected_right_m: float,
    ) -> tuple[tuple[float, tuple[float, float]], ...]:
        supported = (
            self._color_lidar_obstacles
            | self._previous_local_obstacles
            | self._confirmed_local_obstacles
        )
        if not supported:
            return ()
        cells = np.zeros(self.grid.cells.shape, dtype=bool)
        rows, cols = zip(*supported)
        cells[np.asarray(rows), np.asarray(cols)] = True
        components, count = cast(
            tuple[np.ndarray, int],
            label(cells, structure=np.ones((3, 3), dtype=np.uint8)),
        )
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        candidates: list[tuple[float, tuple[float, float]]] = []
        for component in range(1, int(count) + 1):
            component_mask = components == component
            component_rows, component_cols = np.nonzero(component_mask)
            if len(component_rows) == 0:
                continue
            observed_points = tuple(
                self.grid.cell_to_world(int(row), int(col))
                for row, col in zip(component_rows, component_cols)
            )
            camera_errors = []
            expected_bearing = math.atan2(expected_right_m, expected_forward_m)
            expected_range = math.hypot(expected_forward_m, expected_right_m)
            for x_m, y_m in observed_points:
                dx = x_m - pose.x_m
                dy = y_m - pose.y_m
                forward_m = cy * dx + sy * dy
                right_m = sy * dx - cy * dy
                if forward_m < 0.05:
                    continue
                bearing = math.atan2(right_m, forward_m)
                bearing_error = abs(math.atan2(
                    math.sin(bearing - expected_bearing),
                    math.cos(bearing - expected_bearing),
                ))
                camera_errors.append((
                    bearing_error,
                    abs(math.hypot(dx, dy) - expected_range),
                ))
            if not camera_errors:
                continue
            bearing_error, range_error = min(
                camera_errors,
                key=lambda error: (
                    error[0] / math.radians(12.0)
                    + error[1] / COLOR_LIDAR_RANGE_ERROR_M
                ),
            )
            if (
                bearing_error > math.radians(12.0)
                or range_error > COLOR_LIDAR_RANGE_ERROR_M
            ):
                continue
            fitted_rows, fitted_cols = np.nonzero(inscribed_obstacle_mask(component_mask))
            if not len(fitted_rows):
                continue
            fitted_points = tuple(
                self.grid.cell_to_world(int(row), int(col))
                for row, col in zip(fitted_rows, fitted_cols)
            )
            anchor = (
                sum(point[0] for point in fitted_points) / len(fitted_points),
                sum(point[1] for point in fitted_points) / len(fitted_points),
            )
            candidates.append((
                bearing_error / math.radians(12.0)
                + range_error / COLOR_LIDAR_RANGE_ERROR_M,
                anchor,
            ))
        return tuple(candidates)

    def _inside_direction_bounds(self, x_m: float, y_m: float, direction: Direction) -> bool:
        if direction == Direction.UNKNOWN:
            return self.grid.inside_navigable_bounds(x_m, y_m)
        if not -1.5 <= y_m <= 1.5:
            return False
        half_w = self.grid.corridor_width_m * 0.5
        x_min, x_max = (-2.5, half_w) if direction == Direction.LEFT else (-half_w, 2.5)
        center_x = (x_min + x_max) * 0.5
        return x_min <= x_m <= x_max and not (
            center_x - 0.5 < x_m < center_x + 0.5 and -0.5 < y_m < 0.5
        )

    def _obstacle_section_mask(
        self,
        x_m: np.ndarray,
        y_m: np.ndarray,
    ) -> np.ndarray:
        direction = self.grid.direction
        if direction == Direction.UNKNOWN:
            return np.ones(np.broadcast_shapes(x_m.shape, y_m.shape), dtype=np.bool_)
        half_w = self.grid.corridor_width_m * 0.5
        x_min, x_max = (
            (-2.5, half_w)
            if direction == Direction.LEFT
            else (-half_w, 2.5)
        )
        center_x = (x_min + x_max) * 0.5
        center_limit = half_w + 2.5 * self.grid.resolution_m
        return (np.abs(x_m - center_x) <= center_limit) | (
            np.abs(y_m) <= center_limit
        )

    def _ensure_obstacle_program(self, pose: PoseEstimate) -> None:
        self._handled_obstacle_anchors = [
            anchor for anchor in self._handled_obstacle_anchors
            if math.dist(anchor, (pose.x_m, pose.y_m)) <= 1.0
        ]
        if self._obstacle_program_anchor is not None:
            return
        if self._last_motion_direction < 0:
            return
        course_target = (
            (0.0, 1.0)
            if self._route_target_index is None
            else self._corner_centers()[self._route_target_index]
        )
        route_dx = course_target[0] - pose.x_m
        route_dy = course_target[1] - pose.y_m
        route_length = math.hypot(route_dx, route_dy)
        if route_length < 0.05:
            return
        route_forward = (route_dx / route_length, route_dy / route_length)
        approach_yaw = math.pi * 0.5
        section_center = None
        if self._route_target_index is not None:
            centers = self._corner_centers()
            previous = centers[(self._route_target_index - self._route_step) % len(centers)]
            approach_yaw = math.atan2(
                course_target[1] - previous[1], course_target[0] - previous[0],
            )
            section_center = (
                (course_target[0] + previous[0]) * 0.5,
                (course_target[1] + previous[1]) * 0.5,
            )
        section_forward = (math.cos(approach_yaw), math.sin(approach_yaw))
        cy, sy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        candidates = []
        for obstacle in self._tracked_obstacles:
            if obstacle.observations < 2:
                continue
            if section_center is not None and abs(
                (obstacle.anchor[0] - section_center[0]) * section_forward[0]
                + (obstacle.anchor[1] - section_center[1]) * section_forward[1]
            ) > self.grid.corridor_width_m * 0.5 + 2.5 * self.grid.resolution_m:
                continue
            if any(
                math.dist(obstacle.anchor, anchor)
                <= TRACKED_OBSTACLE_ASSOCIATION_M
                for anchor in (
                    *self._handled_obstacle_anchors,
                    *self._completed_u_turn_anchors,
                )
            ):
                continue
            dx, dy = obstacle.anchor[0] - pose.x_m, obstacle.anchor[1] - pose.y_m
            forward = cy * dx + sy * dy
            lateral = -sy * dx + cy * dy
            route_progress = dx * route_forward[0] + dy * route_forward[1]
            route_lateral = abs(-route_forward[1] * dx + route_forward[0] * dy)
            if (
                0.05 <= forward <= 0.90
                and abs(lateral) <= 0.45
                and OBSTACLE_APPROACH_PROGRESS_M <= route_progress <= route_length + 0.20
                and route_lateral <= 0.30
            ):
                candidates.append((forward, abs(lateral), obstacle))
        if not candidates:
            return
        obstacle = min(candidates, key=lambda item: (item[0], item[1]))[2]
        self._obstacle_program_anchor = obstacle.anchor
        self._obstacle_program_actions = obstacle.actions
        self._obstacle_program_index = 0
        self._obstacle_program_approach_yaw = approach_yaw
        self._obstacle_program_gate_passed = False
        self._reset_obstacle_turn()

    def _program_obstacle(self) -> TrackedObstacle | None:
        anchor = self._obstacle_program_anchor
        if anchor is None:
            return None
        return min(
            (
                obstacle for obstacle in self._tracked_obstacles
                if math.dist(obstacle.anchor, anchor)
                <= TRACKED_OBSTACLE_ASSOCIATION_M
            ),
            key=lambda obstacle: math.dist(obstacle.anchor, anchor),
            default=None,
        )

    def _route_obstacle_passes(
        self, pose: PoseEstimate,
    ) -> tuple[tuple[tuple[float, float], str, float], ...]:
        passes = []
        action = self._program_action()
        anchor = self._obstacle_program_anchor
        if anchor is not None and action in ("left", "right"):
            passes.append((anchor, action, self._obstacle_program_approach_yaw))
        if action == "u_turn" or (
            anchor is not None
            and self._obstacle_program_index + 1 < len(self._obstacle_program_actions)
        ):
            return tuple(passes)
        centers = self._corner_centers()
        target_index = self._route_target_index or 0
        target = centers[target_index]
        previous = centers[(target_index - self._route_step) % len(centers)]
        yaw = math.atan2(target[1] - previous[1], target[0] - previous[0])
        forward = (math.cos(yaw), math.sin(yaw))
        center = ((target[0] + previous[0]) * 0.5, (target[1] + previous[1]) * 0.5)
        candidates = []
        for obstacle in self._tracked_obstacles:
            if obstacle.observations < 2 or not obstacle.actions or obstacle.actions[0] not in ("left", "right"):
                continue
            if any(math.dist(obstacle.anchor, handled) <= TRACKED_OBSTACLE_ASSOCIATION_M for handled in (
                *self._handled_obstacle_anchors, *self._completed_u_turn_anchors,
                *((anchor,) if anchor is not None else ()),
            )):
                continue
            dx, dy = obstacle.anchor[0] - center[0], obstacle.anchor[1] - center[1]
            if (
                abs(dx * forward[0] + dy * forward[1]) > 0.5 + 2.5 * self.grid.resolution_m
                or abs(-dx * forward[1] + dy * forward[0]) > 0.5
            ):
                continue
            progress = ((obstacle.anchor[0] - pose.x_m) * forward[0]
                        + (obstacle.anchor[1] - pose.y_m) * forward[1])
            if progress >= OBSTACLE_APPROACH_PROGRESS_M:
                candidates.append((progress, obstacle))
        for _, obstacle in sorted(candidates, key=lambda item: item[0]):
            passes.append((obstacle.anchor, obstacle.actions[0], yaw))
            if len(obstacle.actions) > 1:
                break
        return tuple(passes)

    def _course_tangent(self, pose: PoseEstimate) -> tuple[float, float]:
        if self.grid.direction == Direction.UNKNOWN:
            return (0.0, 1.0)
        centers = self._corner_centers()[::self._route_step]
        route = (*centers, centers[0])
        progress = self._nearest_route_progress(route, pose.x_m, pose.y_m)
        before = self._route_point_at(route, progress - 0.40)
        after = self._route_point_at(route, progress + 0.40)
        dx, dy = after[0] - before[0], after[1] - before[1]
        length = math.hypot(dx, dy)
        return (dx / length, dy / length)

    def _program_action(self) -> str | None:
        actions = self._obstacle_program_actions
        if not actions:
            obstacle = self._program_obstacle()
            actions = () if obstacle is None else obstacle.actions
        if self._obstacle_program_index >= len(actions):
            return None
        return actions[self._obstacle_program_index]

    def _advance_obstacle_program(self, pose: PoseEstimate) -> None:
        obstacle = self._program_obstacle()
        actions = self._obstacle_program_actions or (
            () if obstacle is None else obstacle.actions
        )
        self._obstacle_program_index += 1
        self._obstacle_program_approach_yaw = pose.yaw_rad
        self._obstacle_program_gate_passed = False
        self._reset_obstacle_turn()
        if self._program_action() is None:
            if self._obstacle_program_anchor is not None:
                handled = (
                    self._completed_u_turn_anchors
                    if "u_turn" in actions
                    else self._handled_obstacle_anchors
                )
                handled.append(self._obstacle_program_anchor)
            self._clear_obstacle_program()

    def _clear_obstacle_program(self) -> None:
        self._obstacle_program_anchor = None
        self._obstacle_program_actions = ()
        self._obstacle_program_index = 0
        self._obstacle_program_gate_passed = False
        self._reset_obstacle_turn()

    def _reset_obstacle_turn(self) -> None:
        self._obstacle_program_turn_start_yaw = None
        self._obstacle_program_turn_last_yaw = 0.0
        self._obstacle_program_turn_radians = 0.0
        self._obstacle_program_turn_sign = 0
        self._obstacle_program_stage_target = None
        self._obstacle_program_stage_direction = 0
        self._obstacle_program_stage_settling = False

    def _obstacle_pass_target(
        self,
        pose: PoseEstimate,
        action: str,
    ) -> tuple[float, float] | None:
        anchor = self._obstacle_program_anchor
        if anchor is None or action not in ("left", "right"):
            return None
        yaw = self._obstacle_program_approach_yaw
        forward = (math.cos(yaw), math.sin(yaw))
        left = (-forward[1], forward[0])
        side_sign = 1.0 if action == "left" else -1.0
        side_offset = self._obstacle_pass_side_offset(
            anchor, forward, left, side_sign,
        )
        relative = (pose.x_m - anchor[0], pose.y_m - anchor[1])
        passed = relative[0] * forward[0] + relative[1] * forward[1]
        side = side_sign * (relative[0] * left[0] + relative[1] * left[1])
        target_progress = 0.20
        if passed >= OBSTACLE_PASS_CLEAR_PROGRESS_M and side < 0.10:
            target_progress = min(0.40, passed + 0.20)
        target = (
            anchor[0] + target_progress * forward[0] + side_sign * side_offset * left[0],
            anchor[1] + target_progress * forward[1] + side_sign * side_offset * left[1],
        )
        if passed >= 0.40 or (
            passed >= OBSTACLE_PASS_CLEAR_PROGRESS_M
            and side >= 0.10
        ):
            self._advance_obstacle_program(pose)
            return None
        return target

    def _obstacle_pass_side_offset(
        self,
        anchor: tuple[float, float],
        forward: tuple[float, float],
        left: tuple[float, float],
        side_sign: float,
    ) -> float:
        side = (side_sign * left[0], side_sign * left[1])
        nearest_wall = math.inf
        for along in (-0.18, 0.0, 0.10, 0.14):
            base = (
                anchor[0] + along * forward[0],
                anchor[1] + along * forward[1],
            )
            for distance in np.arange(
                self.grid.resolution_m,
                0.65 + self.grid.resolution_m * 0.5,
                self.grid.resolution_m,
            ):
                cell = self.grid.world_to_cell(
                    base[0] + float(distance) * side[0],
                    base[1] + float(distance) * side[1],
                )
                if cell is None:
                    break
                if self.grid.cells[cell] in (int(Cell.MAP_WALL), int(Cell.WALL)):
                    nearest_wall = min(nearest_wall, float(distance))
                    break
        if math.isfinite(nearest_wall):
            return max(0.14, min(0.20, nearest_wall * 0.5 - 0.025))
        return 0.17

    def _obstacle_u_turn_plan(self, pose: PoseEstimate) -> Plan | None:
        if self._program_action() != "u_turn":
            return None
        if self._obstacle_program_stage_settling:
            if pose.odometry_motion_m > 0.004:
                return Plan(
                    (), ((pose.x_m, pose.y_m),), None,
                    0.0, 1, "obstacle-u-turn-stage", 0.0,
                )
            self._obstacle_program_stage_settling = False
        radius = self.wheelbase_m / math.tan(self.max_steering_angle_rad)
        required_clearance = math.hypot(
            self.robot_length_m * 0.5,
            self.robot_width_m * 0.5,
        ) + 0.01 + self.grid.resolution_m * 0.5
        clearance = self._clearance_field()
        if self._obstacle_program_stage_target is not None:
            remaining = max(
                0.0, math.pi - self._obstacle_program_turn_radians,
            )
            stage = self._u_turn_stage_plan(
                pose,
                radius,
                required_clearance,
                clearance,
                remaining,
                self._obstacle_program_turn_sign or None,
            )
            if stage is not None:
                return stage
            self._obstacle_program_turn_last_yaw = pose.yaw_rad
        if self._obstacle_program_turn_sign == 0:
            candidates = {
                sign: self._u_turn_trajectory(pose, sign, math.pi, radius)
                for sign in (-1, 1)
            }
            clearances = {
                sign: self._trajectory_clearance(trajectory, clearance)
                for sign, trajectory in candidates.items()
            }
            best_sign = max(
                clearances, key=lambda sign: clearances[sign],
            )
            if clearances[best_sign] < required_clearance:
                stage = self._u_turn_stage_plan(
                    pose, radius, required_clearance, clearance, math.pi, None,
                )
                if stage is not None:
                    return stage
                if self._route_target_index is None:
                    return Plan(
                        (), ((pose.x_m, pose.y_m),), None,
                        0.0, 1, "obstacle-u-turn-blocked", 0.0,
                    )
                target = self._corner_centers()[self._route_target_index]
                waypoints = graph_waypoints(
                    self, pose, (target,), self._route_block_points(),
                )
                trajectory, steering, direction = self._simulate_trajectory(
                    pose, waypoints,
                )
                return Plan(
                    waypoints, trajectory, target, steering, direction,
                    "obstacle-u-turn-stage",
                    0.0 if not waypoints else self._path_speed_scale(steering),
                )
            self._obstacle_program_turn_sign = best_sign
            self._obstacle_program_stage_target = None
            self._obstacle_program_turn_start_yaw = pose.yaw_rad
            self._obstacle_program_turn_last_yaw = pose.yaw_rad
        elif self._obstacle_program_turn_start_yaw is None:
            self._obstacle_program_turn_start_yaw = pose.yaw_rad
            self._obstacle_program_turn_last_yaw = pose.yaw_rad
        turn_sign = float(self._obstacle_program_turn_sign)
        delta = math.atan2(
            math.sin(pose.yaw_rad - self._obstacle_program_turn_last_yaw),
            math.cos(pose.yaw_rad - self._obstacle_program_turn_last_yaw),
        )
        self._obstacle_program_turn_radians += max(0.0, turn_sign * delta)
        self._obstacle_program_turn_last_yaw = pose.yaw_rad
        if self._obstacle_program_turn_radians >= math.radians(170.0):
            old_step = self._route_step
            self._route_step = -old_step
            if self._route_target_index is not None:
                self._route_target_index = (
                    self._route_target_index - old_step
                ) % len(self._corner_centers())
            self._parked_target_index = None
            self._corner_phase = 0
            self._corner_index = None
            self._advance_obstacle_program(pose)
            return Plan((), ((pose.x_m, pose.y_m),), None, 0.0, 1, "obstacle-u-turn-settle", 0.0)

        remaining = max(0.0, math.pi - self._obstacle_program_turn_radians)
        trajectory = self._u_turn_trajectory(pose, int(turn_sign), remaining, radius)
        if self._trajectory_clearance(trajectory, clearance) < required_clearance:
            stage = self._u_turn_stage_plan(
                pose,
                radius,
                required_clearance,
                clearance,
                remaining,
                int(turn_sign),
            )
            if stage is not None:
                return stage
            return Plan((), ((pose.x_m, pose.y_m),), None, 0.0, 1, "obstacle-u-turn-blocked", 0.0)
        return Plan(
            (trajectory[-1],), trajectory, trajectory[-1],
            turn_sign * self.max_steering_angle_rad, 1, "obstacle-u-turn", 0.75,
        )

    def _u_turn_trajectory(
        self,
        pose: PoseEstimate,
        turn_sign: int,
        angle_rad: float,
        radius_m: float,
    ) -> tuple[tuple[float, float], ...]:
        left = (-math.sin(pose.yaw_rad), math.cos(pose.yaw_rad))
        center = (
            pose.x_m + turn_sign * radius_m * left[0],
            pose.y_m + turn_sign * radius_m * left[1],
        )
        start_angle = math.atan2(pose.y_m - center[1], pose.x_m - center[0])
        samples = max(
            2,
            int(math.ceil(angle_rad * radius_m / self.trajectory_step_m)) + 1,
        )
        return tuple(
            (
                center[0] + radius_m * math.cos(
                    start_angle + turn_sign * angle_rad * i / (samples - 1),
                ),
                center[1] + radius_m * math.sin(
                    start_angle + turn_sign * angle_rad * i / (samples - 1),
                ),
            )
            for i in range(samples)
        )

    def _trajectory_clearance(
        self,
        trajectory: tuple[tuple[float, float], ...],
        clearance: np.ndarray | None = None,
    ) -> float:
        if clearance is None:
            clearance = self._clearance_field()
        points = np.asarray(trajectory, dtype=np.float64)
        cols = np.rint(points[:, 0] / self.grid.resolution_m).astype(np.intp) + self.grid.origin_cell
        rows = self.grid.origin_cell - np.rint(points[:, 1] / self.grid.resolution_m).astype(np.intp)
        valid = (
            (rows >= 0) & (rows < self.grid.size_cells)
            & (cols >= 0) & (cols < self.grid.size_cells)
        )
        if not bool(valid.all()):
            return 0.0
        return float(clearance[rows, cols].min(initial=math.inf))

    def _clearance_field(self) -> np.ndarray:
        occupied = (
            (self.grid.cells == int(Cell.MAP_WALL))
            | (self.grid.cells == int(Cell.WALL))
            | (self.grid.cells == int(Cell.UNKNOWN_OBSTRUCTION))
            | (self.grid.cells == int(Cell.LIVE_OBSTACLE))
        )
        if self._confirmed_local_obstacles or self._previous_local_obstacles:
            occupied = occupied.copy()
            cells = self._confirmed_local_obstacles | self._previous_local_obstacles
            rows, cols = zip(*cells)
            occupied[np.asarray(rows), np.asarray(cols)] = True
        return np.asarray(
            distance_transform_edt(~occupied), dtype=np.float32,
        ) * np.float32(self.grid.resolution_m)

    def _u_turn_stage_plan(
        self,
        pose: PoseEstimate,
        radius_m: float,
        required_clearance_m: float,
        clearance: np.ndarray,
        remaining_angle_rad: float,
        turn_sign: int | None,
    ) -> Plan | None:
        target = self._obstacle_program_stage_target
        if target is None:
            forward = (math.cos(pose.yaw_rad), math.sin(pose.yaw_rad))
            current_clearance = self._trajectory_clearance(
                ((pose.x_m, pose.y_m),), clearance,
            )
            segment_required_clearance = min(
                required_clearance_m, current_clearance,
            )
            for distance_m in np.arange(0.15, 0.46, 0.05):
                for direction in (-1, 1):
                    candidate = (
                        pose.x_m + direction * float(distance_m) * forward[0],
                        pose.y_m + direction * float(distance_m) * forward[1],
                    )
                    segment = tuple(
                        (
                            pose.x_m + fraction * (candidate[0] - pose.x_m),
                            pose.y_m + fraction * (candidate[1] - pose.y_m),
                        )
                        for fraction in np.linspace(0.0, 1.0, 7)
                    )
                    candidate_pose = PoseEstimate(
                        x_m=candidate[0], y_m=candidate[1], yaw_rad=pose.yaw_rad,
                    )
                    signs = (-1, 1) if turn_sign is None else (turn_sign,)
                    arc_clearance = max(
                        self._trajectory_clearance(
                            self._u_turn_trajectory(
                                candidate_pose, sign, remaining_angle_rad, radius_m,
                            ),
                            clearance,
                        )
                        for sign in signs
                    )
                    if (
                        self._trajectory_clearance(segment, clearance)
                        >= segment_required_clearance
                        and arc_clearance >= required_clearance_m
                    ):
                        target = candidate
                        self._obstacle_program_stage_target = target
                        self._obstacle_program_stage_direction = direction
                        break
                if target is not None:
                    break
        if target is None:
            return None
        dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
        distance = math.hypot(dx, dy)
        if distance <= 0.04:
            self._obstacle_program_stage_target = None
            self._obstacle_program_stage_direction = 0
            self._obstacle_program_stage_settling = True
            return Plan(
                (), ((pose.x_m, pose.y_m),), None,
                0.0, 1, "obstacle-u-turn-stage", 0.0,
            )
        target_yaw = math.atan2(dy, dx)
        direction = self._obstacle_program_stage_direction
        if direction == 0:
            forward_projection = (
                dx * math.cos(pose.yaw_rad) + dy * math.sin(pose.yaw_rad)
            )
            direction = 1 if forward_projection >= 0.0 else -1
            self._obstacle_program_stage_direction = direction
        drive_yaw = pose.yaw_rad if direction > 0 else pose.yaw_rad + math.pi
        alpha = math.atan2(
            math.sin(target_yaw - drive_yaw),
            math.cos(target_yaw - drive_yaw),
        )
        steering = direction * math.atan2(
            2.0 * self.wheelbase_m * math.sin(alpha), max(distance, 0.05),
        )
        steering = max(
            -self.max_steering_angle_rad,
            min(self.max_steering_angle_rad, steering),
        )
        return Plan(
            (target,), ((pose.x_m, pose.y_m), target), target,
            steering, direction, "obstacle-u-turn-stage", 0.65,
        )

    def _corridor_plan(self, pose: PoseEstimate) -> Plan:
        points = self._route_points(pose)
        waypoints = graph_waypoints(self, pose, points, ((0.0, -1.0),))
        target = points[-1]
        trajectory, steering, first_direction = self._simulate_trajectory(pose, waypoints)
        return Plan(
            waypoints=waypoints,
            trajectory=trajectory,
            target=target,
            steering_angle_rad=steering,
            motion_direction=first_direction,
            reason="corridor",
            speed_scale=(
                0.0 if self._uncolored_obstacle_hold or first_direction == 0
                else self._path_speed_scale(steering)
            ),
        )

    def _grid_plan(self, pose: PoseEstimate, progress_pose: PoseEstimate) -> Plan:
        route_points = self._route_points(progress_pose, planning_pose=pose)
        waypoints = graph_waypoints(self, pose, route_points, self._route_block_points())
        trajectory, steering, direction = self._simulate_trajectory(pose, waypoints)
        return Plan(
            waypoints=waypoints,
            trajectory=trajectory,
            target=route_points[-1] if route_points else None,
            steering_angle_rad=steering,
            motion_direction=direction,
            reason="route",
            speed_scale=(
                0.0 if self._uncolored_obstacle_hold or not waypoints or direction == 0
                else self._path_speed_scale(steering)
            ),
        )

    def _path_speed_scale(self, steering: float) -> float:
        if self._lateral_maneuver_hold:
            return 0.0
        steering_fraction = min(1.0, abs(steering) / self.max_steering_angle_rad)
        return 1.0 - 0.18 * steering_fraction

    def _corner_plan(self, pose: PoseEstimate, rear_wall_m: float) -> Plan | None:
        index = self._route_target_index
        if index is None or self.grid.direction == Direction.UNKNOWN or self.course_complete:
            self._corner_phase = 0
            return None
        if self._parked_target_index == index:
            return None
        center = self._corner_centers()[index]
        center_distance = math.dist((pose.x_m, pose.y_m), center)
        if self.completed_laps > 0:
            self._corner_phase = 0
            self._corner_index = None
            if center_distance <= CORNER_ENTRY_RADIUS_M:
                self._parked_target_index = index
                self._advance_route_target(index)
                self._ensure_obstacle_program(pose)
            return None
        if self._corner_phase == 0:
            if self._lateral_maneuver_target is not None:
                return None
            if center_distance > CORNER_ENTRY_RADIUS_M:
                return None
            desired = self._corner_yaws()[index]
            outgoing = (math.cos(desired), math.sin(desired))
            center_dx, center_dy = pose.x_m - center[0], pose.y_m - center[1]
            lateral = -center_dx * outgoing[1] + center_dy * outgoing[0]
            if abs(lateral) > 0.15:
                return None
            self._corner_index = index
            self._set_corner_phase(1, pose)
        elif self._corner_index != index:
            self._corner_phase = 0
            return None

        if self._corner_phase < 6 and math.hypot(
            pose.x_m - center[0], pose.y_m - center[1],
        ) > CORNER_MANEUVER_RADIUS_M:
            self._corner_phase = 0
            self._corner_index = None
            return None

        now = time.monotonic()
        if self._corner_phase == 8:
            desired = self._corner_yaws()[index]
            outgoing = (math.cos(desired), math.sin(desired))
            target = (center[0] - 0.15 * outgoing[0], center[1] - 0.15 * outgoing[1])
            if abs(pose.odometry_speed_mps) > 0.04:
                return self._corner_command(
                    pose, target, 0, 0.0, "corner-braking",
                )
            yaw_error = math.atan2(
                math.sin(desired - pose.yaw_rad), math.cos(desired - pose.yaw_rad),
            )
            lateral = (
                -(pose.x_m - center[0]) * outgoing[1]
                + (pose.y_m - center[1]) * outgoing[0]
            )
            if (
                abs(yaw_error) > CORNER_HEADING_TOLERANCE_RAD
                or abs(lateral) > CORNER_PARK_LATERAL_TOLERANCE_M
                or math.dist((pose.x_m, pose.y_m), center) > CORNER_ENTRY_RADIUS_M
            ):
                self._corner_phase = 0
                self._corner_index = None
                return None
            self._parked_target_index = index
            self._corner_phase = 0
            self._corner_index = None
            self._advance_route_target(index)
            return self._corner_command(pose, target, 0, 0.0, "corner-parked")
        if self._corner_phase == 7:
            traveled = self._corner_distance(pose)
            if traveled >= 0.06:
                resume_phase = self._corner_recovery_resume_phase
                self._corner_recovery_attempted_phase = 0
                self._set_corner_phase(resume_phase, pose)
                return self._corner_command(
                    pose, center, 0, 0.0, "corner-recovery-settle",
                )
            if now - self._corner_phase_time < 0.10:
                return self._corner_command(
                    pose, center, 0, 0.0, "corner-recovery-settle",
                )
            return self._corner_command(
                pose,
                center,
                self._corner_recovery_direction,
                0.0,
                "corner-recovery-straight",
                1.0,
            )

        if self._corner_phase in (1, 3, 5):
            if now - self._corner_phase_time < 0.10:
                return self._corner_command(pose, center, 0, 0.0, "corner-settle")
            if self._corner_phase == 3:
                self._set_corner_phase(4, pose)
            elif self._corner_phase == 5:
                self._set_corner_phase(1, pose)
            else:
                desired = self._corner_yaws()[index]
                error = math.atan2(math.sin(desired - pose.yaw_rad), math.cos(desired - pose.yaw_rad))
                if abs(error) <= CORNER_HEADING_TOLERANCE_RAD:
                    self._set_corner_phase(6, pose)
                else:
                    self._corner_turn_sign = 1 if error > 0.0 else -1
                    self._corner_speed_scale = 1.0
                    desired_leg_m = (
                        abs(error) * self.wheelbase_m
                        / (2.0 * math.tan(self.max_steering_angle_rad))
                    )
                    self._corner_leg_m = max(
                        0.03,
                        min(0.20, desired_leg_m - 0.02),
                    )
                    self._corner_leg_yaw_rad = (
                        self._corner_leg_m
                        * math.tan(self.max_steering_angle_rad)
                        / self.wheelbase_m
                    )
                    forward_margin = self._corner_leg_wall_margin(pose, 1)
                    reverse_margin = self._corner_leg_wall_margin(pose, -1)
                    if forward_margin < 0.03 and reverse_margin < 0.03:
                        self._corner_phase = 0
                        self._corner_index = None
                        return None
                    self._corner_first_direction = (
                        -1
                        if forward_margin < 0.02 and reverse_margin > forward_margin
                        else 1
                    )
                    self._set_corner_phase(2, pose)

        if self._corner_phase in (2, 4):
            desired = self._corner_yaws()[index]
            yaw_error = math.atan2(
                math.sin(desired - pose.yaw_rad), math.cos(desired - pose.yaw_rad),
            )
            if abs(yaw_error) <= math.radians(8.0):
                self._set_corner_phase(5, pose)
                return self._corner_command(pose, center, 0, 0.0, "corner-settle")
            yaw_progress = self._corner_turn_sign * math.atan2(
                math.sin(pose.yaw_rad - self._corner_phase_yaw_rad),
                math.cos(pose.yaw_rad - self._corner_phase_yaw_rad),
            )
            leg_distance = self._corner_distance(pose)
            if (
                yaw_progress >= self._corner_leg_yaw_rad
                or leg_distance >= self._corner_leg_m + 0.04
            ):
                self._set_corner_phase(self._corner_phase + 1, pose)
                return self._corner_command(pose, center, 0, 0.0, "corner-settle")
            direction = (
                self._corner_first_direction
                if self._corner_phase == 2
                else -self._corner_first_direction
            )
            if self._corner_leg_wall_margin(pose, direction) < 0.03:
                if self._corner_leg_wall_margin(pose, -direction) >= 0.03:
                    self._set_corner_phase(self._corner_phase + 1, pose)
                    return self._corner_command(
                        pose, center, 0, 0.0, "corner-settle",
                    )
                self._corner_phase = 0
                self._corner_index = None
                return None
            if (
                self._corner_recovery_attempted_phase != self._corner_phase
                and now - self._corner_phase_time >= 0.75
                and leg_distance < 0.01
                and yaw_progress < math.radians(3.0)
            ):
                if direction < 0 and rear_wall_m < 0.14:
                    direction = 1
                self._corner_recovery_attempted_phase = self._corner_phase
                self._corner_recovery_resume_phase = self._corner_phase
                self._corner_recovery_direction = direction
                self._set_corner_phase(7, pose)
                return self._corner_command(
                    pose, center, 0, 0.0, "corner-recovery-settle",
                )
            steering = self._corner_turn_sign * self.max_steering_angle_rad * direction
            return self._corner_command(
                pose, center, direction, steering, "corner-align", self._corner_speed_scale,
            )

        if self._corner_phase == 6:
            desired = self._corner_yaws()[index]
            outgoing = (math.cos(desired), math.sin(desired))
            target = (center[0] - 0.15 * outgoing[0], center[1] - 0.15 * outgoing[1])
            center_dx, center_dy = pose.x_m - center[0], pose.y_m - center[1]
            longitudinal = center_dx * outgoing[0] + center_dy * outgoing[1]
            lateral = -center_dx * outgoing[1] + center_dy * outgoing[0]
            if abs(lateral) > CORNER_PARK_LATERAL_TOLERANCE_M:
                self._corner_phase = 0
                self._corner_index = None
                return None
            yaw_error = math.atan2(
                math.sin(desired - pose.yaw_rad), math.cos(desired - pose.yaw_rad),
            )
            aligned = abs(yaw_error) <= CORNER_HEADING_TOLERANCE_RAD
            if (
                aligned
                and longitudinal
                + min(0.0, pose.odometry_speed_mps)
                * (self.actuation_delay_s + self.decel_tau_s)
                <= -0.10
                and abs(lateral) <= CORNER_PARK_LATERAL_TOLERANCE_M
            ):
                if abs(pose.odometry_speed_mps) <= 0.04:
                    self._parked_target_index = index
                    self._corner_phase = 0
                    self._corner_index = None
                    self._advance_route_target(index)
                    return self._corner_command(
                        pose, target, 0, 0.0, "corner-parked",
                    )
                self._set_corner_phase(8, pose)
                return self._corner_command(pose, target, 0, 0.0, "corner-braking")
            if longitudinal <= -0.18:
                self._set_corner_phase(1, pose)
                return self._corner_command(pose, target, 0, 0.0, "corner-settle")
            steering = max(
                -math.radians(7.0), min(math.radians(7.0), -yaw_error),
            )
            return self._corner_command(pose, target, -1, steering, "corner-backup", 1.0)
        return self._corner_command(pose, center, 0, 0.0, "corner-settle")

    def _corner_command(
        self,
        pose: PoseEstimate,
        target: tuple[float, float],
        direction: int,
        steering: float,
        reason: str,
        speed_scale: float = 0.0,
    ) -> Plan:
        distance = 0.30 * direction
        endpoint = (
            pose.x_m + distance * math.cos(pose.yaw_rad),
            pose.y_m + distance * math.sin(pose.yaw_rad),
        )
        return Plan(
            (target,), ((pose.x_m, pose.y_m), endpoint), target,
            steering, direction or 1, reason, speed_scale,
        )

    def _set_corner_phase(self, phase: int, pose: PoseEstimate) -> None:
        if phase in (1, 3, 5, 6):
            self._corner_recovery_attempted_phase = 0
        self._corner_phase = phase
        self._corner_phase_odom_m = pose.odometry_travel_m
        self._corner_phase_yaw_rad = pose.yaw_rad
        self._corner_phase_time = time.monotonic()

    def _corner_leg_wall_margin(self, pose: PoseEstimate, direction: int) -> float:
        remaining_m = self._corner_leg_m
        if self._corner_phase in (2, 4):
            remaining_m = max(0.0, remaining_m - self._corner_distance(pose))
        distance = min(remaining_m + 0.06, 0.16)
        step = distance / 8.0
        x_m, y_m, yaw = pose.x_m, pose.y_m, pose.yaw_rad
        yaw_step = step * math.tan(
            self._corner_turn_sign * self.max_steering_angle_rad,
        ) / self.wheelbase_m
        half_length = self.robot_length_m * 0.5 + 0.03
        half_width = self.robot_width_m * 0.5 + 0.03
        x_min, x_max = (
            (-2.5, 0.5)
            if self.grid.direction == Direction.LEFT
            else (-0.5, 2.5)
        )
        margin = math.inf
        for _ in range(8):
            mid_yaw = yaw + yaw_step * 0.5
            x_m += direction * step * math.cos(mid_yaw)
            y_m += direction * step * math.sin(mid_yaw)
            yaw += yaw_step
            cy, sy = math.cos(yaw), math.sin(yaw)
            for obstacle in self._tracked_obstacles:
                if obstacle.observations < 2:
                    continue
                dx = obstacle.anchor[0] - x_m
                dy = obstacle.anchor[1] - y_m
                if (
                    abs(dx * cy + dy * sy) <= half_length + 0.04
                    and abs(-dx * sy + dy * cy) <= half_width + 0.04
                ):
                    return -1.0
            for longitudinal in (-half_length, half_length):
                for lateral in (-half_width, half_width):
                    corner_x = x_m + longitudinal * cy - lateral * sy
                    corner_y = y_m + longitudinal * sy + lateral * cy
                    cell = self.grid.world_to_cell(corner_x, corner_y)
                    if cell is None or self.grid.cells[cell] == int(Cell.MAP_WALL):
                        return -1.0
                    margin = min(
                        margin,
                        corner_x - x_min,
                        x_max - corner_x,
                        corner_y + 1.5,
                        1.5 - corner_y,
                    )
        return margin

    def _corner_distance(self, pose: PoseEstimate) -> float:
        return max(0.0, pose.odometry_travel_m - self._corner_phase_odom_m)

    def _route_points(
        self,
        pose: PoseEstimate,
        planning_pose: PoseEstimate | None = None,
    ) -> tuple[tuple[float, float], ...]:
        centers = self._corner_centers()
        if self._route_target_index is None:
            if self.grid.direction == Direction.UNKNOWN:
                course_target = (0.0, 1.0)
            else:
                self._route_target_index = self._initial_route_target(centers, pose)
                self._initial_route_target_index = self._route_target_index
                course_target = centers[self._route_target_index]
        else:
            course_target = centers[self._route_target_index]
        action = self._program_action()
        if action in ("left", "right"):
            target = self._obstacle_pass_target(pose, action)
            if target is not None:
                anchor = self._obstacle_program_anchor
                assert anchor is not None
                yaw = self._obstacle_program_approach_yaw
                forward = (math.cos(yaw), math.sin(yaw))
                left = (-forward[1], forward[0])
                side_sign = 1.0 if action == "left" else -1.0
                side_offset = self._obstacle_pass_side_offset(
                    anchor, forward, left, side_sign,
                )
                entry = (
                    anchor[0]
                    - OBSTACLE_PASS_ENTRY_LEAD_M * forward[0]
                    + side_sign * side_offset * left[0],
                    anchor[1]
                    - OBSTACLE_PASS_ENTRY_LEAD_M * forward[1]
                    + side_sign * side_offset * left[1],
                )
                gate = (
                    anchor[0] - 0.18 * forward[0]
                    + side_sign * side_offset * left[0],
                    anchor[1] - 0.18 * forward[1]
                    + side_sign * side_offset * left[1],
                )
                relative = (pose.x_m - anchor[0], pose.y_m - anchor[1])
                progress = relative[0] * forward[0] + relative[1] * forward[1]
                route_pose = pose if planning_pose is None else planning_pose
                route_relative = (
                    route_pose.x_m - anchor[0],
                    route_pose.y_m - anchor[1],
                )
                route_progress = (
                    route_relative[0] * forward[0]
                    + route_relative[1] * forward[1]
                )
                side = side_sign * (
                    relative[0] * left[0] + relative[1] * left[1]
                )
                if (
                    math.dist((pose.x_m, pose.y_m), gate) <= 0.12
                    or (
                        progress >= -0.18
                        and side >= 0.12
                    )
                ):
                    self._obstacle_program_gate_passed = True
                if progress >= 0.0 and side < 0.12 and not self._obstacle_program_gate_passed:
                    return (gate,)
                pass_points = (
                    (target,)
                    if self._obstacle_program_gate_passed
                    else (
                        (entry, gate, target)
                        if (
                            route_progress <= -(OBSTACLE_PASS_ENTRY_LEAD_M - 0.08)
                        )
                        else (gate, target)
                    )
                )
                actions = self._obstacle_program_actions
                final_action = self._obstacle_program_index + 1 >= len(actions)
                return (*pass_points, course_target) if final_action else pass_points
        return (course_target,)

    def _corner_centers(self) -> tuple[tuple[float, float], ...]:
        side = -2.0 if self.grid.direction == Direction.LEFT else 2.0
        return ((0.0, 1.0), (side, 1.0), (side, -1.0), (0.0, -1.0))

    def _corner_yaws(self) -> tuple[float, float, float, float]:
        centers = self._corner_centers()
        return cast(tuple[float, float, float, float], tuple(
            math.atan2(
                centers[(index + self._route_step) % len(centers)][1] - center[1],
                centers[(index + self._route_step) % len(centers)][0] - center[0],
            )
            for index, center in enumerate(centers)
        ))

    def _route_block_points(self) -> tuple[tuple[float, float], ...]:
        centers = self._corner_centers()
        if self._route_target_index is None or len(centers) < 4:
            return ()
        return (centers[(self._route_target_index + 2) % len(centers)],)

    def _course_centerline(self, direction: Direction | None = None) -> tuple[tuple[float, float], ...]:
        points = self._course_guide_points(direction)
        return (*points, points[0])

    def _course_guide_points(self, direction: Direction | None = None) -> tuple[tuple[float, float], ...]:
        effective_direction = self.grid.direction if direction is None else direction
        side_x = -2.0 if effective_direction == Direction.LEFT else 2.0
        centers = ((0.0, 1.0), (side_x, 1.0), (side_x, -1.0), (0.0, -1.0))
        yaws = tuple(
            math.atan2(
                centers[(index + self._route_step) % len(centers)][1] - center[1],
                centers[(index + self._route_step) % len(centers)][0] - center[0],
            )
            for index, center in enumerate(centers)
        )
        offset_m = 0.25
        return tuple(
            point
            for center, yaw in zip(centers, yaws)
            for point in (
                (center[0] - offset_m * math.cos(yaw), center[1] - offset_m * math.sin(yaw)),
                (center[0] + offset_m * math.cos(yaw), center[1] + offset_m * math.sin(yaw)),
            )
        )

    def _update_lap_progress(
        self,
        pose: PoseEstimate,
        allow_completion: bool,
    ) -> None:
        if self._lap_start_pose is not None:
            start_x, start_y, start_yaw = self._lap_start_pose
            dx, dy = pose.x_m - start_x, pose.y_m - start_y
            start_distance = math.hypot(dx, dy)
            longitudinal = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
            lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
            crossed_start = abs(longitudinal) <= 0.05 and abs(lateral) <= 0.50
            previous = self._lap_previous_progress_pose
            if previous is not None:
                previous_dx = previous[0] - start_x
                previous_dy = previous[1] - start_y
                previous_longitudinal = (
                    math.cos(start_yaw) * previous_dx
                    + math.sin(start_yaw) * previous_dy
                )
                previous_lateral = (
                    -math.sin(start_yaw) * previous_dx
                    + math.cos(start_yaw) * previous_dy
                )
                denominator = previous_longitudinal - longitudinal
                if previous_longitudinal * longitudinal <= 0.0 and abs(denominator) > 1.0e-6:
                    fraction = previous_longitudinal / denominator
                    crossing_lateral = previous_lateral + fraction * (
                        lateral - previous_lateral
                    )
                    crossed_start = crossed_start or abs(crossing_lateral) <= 0.50
            self._lap_previous_progress_pose = (pose.x_m, pose.y_m)
            if start_distance >= 0.60:
                self._lap_departed_start = True
            if (
                allow_completion
                and self._lap_return_pending
                and self._lap_departed_start
                and crossed_start
            ):
                self.completed_laps += 1
                self._lap_return_pending = False
                self._lap_departed_start = False
                self._lap_visited_targets.clear()

    def _advance_route_target(self, completed_index: int) -> None:
        centers = self._corner_centers()
        if self._route_target_index != completed_index or not centers:
            return
        self._lap_visited_targets.add(completed_index)
        next_index = (completed_index + self._route_step) % len(centers)
        self._route_target_index = next_index
        if len(self._lap_visited_targets) == len(centers):
            self._lap_return_pending = True

    @property
    def course_complete(self) -> bool:
        return self.completed_laps >= self.target_laps

    def _initial_route_target(self, centers: tuple[tuple[float, float], ...], pose: PoseEstimate) -> int:
        nearest = min(
            range(len(centers)),
            key=lambda i: math.hypot(centers[i][0] - pose.x_m, centers[i][1] - pose.y_m),
        )
        if math.hypot(centers[nearest][0] - pose.x_m, centers[nearest][1] - pose.y_m) <= 0.20:
            return nearest
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

    def _simulate_trajectory(
        self,
        pose: PoseEstimate,
        guide_points: tuple[tuple[float, float], ...],
    ) -> tuple[tuple[tuple[float, float], ...], float, int]:
        trajectory = ((pose.x_m, pose.y_m), *guide_points)
        return trajectory, self._heading_steering_rad, self._heading_motion_direction
