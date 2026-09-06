from __future__ import annotations

import math
import unittest
import unittest.mock

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt

from planner.graph_path import (
    _arc_clears_geometry,
    _arc_follows_course,
    _bounded_clearance,
    _feasible_route_entry,
    _footprint_overlap_counts,
    _kinematic_arc_poses,
    _lateral_maneuver_path,
    _path_max_curvature,
    _route_command,
    _spline_path_candidate,
    _uncolored_obstacle_path,
    active_obstacle_component_mask,
    graph_waypoints,
    tracked_obstacle_overlap,
)
from planner.grid import Cell, Direction, GridMap
from planner.localize_types import PoseEstimate
from planner.planner import GridPlanner, TrackedObstacle


class GraphPathTest(unittest.TestCase):
    def setUp(self) -> None:
        self.planner = GridPlanner(GridMap())
        self.planner.grid.lock_direction(Direction.RIGHT)
        self.planner._route_target_index = 2
        self.pose = PoseEstimate(
            x_m=0.38,
            y_m=-0.77,
            yaw_rad=math.radians(-13.0),
        )
        cells = (
            self.planner.grid.world_to_cell(0.35, -0.85),
            self.planner.grid.world_to_cell(0.40, -0.85),
        )
        self.obstacle_cells = {cell for cell in cells if cell is not None}

    @staticmethod
    def _through_target(
        path: tuple[tuple[float, float], ...],
        target: tuple[float, float],
    ) -> tuple[tuple[float, float], ...]:
        closest = min(
            range(len(path)), key=lambda index: math.dist(path[index], target),
        )
        return path[:closest + 1]

    def test_one_frame_cluster_does_not_flip_whole_route_to_reverse(self) -> None:
        self.planner._previous_local_obstacles = self.obstacle_cells

        path = graph_waypoints(self.planner, self.pose, ((1.40, -0.65),))

        self.assertTrue(path)
        self.assertEqual(self.planner._heading_motion_direction, 1)

    def test_prelock_filtered_wall_fragment_does_not_trigger_color_backup(self) -> None:
        planner = GridPlanner(GridMap())
        pose = PoseEstimate(0.0, -0.30, math.radians(90.0))
        wall_fragment = planner.grid.world_to_cell(0.40, 0.70)
        self.assertIsNotNone(wall_fragment)
        assert wall_fragment is not None
        planner._previous_local_obstacles = {wall_fragment}
        planner._color_lidar_obstacles = set()
        path = np.asarray(((0.0, -0.30), (0.0, 1.0)), dtype=np.float32)

        result, backing, holding = _uncolored_obstacle_path(
            planner, pose, path, False,
        )

        np.testing.assert_array_equal(result, path)
        self.assertFalse(backing)
        self.assertFalse(holding)
        self.assertIsNone(planner._uncolored_obstacle_anchor)

    def test_active_uncolored_anchor_survives_live_lidar_dropout(self) -> None:
        anchor = (0.65, -0.75)
        cell = self.planner.grid.world_to_cell(*anchor)
        self.assertIsNotNone(cell)
        assert cell is not None
        self.planner._confirmed_local_obstacles = {cell}
        self.planner._uncolored_obstacle_anchor = anchor
        self.planner._uncolored_obstacle_started_at = 0.0
        path = np.asarray(
            ((self.pose.x_m, self.pose.y_m), (0.80, -0.70)),
            dtype=np.float32,
        )

        _path, backing, holding = _uncolored_obstacle_path(
            self.planner, self.pose, path, False,
        )

        self.assertEqual(self.planner._uncolored_obstacle_anchor, anchor)
        self.assertTrue(backing or holding)

    def test_stale_unknown_cell_does_not_block_color_recovery_backup(self) -> None:
        pose = PoseEstimate(1.50, -1.0, 0.0)
        anchor = (1.80, -1.0)
        anchor_cell = self.planner.grid.world_to_cell(*anchor)
        stale_cell = self.planner.grid.world_to_cell(1.35, -1.0)
        self.assertIsNotNone(anchor_cell)
        self.assertIsNotNone(stale_cell)
        assert anchor_cell is not None and stale_cell is not None
        self.planner._confirmed_local_obstacles = {anchor_cell}
        self.planner.grid.cells[stale_cell] = int(Cell.UNKNOWN_OBSTRUCTION)
        self.planner._uncolored_obstacle_anchor = anchor
        self.planner._uncolored_obstacle_started_at = 0.0
        path = np.asarray(((1.50, -1.0), (2.0, -1.0)), dtype=np.float32)

        _path, backing, holding = _uncolored_obstacle_path(
            self.planner, pose, path, False,
        )

        self.assertTrue(backing)
        self.assertFalse(holding)

    def test_color_recovery_uses_route_when_backup_hits_outer_wall(self) -> None:
        planner = GridPlanner(GridMap())
        planner.grid.lock_direction(Direction.LEFT)
        pose = PoseEstimate(-2.15, -1.0, 0.0)
        anchor = (-1.70, -1.0)
        anchor_cell = planner.grid.world_to_cell(*anchor)
        self.assertIsNotNone(anchor_cell)
        assert anchor_cell is not None
        planner._confirmed_local_obstacles = {anchor_cell}
        planner._uncolored_obstacle_anchor = anchor
        planner._uncolored_obstacle_started_at = 0.0
        path = np.asarray(((-2.15, -1.0), (0.0, -1.0)), dtype=np.float32)

        _path, backing, holding = _uncolored_obstacle_path(
            planner, pose, path, False,
        )

        self.assertFalse(backing)
        self.assertFalse(holding)

    def test_color_recovery_holds_before_close_obstacle_when_backup_blocked(self) -> None:
        planner = GridPlanner(GridMap())
        planner.grid.lock_direction(Direction.LEFT)
        pose = PoseEstimate(-2.15, -1.0, 0.0)
        anchor = (-1.82, -1.0)
        anchor_cell = planner.grid.world_to_cell(*anchor)
        self.assertIsNotNone(anchor_cell)
        assert anchor_cell is not None
        planner._confirmed_local_obstacles = {anchor_cell}
        planner._uncolored_obstacle_anchor = anchor
        planner._uncolored_obstacle_started_at = 0.0
        path = np.asarray(((-2.15, -1.0), (0.0, -1.0)), dtype=np.float32)

        _path, backing, holding = _uncolored_obstacle_path(
            planner, pose, path, False,
        )

        self.assertFalse(backing)
        self.assertTrue(holding)

    def test_localization_shifted_copy_of_colored_obstacle_is_not_unknown(self) -> None:
        colored_anchor = (0.65, -0.75)
        duplicate_cell = self.planner.grid.world_to_cell(0.90, -0.75)
        self.assertIsNotNone(duplicate_cell)
        assert duplicate_cell is not None
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), colored_anchor, math.inf, 3),
        ]
        self.planner._color_lidar_obstacles = {duplicate_cell}
        path = np.asarray(
            ((self.pose.x_m, self.pose.y_m), (1.20, -0.75)),
            dtype=np.float32,
        )

        result, backing, holding = _uncolored_obstacle_path(
            self.planner, self.pose, path, False,
        )

        np.testing.assert_array_equal(result, path)
        self.assertFalse(backing)
        self.assertFalse(holding)
        self.assertIsNone(self.planner._uncolored_obstacle_anchor)

    def test_colored_obstacle_component_edges_are_not_new_obstacles(self) -> None:
        colored_anchor = (0.65, -0.75)
        component = {
            self.planner.grid.world_to_cell(x_m, -0.75)
            for x_m in np.arange(0.65, 1.01, 0.05)
        }
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), colored_anchor, math.inf, 3),
        ]
        self.planner._color_lidar_obstacles = {
            cell for cell in component if cell is not None
        }
        path = np.asarray(
            ((self.pose.x_m, self.pose.y_m), (1.20, -0.75)),
            dtype=np.float32,
        )

        result, backing, holding = _uncolored_obstacle_path(
            self.planner, self.pose, path, False,
        )

        np.testing.assert_array_equal(result, path)
        self.assertFalse(backing)
        self.assertFalse(holding)
        self.assertIsNone(self.planner._uncolored_obstacle_anchor)

    def test_separate_component_still_requires_color(self) -> None:
        colored_anchor = (0.65, -0.75)
        colored_cell = self.planner.grid.world_to_cell(*colored_anchor)
        unknown_cell = self.planner.grid.world_to_cell(0.95, -0.75)
        self.assertIsNotNone(colored_cell)
        self.assertIsNotNone(unknown_cell)
        assert colored_cell is not None and unknown_cell is not None
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), colored_anchor, math.inf, 3),
        ]
        self.planner._color_lidar_obstacles = {colored_cell, unknown_cell}
        path = np.asarray(
            ((self.pose.x_m, self.pose.y_m), (1.20, -0.75)),
            dtype=np.float32,
        )

        _result, backing, holding = _uncolored_obstacle_path(
            self.planner, self.pose, path, False,
        )

        self.assertTrue(backing or holding)
        self.assertIsNotNone(self.planner._uncolored_obstacle_anchor)

    def test_color_recovery_retries_safe_backup_after_camera_wait(self) -> None:
        pose = PoseEstimate(1.50, -1.0, 0.0)
        anchor = (1.80, -1.0)
        anchor_cell = self.planner.grid.world_to_cell(*anchor)
        self.assertIsNotNone(anchor_cell)
        assert anchor_cell is not None
        self.planner._confirmed_local_obstacles = {anchor_cell}
        self.planner._uncolored_obstacle_anchor = anchor
        self.planner._uncolored_obstacle_started_at = 1.0
        self.planner._uncolored_backup_done = True
        path = np.asarray(((1.50, -1.0), (2.0, -1.0)), dtype=np.float32)

        with unittest.mock.patch("planner.graph_path.time.monotonic", return_value=2.1):
            _path, backing, holding = _uncolored_obstacle_path(
                self.planner, pose, path, False,
            )

        self.assertTrue(backing)
        self.assertFalse(holding)

    def test_near_start_path_fold_does_not_reverse_forward_route(self) -> None:
        path = np.asarray(
            ((0.0, 0.0), (-0.08, 0.0), (-0.02, 0.0), (0.18, 0.0)),
            dtype=np.float32,
        )

        direction, _steering = _route_command(
            self.planner, PoseEstimate(yaw_rad=0.0), path,
        )

        self.assertEqual(direction, 1)

    def test_short_guide_cusp_does_not_reverse_forward_course(self) -> None:
        path = np.asarray((
            (-0.12, 0.91),
            (-0.16, 0.89),
            (-0.15, 0.91),
            (-0.12, 0.94),
            (-0.04, 0.98),
            (0.00, 1.00),
        ), dtype=np.float32)

        direction, _steering = _route_command(
            self.planner,
            PoseEstimate(x_m=-0.12, y_m=0.91, yaw_rad=math.radians(63.0)),
            path,
        )

        self.assertEqual(direction, 1)

    def test_direction_follows_executable_start_of_curved_route(self) -> None:
        path = np.asarray((
            (-0.13, 0.47),
            (-0.22, 0.57),
            (-0.20, 0.66),
            (-0.12, 0.79),
            (-0.07, 0.87),
            (0.00, 1.00),
        ), dtype=np.float32)

        direction, _steering = _route_command(
            self.planner,
            PoseEstimate(x_m=-0.13, y_m=0.47, yaw_rad=math.radians(-174.0)),
            path,
        )

        self.assertEqual(direction, 1)

    def test_lateral_route_jitter_does_not_flip_motion_direction(self) -> None:
        pose = PoseEstimate(x_m=-1.79, y_m=0.95, yaw_rad=math.radians(-90.0))
        paths = (
            np.asarray(((-1.79, 0.95), (-1.84, 1.00), (-2.00, 1.00))),
            np.asarray(((-1.79, 0.95), (-1.84, 0.90), (-2.00, 1.00))),
        )

        directions = tuple(_route_command(self.planner, pose, path)[0] for path in paths)

        self.assertEqual(directions, (1, 1))

    def test_clear_route_near_corner_does_not_make_heading_prefix_loop(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 0
        pose = PoseEstimate(
            x_m=-0.10,
            y_m=1.30,
            yaw_rad=math.radians(35.0),
        )
        target = (0.0, 1.0)

        path = graph_waypoints(
            self.planner,
            pose,
            (target,),
            self.planner._route_block_points(),
        )

        self.assertTrue(path)
        distances = [math.dist(point, target) for point in path]
        self.assertLessEqual(max(distances), math.dist((pose.x_m, pose.y_m), target) + 0.01)

    def test_blocked_forward_entry_materializes_reverse_arc(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 1
        obstacle = TrackedObstacle(
            "green", ("left",), (-1.52, 0.88), math.inf, 3,
        )
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = math.pi
        obstacle_cell = self.planner.grid.world_to_cell(*obstacle.anchor)
        self.assertIsNotNone(obstacle_cell)
        assert obstacle_cell is not None
        self.planner._confirmed_local_obstacles = {obstacle_cell}
        self.planner._previous_local_obstacles = {obstacle_cell}
        pose = PoseEstimate(
            x_m=-1.407,
            y_m=0.668,
            yaw_rad=math.radians(-127.6),
        )

        path = graph_waypoints(
            self.planner,
            pose,
            self.planner._route_points(pose, planning_pose=pose),
            self.planner._route_block_points(),
        )

        self.assertTrue(path)
        self.assertTrue(self.planner._route_entry_recovery)
        self.assertEqual(self.planner._heading_motion_direction, -1)
        heading = (math.cos(pose.yaw_rad), math.sin(pose.yaw_rad))
        first_step = (path[0][0] - pose.x_m, path[0][1] - pose.y_m)
        self.assertLess(first_step[0] * heading[0] + first_step[1] * heading[1], 0.0)

    def test_recorded_wall_contact_selects_departing_reverse(self) -> None:
        planner = GridPlanner(GridMap())
        planner.grid.lock_direction(Direction.LEFT)
        pose = PoseEstimate(-1.473, 0.608, math.radians(-145.8))
        geometry = planner.grid.cells == int(Cell.MAP_WALL)
        clearance = np.asarray(distance_transform_edt(~geometry)) * planner.grid.resolution_m
        path = np.asarray(((-1.473, 0.608), (-1.65, 0.69), (-2.0, 1.0)))

        _path, command = _feasible_route_entry(planner, pose, path, geometry, clearance)

        self.assertIsNotNone(command)
        assert command is not None
        self.assertEqual(command[0], -1)
        arc = _kinematic_arc_poses(planner, pose, *command, 0.20)
        self.assertTrue(_arc_clears_geometry(planner, arc, geometry))
        crossing = _kinematic_arc_poses(planner, pose, 1, math.radians(-24), 0.20)
        self.assertFalse(_arc_clears_geometry(planner, crossing, geometry))

    def test_tracked_rectangle_overlap_matches_opencv(self) -> None:
        obstacle = TrackedObstacle("red", ("right",), (0.023, -0.017), math.inf, 3)
        self.planner._tracked_obstacles = [obstacle]
        rng = np.random.default_rng(11)
        poses = rng.uniform((-.20, -.20, -math.pi), (.20, .20, math.pi), (1000, 3))

        overlap = tracked_obstacle_overlap(self.planner, poses)[:, 0] > 1.0e-6
        expected = []
        for x_m, y_m, yaw in poses:
            intersection, _ = cv2.rotatedRectangleIntersection(
                ((float(x_m), float(y_m)), (0.22, 0.15), math.degrees(float(yaw))),
                (obstacle.anchor, (0.05, 0.05), 0.0),
            )
            expected.append(intersection != cv2.INTERSECT_NONE)
        np.testing.assert_array_equal(overlap, expected)

    def test_off_grid_obstacle_boundary_blocks_contact_and_allows_departure(self) -> None:
        obstacle = TrackedObstacle("red", ("right",), (-2.17, -0.47), math.inf, 3)
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._tracked_obstacles = [obstacle]
        geometry = self.planner.grid.cells == int(Cell.MAP_WALL)
        cell = self.planner.grid.world_to_cell(*obstacle.anchor)
        assert cell is not None
        geometry[cell] = True
        pose = PoseEstimate(-2.31, -0.47, 0.0)

        crossing = _kinematic_arc_poses(self.planner, pose, 1, 0.0, .02)
        departing = _kinematic_arc_poses(
            self.planner, PoseEstimate(-2.29, -.47, 0.0), -1, 0.0, .03,
        )

        self.assertFalse(np.any(_footprint_overlap_counts(self.planner, crossing, geometry)))
        self.assertFalse(_arc_clears_geometry(self.planner, crossing, geometry))
        self.assertTrue(_arc_clears_geometry(self.planner, departing, geometry))

    def test_recorded_recovery_passes_obstacle_without_direction_limit_cycle(self) -> None:
        for mirror in (1, -1):
            with self.subTest(mirror=mirror):
                grid = GridMap()
                grid.lock_direction(Direction.LEFT if mirror == 1 else Direction.RIGHT)
                planner = GridPlanner(grid)
                planner._route_target_index = 2
                obstacle = TrackedObstacle(
                    "red" if mirror == 1 else "green",
                    ("right",) if mirror == 1 else ("left",),
                    (-2.17 * mirror, -.47), math.inf, 3,
                )
                planner._tracked_obstacles = [obstacle]
                planner._obstacle_program_anchor = obstacle.anchor
                planner._obstacle_program_actions = obstacle.actions
                planner._obstacle_program_approach_yaw = -math.pi / 2.0
                cell = grid.world_to_cell(*obstacle.anchor)
                assert cell is not None
                planner._confirmed_local_obstacles = {cell}
                planner._previous_local_obstacles = {cell}
                planner._colored_obstacle_anchors = [obstacle.anchor]
                geometry = grid.cells == int(Cell.MAP_WALL)
                geometry[cell] = True
                yaw = math.radians(-70)
                pose = PoseEstimate(-2.2 * mirror, -.1, yaw if mirror == 1 else math.pi - yaw)
                target = planner._corner_centers()[2]
                directions = []
                passed = False
                for _ in range(80):
                    path = graph_waypoints(planner, pose, planner._route_points(pose), planner._route_block_points())
                    self.assertTrue(path)
                    direction, steering = planner._heading_motion_direction, planner._heading_steering_rad
                    directions.append(direction)
                    arc = _kinematic_arc_poses(planner, pose, direction, steering, .02)
                    self.assertTrue(_arc_clears_geometry(planner, arc, geometry))
                    self.assertFalse(np.any(tracked_obstacle_overlap(planner, arc) > 1.0e-6))
                    x_m, y_m, yaw = map(float, arc[-1])
                    pose = PoseEstimate(x_m, y_m, yaw)
                    if not passed and pose.y_m <= obstacle.anchor[1]:
                        self.assertLess(mirror * (pose.x_m - obstacle.anchor[0]), -.10)
                        passed = True
                    if math.dist((pose.x_m, pose.y_m), target) < .15:
                        break
                self.assertTrue(passed)
                self.assertLess(math.dist((pose.x_m, pose.y_m), target), .15)
                self.assertLessEqual(sum(a * b < 0 for a, b in zip(directions, directions[1:])), 12)

    def test_lateral_maneuver_stops_with_delay_and_coast_lead(self) -> None:
        self.planner._lateral_maneuver_target = (0.0, 0.10)
        self.planner._lateral_maneuver_phase = 4
        self.planner._lateral_maneuver_arc_m = 0.12
        self.planner._lateral_maneuver_sign = 1
        self.planner._lateral_maneuver_direction = -1
        path = np.asarray(((0.0, 0.0), (0.0, 0.10)), dtype=np.float32)
        clearance = np.ones(self.planner.grid.cells.shape, dtype=np.float32)

        _path, command = _lateral_maneuver_path(
            self.planner,
            PoseEstimate(odometry_travel_m=0.04),
            path,
            clearance,
        )

        self.assertEqual(self.planner._lateral_maneuver_phase, 5)
        self.assertTrue(self.planner._lateral_maneuver_hold)
        self.assertEqual(command, (-1, 0.0))

    def test_lateral_maneuver_aborts_when_remaining_arc_loses_clearance(self) -> None:
        self.planner._lateral_maneuver_target = (0.0, 0.10)
        self.planner._lateral_maneuver_phase = 2
        self.planner._lateral_maneuver_arc_m = 0.15
        self.planner._lateral_maneuver_sign = 1
        self.planner._lateral_maneuver_direction = -1
        path = np.asarray(((0.0, 0.0), (0.0, 0.10)), dtype=np.float32)
        clearance = np.zeros(self.planner.grid.cells.shape, dtype=np.float32)

        returned, command = _lateral_maneuver_path(
            self.planner,
            PoseEstimate(),
            path,
            clearance,
        )

        np.testing.assert_array_equal(returned, path)
        self.assertEqual(self.planner._lateral_maneuver_phase, 5)
        self.assertTrue(self.planner._lateral_maneuver_hold)
        self.assertEqual(command, (-1, 0.0))

    def test_forced_backup_does_not_steer_toward_later_forward_route(self) -> None:
        pose = PoseEstimate(x_m=-1.82, y_m=-0.45, yaw_rad=math.radians(-90.0))
        path = np.asarray((
            (-1.82, -0.45),
            (-1.82, -0.35),
            (-1.70, -0.60),
            (-2.00, -1.00),
        ))

        direction, steering = _route_command(
            self.planner, pose, path, forced_direction=-1,
        )

        self.assertEqual(direction, -1)
        self.assertAlmostEqual(steering, 0.0, places=5)

    def test_confirmed_cluster_retains_existing_overlap_escape(self) -> None:
        self.planner._route_step = -1
        self.planner._confirmed_local_obstacles = self.obstacle_cells

        path = graph_waypoints(self.planner, self.pose, ((1.40, -0.65),))

        self.assertTrue(path)
        self.assertFalse(self.planner._route_entry_recovery)
        self.assertEqual(self.planner._heading_motion_direction, -1)

    def test_route_plans_both_passing_sides_before_first_obstacle(self) -> None:
        for direction in (Direction.LEFT, Direction.RIGHT):
            for step in (1, -1):
                for target_index in range(4):
                    with self.subTest(direction=direction, step=step, target=target_index):
                        planner = GridPlanner(GridMap())
                        planner.grid.lock_direction(direction)
                        planner._route_step = step
                        planner._route_target_index = target_index
                        centers = planner._corner_centers()
                        start = np.asarray(centers[(target_index - step) % 4])
                        target = centers[target_index]
                        forward = (np.asarray(target) - start) / 2.0
                        left = np.asarray((-forward[1], forward[0]))
                        yaw = math.atan2(forward[1], forward[0])
                        pose = PoseEstimate(float(start[0]), float(start[1]), yaw)
                        for label, action, distance in (("red", "right", 0.55), ("green", "left", 1.45)):
                            anchor = tuple(start + distance * forward)
                            planner._tracked_obstacles.append(TrackedObstacle(label, (action,), anchor, math.inf, 3))
                            cell = planner.grid.world_to_cell(*anchor)
                            assert cell is not None
                            planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
                            planner._confirmed_local_obstacles.add(cell)
                            planner._previous_local_obstacles.add(cell)
                        planner._ensure_obstacle_program(pose)
                        passes = planner._route_obstacle_passes(pose)
                        self.assertEqual(len(passes), 2)
                        path = np.asarray(graph_waypoints(planner, pose, planner._route_points(pose)))
                        self.assertTrue(planner._graph_search_connected)
                        np.testing.assert_allclose(path[-1], target, atol=0.05)
                        for obstacle in planner._tracked_obstacles:
                            relative = path - obstacle.anchor
                            crossing = np.argmin(np.abs(relative @ forward))
                            sign = 1.0 if obstacle.actions == ("left",) else -1.0
                            self.assertGreater(sign * (relative[crossing] @ left), 0.12)

    def test_course_constraint_allows_backup_but_not_backward_progress(self) -> None:
        for direction in (Direction.LEFT, Direction.RIGHT):
            for step in (1, -1):
                planner = GridPlanner(GridMap())
                planner.grid.lock_direction(direction)
                planner._route_step = step
                for index in range(4):
                    centers = planner._corner_centers()
                    a, b = np.asarray(centers[index]), np.asarray(centers[(index + step) % 4])
                    center = (a + b) * 0.5
                    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                    forward_pose = PoseEstimate(float(center[0]), float(center[1]), yaw)
                    course = planner._course_tangent(forward_pose)
                    backup = _kinematic_arc_poses(planner, forward_pose, -1, 0.0, 0.20)
                    self.assertTrue(_arc_follows_course(backup, -1, course))
                    backward_pose = PoseEstimate(float(center[0]), float(center[1]), yaw + math.pi)
                    backward_progress = _kinematic_arc_poses(planner, backward_pose, -1, 0.0, 0.20)
                    self.assertFalse(_arc_follows_course(backward_progress, -1, course))
                    near_limit = PoseEstimate(float(center[0]), float(center[1]), yaw + math.radians(85))
                    pivot = _kinematic_arc_poses(planner, near_limit, 1, planner.max_steering_angle_rad, 0.20)
                    self.assertFalse(_arc_follows_course(pivot, 1, course))

    def test_clear_route_does_not_drive_backwards_around_course(self) -> None:
        planner = GridPlanner(GridMap())
        planner.grid.lock_direction(Direction.RIGHT)
        planner._route_target_index = 1
        pose = PoseEstimate(1.0, 1.0, math.pi)
        geometry = planner.grid.cells == int(Cell.MAP_WALL)
        clearance = np.asarray(distance_transform_edt(~geometry)) * planner.grid.resolution_m
        path = np.asarray(((1.0, 1.0), (1.3, 1.0), (2.0, 1.0)))

        _, command = _feasible_route_entry(planner, pose, path, geometry, clearance)

        self.assertIsNotNone(command)
        assert command is not None
        self.assertEqual(command[0], 1)
        arc = _kinematic_arc_poses(planner, pose, *command, 0.20)
        self.assertTrue(_arc_clears_geometry(planner, arc, geometry))
        self.assertTrue(_arc_follows_course(arc, command[0], planner._course_tangent(pose)))

    def test_clear_corner_route_remains_smooth_after_relaxation(self) -> None:
        path = graph_waypoints(self.planner, self.pose, ((2.0, -1.0),))

        self.assertTrue(path)
        steering_curvature = math.tan(
            self.planner.max_steering_angle_rad,
        ) / self.planner.wheelbase_m
        self.assertLess(self.planner._path_max_curvature, steering_curvature)
        self.assertGreater(self.planner._path_min_clearance, 0.25)

    def test_disk_clear_spline_is_not_rejected_by_heading_overlap_raster(self) -> None:
        pose = PoseEstimate(1.227, 0.719, math.radians(9.9))
        self.planner._route_target_index = 1
        for point in ((1.10, 0.80), (1.10, 0.85), (1.05, 0.80), (1.05, 0.85)):
            cell = self.planner.grid.world_to_cell(*point)
            self.assertIsNotNone(cell)
            assert cell is not None
            self.planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
            self.planner._confirmed_local_obstacles.add(cell)

        path = graph_waypoints(
            self.planner,
            pose,
            ((2.0, 1.0),),
            self.planner._route_block_points(),
        )

        self.assertTrue(path)
        self.assertTrue(self.planner._graph_search_connected)
        self.assertLess(self.planner._path_max_curvature, 3.2)
        self.assertGreater(self.planner._path_min_clearance, 0.15)

    def test_spline_can_smooth_route_starting_inside_clearance_margin(self) -> None:
        points = np.asarray((
            (0.0, 0.0),
            (0.0, 0.30),
            (0.10, 0.50),
            (0.40, 0.60),
            (0.80, 0.60),
        ), dtype=np.float32)
        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        distance = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        wanted = np.linspace(0.0, float(distance[-1]), 30, dtype=np.float32)
        path = np.column_stack((
            np.interp(wanted, distance, points[:, 0]),
            np.interp(wanted, distance, points[:, 1]),
        )).astype(np.float32)
        clearance = np.ones(self.planner.grid.cells.shape, dtype=np.float32)
        start = self.planner.grid.world_to_cell(*path[0])
        self.assertIsNotNone(start)
        assert start is not None
        clearance[start] = 0.05
        footprint_overlap = np.zeros_like(clearance)

        smoothed = _spline_path_candidate(
            self.planner,
            points,
            distance,
            path,
            clearance,
            hard_clearance=0.15,
            footprint_overlap=footprint_overlap,
        )

        np.testing.assert_array_equal(smoothed[0], path[0])
        self.assertLess(_path_max_curvature(smoothed), _path_max_curvature(path))

    def test_route_recovers_away_from_outer_wall_localization_error(self) -> None:
        pose = PoseEstimate(-0.36, -0.85, math.radians(-170.0))

        path = graph_waypoints(self.planner, pose, ((0.0, -1.0),))

        self.assertTrue(path)
        self.assertTrue(self.planner._graph_search_connected)
        self.assertGreater(min(point[0] for point in path), -0.37)
        self.assertGreater(path[-1][0], -0.05)

    def test_bounded_clearance_matches_full_field_inside_course(self) -> None:
        mask = np.zeros(self.planner.grid.cells.shape, dtype=np.bool_)
        for point in ((0.4, -0.8), (1.5, 0.9), (1.9, -0.4)):
            cell = self.planner.grid.world_to_cell(*point)
            self.assertIsNotNone(cell)
            mask[cell] = True
        map_rows, map_cols = np.nonzero(
            self.planner.grid.cells == int(Cell.MAP_WALL),
        )
        bounds = (
            int(map_rows.min()), int(map_rows.max()) + 1,
            int(map_cols.min()), int(map_cols.max()) + 1,
        )

        bounded = _bounded_clearance(mask, self.planner.grid.resolution_m, bounds)
        full = np.asarray(distance_transform_edt(~mask), dtype=np.float32)
        full *= self.planner.grid.resolution_m
        row0, row1, col0, col1 = bounds

        np.testing.assert_allclose(
            bounded[row0:row1, col0:col1],
            full[row0:row1, col0:col1],
            rtol=1.0e-6,
        )

    def test_left_action_approaches_obstacle_on_left_side(self) -> None:
        pose = PoseEstimate(0.65, -0.82, math.radians(37.0))
        obstacle = TrackedObstacle(
            "yellow", ("left",), (1.52, -0.88), math.inf, 3,
        )
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_approach_yaw = pose.yaw_rad
        for x_m in (1.45, 1.50, 1.55):
            for y_m in (-0.95, -0.90, -0.85):
                cell = self.planner.grid.world_to_cell(x_m, y_m)
                self.assertIsNotNone(cell)
                assert cell is not None
                self.planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
                self.planner._confirmed_local_obstacles.add(cell)
        target = self.planner._obstacle_pass_target(pose, "left")
        self.assertIsNotNone(target)
        assert target is not None

        path = graph_waypoints(
            self.planner,
            pose,
            (target,),
            self.planner._route_block_points(),
        )

        forward = (math.cos(pose.yaw_rad), math.sin(pose.yaw_rad))
        left = (-forward[1], forward[0])
        relative = np.asarray(path) - np.asarray(obstacle.anchor)
        along = relative[:, 0] * forward[0] + relative[:, 1] * forward[1]
        side = relative[:, 0] * left[0] + relative[:, 1] * left[1]
        crossing = int(np.argmin(np.abs(along)))
        self.assertGreater(side[crossing], 0.12)
        self.assertGreater(self.planner._path_min_clearance, 0.05)

    def test_right_action_path_crosses_the_declared_side(self) -> None:
        pose = PoseEstimate(0.24, 0.61, math.radians(88.0))
        obstacle = TrackedObstacle(
            "red", ("right",), (0.50, 0.88), math.inf, 3,
        )
        self.planner._route_target_index = 0
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_approach_yaw = pose.yaw_rad
        for x_m in (0.45, 0.50, 0.55):
            for y_m in (0.85, 0.90):
                cell = self.planner.grid.world_to_cell(x_m, y_m)
                self.assertIsNotNone(cell)
                assert cell is not None
                self.planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
                self.planner._confirmed_local_obstacles.add(cell)

        pass_route = self.planner._route_points(pose)
        path = graph_waypoints(self.planner, pose, pass_route)

        forward = np.asarray((math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)))
        right = np.asarray((forward[1], -forward[0]))
        relative = np.asarray(path) - np.asarray(obstacle.anchor)
        crossing = int(np.argmin(np.abs(relative @ forward)))
        self.assertGreater(relative[crossing] @ right, 0.12)
        self.assertTrue(self.planner._graph_search_connected)

    def test_active_obstacle_noise_does_not_create_large_route_loop(self) -> None:
        pose = PoseEstimate(0.34, 0.73, math.radians(-30.0))
        obstacle = TrackedObstacle(
            "red", ("right",), (0.47, 0.88), math.inf, 3,
        )
        self.planner._route_target_index = 1
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_approach_yaw = 0.0
        for point in (
            (0.45, 0.85),
            (0.50, 0.85),
            (0.50, 0.90),
            (0.25, 0.70),
        ):
            cell = self.planner.grid.world_to_cell(*point)
            self.assertIsNotNone(cell)
            assert cell is not None
            self.planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
            self.planner._confirmed_local_obstacles.add(cell)

        route = self.planner._route_points(pose)
        path = graph_waypoints(
            self.planner,
            pose,
            route,
        )
        local_path = self._through_target(path, route[-2])

        self.assertTrue(path)
        self.assertLess(max(point[1] for point in local_path), 0.90)
        self.assertLess(
            sum(math.dist(a, b) for a, b in zip(local_path, local_path[1:])),
            0.45,
        )

    def test_single_frame_endpoint_does_not_outweigh_direct_pass_route(self) -> None:
        pose = PoseEstimate(0.40, 0.78, math.radians(-32.0))
        obstacle = TrackedObstacle(
            "red", ("right",), (0.49, 0.87), math.inf, 3,
        )
        self.planner._route_target_index = 1
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = 0.0
        self.planner._obstacle_program_gate_passed = True
        anchor_cell = self.planner.grid.world_to_cell(*obstacle.anchor)
        transient_cell = self.planner.grid.world_to_cell(0.35, 0.70)
        self.assertIsNotNone(anchor_cell)
        self.assertIsNotNone(transient_cell)
        assert anchor_cell is not None and transient_cell is not None
        self.planner.grid.cells[anchor_cell] = int(Cell.UNKNOWN_OBSTRUCTION)
        self.planner._confirmed_local_obstacles.add(anchor_cell)
        self.planner._previous_local_obstacles.add(transient_cell)

        route = self.planner._route_points(pose)
        path = graph_waypoints(
            self.planner,
            pose,
            route,
        )
        local_path = self._through_target(path, route[-2])

        self.assertTrue(self.planner._graph_search_connected)
        self.assertEqual(self.planner._heading_motion_direction, 1)
        self.assertLess(max(point[1] for point in local_path), 0.85)
        self.assertLess(
            sum(math.dist(a, b) for a, b in zip(local_path, local_path[1:])),
            0.40,
        )

    def test_active_obstacle_wall_fit_does_not_materialize_route_loop(self) -> None:
        pose = PoseEstimate(0.68, -0.77, math.radians(146.0))
        obstacle = TrackedObstacle(
            "red", ("right",), (0.58, -0.88), math.inf, 3,
        )
        self.planner._route_target_index = 3
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = math.pi
        self.planner._obstacle_program_gate_passed = True
        for point in ((0.50, -0.85), (0.55, -0.85), (0.55, -0.90), (0.60, -0.90)):
            cell = self.planner.grid.world_to_cell(*point)
            self.assertIsNotNone(cell)
            assert cell is not None
            self.planner.grid.cells[cell] = int(Cell.WALL)
            self.planner._confirmed_local_obstacles.add(cell)

        route = self.planner._route_points(pose)
        path = graph_waypoints(
            self.planner,
            pose,
            route,
        )
        local_path = self._through_target(path, route[-2])

        self.assertTrue(self.planner._graph_search_connected)
        self.assertEqual(self.planner._heading_motion_direction, 1)
        self.assertGreater(min(point[1] for point in local_path), -0.85)
        self.assertLess(
            sum(math.dist(a, b) for a, b in zip(local_path, local_path[1:])),
            0.45,
        )

    def test_active_obstacle_compacts_connected_tail_only(self) -> None:
        anchor = (0.50, 0.85)
        connected = (
            (0.50, 0.85), (0.55, 0.85), (0.60, 0.85), (0.65, 0.85),
            (0.70, 0.85), (0.75, 0.85), (0.80, 0.85), (0.85, 0.85),
        )
        separate = (0.85, 0.65)
        for point in (*connected, separate):
            cell = self.planner.grid.world_to_cell(*point)
            self.assertIsNotNone(cell)
            assert cell is not None
            self.planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
            self.planner._confirmed_local_obstacles.add(cell)

        compact = active_obstacle_component_mask(self.planner, anchor)

        for point in connected:
            cell = self.planner.grid.world_to_cell(*point)
            assert cell is not None
            self.assertTrue(compact[cell])
        separate_cell = self.planner.grid.world_to_cell(*separate)
        assert separate_cell is not None
        self.assertFalse(compact[separate_cell])

    def test_active_pass_preserves_current_obstacle_footprint(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 1
        obstacle = TrackedObstacle(
            "green", ("left",), (-1.38, 0.88), math.inf, 3,
        )
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = math.pi
        for point in (
            (-1.40, 0.95), (-1.35, 0.95),
            (-1.40, 0.90), (-1.35, 0.90),
        ):
            cell = self.planner.grid.world_to_cell(*point)
            self.assertIsNotNone(cell)
            assert cell is not None
            self.planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
            self.planner._confirmed_local_obstacles.add(cell)
            self.planner._previous_local_obstacles.add(cell)
        pose = PoseEstimate(-1.09, 1.19, math.radians(-161.0))

        path = graph_waypoints(
            self.planner,
            pose,
            self.planner._route_points(pose),
            self.planner._route_block_points(),
        )

        self.assertTrue(path)
        self.assertTrue(self.planner._graph_search_connected)
        self.assertGreaterEqual(self.planner._path_min_clearance, 0.125)

    def test_active_obstacle_overlap_escapes_toward_declared_side(self) -> None:
        pose = PoseEstimate(0.60, -0.76, math.radians(168.0))
        obstacle = TrackedObstacle(
            "red", ("right",), (0.51, -0.87), math.inf, 3,
        )
        self.planner._route_target_index = 3
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = math.pi
        self.planner._obstacle_program_gate_passed = True
        anchor_cell = self.planner.grid.world_to_cell(*obstacle.anchor)
        self.assertIsNotNone(anchor_cell)
        assert anchor_cell is not None
        self.planner.grid.cells[anchor_cell] = int(Cell.UNKNOWN_OBSTRUCTION)
        self.planner._confirmed_local_obstacles.add(anchor_cell)

        route = self.planner._route_points(pose)
        path = graph_waypoints(
            self.planner,
            pose,
            route,
        )
        local_path = self._through_target(path, route[-2])

        self.assertTrue(path)
        self.assertTrue(self.planner._graph_search_connected)
        self.assertEqual(self.planner._heading_motion_direction, 1)
        self.assertGreater(min(point[1] for point in local_path), -0.85)
        self.assertLess(
            sum(math.dist(a, b) for a, b in zip(local_path, local_path[1:])),
            0.45,
        )

    def test_pass_smoothing_preserves_declared_side_gate(self) -> None:
        pose = PoseEstimate(-0.20, 0.43, math.radians(107.5))
        obstacle = TrackedObstacle(
            "green", ("left",), (-0.12, 0.53), math.inf, 3,
        )
        self.planner._route_target_index = 0
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = math.pi * 0.5
        for x_m in (-0.15, -0.10):
            for y_m in (0.50, 0.55):
                cell = self.planner.grid.world_to_cell(x_m, y_m)
                self.assertIsNotNone(cell)
                assert cell is not None
                self.planner.grid.cells[cell] = int(Cell.UNKNOWN_OBSTRUCTION)
                self.planner._confirmed_local_obstacles.add(cell)

        route = self.planner._route_points(pose)
        with unittest.mock.patch(
            "planner.graph_path._feasible_route_entry", wraps=_feasible_route_entry,
        ) as entry:
            graph_waypoints(self.planner, pose, route)
        points = entry.call_args.args[2]
        crossing = int(np.argmin(np.abs(points[:, 1] - obstacle.anchor[1])))

        self.assertTrue(self.planner._graph_search_connected)
        self.assertGreater(obstacle.anchor[0] - points[crossing, 0], 0.14)

    def test_active_obstacle_near_inflation_edge_escapes_declared_side(self) -> None:
        pose = PoseEstimate(0.60, -0.76, math.radians(146.0))
        obstacle = TrackedObstacle(
            "red", ("right",), (0.48, -0.84), math.inf, 3,
        )
        self.planner._route_target_index = 3
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = math.pi
        self.planner._obstacle_program_gate_passed = True
        anchor_cell = self.planner.grid.world_to_cell(*obstacle.anchor)
        self.assertIsNotNone(anchor_cell)
        assert anchor_cell is not None
        self.planner.grid.cells[anchor_cell] = int(Cell.UNKNOWN_OBSTRUCTION)
        self.planner._confirmed_local_obstacles.add(anchor_cell)

        route = self.planner._route_points(pose)
        path = graph_waypoints(
            self.planner,
            pose,
            route,
        )
        local_path = self._through_target(path, route[-2])

        self.assertTrue(self.planner._graph_search_connected)
        self.assertEqual(self.planner._heading_motion_direction, 1)
        self.assertGreater(min(point[1] for point in local_path), -0.85)
        self.assertLess(
            sum(math.dist(a, b) for a, b in zip(local_path, local_path[1:])),
            0.45,
        )

if __name__ == "__main__":
    unittest.main()
