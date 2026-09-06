from __future__ import annotations

import math
import unittest
from unittest.mock import patch

from planner.camera_color import ColorObstacle
from planner.grid import Cell, Direction, GridMap
from planner.local_grid import LocalGrid
from planner.localize_types import PoseEstimate
from planner.obstacle_policy import obstacle_actions
from planner.planner import GridPlanner, Plan, TrackedObstacle


class ObstacleProgramTest(unittest.TestCase):
    def setUp(self) -> None:
        grid = GridMap()
        grid.lock_direction(Direction.RIGHT)
        self.planner = GridPlanner(grid)

    def test_policy_is_an_ordered_action_program(self) -> None:
        self.assertEqual(obstacle_actions()["red"], ("right",))
        self.assertEqual(
            obstacle_actions()["yellow"],
            ("left", "u_turn", "left"),
        )

    def test_unmatched_fitted_line_becomes_compact_obstacle_evidence(self) -> None:
        local = LocalGrid(size_m=2.4, resolution_m=0.05)
        local.mark_scan_points(
            ((0.20, lateral) for lateral in (-0.10, -0.05, 0.0, 0.05, 0.10)),
            Cell.WALL,
        )
        pose = PoseEstimate(0.0, -0.80, 0.0)

        mask = self.planner._local_obstacle_mask(local, pose)
        clusters = self.planner._local_obstacle_clusters(local, pose)

        self.assertEqual(int(mask.sum()), 1)
        self.assertEqual(len(clusters), 1)

    def test_fitted_line_on_known_map_wall_is_not_obstacle_evidence(self) -> None:
        local = LocalGrid(size_m=2.4, resolution_m=0.05)
        local.mark_scan_points(
            ((forward, -0.30) for forward in (-0.10, -0.05, 0.0, 0.05, 0.10)),
            Cell.WALL,
        )
        pose = PoseEstimate(0.0, -1.20, 0.0)

        mask = self.planner._local_obstacle_mask(local, pose)

        self.assertFalse(bool(mask.any()))

    def test_corner_ghost_cannot_become_obstacle_support(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        local = LocalGrid(size_m=2.4, resolution_m=0.05)
        local.mark_scan_points(
            ((forward, lateral) for forward in (0.00, 0.05) for lateral in (0.00, 0.05)),
            Cell.UNKNOWN_OBSTRUCTION,
        )

        self.planner._update_local_obstacle_support(
            PoseEstimate(-2.28, 1.15, 0.0), local,
        )

        self.assertFalse(self.planner._previous_local_obstacles)
        self.assertFalse(self.planner._color_lidar_obstacles)

    def test_side_section_boundary_remains_obstacle_support(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        local = LocalGrid(size_m=2.4, resolution_m=0.05)
        local.mark_scan_points(
            ((forward, lateral) for forward in (0.00, 0.05) for lateral in (0.00, 0.05)),
            Cell.UNKNOWN_OBSTRUCTION,
        )

        self.planner._update_local_obstacle_support(
            PoseEstimate(-2.08, 0.53, 0.0), local,
        )

        self.assertTrue(self.planner._previous_local_obstacles)
        self.assertTrue(self.planner._color_lidar_obstacles)

    def test_prelock_fitted_line_cannot_become_colored_obstacle_anchor(self) -> None:
        planner = GridPlanner(GridMap())
        local = LocalGrid(size_m=2.4, resolution_m=0.05)
        local.mark_scan_points(
            ((0.20, lateral) for lateral in (-0.10, -0.05, 0.0, 0.05, 0.10)),
            Cell.WALL,
        )
        pose = PoseEstimate(0.0, -0.80, 0.0)

        planner._update_local_obstacle_support(pose, local)

        self.assertTrue(planner._previous_local_obstacles)
        self.assertFalse(planner._color_lidar_obstacles)

    def test_prelock_open_side_boundary_cannot_receive_camera_color(self) -> None:
        planner = GridPlanner(GridMap())
        local = LocalGrid(size_m=2.4, resolution_m=0.05)
        local.mark_scan_points(
            ((forward, lateral) for forward in (0.45, 0.50) for lateral in (1.05, 1.10)),
            Cell.UNKNOWN_OBSTRUCTION,
        )

        planner._update_local_obstacle_support(PoseEstimate(0.0, 0.0, 0.0), local)

        self.assertTrue(planner._previous_local_obstacles)
        self.assertFalse(planner._color_lidar_obstacles)

    def test_prelock_center_obstacle_remains_camera_eligible(self) -> None:
        planner = GridPlanner(GridMap())
        local = LocalGrid(size_m=2.4, resolution_m=0.05)
        local.mark_scan_points(
            ((forward, lateral) for forward in (0.05, 0.10) for lateral in (0.45, 0.50)),
            Cell.UNKNOWN_OBSTRUCTION,
        )

        planner._update_local_obstacle_support(PoseEstimate(0.0, 0.0, 0.0), local)

        self.assertTrue(planner._color_lidar_obstacles)

    def test_colored_display_follows_lidar_without_moving_track(self) -> None:
        old_anchor = (0.50, 0.50)
        new_cells = {
            self.planner.grid.world_to_cell(0.55, 0.50),
            self.planner.grid.world_to_cell(0.55, 0.55),
        }
        current = {cell for cell in new_cells if cell is not None}
        self.planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), old_anchor, math.inf, 4),
        ]
        self.planner._color_lidar_obstacles = current

        displayed = self.planner.tracked_obstacle_display()

        self.assertEqual(displayed, (("green", (0.55, 0.50)),))
        self.assertEqual(self.planner._tracked_obstacles[0].anchor, old_anchor)

    def test_colored_display_does_not_jump_to_distant_lidar_component(self) -> None:
        anchor = (0.50, 0.50)
        distant = self.planner.grid.world_to_cell(0.85, 0.50)
        self.assertIsNotNone(distant)
        assert distant is not None
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), anchor, math.inf, 4),
        ]

        self.planner._color_lidar_obstacles = {distant}

        self.assertEqual(
            self.planner.tracked_obstacle_display(),
            (("red", anchor),),
        )
        self.assertEqual(self.planner._tracked_obstacles[0].anchor, anchor)

    def test_initial_corridor_obeys_colored_obstacle_before_direction_lock(self) -> None:
        planner = GridPlanner(GridMap())
        planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), (-0.10, 0.50), math.inf, 2),
        ]
        pose = PoseEstimate(x_m=0.0, y_m=0.0, yaw_rad=math.pi * 0.5)

        planner._ensure_obstacle_program(pose)
        points = planner._route_points(pose)

        self.assertEqual(planner._program_action(), "left")
        self.assertGreaterEqual(len(points), 3)
        self.assertLess(points[0][0], -0.10)

    def test_left_action_targets_left_side_and_advances_after_passing(self) -> None:
        anchor = (0.80, 1.0)
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("left",), anchor, math.inf, 3),
        ]
        self.planner._route_target_index = 1
        approach = PoseEstimate(x_m=0.30, y_m=1.0, yaw_rad=0.0)
        self.planner._ensure_obstacle_program(approach)

        self.assertEqual(self.planner._program_action(), "left")

        route = self.planner._route_points(approach)
        gate, target, course_target = route[-3:]
        yaw = 0.0
        forward = (math.cos(yaw), math.sin(yaw))
        left = (-forward[1], forward[0])
        side_offset = self.planner._obstacle_pass_side_offset(
            anchor, forward, left, 1.0,
        )
        self.assertAlmostEqual(
            gate[0], anchor[0] - 0.18 * forward[0] + side_offset * left[0],
        )
        self.assertAlmostEqual(
            gate[1], anchor[1] - 0.18 * forward[1] + side_offset * left[1],
        )
        self.assertAlmostEqual(
            target[0], anchor[0] + 0.20 * forward[0] + side_offset * left[0],
        )
        self.assertAlmostEqual(
            target[1], anchor[1] + 0.20 * forward[1] + side_offset * left[1],
        )
        self.assertEqual(course_target, (2.0, 1.0))

        passed = PoseEstimate(
            x_m=anchor[0] + 0.12 * forward[0] + side_offset * left[0],
            y_m=anchor[1] + 0.12 * forward[1] + side_offset * left[1],
            yaw_rad=yaw,
        )
        self.planner._route_points(passed)
        self.assertEqual(self.planner._program_action(), "left")

        passed.x_m = anchor[0] + 0.18 * forward[0] + side_offset * left[0]
        passed.y_m = anchor[1] + 0.18 * forward[1] + side_offset * left[1]
        self.planner._route_points(passed)
        self.assertIsNone(self.planner._program_action())
        self.planner._ensure_obstacle_program(passed)
        self.assertIsNone(self.planner._program_action())

        far_away = PoseEstimate(x_m=2.0, y_m=1.0, yaw_rad=math.pi)
        self.planner._ensure_obstacle_program(far_away)
        self.planner._ensure_obstacle_program(approach)
        self.assertEqual(self.planner._program_action(), "left")

    def test_extra_lateral_clearance_does_not_delay_completed_pass(self) -> None:
        obstacle = TrackedObstacle("green", ("left",), (1.52, -0.77), math.inf, 3)
        self.planner._tracked_obstacles = [obstacle]
        self.planner._route_target_index = 3
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._obstacle_program_actions = obstacle.actions
        self.planner._obstacle_program_approach_yaw = math.pi

        self.planner._route_points(
            PoseEstimate(x_m=1.30, y_m=-1.10, yaw_rad=math.pi),
        )

        self.assertIsNone(self.planner._program_action())
        self.assertIn(obstacle.anchor, self.planner._handled_obstacle_anchors)

    def test_pass_target_uses_center_of_obstacle_wall_gap(self) -> None:
        offset = self.planner._obstacle_pass_side_offset(
            (-0.12, 0.53), (0.0, 1.0), (-1.0, 0.0), 1.0,
        )

        self.assertAlmostEqual(offset, 0.175)

    def test_wrong_side_crossing_recovers_to_gate_without_route_loop(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("left",), (0.80, 1.0), math.inf, 3),
        ]
        self.planner._route_target_index = 1
        self.planner._ensure_obstacle_program(
            PoseEstimate(x_m=0.50, y_m=1.0, yaw_rad=0.0),
        )

        route = self.planner._route_points(
            PoseEstimate(x_m=0.95, y_m=0.88, yaw_rad=0.0),
        )

        self.assertEqual(self.planner._program_action(), "left")
        self.assertEqual(len(route), 1)
        gate = route[0]
        self.assertLess(gate[0], 0.80)
        self.assertGreater(gate[1], 1.0)

    def test_lateral_predicted_pose_does_not_clear_pass_action(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), (0.47, 0.90), math.inf, 3),
        ]
        self.planner._route_target_index = 1
        self.planner._obstacle_program_anchor = (0.47, 0.90)
        self.planner._obstacle_program_approach_yaw = math.radians(-6.0)

        route = self.planner._route_points(PoseEstimate(
            x_m=0.473,
            y_m=1.186,
            yaw_rad=math.radians(16.7),
        ))

        self.assertEqual(self.planner._program_action(), "left")
        self.assertTrue(route)

    def test_pass_action_ends_after_correct_side_crossing(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), (0.47, 0.90), math.inf, 3),
        ]
        self.planner._route_target_index = 1
        self.planner._obstacle_program_anchor = (0.47, 0.90)
        self.planner._obstacle_program_approach_yaw = 0.0
        pose = PoseEstimate(
            x_m=0.65,
            y_m=1.10,
            yaw_rad=math.radians(16.0),
        )

        route = self.planner._route_points(pose)

        self.assertIsNone(self.planner._program_action())
        self.assertEqual(route, ((2.0, 1.0),))

    def test_pass_action_does_not_end_far_outside_pass_corridor(self) -> None:
        self.planner._route_target_index = 3
        self.planner._obstacle_program_anchor = (-0.12, 0.53)
        self.planner._obstacle_program_actions = ("left", "u_turn", "left")
        self.planner._obstacle_program_index = 2
        self.planner._obstacle_program_approach_yaw = math.radians(-50.9)

        route = self.planner._route_points(PoseEstimate(
            x_m=0.851,
            y_m=1.155,
            yaw_rad=math.radians(-60.0),
        ))

        self.assertEqual(self.planner._program_action(), "left")
        self.assertEqual(route[-1], (0.0, -1.0))
        self.assertEqual(len(route), 2)

    def test_late_side_correction_stays_ahead_of_robot(self) -> None:
        self.planner._route_target_index = 0
        self.planner._obstacle_program_anchor = (-0.12, 0.53)
        self.planner._obstacle_program_actions = ("left",)
        self.planner._obstacle_program_approach_yaw = math.pi / 2.0
        self.planner._obstacle_program_gate_passed = True
        pose = PoseEstimate(x_m=-0.15, y_m=0.72, yaw_rad=math.pi / 2.0)

        route = self.planner._route_points(pose)

        self.assertEqual(self.planner._program_action(), "left")
        self.assertGreater(route[0][1], pose.y_m)
        self.assertLess(route[0][0], -0.20)

    def test_pass_action_ends_once_obstacle_is_well_behind(self) -> None:
        self.planner._route_target_index = 0
        self.planner._obstacle_program_anchor = (-0.12, 0.53)
        self.planner._obstacle_program_actions = ("left",)
        self.planner._obstacle_program_approach_yaw = math.pi / 2.0
        self.planner._obstacle_program_gate_passed = True

        route = self.planner._route_points(PoseEstimate(
            x_m=-0.15, y_m=0.95, yaw_rad=math.pi / 2.0,
        ))

        self.assertIsNone(self.planner._program_action())
        self.assertEqual(route, ((0.0, 1.0),))

    def test_pass_action_remains_while_reversing_toward_obstacle(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), (0.47, 0.90), math.inf, 3),
        ]
        self.planner._route_target_index = 3
        self.planner._obstacle_program_anchor = (0.47, 0.90)
        self.planner._obstacle_program_approach_yaw = 0.0
        self.planner._last_motion_direction = -1
        pose = PoseEstimate(
            x_m=-0.04,
            y_m=1.20,
            yaw_rad=math.radians(-124.0),
        )

        route = self.planner._route_points(pose)

        self.assertEqual(self.planner._program_action(), "left")
        self.assertNotEqual(route, ((0.0, -1.0),))

    def test_completed_side_gate_is_not_routed_again(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("left",), (0.30, 0.0), math.inf, 3),
        ]
        self.planner._route_target_index = 1
        self.planner._obstacle_program_anchor = (0.30, 0.0)
        self.planner._obstacle_program_approach_yaw = 0.0

        route = self.planner._route_points(
            PoseEstimate(x_m=0.12, y_m=0.22, yaw_rad=0.0),
        )

        self.assertTrue(self.planner._obstacle_program_gate_passed)
        self.assertEqual(len(route), 2)
        self.assertAlmostEqual(route[0][0], 0.50)
        self.assertAlmostEqual(
            route[0][1],
            self.planner._obstacle_pass_side_offset(
                (0.30, 0.0), (1.0, 0.0), (0.0, 1.0), 1.0,
            ),
        )
        self.assertEqual(route[1], (2.0, 1.0))

    def test_pass_frame_uses_map_course_not_transient_chassis_yaw(self) -> None:
        self.planner._route_target_index = 3
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (0.50, -0.88), math.inf, 3),
        ]
        pose = PoseEstimate(
            x_m=1.35,
            y_m=-1.09,
            yaw_rad=math.radians(158.0),
        )

        self.planner._ensure_obstacle_program(pose)

        expected = math.pi
        self.assertAlmostEqual(self.planner._obstacle_program_approach_yaw, expected)
        self.assertGreater(
            abs(self.planner._obstacle_program_approach_yaw - pose.yaw_rad),
            math.radians(10.0),
        )

    def test_u_turn_drives_forward_and_flips_route_after_180_degrees(self) -> None:
        self.planner.max_steering_angle_rad = 0.4188
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._tracked_obstacles = [
            TrackedObstacle(
                "yellow", ("left", "u_turn", "left"),
                (0.30, 0.0), math.inf, 3,
            ),
        ]
        self.planner._obstacle_program_anchor = (0.30, 0.0)
        self.planner._obstacle_program_index = 1
        self.planner._obstacle_program_approach_yaw = 0.0
        self.planner._parked_target_index = 2

        plan = self.planner._obstacle_u_turn_plan(
            PoseEstimate(x_m=0.0, y_m=-1.0, yaw_rad=0.0),
        )
        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.motion_direction, 1)
        self.assertNotEqual(plan.steering_angle_rad, 0.0)
        self.assertEqual(self.planner._route_step, 1)

        turn_sign = 1.0 if plan.steering_angle_rad > 0.0 else -1.0
        settled = self.planner._obstacle_u_turn_plan(
            PoseEstimate(
                x_m=0.10,
                y_m=0.50,
                yaw_rad=turn_sign * (math.pi - 0.05),
            ),
        )
        self.assertIsNotNone(settled)
        self.assertEqual(self.planner._route_step, -1)
        self.assertEqual(self.planner._program_action(), "left")
        self.assertIsNone(self.planner._parked_target_index)

    def test_measured_turning_radius_stages_instead_of_cutting_walls(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._tracked_obstacles = [
            TrackedObstacle("yellow", ("u_turn",), (0.30, 0.0), math.inf, 3),
        ]
        self.planner._obstacle_program_anchor = (0.30, 0.0)
        self.planner._obstacle_program_approach_yaw = 0.0

        plan = self.planner._obstacle_u_turn_plan(
            PoseEstimate(x_m=0.0, y_m=-1.0, yaw_rad=0.0),
        )

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "obstacle-u-turn-stage")
        self.assertEqual(self.planner._obstacle_program_turn_sign, 0)
        self.assertEqual(self.planner._route_step, 1)

    def test_u_turn_retreats_from_obstacle_until_forward_arc_fits(self) -> None:
        self.planner.max_steering_angle_rad = 0.4188
        self.planner._tracked_obstacles = [
            TrackedObstacle("yellow", ("u_turn",), (1.50, -0.90), math.inf, 3),
        ]
        self.planner._obstacle_program_anchor = (1.50, -0.90)
        for x_m in (-0.15, -0.10, -0.05):
            for y_m in (-0.55, -0.50, -0.45):
                cell = self.planner.grid.world_to_cell(x_m, y_m)
                assert cell is not None
                self.planner.grid.cells[cell] = int(Cell.LIVE_OBSTACLE)
        pose = PoseEstimate(
            x_m=-0.085,
            y_m=-0.678,
            yaw_rad=math.radians(71.5),
        )

        plan = self.planner._obstacle_u_turn_plan(pose)

        self.assertIsNotNone(plan)
        assert plan is not None and plan.target is not None
        self.assertEqual(plan.reason, "obstacle-u-turn-stage")
        self.assertEqual(plan.motion_direction, -1)
        toward_target = (
            (plan.target[0] - pose.x_m) * math.cos(pose.yaw_rad)
            + (plan.target[1] - pose.y_m) * math.sin(pose.yaw_rad)
        )
        self.assertLess(toward_target, -0.20)

    def test_u_turn_stage_can_move_forward_to_open_turning_space(self) -> None:
        self.planner._obstacle_program_stage_target = (0.20, 0.0)

        plan = self.planner._u_turn_stage_plan(
            PoseEstimate(yaw_rad=0.0),
            radius_m=0.30,
            required_clearance_m=0.14,
            clearance=self.planner._clearance_field(),
            remaining_angle_rad=math.pi,
            turn_sign=1,
        )

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "obstacle-u-turn-stage")
        self.assertEqual(plan.motion_direction, 1)

    def test_partial_u_turn_stages_away_from_outer_wall_cell_margin(self) -> None:
        self.planner._obstacle_program_anchor = (-0.12, 0.53)
        self.planner._obstacle_program_actions = ("u_turn",)
        self.planner._obstacle_program_turn_sign = -1
        self.planner._obstacle_program_turn_radians = math.radians(39.0)
        self.planner._obstacle_program_turn_start_yaw = math.radians(90.0)
        self.planner._obstacle_program_turn_last_yaw = math.radians(52.0)
        pose = PoseEstimate(
            x_m=-0.35,
            y_m=0.72,
            yaw_rad=math.radians(52.0),
        )

        plan = self.planner._obstacle_u_turn_plan(pose)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "obstacle-u-turn-stage")
        self.assertEqual(plan.motion_direction, 1)
        self.assertIsNotNone(plan.target)
        assert plan.target is not None
        self.assertGreater(plan.target[0], pose.x_m)
        self.assertAlmostEqual(plan.steering_angle_rad, 0.0)

    def test_u_turn_stage_direction_does_not_flip_with_pose_projection(self) -> None:
        self.planner._obstacle_program_stage_target = (0.20, 0.0)
        self.planner._obstacle_program_stage_direction = 1

        plan = self.planner._u_turn_stage_plan(
            PoseEstimate(x_m=0.05, yaw_rad=math.pi),
            radius_m=0.30,
            required_clearance_m=0.14,
            clearance=self.planner._clearance_field(),
            remaining_angle_rad=math.pi,
            turn_sign=1,
        )

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.motion_direction, 1)

    def test_u_turn_does_not_count_staging_yaw_as_turn_progress(self) -> None:
        self.planner.max_steering_angle_rad = 0.4188
        self.planner._tracked_obstacles = [
            TrackedObstacle("yellow", ("u_turn",), (1.50, -0.90), math.inf, 3),
        ]
        self.planner._obstacle_program_anchor = (1.50, -0.90)
        for x_m in (-0.15, -0.10, -0.05):
            for y_m in (-0.55, -0.50, -0.45):
                cell = self.planner.grid.world_to_cell(x_m, y_m)
                assert cell is not None
                self.planner.grid.cells[cell] = int(Cell.LIVE_OBSTACLE)
        staged = self.planner._obstacle_u_turn_plan(PoseEstimate(
            x_m=-0.085,
            y_m=-0.678,
            yaw_rad=math.radians(71.5),
        ))
        self.assertIsNotNone(staged)
        assert staged is not None and staged.target is not None
        self.assertEqual(staged.reason, "obstacle-u-turn-stage")

        self.planner.grid.map_direction(Direction.RIGHT)
        settling = self.planner._obstacle_u_turn_plan(PoseEstimate(
            x_m=staged.target[0],
            y_m=staged.target[1],
            yaw_rad=0.0,
            odometry_motion_m=0.02,
        ))

        self.assertIsNotNone(settling)
        assert settling is not None
        self.assertEqual(settling.reason, "obstacle-u-turn-stage")
        self.assertEqual(settling.speed_scale, 0.0)

        still_settling = self.planner._obstacle_u_turn_plan(PoseEstimate(
            x_m=staged.target[0],
            y_m=staged.target[1],
            yaw_rad=0.0,
            odometry_motion_m=0.01,
        ))
        self.assertIsNotNone(still_settling)
        assert still_settling is not None
        self.assertEqual(still_settling.reason, "obstacle-u-turn-stage")

        started = self.planner._obstacle_u_turn_plan(PoseEstimate(
            x_m=staged.target[0],
            y_m=staged.target[1],
            yaw_rad=0.0,
            odometry_motion_m=0.0,
        ))

        self.assertIsNotNone(started)
        assert started is not None
        self.assertEqual(started.reason, "obstacle-u-turn")
        self.assertEqual(self.planner._obstacle_program_turn_radians, 0.0)

    def test_blocked_partial_u_turn_preserves_completed_angle(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("yellow", ("u_turn",), (0.30, 0.0), math.inf, 3),
        ]
        self.planner._obstacle_program_anchor = (0.30, 0.0)
        self.planner._obstacle_program_turn_sign = 1
        self.planner._obstacle_program_turn_start_yaw = 0.0
        self.planner._obstacle_program_turn_last_yaw = 1.0
        self.planner._obstacle_program_turn_radians = 1.0
        stage_result = Plan(
            (), ((0.0, 0.0),), None, 0.0, -1, "obstacle-u-turn-stage", 0.65,
        )

        with (
            patch.object(self.planner, "_trajectory_clearance", return_value=0.0),
            patch.object(
                self.planner,
                "_u_turn_stage_plan",
                return_value=stage_result,
            ) as stage_plan,
        ):
            plan = self.planner._obstacle_u_turn_plan(PoseEstimate(yaw_rad=1.1))

        self.assertIsNotNone(plan)
        self.assertEqual(self.planner._obstacle_program_turn_sign, 1)
        self.assertAlmostEqual(self.planner._obstacle_program_turn_radians, 1.1)
        self.assertAlmostEqual(stage_plan.call_args.args[4], math.pi - 1.1)
        self.assertEqual(stage_plan.call_args.args[5], 1)

    def test_u_turn_staging_does_not_add_yaw_to_partial_turn(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("yellow", ("u_turn",), (0.30, 0.0), math.inf, 3),
        ]
        self.planner._obstacle_program_anchor = (0.30, 0.0)
        self.planner._obstacle_program_turn_sign = 1
        self.planner._obstacle_program_turn_start_yaw = 0.0
        self.planner._obstacle_program_turn_last_yaw = 1.0
        self.planner._obstacle_program_turn_radians = 1.0
        self.planner._obstacle_program_stage_target = (-0.20, 0.0)
        stage_result = Plan(
            (), ((0.0, 0.0),), None, 0.0, -1, "obstacle-u-turn-stage", 0.65,
        )

        with patch.object(
            self.planner,
            "_u_turn_stage_plan",
            return_value=stage_result,
        ):
            plan = self.planner._obstacle_u_turn_plan(PoseEstimate(yaw_rad=1.4))

        self.assertIsNotNone(plan)
        self.assertAlmostEqual(self.planner._obstacle_program_turn_radians, 1.0)

    def test_corner_alignment_replans_before_lateral_wall_sweep(self) -> None:
        self.planner._route_target_index = 3
        self.planner._corner_index = 3
        self.planner._corner_phase = 2
        self.planner._corner_turn_sign = 1
        self.planner._corner_first_direction = -1
        self.planner._corner_leg_m = 0.14
        pose = PoseEstimate(
            x_m=-0.33,
            y_m=-0.93,
            yaw_rad=math.radians(-157.0),
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNone(plan)
        self.assertEqual(self.planner._corner_phase, 0)
        self.assertIsNone(self.planner._corner_index)

    def test_corner_alignment_switches_to_clear_opposite_arc(self) -> None:
        self.planner._route_target_index = 3
        self.planner._corner_index = 3
        self.planner._corner_phase = 2
        self.planner._corner_turn_sign = 1
        self.planner._corner_first_direction = -1
        self.planner._corner_leg_m = 0.14
        pose = PoseEstimate(
            x_m=-0.10,
            y_m=-1.00,
            yaw_rad=math.radians(-157.0),
        )

        with patch.object(
            self.planner,
            "_corner_leg_wall_margin",
            side_effect=lambda _pose, direction: 0.20 if direction > 0 else -1.0,
        ):
            plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-settle")
        self.assertEqual(self.planner._corner_phase, 3)
        self.assertEqual(self.planner._corner_index, 3)

    def test_corner_arc_uses_safe_prefix_before_close_obstacle(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 4
        self.planner._corner_turn_sign = 1
        self.planner._corner_first_direction = 1
        self.planner._corner_leg_m = 0.20
        self.planner._corner_leg_yaw_rad = math.radians(35.0)
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (-0.04, 0.71), math.inf, 5),
        ]
        pose = PoseEstimate(
            x_m=0.054,
            y_m=1.055,
            yaw_rad=math.radians(63.5),
            odometry_travel_m=1.0,
        )
        self.planner._set_corner_phase(4, pose)

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-align")
        self.assertEqual(plan.motion_direction, -1)
        self.assertEqual(self.planner._corner_phase, 4)

    def test_corner_alignment_checks_only_remaining_arc_distance(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        self.planner._route_target_index = 1
        self.planner._corner_index = 1
        self.planner._corner_phase = 2
        self.planner._corner_turn_sign = -1
        self.planner._corner_first_direction = 1
        self.planner._corner_leg_m = 0.20
        self.planner._corner_leg_yaw_rad = math.radians(37.0)
        self.planner._corner_phase_yaw_rad = 0.0
        self.planner._corner_phase_odom_m = 1.0
        pose = PoseEstimate(
            x_m=2.133,
            y_m=1.000,
            yaw_rad=math.radians(-18.4),
            odometry_travel_m=1.15,
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-align")
        self.assertEqual(self.planner._corner_phase, 2)

    def test_ninety_degree_corner_leg_matches_calibrated_curvature(self) -> None:
        self.planner._route_target_index = 0
        desired = self.planner._corner_yaws()[0]

        pose = PoseEstimate(
            x_m=0.0,
            y_m=1.0,
            yaw_rad=desired + math.pi * 0.5,
        )
        settling = self.planner._corner_plan(pose, math.inf)
        self.planner._corner_phase_time -= 0.11
        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(settling)
        assert settling is not None
        self.assertEqual(settling.reason, "corner-settle")
        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-align")
        self.assertAlmostEqual(self.planner._corner_leg_m, 0.20)
        self.assertAlmostEqual(
            self.planner._corner_leg_yaw_rad,
            self.planner._corner_leg_m * math.tan(self.planner.max_steering_angle_rad)
            / self.planner.wheelbase_m,
        )
        self.assertGreater(self.planner._corner_leg_yaw_rad, math.radians(20.0))
        self.assertLess(self.planner._corner_leg_yaw_rad, math.radians(23.0))

    def test_corner_acceptance_and_parking_use_same_yaw_tolerance(self) -> None:
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 1
        self.planner._corner_phase_time -= 0.11
        center = self.planner._corner_centers()[0]
        desired = self.planner._corner_yaws()[0]
        pose = PoseEstimate(
            x_m=center[0] - 0.15 * math.cos(desired),
            y_m=center[1] - 0.15 * math.sin(desired),
            yaw_rad=desired + math.radians(16.5),
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-parked")
        self.assertEqual(self.planner._corner_phase, 0)

    def test_corner_accepts_small_route_correctable_heading_error(self) -> None:
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 1
        self.planner._corner_phase_time -= 0.11
        center = self.planner._corner_centers()[0]
        desired = self.planner._corner_yaws()[0]

        plan = self.planner._corner_plan(PoseEstimate(
            x_m=center[0] - 0.15 * math.cos(desired),
            y_m=center[1] - 0.15 * math.sin(desired),
            yaw_rad=desired + math.radians(22.0),
        ), math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-parked")
        self.assertEqual(self.planner._corner_phase, 0)

    def test_corner_parking_keeps_control_after_safe_alignment_drift(self) -> None:
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 6
        center = self.planner._corner_centers()[0]
        desired = self.planner._corner_yaws()[0]
        outgoing = (math.cos(desired), math.sin(desired))
        lateral = (-outgoing[1], outgoing[0])
        pose = PoseEstimate(
            x_m=center[0] + 0.14 * outgoing[0] + 0.18 * lateral[0],
            y_m=center[1] + 0.14 * outgoing[1] + 0.18 * lateral[1],
            yaw_rad=desired,
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-backup")
        self.assertEqual(plan.motion_direction, -1)
        self.assertEqual(self.planner._corner_phase, 6)

    def test_corner_alignment_captures_route_inside_safe_section_region(self) -> None:
        self.planner._route_target_index = 1
        center = self.planner._corner_centers()[1]
        desired = self.planner._corner_yaws()[1]
        pose = PoseEstimate(
            x_m=center[0] - 0.10,
            y_m=center[1] - 0.34,
            yaw_rad=desired + math.pi * 0.5,
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-settle")
        self.assertEqual(self.planner._corner_phase, 1)

    def test_corner_parking_accounts_for_measured_stopping_distance(self) -> None:
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 6
        center = self.planner._corner_centers()[0]
        desired = self.planner._corner_yaws()[0]
        outgoing = (math.cos(desired), math.sin(desired))
        pose = PoseEstimate(
            x_m=center[0] - 0.02 * outgoing[0],
            y_m=center[1] - 0.02 * outgoing[1],
            yaw_rad=desired,
            odometry_speed_mps=-0.20,
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-braking")
        self.assertEqual(plan.speed_scale, 0.0)
        self.assertEqual(self.planner._corner_phase, 8)

        pose.odometry_speed_mps = -0.02
        parked = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(parked)
        assert parked is not None
        self.assertEqual(parked.reason, "corner-parked")
        self.assertEqual(self.planner._corner_phase, 0)

    def test_corner_parking_still_backs_when_stationary_before_threshold(self) -> None:
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 6
        center = self.planner._corner_centers()[0]
        desired = self.planner._corner_yaws()[0]
        outgoing = (math.cos(desired), math.sin(desired))
        pose = PoseEstimate(
            x_m=center[0] - 0.02 * outgoing[0],
            y_m=center[1] - 0.02 * outgoing[1],
            yaw_rad=desired,
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-backup")
        self.assertEqual(plan.motion_direction, -1)

    def test_braking_drift_cannot_complete_corner(self) -> None:
        for pose in (
            PoseEstimate(-2.225, 1.363, math.radians(-36.2)),
            PoseEstimate(-2.0, 1.15, math.radians(-36.2)),
            PoseEstimate(-2.28, 1.15, -math.pi / 2.0),
        ):
            with self.subTest(pose=pose):
                self.planner.grid.lock_direction(Direction.LEFT)
                self.planner._route_target_index = 1
                self.planner._corner_index = 1
                self.planner._corner_phase = 8

                plan = self.planner._corner_plan(pose, math.inf)

                self.assertIsNone(plan)
                self.assertEqual(self.planner._route_target_index, 1)
                self.assertIsNone(self.planner._parked_target_index)
                self.assertEqual(self.planner._corner_phase, 0)

    def test_corner_leg_ends_from_imu_yaw_without_wheel_distance(self) -> None:
        self.planner._route_target_index = 3
        self.planner._corner_index = 3
        self.planner._corner_phase = 2
        self.planner._corner_turn_sign = 1
        self.planner._corner_first_direction = 1
        self.planner._corner_leg_m = 0.14
        self.planner._corner_leg_yaw_rad = math.radians(20.0)
        self.planner._corner_phase_yaw_rad = math.radians(-90.0)
        self.planner._corner_phase_odom_m = 1.0
        pose = PoseEstimate(
            x_m=-0.10,
            y_m=-1.00,
            yaw_rad=math.radians(-68.0),
            odometry_travel_m=1.0,
        )

        plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.reason, "corner-settle")
        self.assertEqual(self.planner._corner_phase, 3)

    def test_stalled_corner_arc_makes_straight_clearance_before_retry(self) -> None:
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 4
        self.planner._corner_turn_sign = 1
        self.planner._corner_first_direction = 1
        self.planner._corner_leg_m = 0.14
        self.planner._corner_leg_yaw_rad = math.radians(20.0)
        self.planner._corner_phase_yaw_rad = math.radians(90.0)
        self.planner._corner_phase_odom_m = 1.0
        self.planner._corner_phase_time -= 0.80
        pose = PoseEstimate(
            x_m=-0.15,
            y_m=1.20,
            yaw_rad=math.radians(90.0),
            odometry_travel_m=1.0,
        )

        settling = self.planner._corner_plan(pose, math.inf)
        self.planner._corner_phase_time -= 0.11
        recovery = self.planner._corner_plan(pose, math.inf)
        resumed = self.planner._corner_plan(PoseEstimate(
            x_m=-0.15,
            y_m=1.14,
            yaw_rad=math.radians(90.0),
            odometry_travel_m=1.06,
        ), math.inf)

        self.assertIsNotNone(settling)
        self.assertIsNotNone(recovery)
        self.assertIsNotNone(resumed)
        assert settling is not None and recovery is not None and resumed is not None
        self.assertEqual(settling.reason, "corner-recovery-settle")
        self.assertEqual(recovery.reason, "corner-recovery-straight")
        self.assertEqual(recovery.motion_direction, -1)
        self.assertAlmostEqual(recovery.steering_angle_rad, 0.0)
        self.assertEqual(resumed.reason, "corner-recovery-settle")
        self.assertEqual(self.planner._corner_phase, 4)

        self.planner._corner_phase_time -= 0.80
        retried = self.planner._corner_plan(PoseEstimate(
            x_m=-0.15,
            y_m=1.14,
            yaw_rad=math.radians(90.0),
            odometry_travel_m=1.06,
        ), math.inf)

        self.assertIsNotNone(retried)
        assert retried is not None
        self.assertEqual(retried.reason, "corner-recovery-settle")
        self.assertEqual(self.planner._corner_phase, 7)

    def test_corner_leg_rejects_tracked_obstacle_inside_footprint(self) -> None:
        self.planner._corner_leg_m = 0.14
        self.planner._corner_turn_sign = 1
        pose = PoseEstimate(x_m=-1.70, y_m=-1.00, yaw_rad=0.0)
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (-1.60, -1.00), math.inf, 2),
        ]

        margin = self.planner._corner_leg_wall_margin(pose, 1)

        self.assertLess(margin, 0.0)

    def test_corner_does_not_advance_with_wrong_heading_when_alignment_is_blocked(self) -> None:
        self.planner._route_target_index = 0
        self.planner._corner_index = 0
        self.planner._corner_phase = 1
        self.planner._corner_phase_time -= 0.11
        center = self.planner._corner_centers()[0]
        pose = PoseEstimate(
            x_m=center[0],
            y_m=center[1],
            yaw_rad=self.planner._corner_yaws()[0] + math.pi * 0.5,
        )

        with patch.object(
            self.planner, "_corner_leg_wall_margin", return_value=-1.0,
        ):
            plan = self.planner._corner_plan(pose, math.inf)

        self.assertIsNone(plan)
        self.assertEqual(self.planner._route_target_index, 0)
        self.assertIsNone(self.planner._parked_target_index)
        self.assertEqual(self.planner._corner_phase, 0)

    def test_repeat_lap_cancels_active_parking_without_stopping(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 0
        self.planner._initial_route_target_index = 0
        self.planner.completed_laps = 1
        self.planner._corner_index = 0
        self.planner._corner_phase = 6
        desired = self.planner._corner_yaws()[0]

        plan = self.planner._corner_plan(
            PoseEstimate(x_m=0.05, y_m=1.02, yaw_rad=desired), math.inf,
        )

        self.assertIsNone(plan)
        self.assertEqual(self.planner._route_target_index, 1)
        self.assertEqual(self.planner._corner_phase, 0)
        self.assertIsNone(self.planner._corner_index)

    def test_repeat_lap_continues_from_corner_without_realignment(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 1
        self.planner.completed_laps = 1
        center = self.planner._corner_centers()[1]
        desired = self.planner._corner_yaws()[1]

        plan = self.planner._corner_plan(
            PoseEstimate(
                x_m=center[0] + 0.05,
                y_m=center[1],
                yaw_rad=desired,
            ),
            math.inf,
        )

        self.assertIsNone(plan)
        self.assertEqual(self.planner._route_target_index, 2)
        self.assertEqual(self.planner._parked_target_index, 1)
        self.assertEqual(self.planner._corner_phase, 0)

    def test_repeat_lap_advances_incoming_heading_without_alignment(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 1
        self.planner.completed_laps = 1
        center = self.planner._corner_centers()[1]
        desired = self.planner._corner_yaws()[1]

        plan = self.planner._corner_plan(
            PoseEstimate(
                x_m=center[0] + 0.05,
                y_m=center[1],
                yaw_rad=desired + math.pi * 0.5,
            ),
            math.inf,
        )

        self.assertIsNone(plan)
        self.assertEqual(self.planner._route_target_index, 2)
        self.assertEqual(self.planner._corner_phase, 0)

    def test_repeat_lap_leaves_outgoing_obstacle_to_route_planner(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 0
        self.planner._initial_route_target_index = 0
        self.planner.completed_laps = 1
        self.planner._corner_index = 0
        self.planner._corner_phase = 6
        desired = self.planner._corner_yaws()[0]
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (-0.50, 1.05), math.inf, 3),
        ]

        plan = self.planner._corner_plan(
            PoseEstimate(x_m=0.05, y_m=1.02, yaw_rad=desired), math.inf,
        )

        self.assertIsNone(plan)
        self.assertEqual(self.planner._route_target_index, 1)
        self.assertEqual(self.planner._corner_phase, 0)

    def test_repeat_lap_advances_each_corner_on_entry_not_before(self) -> None:
        for direction in (Direction.LEFT, Direction.RIGHT):
            for index in range(4):
                with self.subTest(direction=direction, index=index):
                    grid = GridMap()
                    grid.lock_direction(direction)
                    planner = GridPlanner(grid)
                    planner.completed_laps = 1
                    planner._route_target_index = index
                    x_m, y_m = planner._corner_centers()[index]
                    incoming = planner._corner_yaws()[(index - 1) % 4]
                    far = PoseEstimate(
                        x_m=x_m - 0.41 * math.cos(incoming),
                        y_m=y_m - 0.41 * math.sin(incoming),
                        yaw_rad=incoming,
                    )
                    near = PoseEstimate(
                        x_m=x_m - 0.39 * math.cos(incoming),
                        y_m=y_m - 0.39 * math.sin(incoming),
                        yaw_rad=incoming,
                    )

                    self.assertIsNone(planner._corner_plan(far, math.inf))
                    self.assertEqual(planner._route_target_index, index)
                    self.assertIsNone(planner._corner_plan(near, math.inf))
                    self.assertEqual(planner._route_target_index, (index + 1) % 4)
                    self.assertIsNone(planner._corner_plan(near, math.inf))
                    self.assertEqual(planner._route_target_index, (index + 1) % 4)
                    self.assertEqual(planner._corner_phase, 0)

    def test_recorded_repeat_corner_uses_next_obstacle_without_parking(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner.completed_laps = 1
        self.planner._route_target_index = 0
        obstacle = TrackedObstacle("red", ("right",), (-0.53, 1.12), math.inf, 3)
        self.planner._tracked_obstacles = [obstacle]
        pose = PoseEstimate(-0.055, 1.256, math.radians(172.7))

        plan = self.planner.plan(pose)

        self.assertEqual(self.planner._route_target_index, 1)
        self.assertEqual(self.planner._obstacle_program_anchor, obstacle.anchor)
        self.assertEqual(self.planner._program_action(), "right")
        self.assertEqual(plan.reason, "route")
        self.assertEqual(plan.motion_direction, 1)
        self.assertGreater(plan.speed_scale, 0.0)
        self.assertEqual(plan.target, (-2.0, 1.0))

    def test_lap_counts_when_pose_samples_cross_start_line(self) -> None:
        self.planner._lap_start_pose = (0.0, 0.0, math.pi / 2.0)
        self.planner._lap_previous_progress_pose = (0.10, 0.08)
        self.planner._lap_departed_start = True
        self.planner._lap_return_pending = True

        self.planner._update_lap_progress(
            PoseEstimate(x_m=0.10, y_m=-0.08),
            allow_completion=True,
        )

        self.assertEqual(self.planner.completed_laps, 1)
        self.assertFalse(self.planner._lap_return_pending)

    def test_lap_requires_all_corner_sections_after_direction_change(self) -> None:
        self.planner._route_target_index = 0
        for index in (0, 1, 0, 3):
            self.planner._route_target_index = index
            self.planner._advance_route_target(index)

        self.assertFalse(self.planner._lap_return_pending)

        self.planner._route_target_index = 2
        self.planner._advance_route_target(2)

        self.assertTrue(self.planner._lap_return_pending)

    def test_completed_u_turn_program_is_not_reacquired(self) -> None:
        obstacle = TrackedObstacle(
            "yellow", ("u_turn",), (0.30, 0.0), math.inf, 3,
        )
        self.planner._tracked_obstacles = [obstacle]
        self.planner._obstacle_program_anchor = obstacle.anchor
        self.planner._advance_obstacle_program(PoseEstimate())
        self.assertIsNone(self.planner._program_action())

        self.planner._route_target_index = 1
        self.planner._ensure_obstacle_program(PoseEstimate(
            x_m=1.50,
            y_m=0.0,
            yaw_rad=math.pi,
        ))
        self.planner._ensure_obstacle_program(PoseEstimate(
            x_m=0.0,
            y_m=0.0,
            yaw_rad=0.0,
        ))

        self.assertIsNone(self.planner._program_action())

    def test_only_acquires_obstacle_on_forward_route(self) -> None:
        self.planner._route_target_index = 0
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (0.45, 0.45), math.inf, 3),
        ]
        pose = PoseEstimate(x_m=0.0, y_m=0.0, yaw_rad=math.pi / 2.0)

        self.planner._ensure_obstacle_program(pose)
        self.assertIsNone(self.planner._program_action())

        self.planner._tracked_obstacles[0] = TrackedObstacle(
            "red", ("right",), (0.10, 0.45), math.inf, 3,
        )
        self.planner._last_motion_direction = -1
        self.planner._ensure_obstacle_program(pose)
        self.assertIsNone(self.planner._program_action())

        self.planner._last_motion_direction = 1
        self.planner._ensure_obstacle_program(pose)
        self.assertEqual(self.planner._program_action(), "right")

    def test_acquires_offset_obstacle_before_reaching_its_turn(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 1
        self.planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), (-1.38, 0.88), math.inf, 3),
        ]
        pose = PoseEstimate(
            x_m=-0.81,
            y_m=1.30,
            yaw_rad=math.radians(-174.0),
        )

        self.planner._ensure_obstacle_program(pose)

        self.assertEqual(self.planner._program_action(), "left")

    def test_does_not_acquire_color_from_one_camera_observation(self) -> None:
        self.planner._route_target_index = 1
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (0.45, 0.90), 1.75, 1),
        ]

        self.planner._ensure_obstacle_program(PoseEstimate(
            x_m=0.0,
            y_m=0.90,
            yaw_rad=0.0,
        ))

        self.assertIsNone(self.planner._program_action())

    def test_does_not_acquire_obstacle_lateral_to_corner_approach(self) -> None:
        self.planner._route_target_index = 0
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (0.45, 0.85), math.inf, 3),
        ]
        pose = PoseEstimate(
            x_m=0.18,
            y_m=0.61,
            yaw_rad=math.radians(84.0),
        )

        self.planner._ensure_obstacle_program(pose)

        self.assertIsNone(self.planner._program_action())

    def test_next_section_obstacle_waits_for_corner_handoff(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        self.planner._route_target_index = 1
        obstacle = TrackedObstacle("red", ("right",), (-2.03, 0.53), math.inf, 3)
        self.planner._tracked_obstacles = [obstacle]

        self.planner._ensure_obstacle_program(PoseEstimate(
            -1.667, 0.516, math.radians(-166.8),
        ))

        self.assertIsNone(self.planner._program_action())

        self.planner._route_target_index = 2
        self.planner._ensure_obstacle_program(PoseEstimate(
            -2.0, 1.0, -math.pi / 2.0,
        ))

        self.assertEqual(self.planner._program_action(), "right")
        self.assertEqual(self.planner._obstacle_program_anchor, obstacle.anchor)
        self.assertAlmostEqual(self.planner._obstacle_program_approach_yaw, -math.pi / 2.0)

    def test_active_program_survives_transient_tracking_loss(self) -> None:
        self.planner._route_target_index = 1
        obstacle = TrackedObstacle(
            "red", ("right",), (0.45, 0.90), math.inf, 3,
        )
        self.planner._tracked_obstacles = [obstacle]
        pose = PoseEstimate(x_m=0.0, y_m=0.90, yaw_rad=0.0)
        self.planner._ensure_obstacle_program(pose)
        self.planner._obstacle_program_gate_passed = True

        self.planner._tracked_obstacles = []
        self.planner._ensure_obstacle_program(pose)

        self.assertEqual(self.planner._program_action(), "right")
        self.assertEqual(self.planner._obstacle_program_anchor, obstacle.anchor)
        self.assertTrue(self.planner._obstacle_program_gate_passed)

    def test_same_color_does_not_merge_distinct_obstacles(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (0.0, 0.0), math.inf, 4),
        ]

        self.assertIsNone(
            self.planner._matching_tracked_obstacle("red", (0.35, 0.0)),
        )

    def test_same_placement_column_keeps_one_obstacle_identity(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (0.28, -0.55), math.inf, 4),
        ]

        self.assertEqual(
            self.planner._matching_tracked_obstacle("red", (-0.12, -0.53)),
            0,
        )

    def test_different_placement_columns_remain_distinct(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        self.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (-0.12, -0.50), math.inf, 4),
        ]

        self.assertIsNone(
            self.planner._matching_tracked_obstacle("red", (-0.12, 0.0)),
        )

    def test_far_same_ray_color_surface_cannot_claim_another_anchor(self) -> None:
        observations = (
            ColorObstacle("red", ("right",), depth_m=0.14, x_norm=-0.20),
            ColorObstacle("red", ("right",), depth_m=0.90, x_norm=-0.20),
        )
        candidates = ((0.0, (0.30, 0.0)), (0.1, (1.05, 0.0)))

        with patch.object(
            self.planner,
            "_supported_obstacle_candidates",
            return_value=candidates,
        ):
            assigned = self.planner._assign_color_observations(
                PoseEstimate(yaw_rad=0.0), observations,
            )

        self.assertEqual(assigned, ((0.30, 0.0), None))

    def test_close_camera_block_cannot_color_far_same_bearing_lidar_cluster(self) -> None:
        pose = PoseEstimate(x_m=0.0, y_m=-0.05, yaw_rad=math.pi * 0.5)
        far_cell = self.planner.grid.world_to_cell(0.28, 0.80)
        self.assertIsNotNone(far_cell)
        assert far_cell is not None
        self.planner._color_lidar_obstacles = {far_cell}

        candidates = self.planner._supported_obstacle_candidates(
            pose,
            expected_forward_m=0.50,
            expected_right_m=0.10,
        )

        self.assertEqual(candidates, ())

    def test_far_same_ray_color_surface_is_not_persisted(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        pose = PoseEstimate(yaw_rad=0.0)
        observations = (
            ColorObstacle("green", ("left",), depth_m=0.14, x_norm=-0.20),
            ColorObstacle("green", ("left",), depth_m=0.90, x_norm=-0.20),
        )
        candidates = ((0.0, (0.30, 0.0)), (0.1, (1.05, 0.0)))

        with patch.object(
            self.planner,
            "_supported_obstacle_candidates",
            return_value=candidates,
        ):
            self.planner.plan(
                pose,
                color_observations=observations,
                observation_time=1.0,
            )

        self.assertEqual(len(self.planner._tracked_obstacles), 1)
        self.assertEqual(self.planner._tracked_obstacles[0].anchor, (0.30, 0.0))
        self.assertEqual(self.planner._pending_colors, [])

    def test_anchor_identity_precedes_color_label(self) -> None:
        self.planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), (0.0, 0.0), math.inf, 2),
            TrackedObstacle("red", ("right",), (0.35, 0.0), math.inf, 2),
        ]

        self.assertEqual(
            self.planner._matching_tracked_obstacle("red", (0.04, 0.0)),
            0,
        )

    def test_rejected_batch_color_cannot_bypass_assignment(self) -> None:
        self.planner._uncolored_obstacle_anchor = (0.50, 0.0)
        observation = ColorObstacle(
            "green", ("left",), depth_m=0.337, x_norm=0.0,
        )

        self.planner.observe_colored_obstacle(
            PoseEstimate(yaw_rad=0.0),
            observation,
            now=1.0,
            direction_hint=Direction.RIGHT,
            assigned_anchor=None,
            association_resolved=True,
        )

        self.assertEqual(self.planner._tracked_obstacles, [])
        self.assertEqual(len(self.planner._pending_colors), 1)

    def test_far_red_cannot_overwrite_close_green_from_recorded_frame(self) -> None:
        self.planner.grid.lock_direction(Direction.LEFT)
        pose = PoseEstimate(-1.063, 1.206, math.radians(-151.1))
        anchor = (-1.43, 0.78)
        cell = self.planner.grid.world_to_cell(*anchor)
        assert cell is not None
        self.planner._uncolored_obstacle_anchor = anchor
        self.planner._color_lidar_obstacles = {cell}
        observations = (
            ColorObstacle("green", ("left",), 0.24, -0.44),
            ColorObstacle("red", ("right",), 0.93, -0.22),
        )

        for frame in range(3):
            assigned = self.planner._assign_color_observations(pose, observations)
            self.assertIsNotNone(assigned[0])
            self.assertIsNone(assigned[1])
            for observation, selected in zip(observations, assigned):
                self.planner.observe_colored_obstacle(
                    pose, observation, now=1.0 + frame * 0.1,
                    assigned_anchor=selected, association_resolved=True,
                )

        self.assertEqual(len(self.planner._tracked_obstacles), 1)
        tracked = self.planner._tracked_obstacles[0]
        self.assertEqual(tracked.label, "green")
        self.assertEqual(tracked.actions, ("left",))
        self.assertGreaterEqual(tracked.observations, 3)

    def test_early_camera_color_associates_when_lidar_anchor_appears(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        observation = ColorObstacle(
            "red", ("right",), depth_m=0.50, x_norm=0.0,
        )
        camera_pose = PoseEstimate(x_m=0.0, y_m=0.8, yaw_rad=0.0)
        self.planner.observe_colored_obstacle(camera_pose, observation, now=1.0)
        self.planner.observe_colored_obstacle(camera_pose, observation, now=1.1)

        self.assertEqual(self.planner._tracked_obstacles, [])
        self.assertEqual(len(self.planner._pending_colors), 1)

        self.planner._uncolored_obstacle_anchor = (0.68, 0.80)
        anchor_cell = self.planner.grid.world_to_cell(0.68, 0.80)
        self.assertIsNotNone(anchor_cell)
        assert anchor_cell is not None
        self.planner._confirmed_local_obstacles = {anchor_cell}
        self.planner._associate_pending_colors(camera_pose, now=1.7)

        self.assertEqual(len(self.planner._tracked_obstacles), 1)
        tracked = self.planner._tracked_obstacles[0]
        self.assertEqual(tracked.label, "red")
        self.assertGreaterEqual(tracked.observations, 2)
        self.assertEqual(tracked.anchor, (0.68, 0.80))
        self.assertEqual(self.planner._pending_colors, [])

    def test_single_early_camera_frame_cannot_color_later_lidar_anchor(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        observation = ColorObstacle(
            "green", ("left",), depth_m=0.50, x_norm=0.0,
        )
        pose = PoseEstimate(x_m=0.0, y_m=0.8, yaw_rad=0.0)
        self.planner.observe_colored_obstacle(pose, observation, now=1.0)
        self.planner._uncolored_obstacle_anchor = (0.68, 0.80)

        self.planner._associate_pending_colors(pose, now=2.0)

        self.assertEqual(self.planner._tracked_obstacles, [])
        self.assertEqual(self.planner._pending_colors, [])

    def test_transient_uncolored_anchor_cannot_claim_pending_color(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        observation = ColorObstacle(
            "red", ("right",), depth_m=0.50, x_norm=0.0,
        )
        pose = PoseEstimate(x_m=0.0, y_m=0.8, yaw_rad=0.0)
        self.planner.observe_colored_obstacle(pose, observation, now=1.0)
        self.planner.observe_colored_obstacle(pose, observation, now=1.1)
        self.planner._uncolored_obstacle_anchor = (0.68, 0.80)

        self.planner._associate_pending_colors(pose, now=1.7)

        self.assertEqual(self.planner._tracked_obstacles, [])
        self.assertEqual(len(self.planner._pending_colors), 1)

    def test_fresh_lidar_color_overrides_conflicting_pending_projection(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        pose = PoseEstimate(x_m=0.0, y_m=0.8, yaw_rad=0.0)
        red = ColorObstacle("red", ("right",), depth_m=0.50, x_norm=0.0)
        self.planner.observe_colored_obstacle(pose, red, now=1.0)
        self.planner.observe_colored_obstacle(pose, red, now=1.1)
        anchor = (0.68, 0.80)
        anchor_cell = self.planner.grid.world_to_cell(*anchor)
        self.assertIsNotNone(anchor_cell)
        assert anchor_cell is not None
        self.planner._confirmed_local_obstacles = {anchor_cell}
        self.planner._uncolored_obstacle_anchor = anchor
        self.planner._tracked_obstacles = [
            TrackedObstacle("green", ("left",), anchor, math.inf, 1),
        ]

        self.planner._associate_pending_colors(pose, now=1.2)

        self.assertEqual(len(self.planner._tracked_obstacles), 1)
        self.assertEqual(self.planner._tracked_obstacles[0].label, "green")
        self.assertEqual(len(self.planner._pending_colors), 1)

    def test_unmatched_camera_blob_does_not_stop_repeat_lap(self) -> None:
        self.planner.grid.lock_direction(Direction.RIGHT)
        self.planner.completed_laps = 1
        observation = ColorObstacle(
            "red", ("right",), depth_m=0.50, x_norm=0.0,
        )

        self.planner.observe_colored_obstacle(
            PoseEstimate(y_m=0.8, yaw_rad=0.0),
            observation,
            now=10.0,
            direction_hint=Direction.RIGHT,
            assigned_anchor=None,
            association_resolved=True,
        )

        self.assertEqual(self.planner._pending_camera_hold_until, 0.0)
        self.assertEqual(
            self.planner.color_observation_status(), "camera-known-course",
        )


if __name__ == "__main__":
    unittest.main()
