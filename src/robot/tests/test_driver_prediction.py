from __future__ import annotations

import math
import threading
import unittest
from collections import deque
from unittest.mock import MagicMock, patch

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from planner.config import DriverConfig
from planner.driver import Driver
from planner.grid import Direction, GridMap
from planner.local_grid import LocalGrid
from planner.localize_types import PoseEstimate
from planner.planner import GridPlanner, TrackedObstacle


class DriverPredictionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.driver = Driver.__new__(Driver)
        self.driver.config = DriverConfig()
        grid = GridMap()
        grid.lock_direction(Direction.RIGHT)
        self.driver.planner = GridPlanner(grid)
        self.driver.last_prediction_block_reason = "none"
        self.driver.last_prediction_block_point = None
        self.driver.last_prediction_escape_cells = 0
        self.driver.command_history = []
        self.local_grid = LocalGrid(size_m=2.4)
        noisy_cell = grid.world_to_cell(0.60, -0.80)
        self.assertIsNotNone(noisy_cell)
        assert noisy_cell is not None
        self.driver.planner._confirmed_local_obstacles.add(noisy_cell)

    def _escape_pose_is_clear(self) -> bool:
        scan_pose = PoseEstimate(0.56, -0.735, math.radians(175.0))
        return self.driver._prediction_pose_clear(
            0.57,
            -0.735,
            math.radians(175.0),
            scan_pose.x_m,
            scan_pose.y_m,
            scan_pose.yaw_rad,
            scan_pose,
            self.local_grid,
            set(),
        )

    def test_raw_persisted_cell_blocks_overlapping_footprint(self) -> None:
        self.assertFalse(self._escape_pose_is_clear())
        self.assertEqual(
            self.driver.last_prediction_block_reason,
            "persisted-obstacle",
        )

    def test_tracked_active_obstacle_uses_compact_footprint(self) -> None:
        obstacle = TrackedObstacle(
            "red", ("right",), (0.47, -0.88), math.inf, 3,
        )
        assert self.driver.planner is not None
        self.driver.planner._tracked_obstacles = [obstacle]
        self.driver.planner._obstacle_program_anchor = obstacle.anchor

        self.assertTrue(self._escape_pose_is_clear())

    def test_wall_contact_allows_departure_but_not_deeper_contact(self) -> None:
        self.driver.planner = GridPlanner(GridMap())
        self.driver.planner.grid.lock_direction(Direction.LEFT)
        pose = PoseEstimate(-1.473, 0.608, math.radians(-145.8))
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                clear = self.driver._prediction_pose_clear(
                    pose.x_m + direction * 0.01 * math.cos(pose.yaw_rad),
                    pose.y_m + direction * 0.01 * math.sin(pose.yaw_rad),
                    pose.yaw_rad, pose.x_m, pose.y_m, pose.yaw_rad,
                    pose, self.local_grid, set(),
                )
                self.assertEqual(clear, direction == -1)

    def test_tracked_obstacle_contact_uses_unrounded_anchor(self) -> None:
        self.driver.planner = GridPlanner(GridMap())
        self.driver.planner.grid.lock_direction(Direction.LEFT)
        self.driver.planner._tracked_obstacles = [
            TrackedObstacle("red", ("right",), (-2.17, -.47), math.inf, 3),
        ]
        pose = PoseEstimate(-2.31, -.47, 0.0)

        self.assertFalse(self.driver._prediction_pose_clear(
            -2.29, -.47, 0.0, pose.x_m, pose.y_m, pose.yaw_rad,
            pose, self.local_grid, set(),
        ))
        self.assertEqual(self.driver.last_prediction_block_reason, "tracked-obstacle")
        self.assertTrue(self.driver._prediction_pose_clear(
            -2.31, -.47, 0.0, -2.29, -.47, 0.0,
            pose, self.local_grid, set(),
        ))

    def _prepare_sensor_tick(self) -> None:
        self.localizer_mock = MagicMock()
        self.driver.localizer = self.localizer_mock
        self.driver.state_lock = threading.Lock()
        self.driver.latest_scan = MagicMock()
        self.driver.latest_scan_sim_pose = None
        self.driver.latest_scan_stamp = 10.0
        self.driver.odometry_history = deque(((10.0, (0.0, 0.0, 0.0, 0.0)),))
        self.driver.last_processed_scan_stamp = 9.0
        self.driver.latest_scan_received_at = 99.9
        self.driver.latest_scan_sample_time = 99.75
        self.driver.latest_color_obstacles = None
        self.driver.imu_history = deque(((9.99, 0.0),))
        self.command_mock = MagicMock()
        self.wait_mock = MagicMock()
        self.driver._command = self.command_mock
        self.driver._log_sensor_wait = self.wait_mock

    def test_pending_imu_does_not_insert_stop_command(self) -> None:
        self._prepare_sensor_tick()
        with patch("planner.driver.time.monotonic", return_value=100.0):
            self.driver._process_tick()

        self.wait_mock.assert_called_once_with("imu")
        self.command_mock.assert_not_called()

    def test_stale_scan_stops_existing_command(self) -> None:
        self._prepare_sensor_tick()
        self.driver.latest_scan_received_at = 99.0
        self.driver.latest_scan_sample_time = 98.85

        with patch("planner.driver.time.monotonic", return_value=100.0):
            self.driver._process_tick()

        self.wait_mock.assert_called_once_with("scan")
        self.command_mock.assert_called_once_with(0, 0)

    def test_odometry_dropout_cancels_existing_command(self) -> None:
        self._prepare_sensor_tick()
        self.driver.odometry_history = deque(((9.7, (0.0, 0.0, 0.0, 0.25)),))
        with patch("planner.driver.time.monotonic", return_value=100.0):
            self.driver._process_tick()

        self.wait_mock.assert_called_once_with("odometry")
        self.command_mock.assert_called_once_with(0, 0)
        self.localizer_mock.apply_odometry.assert_not_called()

    def test_imu_dropout_cancels_existing_command(self) -> None:
        for samples in ((), ((9.7, 0.0),)):
            with self.subTest(samples=samples):
                self._prepare_sensor_tick()
                self.driver.imu_history = deque(samples)
                with patch("planner.driver.time.monotonic", return_value=100.0):
                    self.driver._process_tick()

                self.wait_mock.assert_called_once_with("imu")
                self.command_mock.assert_called_once_with(0, 0)

    def test_odometry_arriving_after_scan_retries_pending_frame(self) -> None:
        self._prepare_sensor_tick()
        self.driver.odometry_history.clear()
        self.driver.work_event = threading.Event()
        message = Odometry()
        message.header.stamp.sec = 10
        message.twist.twist.linear.x = 0.2

        with patch("planner.driver.time.monotonic", return_value=100.0):
            self.driver._process_tick()
            self.driver._odom_cb(message)
            self.assertTrue(self.driver.work_event.is_set())
            self.driver._process_tick()

        self.assertEqual(
            [call.args[0] for call in self.wait_mock.call_args_list],
            ["odometry", "imu"],
        )
        self.command_mock.assert_called_once_with(0, 0)

        self.driver.last_processed_scan_stamp = 10.0
        self.driver.work_event.clear()
        self.driver._odom_cb(message)
        self.assertFalse(self.driver.work_event.is_set())

    def test_measured_twist_direction_is_independent_of_commands(self) -> None:
        for command, measured in ((150, -0.126), (-150, 0.25), (0, -0.10)):
            with self.subTest(command=command, measured=measured):
                self.driver.command_history = [(10.0, command, 0)]
                self.driver.localizer = MagicMock()
                self.driver.last_motion_stamp = 11.0

                self.driver._apply_motion_update(11.1, (0.0, 0.0, 0.0, measured))

                args = self.driver.localizer.apply_odometry.call_args.args
                self.assertEqual(args[0], measured)
                self.assertAlmostEqual(args[1], 0.1)
                self.assertEqual(self.driver.latest_odometry_speed_mps, measured)

    def test_steering_prediction_matches_recorded_half_second_turns(self) -> None:
        windows = (
            (0.259909, 320, -13.1885),
            (0.196613, -320, 12.4936),
            (-0.248088, 320, 12.9057),
        )
        for speed, servo, yaw_degrees in windows:
            with self.subTest(speed=speed, servo=servo):
                with patch.object(self.driver, "_motor_speed", return_value=speed):
                    _, _, yaw, _ = self.driver._integrate_control(
                        0.0, 0.0, 0.0, speed, 150 if speed > 0 else -150,
                        servo, 0.5, [], PoseEstimate(), None, set(), 0.0,
                    )
                self.assertLess(abs(math.degrees(yaw) - yaw_degrees), 3.0)

    def test_route_and_prediction_share_default_turning_radius(self) -> None:
        assert self.driver.config is not None
        assert self.driver.planner is not None
        self.assertAlmostEqual(
            self.driver.config.max_steering_angle_rad,
            self.driver.planner.max_steering_angle_rad,
        )
        radius = self.driver.planner.wheelbase_m / math.tan(
            self.driver.planner.max_steering_angle_rad,
        )
        self.assertGreater(radius, 0.50)
        self.assertLess(radius, 0.57)

    def test_scan_measurement_time_includes_transport_delay(self) -> None:
        self.driver.state_lock = threading.Lock()
        self.driver.latest_sim_pose = None
        self.driver.odometry_history = deque()
        self.driver.get_clock = MagicMock()
        self.driver.get_clock.return_value.now.return_value.nanoseconds = 500_150_000_000
        scan = LaserScan()
        scan.header.stamp.sec = 500

        with patch("planner.driver.time.monotonic", return_value=1000.20):
            self.driver._scan_cb(scan)

        self.assertAlmostEqual(self.driver.latest_scan_received_at, 1000.20)
        self.assertAlmostEqual(self.driver.latest_scan_sample_time, 1000.05)

    def test_prediction_includes_sensor_processing_and_precedes_debug(self) -> None:
        driver = MagicMock(spec=Driver)
        driver.config = DriverConfig(auto_drive_enabled=True, drive_motor=150)
        driver.localizer = MagicMock()
        driver.localizer.grid.resolution_m = 0.05
        driver.planner = MagicMock()
        driver.planner.course_complete = False
        driver.planner.plan.return_value.speed_scale = 1.0
        driver.planner.plan.return_value.motion_direction = 1
        driver.state_lock = threading.Lock()
        driver.latest_scan = object()
        driver.latest_scan_sim_pose = None
        driver.latest_scan_stamp = 10.0
        driver.odometry_history = deque(((10.0, (0.0, 0.0, 0.0, 0.0)),))
        driver._nearest_sample.side_effect = Driver._nearest_sample
        driver.latest_scan_received_at = 99.9
        driver.latest_scan_sample_time = 99.75
        driver.latest_color_obstacles = None
        driver.last_processed_scan_stamp = 0.0
        driver.imu_history = deque(((10.0, 0.0),))
        driver._predict_delayed_pose.return_value = (PoseEstimate(), ())
        driver._servo_for_plan.return_value = 0
        driver.last_prediction_blocked = False
        order = []
        driver._command.side_effect = lambda *_args: order.append("command")
        driver._log_pose_debug.side_effect = lambda *_args: order.append("log")
        driver._queue_debug_frame.side_effect = lambda *_args: order.append("view")

        with patch("planner.driver.time.monotonic", side_effect=(100.0, 100.1)), patch(
            "planner.driver.local_grid_from_scan",
        ):
            Driver._process_tick(driver)

        driver._command.assert_called_once_with(150, 0)
        self.assertEqual(order, ["command", "log", "view"])
        driver._apply_motion_update.assert_called_once_with(
            10.0, driver.odometry_history[0][1],
        )
        self.assertAlmostEqual(driver._predict_delayed_pose.call_args.args[1], 100.1)
        self.assertAlmostEqual(driver._predict_delayed_pose.call_args.args[2], 0.35)


if __name__ == "__main__":
    unittest.main()
