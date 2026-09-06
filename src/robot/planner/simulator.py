#!/usr/bin/env python3
from __future__ import annotations

import math
import random
import threading
import time
from array import array
from dataclasses import dataclass
from typing import cast

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Int32

from .camera_color import OBSTACLE_COLORS_BGR
from .config import DriverConfig
from .simulator_view import SimulatorViewMixin


Segment = tuple[tuple[float, float], tuple[float, float]]


def add_imperfect_rect(
    segments: list[Segment],
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> None:
    for a, b in (
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ):
        _add_imperfect_segment(segments, a, b)


def _add_imperfect_segment(
    segments: list[Segment],
    a: tuple[float, float],
    b: tuple[float, float],
) -> None:
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    if length < 0.25:
        segments.append((a, b))
        return
    ux = (b[0] - a[0]) / length
    uy = (b[1] - a[1]) / length
    pieces = max(2, int(round(length / 0.75)))
    trim = random.uniform(0.005, 0.018)
    gap = random.uniform(0.012, 0.030)
    for i in range(pieces):
        start = max(0.0, length * i / pieces + trim)
        end = min(length, length * (i + 1) / pieces - trim)
        if i > 0:
            start += gap * random.uniform(0.35, 0.65)
        if i < pieces - 1:
            end -= gap * random.uniform(0.35, 0.65)
        if end - start > 0.04:
            segments.append((
                (a[0] + ux * start, a[1] + uy * start),
                (a[0] + ux * end, a[1] + uy * end),
            ))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class RobotState:
    x_m: float = 1.0
    y_m: float = 0.0
    yaw_rad: float = math.pi / 2.0
    motor: int = 0
    servo: int = 0


@dataclass(frozen=True)
class SimObstacle:
    rect: tuple[float, float, float, float]
    label: str
    color_bgr: tuple[int, int, int]


class PlannerIoSimulator(SimulatorViewMixin, Node):
    def __init__(self) -> None:
        super().__init__("wro_planner_io_simulator")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu_data")
        self.declare_parameter("motor_topic", "/microROS/motor_control")
        self.declare_parameter("servo_topic", "/microROS/servo_control")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("lidar_rate_hz", 10.0)
        self.declare_parameter("max_range_m", 2.2)
        self.declare_parameter("noise_std_m", 0.03); self.declare_parameter("random_projection_ratio", 0.10)
        self.declare_parameter("lidar_cardinal_good_width_deg", 12.0)
        self.declare_parameter("lidar_diagonal_keep_ratio", 1.0)
        self.declare_parameter("lidar_diagonal_hash_period_deg", 7.0)
        self.declare_parameter("lidar_ghost_enabled", True)
        self.declare_parameter("lidar_ghost_band_width_deg", 8.0)
        self.declare_parameter("motor_speed_mps_per_unit", 0.0023333333333333335)
        self.declare_parameter("motor_speed_scale", 0.912)
        self.declare_parameter("motor_speed_drift_amplitude", 0.05)
        self.declare_parameter("motor_accel_tau_s", 0.215)
        self.declare_parameter("motor_decel_tau_s", 0.354)
        self.declare_parameter("motor_deadband_command", 90)
        self.declare_parameter("planning_motor_reference", 150)
        self.declare_parameter("command_delay_s", 0.15)
        self.declare_parameter("command_timeout_s", 0.4)
        self.declare_parameter("max_servo", 320)
        self.declare_parameter("wheelbase_m", 0.138)
        self.declare_parameter("max_steering_angle_rad", DriverConfig.max_steering_angle_rad)
        self.declare_parameter("steering_effectiveness", 1.0)
        self.declare_parameter("robot_length_m", 0.22)
        self.declare_parameter("robot_width_m", 0.15)
        self.declare_parameter("initial_x_m", 1.0)
        self.declare_parameter("initial_y_m", 0.0)
        self.declare_parameter("initial_yaw_rad", math.pi / 2.0)
        self.declare_parameter("odometry_linear_scale", 0.95)
        self.declare_parameter("odometry_right_scale", 0.85)
        self.declare_parameter("odometry_left_scale", 1.10)
        self.declare_parameter("odometry_yaw_scale", 0.99)
        self.declare_parameter("odometry_linear_drift_amplitude", 0.08)
        self.declare_parameter("odometry_yaw_drift_amplitude", 0.04)
        self.declare_parameter("odometry_drift_period_s", 45.0)
        self.declare_parameter("imu_yaw_scale", 1.0)
        self.declare_parameter("lidar_range_scale", 1.0)
        self.declare_parameter("lidar_range_drift_amplitude", 0.015)
        self.declare_parameter("obstacle_block_size_m", 0.05)
        self.declare_parameter("obstacle_height_m", 0.10)
        self.declare_parameter("camera_rate_hz", 30.0)
        self.declare_parameter("camera_depth_misalignment_ratio", 0.35)
        self.declare_parameter("camera_depth_misalignment_max_px", 6)
        self.declare_parameter("camera_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("view_enabled", True)
        self.declare_parameter("simulator_seed", -1)

        simulator_seed = self._int_param("simulator_seed")
        if simulator_seed >= 0:
            random.seed(simulator_seed)
        self._camera_rng = random.Random(
            None if simulator_seed < 0 else simulator_seed ^ 0x43414D,
        )

        self.scan_topic = self._str_param("scan_topic")
        self.odom_topic = self._str_param("odom_topic")
        self.imu_topic = self._str_param("imu_topic")
        self.motor_topic = self._str_param("motor_topic")
        self.servo_topic = self._str_param("servo_topic")
        self.rate_hz = self._float_param("rate_hz")
        self.lidar_rate_hz = self._float_param("lidar_rate_hz")
        self.max_range_m = self._float_param("max_range_m")
        self.noise_std_m = self._float_param("noise_std_m"); self.random_projection_ratio = self._float_param("random_projection_ratio")
        self.lidar_cardinal_good_width_deg = self._float_param("lidar_cardinal_good_width_deg")
        self.lidar_diagonal_keep_ratio = self._float_param("lidar_diagonal_keep_ratio")
        self.lidar_diagonal_hash_period_deg = self._float_param("lidar_diagonal_hash_period_deg")
        self.lidar_ghost_enabled = bool(self.get_parameter("lidar_ghost_enabled").value)
        self.lidar_ghost_band_width_deg = self._float_param("lidar_ghost_band_width_deg")
        self.motor_speed_mps_per_unit = self._float_param("motor_speed_mps_per_unit")
        self.motor_speed_scale = self._float_param("motor_speed_scale")
        self.motor_speed_drift_amplitude = self._float_param("motor_speed_drift_amplitude")
        self.motor_accel_tau_s = self._float_param("motor_accel_tau_s")
        self.motor_decel_tau_s = self._float_param("motor_decel_tau_s")
        self.motor_deadband_command = self._int_param("motor_deadband_command")
        self.planning_motor_reference = self._int_param("planning_motor_reference")
        self.command_delay_s = max(0.0, self._float_param("command_delay_s"))
        self.command_timeout_s = self._float_param("command_timeout_s")
        self.max_servo = self._int_param("max_servo")
        self.wheelbase_m = self._float_param("wheelbase_m")
        self.max_steering_angle_rad = self._float_param("max_steering_angle_rad")
        self.steering_effectiveness = self._float_param("steering_effectiveness")
        self.robot_length_m = self._float_param("robot_length_m")
        self.robot_width_m = self._float_param("robot_width_m")
        self.initial_x_m = self._float_param("initial_x_m")
        self.initial_y_m = self._float_param("initial_y_m")
        self.initial_yaw_rad = self._float_param("initial_yaw_rad")
        self.odometry_linear_scale = self._float_param("odometry_linear_scale")
        self.odometry_right_scale = self._float_param("odometry_right_scale")
        self.odometry_left_scale = self._float_param("odometry_left_scale")
        self.odometry_yaw_scale = self._float_param("odometry_yaw_scale")
        self.odometry_linear_drift_amplitude = self._float_param("odometry_linear_drift_amplitude")
        self.odometry_yaw_drift_amplitude = self._float_param("odometry_yaw_drift_amplitude")
        self.odometry_drift_period_s = max(self._float_param("odometry_drift_period_s"), 1.0)
        self.imu_yaw_scale = self._float_param("imu_yaw_scale")
        self.lidar_range_scale = self._float_param("lidar_range_scale")
        self.lidar_range_drift_amplitude = self._float_param("lidar_range_drift_amplitude")
        self.obstacle_block_size_m = self._float_param("obstacle_block_size_m")
        self.obstacle_height_m = self._float_param("obstacle_height_m")
        self.camera_rate_hz = self._float_param("camera_rate_hz")
        self.camera_depth_misalignment_ratio = self._float_param(
            "camera_depth_misalignment_ratio",
        )
        self.camera_depth_misalignment_max_px = self._int_param(
            "camera_depth_misalignment_max_px",
        )
        self.view_enabled = bool(self.get_parameter("view_enabled").value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        camera_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, qos)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, qos)
        self.camera_pub = self.create_publisher(Image, self._str_param("camera_image_topic"), camera_qos)
        self.depth_pub = self.create_publisher(Image, self._str_param("camera_depth_topic"), camera_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, self._str_param("camera_info_topic"), camera_qos)
        self.pose_pub = self.create_publisher(Float32MultiArray, "/wro_sim/pose", qos)
        self.create_subscription(Int32, self.motor_topic, self._motor_cb, qos)
        self.create_subscription(Int32, self.servo_topic, self._servo_cb, qos)

        self.start_x_m = random.choice((-1.0, 1.0)) * abs(self.initial_x_m)
        self.start_y_m = self.initial_y_m
        expected_direction = "LEFT" if self.start_x_m > 0.0 else "RIGHT"
        self.get_logger().info(
            f"SIM_START side_x={self.start_x_m:+.2f} expected_direction={expected_direction}"
        )
        self.robot = self._initial_robot()
        self.pending_motor_commands: list[tuple[float, int]] = []
        self.pending_servo_commands: list[tuple[float, int]] = []
        self.last_time = time.monotonic()
        self.last_time_origin = self.last_time
        self.last_motor_command_time = self.last_time
        self.last_collision_log_time = 0.0
        self.odometry_linear_mps = 0.0
        self.linear_speed_mps = 0.0
        self.odometry_angular_rps = 0.0
        self.odometry_x_m = 0.0
        self.odometry_y_m = 0.0
        self.odometry_yaw_rad = self.initial_yaw_rad
        self.dragging = False
        self.drag_aim: tuple[float, float] | None = None
        self.window = "wro planner io simulator"
        self.camera_window = "wro simulator camera"
        self.latest_camera_view: np.ndarray | None = None
        self.world_extent_m = 2.2
        self.canvas_px = 1280
        self.ui_scale = self.canvas_px / 780.0
        obstacle_rects = self._obstacle_rects()
        labels = ["red", "green"]
        random.shuffle(labels)
        obstacle_labels = [labels[index % len(labels)] for index in range(len(obstacle_rects))]
        self.obstacles = tuple(
            SimObstacle(rect, label, OBSTACLE_COLORS_BGR[label])
            for rect, label in zip(obstacle_rects, obstacle_labels)
        )
        self.get_logger().info(f"SIM_OBSTACLES count={len(self.obstacles)}")
        for obstacle in self.obstacles:
            x0, y0, x1, y1 = obstacle.rect
            self.get_logger().info(
                f"SIM_OBSTACLE color={obstacle.label} "
                f"x={(x0 + x1) * 0.5 - self.start_x_m:+.3f} "
                f"y={(y0 + y1) * 0.5 - self.start_y_m:+.3f}"
            )
        self.block_rects = ((-0.5, -0.5, 0.5, 0.5), *obstacle_rects)
        self.segments = self._build_world()
        self.ghost_segments = self._build_ghost_world()
        self._segment_array = np.asarray(self.segments, dtype=np.float64)
        self._ghost_segment_array = np.asarray(self.ghost_segments, dtype=np.float64)
        self._camera_ray_cache: dict[
            tuple[int, int, float, float, float, float, float],
            tuple[np.ndarray, np.ndarray, np.ndarray],
        ] = {}
        self._camera_geometry = tuple(
            [(rect, 0.10, bgr) for rect, bgr in self._camera_wall_boxes()]
            + [
                (obstacle.rect, self.obstacle_height_m, obstacle.color_bgr)
                for obstacle in self.obstacles
            ]
        )
        if self.view_enabled:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, self.canvas_px, self.canvas_px)
            cv2.setMouseCallback(self.window, self._mouse_cb)
            cv2.namedWindow(self.camera_window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.camera_window, 640, 480)
        self.create_timer(1.0 / max(self.lidar_rate_hz, 1.0), self._lidar_tick)
        if self.view_enabled:
            self.create_timer(1.0 / 20.0, self._view_tick)
        self._physics_stop = threading.Event()
        self._physics_thread = threading.Thread(target=self._physics_loop, name="sim-physics", daemon=True)
        self._camera_thread = threading.Thread(target=self._camera_loop, name="sim-camera", daemon=True)
        self._physics_thread.start()
        self._camera_thread.start()

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(cast(float, self.get_parameter(name).value))

    def _int_param(self, name: str) -> int:
        return int(cast(int, self.get_parameter(name).value))

    def _motor_cb(self, msg: Int32) -> None:
        now = time.monotonic()
        self.pending_motor_commands.append((now + self.command_delay_s, int(msg.data)))
        self.last_motor_command_time = now

    def _servo_cb(self, msg: Int32) -> None:
        value = int(clamp(int(msg.data), -self.max_servo, self.max_servo))
        self.pending_servo_commands.append((time.monotonic() + self.command_delay_s, value))

    def _build_world(self) -> tuple[Segment, ...]:
        wall_segments: list[Segment] = []
        add_imperfect_rect(wall_segments, -1.5, -1.5, 1.5, 1.5)
        add_imperfect_rect(wall_segments, -0.5, -0.5, 0.5, 0.5)
        self.camera_wall_segments = tuple(wall_segments)
        segments = list(wall_segments)
        for rect in self.block_rects[1:]:
            add_imperfect_rect(segments, *rect)
        return tuple(segments)

    @staticmethod
    def _build_ghost_world() -> tuple[Segment, ...]:
        return (
            ((-1.82, -1.82), (1.82, -1.82)),
            ((1.82, -1.82), (1.82, 1.82)),
            ((1.82, 1.82), (-1.82, 1.82)),
            ((-1.82, 1.82), (-1.82, -1.82)),
            ((-1.82, -1.68), (-1.28, -1.82)),
            ((1.28, 1.82), (1.82, 1.68)),
            ((-1.82, 1.18), (-1.42, 1.82)),
            ((1.42, -1.82), (1.82, -1.18)),
        )

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(now - self.last_time, 0.1)
        self.last_time = now
        self._apply_delayed_commands(now)
        self._apply_command_watchdog(now)
        if not self.dragging:
            self._advance_robot(dt)
        else:
            self.linear_speed_mps = 0.0
            self.odometry_linear_mps = 0.0
            self.odometry_angular_rps = 0.0
        self.pose_pub.publish(Float32MultiArray(data=[
            self.robot.x_m - self.start_x_m,
            self.robot.y_m - self.start_y_m,
            self.robot.yaw_rad,
            self.odometry_linear_mps,
        ]))
        self.odom_pub.publish(self._make_odometry(dt))
        self.imu_pub.publish(self._make_imu())

    def _physics_loop(self) -> None:
        period = 1.0 / max(self.rate_hz, 20.0)
        deadline = time.monotonic()
        while not self._physics_stop.is_set() and rclpy.ok():
            self._tick()
            deadline += period
            delay = deadline - time.monotonic()
            if delay <= 0.0:
                deadline = time.monotonic()
                continue
            self._physics_stop.wait(delay)

    def stop_workers(self) -> None:
        self._physics_stop.set()
        self._physics_thread.join(timeout=1.0)
        self._camera_thread.join(timeout=1.0)

    def _lidar_tick(self) -> None:
        self.scan_pub.publish(self._make_scan())

    def _camera_tick(self) -> None:
        color, depth, info = self._make_camera_frame()
        if self.view_enabled:
            self.latest_camera_view = np.frombuffer(color.data, dtype=np.uint8).reshape(
                color.height, color.width, 3,
            ).copy()
        self.depth_pub.publish(depth)
        self.camera_pub.publish(color)
        self.camera_info_pub.publish(info)

    def _camera_loop(self) -> None:
        period = 1.0 / max(self.camera_rate_hz, 1.0)
        deadline = time.monotonic()
        while not self._physics_stop.is_set() and rclpy.ok():
            self._camera_tick()
            deadline += period
            delay = deadline - time.monotonic()
            if delay <= 0.0:
                deadline = time.monotonic()
                continue
            self._physics_stop.wait(delay)

    def _view_tick(self) -> None:
        now = time.monotonic()
        if self.latest_camera_view is not None:
            cv2.imshow(self.camera_window, self.latest_camera_view)
        key = self._draw()
        if key == ord("0"):
            self.get_logger().info("SIM_RESET keyboard")
            self.robot = self._initial_robot()
            self._reset_odometry()
            self.pending_motor_commands.clear()
            self.pending_servo_commands.clear()
            self.last_motor_command_time = now
        elif key in (ord("q"), ord("Q"), 27):
            rclpy.shutdown()

    def _apply_command_watchdog(self, now: float) -> None:
        if now - self.last_motor_command_time > self.command_timeout_s:
            self.robot.motor = 0

    def _apply_delayed_commands(self, now: float) -> None:
        while self.pending_motor_commands and self.pending_motor_commands[0][0] <= now:
            _deadline, self.robot.motor = self.pending_motor_commands.pop(0)
        while self.pending_servo_commands and self.pending_servo_commands[0][0] <= now:
            _deadline, self.robot.servo = self.pending_servo_commands.pop(0)

    def _initial_robot(self) -> RobotState:
        return RobotState(x_m=self.start_x_m, y_m=self.start_y_m, yaw_rad=self.initial_yaw_rad)

    def _reset_odometry(self) -> None:
        self.linear_speed_mps = 0.0
        self.odometry_x_m = 0.0
        self.odometry_y_m = 0.0
        self.odometry_yaw_rad = self.initial_yaw_rad
        self.odometry_linear_mps = 0.0
        self.odometry_angular_rps = 0.0

    def _obstacle_rects(self) -> tuple[tuple[float, float, float, float], ...]:
        half = max(0.02, self.obstacle_block_size_m) * 0.5
        centers = []
        for side_index, side in enumerate(self._side_slots()):
            start_side_index = 1 if self.start_x_m > 0.0 else 3
            if side_index == start_side_index:
                side = tuple(point for point in side if abs(point[1]) > 0.25)
            centers.extend(self._side_obstacle_centers(side))
        return tuple((cx - half, cy - half, cx + half, cy + half) for cx, cy in centers)


    @staticmethod
    def _side_slots() -> tuple[tuple[tuple[float, float], ...], ...]:
        along = (-0.5, 0.0, 0.5)
        inner_outer = (0.875, 1.125)
        top = tuple((x, radial) for x in along for radial in inner_outer)
        right = tuple((radial, -x) for x in along for radial in inner_outer)
        bottom = tuple((-x, -radial) for x in along for radial in inner_outer)
        left = tuple((-radial, x) for x in along for radial in inner_outer)
        return top, right, bottom, left

    @staticmethod
    def _side_obstacle_centers(side: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], ...]:
        if random.randint(1, 2) == 1:
            return (random.choice(side),)
        return random.choice(side[0:2]), random.choice(side[-2:])

    def _advance_robot(self, dt: float) -> None:
        motor = clamp(self.robot.motor, -255, 255)
        magnitude = abs(motor)
        reference = max(1, self.planning_motor_reference)
        deadband = min(max(0, self.motor_deadband_command), reference - 1)
        if 0 < magnitude < reference and deadband > 0:
            magnitude = max(0.0, (magnitude - deadband) * reference / (reference - deadband))
            motor = math.copysign(magnitude, motor)
        elapsed = time.monotonic() - self.last_time_origin
        motor_drift = 1.0 + self.motor_speed_drift_amplitude * math.sin(math.tau * elapsed / 37.0 + 0.35)
        target_speed = motor * self.motor_speed_mps_per_unit * self.motor_speed_scale * motor_drift
        accelerating = (
            self.linear_speed_mps * target_speed >= 0.0
            and abs(target_speed) > abs(self.linear_speed_mps)
        )
        tau = self.motor_accel_tau_s if accelerating else self.motor_decel_tau_s
        alpha = 1.0 - math.exp(-dt / max(tau, 1.0e-3))
        next_speed = self.linear_speed_mps + (target_speed - self.linear_speed_mps) * alpha
        speed = 0.5 * (self.linear_speed_mps + next_speed)
        self.linear_speed_mps = next_speed
        steering_angle = (
            -clamp(self.robot.servo, -self.max_servo, self.max_servo)
            / max(self.max_servo, 1)
            * self.max_steering_angle_rad
            * self.steering_effectiveness
        )
        yaw_rate = speed / max(self.wheelbase_m, 0.01) * math.tan(steering_angle)
        next_yaw = self.robot.yaw_rad + yaw_rate * dt
        travel_yaw = self.robot.yaw_rad + yaw_rate * dt * 0.5
        next_x = self.robot.x_m + math.cos(travel_yaw) * speed * dt
        next_y = self.robot.y_m + math.sin(travel_yaw) * speed * dt
        self.odometry_linear_mps = 0.0
        self.odometry_angular_rps = 0.0
        collision = self._collision_at(next_x, next_y, next_yaw)
        if collision is None:
            self.robot.x_m = next_x
            self.robot.y_m = next_y
            self.robot.yaw_rad = next_yaw
            self.odometry_linear_mps = speed
            self.odometry_angular_rps = yaw_rate
            self._integrate_odometry(speed, yaw_rate, dt)
        else:
            self.linear_speed_mps = 0.0
            if abs(speed) > 1.0e-4 and time.monotonic() - self.last_collision_log_time >= 0.5:
                self.last_collision_log_time = time.monotonic()
                self.get_logger().info(
                    f"SIM_COLLISION map=({next_x:+.3f},{next_y:+.3f},{math.degrees(next_yaw):+.1f}deg) "
                    f"local=({next_x - self.start_x_m:+.3f},{next_y - self.start_y_m:+.3f}) geometry={collision}"
                )

    def _integrate_odometry(self, linear_mps: float, angular_rps: float, dt: float) -> None:
        turn_blend = min(abs(angular_rps) / 1.0, 1.0)
        turn_scale = self.odometry_left_scale if angular_rps > 0.0 else self.odometry_right_scale
        linear_scale = self.odometry_linear_scale + (turn_scale - self.odometry_linear_scale) * turn_blend
        linear_drift, yaw_drift = self._odometry_drift_factors()
        reported_speed = linear_mps * linear_scale * linear_drift
        reported_yaw_rate = angular_rps * self.odometry_yaw_scale * yaw_drift
        self.odometry_yaw_rad += reported_yaw_rate * dt
        self.odometry_x_m += math.cos(self.odometry_yaw_rad) * reported_speed * dt
        self.odometry_y_m += math.sin(self.odometry_yaw_rad) * reported_speed * dt

    def _odometry_drift_factors(self) -> tuple[float, float]:
        phase = math.tau * (time.monotonic() - self.last_time_origin) / self.odometry_drift_period_s
        return (
            1.0 + self.odometry_linear_drift_amplitude * math.sin(phase),
            1.0 + self.odometry_yaw_drift_amplitude * math.sin(phase * 0.73 + 0.8),
        )

    def _pose_is_free(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        return self._collision_at(x_m, y_m, yaw_rad) is None

    def _collision_at(self, x_m: float, y_m: float, yaw_rad: float) -> str | None:
        footprint = self._footprint_corners(x_m, y_m, yaw_rad)
        if any(not (-1.5 <= px <= 1.5 and -1.5 <= py <= 1.5) for px, py in footprint):
            return "outer_wall"
        for x0, y0, x1, y1 in self.block_rects:
            if any(x0 <= px <= x1 and y0 <= py <= y1 for px, py in footprint):
                return f"rect=({x0:+.3f},{y0:+.3f},{x1:+.3f},{y1:+.3f})"
            rect_corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
            if any(self._point_in_robot(rx, ry, x_m, y_m, yaw_rad) for rx, ry in rect_corners):
                return f"rect=({x0:+.3f},{y0:+.3f},{x1:+.3f},{y1:+.3f})"
        return None

    def _footprint_corners(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> tuple[tuple[float, float], ...]:
        half_l = self.robot_length_m * 0.5
        half_w = self.robot_width_m * 0.5
        local = ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))
        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        return tuple((x_m + lx * cy - ly * sy, y_m + lx * sy + ly * cy) for lx, ly in local)

    def _point_in_robot(
        self,
        px: float,
        py: float,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> bool:
        dx = px - robot_x
        dy = py - robot_y
        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
        return abs(local_x) <= self.robot_length_m * 0.5 and abs(local_y) <= self.robot_width_m * 0.5

    def _make_scan(self) -> LaserScan:
        scan_stamp = self.get_clock().now().to_msg()
        robot_x = self.robot.x_m
        robot_y = self.robot.y_m
        robot_yaw = self.robot.yaw_rad
        angle_min = -math.pi
        angle_max = math.pi
        count = 361
        angle_increment = (angle_max - angle_min) / (count - 1)
        elapsed = time.monotonic() - self.last_time_origin
        lidar_drift = 1.0 + self.lidar_range_drift_amplitude * math.sin(math.tau * elapsed / 53.0 + 1.1)
        angles = np.linspace(angle_min, angle_max, count, dtype=np.float64)
        wall_ranges = self._raycast_ranges(
            angles, self._segment_array, robot_x, robot_y, robot_yaw,
        )
        ghost_ranges = self._raycast_ranges(
            angles, self._ghost_segment_array, robot_x, robot_y, robot_yaw,
        )
        ranges: list[float] = []
        for angle, wall_dist, ghost_dist in zip(angles, wall_ranges, ghost_ranges):
            if self._lidar_ghost_angle(angle):
                dist = ghost_dist if ghost_dist > wall_dist + 0.04 else math.inf
            else:
                dist = wall_dist if self._lidar_angle_visible(angle) else math.inf
            if math.isfinite(dist):
                dist *= self.lidar_range_scale * lidar_drift
                if random.random() < clamp(self.random_projection_ratio, 0.0, 1.0):
                    dist = random.uniform(0.05, self.max_range_m)
                else:
                    dist += random.gauss(0.0, self.noise_std_m)
                dist = clamp(dist, 0.05, self.max_range_m)
            ranges.append(float(dist))

        msg = LaserScan()
        msg.header.stamp = scan_stamp
        msg.header.frame_id = "sim_lidar"
        msg.angle_min = float(angle_min); msg.angle_max = float(angle_max); msg.angle_increment = float(angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / max(self.lidar_rate_hz, 1.0)
        msg.range_min = 0.05; msg.range_max = float(self.max_range_m)
        msg.ranges = ranges
        return msg

    def _make_camera_frame(self) -> tuple[Image, Image, CameraInfo]:
        width, height = 640, 480
        fx, fy = 607.97784, 608.07367
        cx, cy = 314.42389, 248.78932
        camera_forward_m = 0.163
        camera_height_m = 0.0778 + 0.0323
        camera_pitch_rad = math.radians(4.25)
        near_m, far_m = 0.10, 2.50
        color, depth = self._raycast_camera_frame(
            width, height, fx, fy, cx, cy, camera_forward_m,
            camera_height_m, camera_pitch_rad, near_m, far_m,
        )
        max_shift = max(0, self.camera_depth_misalignment_max_px)
        if max_shift and self._camera_rng.random() < clamp(
            self.camera_depth_misalignment_ratio, 0.0, 1.0,
        ):
            dx = self._camera_rng.randint(-max_shift, max_shift)
            dy = self._camera_rng.randint(-max_shift, max_shift)
            if dx == 0 and dy == 0:
                dx = max_shift
            depth = self._shift_depth(depth, dx, dy)
        depth_mm = np.where(
            np.isfinite(depth),
            np.clip(depth * 1000.0, 0.0, float(np.iinfo(np.uint16).max)),
            0.0,
        ).astype(np.uint16)

        stamp = self.get_clock().now().to_msg()
        color_msg = Image()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "camera_link"
        color_msg.height = height
        color_msg.width = width
        color_msg.encoding = "bgr8"
        color_msg.is_bigendian = 0
        color_msg.step = width * 3
        color_msg.data = array("B", color.tobytes())

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_link"
        depth_msg.height = height
        depth_msg.width = width
        depth_msg.encoding = "16UC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = width * 2
        depth_msg.data = array("B", depth_mm.tobytes())

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "camera_link"
        info.height = height
        info.width = width
        info.distortion_model = "plumb_bob"
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return color_msg, depth_msg, info

    @staticmethod
    def _shift_depth(depth: np.ndarray, dx: int, dy: int) -> np.ndarray:
        shifted = np.full(depth.shape, np.inf, dtype=depth.dtype)
        height, width = depth.shape
        source_x0, source_x1 = max(0, -dx), min(width, width - dx)
        source_y0, source_y1 = max(0, -dy), min(height, height - dy)
        if source_x0 >= source_x1 or source_y0 >= source_y1:
            return shifted
        shifted[
            source_y0 + dy:source_y1 + dy,
            source_x0 + dx:source_x1 + dx,
        ] = depth[source_y0:source_y1, source_x0:source_x1]
        return shifted

    def _camera_wall_boxes(self) -> tuple[
        tuple[tuple[float, float, float, float], tuple[int, int, int]], ...
    ]:
        half_thickness_m = 0.01
        boxes = []
        for (x0, y0), (x1, y1) in self.camera_wall_segments:
            boxes.append((
                (
                    min(x0, x1) - half_thickness_m,
                    min(y0, y1) - half_thickness_m,
                    max(x0, x1) + half_thickness_m,
                    max(y0, y1) + half_thickness_m,
                ),
                (40, 40, 40),
            ))
        return tuple(boxes)

    def _raycast_camera_frame(
        self,
        width: int,
        height: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        camera_forward_m: float,
        camera_height_m: float,
        pitch_rad: float,
        near_m: float,
        far_m: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        scale = 8
        render_width, render_height = width // scale, height // scale
        ray_key = (render_width, render_height, fx, fy, cx, cy, pitch_rad)
        ray_basis = self._camera_ray_cache.get(ray_key)
        if ray_basis is None:
            cols, rows = np.meshgrid(np.arange(render_width), np.arange(render_height))
            ray_right = (cols * scale + 0.5 * scale - cx) / fx
            ray_down = (rows * scale + 0.5 * scale - cy) / fy
            sin_pitch, cos_pitch = math.sin(pitch_rad), math.cos(pitch_rad)
            local_forward = cos_pitch + ray_down * sin_pitch
            local_up = -sin_pitch - ray_down * cos_pitch
            ray_basis = (local_forward, ray_right, local_up)
            self._camera_ray_cache[ray_key] = ray_basis
        local_forward, ray_right, ray_z = ray_basis
        cyaw, syaw = math.cos(self.robot.yaw_rad), math.sin(self.robot.yaw_rad)
        ray_x = local_forward * cyaw + ray_right * syaw
        ray_y = local_forward * syaw - ray_right * cyaw
        origin = np.asarray((
            self.robot.x_m + cyaw * camera_forward_m,
            self.robot.y_m + syaw * camera_forward_m,
            camera_height_m,
        ))
        depth = np.full((render_height, render_width), np.inf, dtype=np.float32)
        color = np.full((render_height, render_width, 3), (218, 218, 218), dtype=np.uint8)

        ground_t = np.divide(-origin[2], ray_z, out=np.full_like(ray_z, np.inf), where=ray_z < -1.0e-6)
        ground = (ground_t >= near_m) & (ground_t <= far_m)
        depth[ground] = ground_t[ground]
        color[ground] = (242, 242, 242)

        directions = (ray_x, ray_y, ray_z)
        for rect, object_height_m, bgr in self._camera_geometry:
            bounds = ((rect[0], rect[2]), (rect[1], rect[3]), (0.0, object_height_m))
            t_near = np.full(depth.shape, -np.inf)
            t_far = np.full(depth.shape, np.inf)
            for axis, (low, high) in enumerate(bounds):
                direction = directions[axis]
                with np.errstate(divide="ignore", invalid="ignore"):
                    first = (low - origin[axis]) / direction
                    second = (high - origin[axis]) / direction
                t_near = np.maximum(t_near, np.minimum(first, second))
                t_far = np.minimum(t_far, np.maximum(first, second))
            visible = (
                (t_far >= np.maximum(t_near, near_m))
                & (t_near <= far_m)
                & (t_near < depth)
            )
            depth[visible] = t_near[visible]
            color[visible] = bgr

        return (
            cv2.resize(color, (width, height), interpolation=cv2.INTER_NEAREST),
            cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST),
        )

    def _raycast_ranges(
        self,
        angles: np.ndarray,
        segments: np.ndarray,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> np.ndarray:
        if not len(segments):
            return np.full(angles.shape, np.inf, dtype=np.float64)
        world_angles = angles + robot_yaw
        directions = np.stack((np.cos(world_angles), np.sin(world_angles)), axis=1)
        starts = segments[:, 0, :]
        vectors = segments[:, 1, :] - starts
        relative = starts - np.asarray((robot_x, robot_y), dtype=np.float64)
        denominator = (
            directions[:, None, 0] * vectors[None, :, 1]
            - directions[:, None, 1] * vectors[None, :, 0]
        )
        valid_denominator = np.abs(denominator) > 1.0e-9
        safe_denominator = np.where(valid_denominator, denominator, 1.0)
        distance = (
            relative[None, :, 0] * vectors[None, :, 1]
            - relative[None, :, 1] * vectors[None, :, 0]
        ) / safe_denominator
        along_segment = (
            relative[None, :, 0] * directions[:, None, 1]
            - relative[None, :, 1] * directions[:, None, 0]
        ) / safe_denominator
        valid = (
            valid_denominator
            & (distance > 0.0)
            & (along_segment >= 0.0)
            & (along_segment <= 1.0)
            & (distance <= self.max_range_m)
        )
        return np.min(np.where(valid, distance, np.inf), axis=1)

    def _lidar_ghost_angle(self, local_angle_rad: float) -> bool:
        if not self.lidar_ghost_enabled:
            return False
        angle_deg = (math.degrees(local_angle_rad) + 360.0) % 360.0
        half_width = max(0.0, self.lidar_ghost_band_width_deg)
        return any(
            abs(((angle_deg - center + 180.0) % 360.0) - 180.0) <= half_width
            for center in (45.0, 135.0, 225.0, 315.0)
        )

    def _lidar_angle_visible(self, local_angle_rad: float) -> bool:
        cardinal_half_width = max(0.0, self.lidar_cardinal_good_width_deg)
        angle_deg = (math.degrees(local_angle_rad) + 360.0) % 360.0
        nearest_cardinal = round(angle_deg / 90.0) * 90.0
        cardinal_error = abs(((angle_deg - nearest_cardinal + 180.0) % 360.0) - 180.0)
        if cardinal_error <= cardinal_half_width:
            return True

        keep_ratio = clamp(self.lidar_diagonal_keep_ratio, 0.0, 1.0)
        if keep_ratio >= 1.0:
            return True
        if keep_ratio <= 0.0:
            return False
        period_deg = max(self.lidar_diagonal_hash_period_deg, 0.5)
        bucket = int(math.floor(angle_deg / period_deg))
        hashed = (bucket * 1103515245 + 12345) & 0x7FFFFFFF
        return (hashed % 1000) < int(round(keep_ratio * 1000.0))

    def _make_odometry(self, dt: float) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.odometry_x_m
        msg.pose.pose.position.y = self.odometry_y_m
        half = self.odometry_yaw_rad * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        turn_blend = min(abs(self.odometry_angular_rps) / 1.0, 1.0)
        turn_scale = self.odometry_left_scale if self.odometry_angular_rps > 0.0 else self.odometry_right_scale
        linear_scale = self.odometry_linear_scale + (turn_scale - self.odometry_linear_scale) * turn_blend
        linear_drift, yaw_drift = self._odometry_drift_factors()
        msg.twist.twist.linear.x = self.odometry_linear_mps * linear_scale * linear_drift
        msg.twist.twist.angular.z = self.odometry_angular_rps * self.odometry_yaw_scale * yaw_drift
        covariance = 0.0025 if dt > 0.0 else 0.01
        msg.pose.covariance[0] = covariance
        msg.pose.covariance[7] = covariance
        msg.pose.covariance[35] = math.radians(1.0) ** 2
        return msg

    def _make_imu(self) -> Imu:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sim_imu"
        reported_yaw = self.initial_yaw_rad + (self.robot.yaw_rad - self.initial_yaw_rad) * self.imu_yaw_scale
        half = reported_yaw * 0.5
        msg.orientation.z = math.sin(half)
        msg.orientation.w = math.cos(half)
        msg.orientation_covariance[0] = 0.02
        msg.angular_velocity.z = self.odometry_angular_rps
        msg.linear_acceleration.z = -1.0
        return msg


def main() -> None:
    rclpy.init()
    node = PlannerIoSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.stop_workers()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
