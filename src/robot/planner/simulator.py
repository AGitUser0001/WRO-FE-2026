#!/usr/bin/env python3
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import cast

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Int32

from .simulator_view import SimulatorViewMixin

Segment = tuple[tuple[float, float], tuple[float, float]]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class RobotState:
    x_m: float = 1.0
    y_m: float = 0.0
    yaw_rad: float = math.pi / 2.0
    motor: int = 0
    servo: int = 0


class PlannerIoSimulator(SimulatorViewMixin, Node):
    def __init__(self) -> None:
        super().__init__("wro_planner_io_simulator")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu_data")
        self.declare_parameter("motor_topic", "/microROS/motor_control")
        self.declare_parameter("servo_topic", "/microROS/servo_control")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("max_range_m", 2.2)
        self.declare_parameter("noise_std_m", 0.01); self.declare_parameter("random_projection_ratio", 0.05)
        self.declare_parameter("lidar_cardinal_good_width_deg", 12.0)
        self.declare_parameter("lidar_diagonal_keep_ratio", 0.35)
        self.declare_parameter("lidar_diagonal_hash_period_deg", 7.0)
        self.declare_parameter("motor_speed_mps_per_unit", 0.0023333333333333335)
        self.declare_parameter("command_timeout_s", 0.4)
        self.declare_parameter("max_servo", 320)
        self.declare_parameter("wheelbase_m", 0.138)
        self.declare_parameter("max_steering_angle_rad", 0.4188)
        self.declare_parameter("robot_length_m", 0.25)
        self.declare_parameter("robot_width_m", 0.15)
        self.declare_parameter("initial_x_m", 1.0)
        self.declare_parameter("initial_y_m", 0.0)
        self.declare_parameter("initial_yaw_rad", math.pi / 2.0)
        self.declare_parameter("obstacle_block_size_m", 0.05)
        self.declare_parameter("view_enabled", True)

        self.scan_topic = self._str_param("scan_topic")
        self.imu_topic = self._str_param("imu_topic")
        self.motor_topic = self._str_param("motor_topic")
        self.servo_topic = self._str_param("servo_topic")
        self.rate_hz = self._float_param("rate_hz")
        self.max_range_m = self._float_param("max_range_m")
        self.noise_std_m = self._float_param("noise_std_m"); self.random_projection_ratio = self._float_param("random_projection_ratio")
        self.lidar_cardinal_good_width_deg = self._float_param("lidar_cardinal_good_width_deg")
        self.lidar_diagonal_keep_ratio = self._float_param("lidar_diagonal_keep_ratio")
        self.lidar_diagonal_hash_period_deg = self._float_param("lidar_diagonal_hash_period_deg")
        self.motor_speed_mps_per_unit = self._float_param("motor_speed_mps_per_unit")
        self.command_timeout_s = self._float_param("command_timeout_s")
        self.max_servo = self._int_param("max_servo")
        self.wheelbase_m = self._float_param("wheelbase_m")
        self.max_steering_angle_rad = self._float_param("max_steering_angle_rad")
        self.robot_length_m = self._float_param("robot_length_m")
        self.robot_width_m = self._float_param("robot_width_m")
        self.initial_x_m = self._float_param("initial_x_m")
        self.initial_y_m = self._float_param("initial_y_m")
        self.initial_yaw_rad = self._float_param("initial_yaw_rad")
        self.obstacle_block_size_m = self._float_param("obstacle_block_size_m")
        self.view_enabled = bool(self.get_parameter("view_enabled").value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, qos)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, qos)
        self.pose_pub = self.create_publisher(Float32MultiArray, "/wro_sim/pose", qos)
        self.create_subscription(Int32, self.motor_topic, self._motor_cb, qos)
        self.create_subscription(Int32, self.servo_topic, self._servo_cb, qos)

        self.start_x_m = random.choice((-1.0, 1.0)) * abs(self.initial_x_m)
        self.start_y_m = self.initial_y_m
        self.robot = self._initial_robot()
        self.last_time = time.monotonic()
        self.last_motor_command_time = self.last_time
        self.dragging = False
        self.drag_aim: tuple[float, float] | None = None
        self.window = "wro planner io simulator"
        self.world_extent_m = 2.2
        self.canvas_px = 1280
        self.ui_scale = self.canvas_px / 780.0
        self.block_rects = ((-0.5, -0.5, 0.5, 0.5), *self._obstacle_rects())
        self.segments = self._build_world()

        if self.view_enabled:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, self.canvas_px, self.canvas_px)
            cv2.setMouseCallback(self.window, self._mouse_cb)
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._tick)

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(cast(float, self.get_parameter(name).value))

    def _int_param(self, name: str) -> int:
        return int(cast(int, self.get_parameter(name).value))

    def _motor_cb(self, msg: Int32) -> None:
        self.robot.motor = int(msg.data)
        self.last_motor_command_time = time.monotonic()

    def _servo_cb(self, msg: Int32) -> None:
        self.robot.servo = int(clamp(int(msg.data), -self.max_servo, self.max_servo))

    def _build_world(self) -> tuple[Segment, ...]:
        segments: list[Segment] = []
        self._add_rect(segments, -1.5, -1.5, 1.5, 1.5)
        for rect in self.block_rects:
            self._add_rect(segments, *rect)
        return tuple(segments)

    @staticmethod
    def _add_rect(segments: list[Segment], x0: float, y0: float, x1: float, y1: float) -> None:
        segments.extend((((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))))

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(now - self.last_time, 0.1)
        self.last_time = now
        self._apply_command_watchdog(now)
        if not self.dragging:
            self._advance_robot(dt)
        self.pose_pub.publish(Float32MultiArray(data=[self.robot.x_m - self.start_x_m, self.robot.y_m - self.start_y_m, self.robot.yaw_rad]))
        self.imu_pub.publish(self._make_imu())
        self.scan_pub.publish(self._make_scan())
        key = self._draw() if self.view_enabled else -1
        if key in (ord("r"), ord("R")):
            self.robot = self._initial_robot()
            self.last_motor_command_time = now
        elif key in (ord("q"), ord("Q"), 27):
            rclpy.shutdown()

    def _apply_command_watchdog(self, now: float) -> None:
        if now - self.last_motor_command_time > self.command_timeout_s:
            self.robot.motor = 0

    def _initial_robot(self) -> RobotState:
        return RobotState(x_m=self.start_x_m, y_m=self.start_y_m, yaw_rad=self.initial_yaw_rad)

    def _obstacle_rects(self) -> tuple[tuple[float, float, float, float], ...]:
        half = max(0.02, self.obstacle_block_size_m) * 0.5
        centers = [point for side in self._side_slots() for point in self._side_obstacle_centers(side)]
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
        return random.choice(side[0:2]), random.choice(side[4:6])

    def _advance_robot(self, dt: float) -> None:
        motor = clamp(self.robot.motor, -255, 255)
        speed = motor * self.motor_speed_mps_per_unit
        steering_angle = (
            -clamp(self.robot.servo, -self.max_servo, self.max_servo)
            / max(self.max_servo, 1)
            * self.max_steering_angle_rad
        )
        yaw_rate = speed / max(self.wheelbase_m, 0.01) * math.tan(steering_angle)
        next_yaw = self.robot.yaw_rad + yaw_rate * dt
        next_x = self.robot.x_m + math.cos(next_yaw) * speed * dt
        next_y = self.robot.y_m + math.sin(next_yaw) * speed * dt
        if self._pose_is_free(next_x, next_y, next_yaw):
            self.robot.x_m = next_x
            self.robot.y_m = next_y
            self.robot.yaw_rad = next_yaw

    def _pose_is_free(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        footprint = self._footprint_corners(x_m, y_m, yaw_rad)
        if any(not (-1.5 <= px <= 1.5 and -1.5 <= py <= 1.5) for px, py in footprint):
            return False
        for x0, y0, x1, y1 in self.block_rects:
            if any(x0 <= px <= x1 and y0 <= py <= y1 for px, py in footprint):
                return False
            rect_corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
            if any(self._point_in_robot(rx, ry, x_m, y_m, yaw_rad) for rx, ry in rect_corners):
                return False
        return True

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
        angle_min = -math.pi
        angle_max = math.pi
        count = 361
        angle_increment = (angle_max - angle_min) / (count - 1)
        ranges: list[float] = []
        for i in range(count):
            angle = angle_min + i * angle_increment
            dist = self._raycast(angle) if self._lidar_angle_visible(angle) else math.inf
            if math.isfinite(dist):
                if random.random() < clamp(self.random_projection_ratio, 0.0, 1.0):
                    dist = random.uniform(0.05, self.max_range_m)
                else:
                    dist += random.gauss(0.0, self.noise_std_m)
                dist = clamp(dist, 0.05, self.max_range_m)
            ranges.append(float(dist))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sim_lidar"
        msg.angle_min = float(angle_min); msg.angle_max = float(angle_max); msg.angle_increment = float(angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / max(self.rate_hz, 1.0)
        msg.range_min = 0.05; msg.range_max = float(self.max_range_m)
        msg.ranges = ranges
        return msg

    def _raycast(self, local_angle_rad: float) -> float:
        forward = (math.cos(self.robot.yaw_rad), math.sin(self.robot.yaw_rad))
        left = (-math.sin(self.robot.yaw_rad), math.cos(self.robot.yaw_rad))
        dx = forward[0] * math.cos(local_angle_rad) + left[0] * math.sin(local_angle_rad)
        dy = forward[1] * math.cos(local_angle_rad) + left[1] * math.sin(local_angle_rad)
        best = math.inf
        for segment in self.segments:
            hit = self._ray_segment_intersection(
                (self.robot.x_m, self.robot.y_m),
                (dx, dy),
                segment,
            )
            if hit is not None and 0.0 < hit < best:
                best = hit
        return best if best <= self.max_range_m else math.inf

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

    @staticmethod
    def _ray_segment_intersection(
        origin: tuple[float, float],
        direction: tuple[float, float],
        segment: Segment,
    ) -> float | None:
        ox, oy = origin
        dx, dy = direction
        (x1, y1), (x2, y2) = segment
        sx = x2 - x1
        sy = y2 - y1
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-9:
            return None
        qx = x1 - ox
        qy = y1 - oy
        t = (qx * sy - qy * sx) / denom
        u = (qx * dy - qy * dx) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def _make_imu(self) -> Imu:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sim_imu"
        half = self.robot.yaw_rad * 0.5
        msg.orientation.z = math.sin(half)
        msg.orientation.w = math.cos(half)
        msg.orientation_covariance[0] = 0.02
        msg.angular_velocity.z = 0.0
        msg.linear_acceleration.z = -1.0
        return msg


def main() -> None:
    rclpy.init()
    node = PlannerIoSimulator()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
