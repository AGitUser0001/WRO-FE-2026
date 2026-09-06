import math
import queue
import threading
import time
from collections import deque

import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Int32

from .config import DriverConfig
from .camera_color import ColorObstacle, color_obstacles, depth_image_m
from .debug import DebugFrame, DebugWindow
from .grid import Cell, Direction
from .localize_types import PoseEstimate
from .odometry_localize import OdometryLocalizer
from .graph_path import active_obstacle_component_mask, tracked_obstacle_overlap
from .obstacle_policy import ACTIVE_OBSTACLE_CLUSTER_RADIUS_M
from .planner import GridPlanner, Plan
from .sensors import SensorFrame, local_grid_from_scan, yaw_from_quat


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def sampled_path(points: tuple[tuple[float, float], ...], count: int = 6) -> str:
    if not points:
        return "[]"
    count = max(2, min(count, len(points)))
    last = len(points) - 1
    indices = (round(last * i / (count - 1)) for i in range(count))
    return "[" + ",".join(
        f"{points[i][0]:+.2f}:{points[i][1]:+.2f}" for i in indices
    ) + "]"


class PoseDebugLogger:
    def __init__(self) -> None:
        self.queue: queue.Queue[str] = queue.Queue(maxsize=8)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def write(self, line: str) -> None:
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(line)
            except queue.Full:
                pass

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=0.5)

    def _loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                line = self.queue.get(timeout=0.05)
            except queue.Empty:
                continue
            print(line, flush=True)


class Driver(Node):
    def __init__(self):
        super().__init__("wro_planner")
        self.config: DriverConfig | None = None
        self.localizer: OdometryLocalizer | None = None
        self.planner: GridPlanner | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_scan_received_at = 0.0
        self.latest_scan_sample_time = 0.0
        self.latest_scan_stamp = 0.0
        self.last_processed_scan_stamp = -math.inf
        self.latest_scan_sim_pose: tuple[float, float, float] | None = None
        self.latest_imu_yaw: float | None = None
        self.last_accepted_imu: tuple[float, float] | None = None
        self.latest_odometry: tuple[float, float, float, float] | None = None
        self.latest_odometry_speed_mps = 0.0
        self.imu_history: deque[tuple[float, float]] = deque(maxlen=64)
        self.odometry_history: deque[
            tuple[float, tuple[float, float, float, float]]
        ] = deque(maxlen=64)
        self.latest_frame: SensorFrame | None = None
        self.latest_plan: Plan | None = None
        self.latest_sim_pose: tuple[float, float, float] | None = None
        self.latest_sim_speed_mps = 0.0
        self.latest_camera_depth: tuple[np.ndarray, float] | None = None
        self.latest_camera_color: tuple[Image, float] | None = None
        self.last_camera_pair_stamp = -math.inf
        self.latest_camera_observation_time = 0.0
        self.latest_color_obstacles: tuple[tuple[ColorObstacle, ...], float] | None = None
        self.last_color_observation_time = 0.0
        self.debug_window: DebugWindow | None = None
        self.last_debug_render_ms = 0.0
        self.last_pose_debug_ms = 0.0
        self.state_lock = threading.Lock()
        self.debug_lock = threading.Lock()
        self.debug_frame: DebugFrame | None = None
        self.debug_thread: threading.Thread | None = None
        self.debug_stop = threading.Event()
        self.work_event = threading.Event()
        self.work_stop = threading.Event()
        self.work_thread: threading.Thread | None = None
        self.camera_event = threading.Event()
        self.camera_stop = threading.Event()
        self.camera_thread: threading.Thread | None = None
        self.work_busy = False
        self.last_motion_stamp: float | None = None
        self.last_prediction_blocked = False
        self.last_prediction_block_time_s = math.inf
        self.last_prediction_block_reason = "none"
        self.last_prediction_block_point: tuple[float, float] | None = None
        self.last_prediction_escape_cells = 0
        self.last_motor_command = 0
        self.last_motor_output = 0
        self.last_sensor_wait_reason = ""
        self.last_sensor_wait_log_time = -math.inf
        self.command_history: list[tuple[float, int, int]] = []
        self.pose_debug_logger: PoseDebugLogger | None = None
        self.motor_pub = None
        self.servo_pub = None

    def configure(
        self,
        config: DriverConfig,
        localizer: OdometryLocalizer,
        planner: GridPlanner,
        debug_window: DebugWindow | None = None,
    ) -> None:
        self.config = config
        self.localizer = localizer
        self.planner = planner
        self.debug_window = debug_window
        if self.debug_window is not None and config.debug_view_enabled:
            self.debug_thread = threading.Thread(target=self._debug_loop, daemon=True)
            self.debug_thread.start()
        if config.pose_debug:
            self.pose_debug_logger = PoseDebugLogger()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.motor_pub = self.create_publisher(Int32, config.motor_topic, qos)
        self.servo_pub = self.create_publisher(Int32, config.servo_topic, qos)
        self.work_thread = threading.Thread(target=self._work_loop, daemon=True)
        self.work_thread.start()
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
        self.create_subscription(
            LaserScan,
            config.scan_topic,
            self._scan_cb,
            qos,
        )
        self.create_subscription(Odometry, config.odom_topic, self._odom_cb, qos)
        self.create_subscription(Imu, config.imu_topic, self._imu_cb, qos)
        self.create_subscription(Float32MultiArray, "/wro_sim/pose", self._sim_pose_cb, qos)
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, config.camera_depth_topic, self._camera_depth_cb, camera_qos)
        self.create_subscription(Image, config.camera_image_topic, self._camera_color_cb, camera_qos)
        self.create_timer(1.0 / max(config.tick_rate, 1.0), self._tick)

    def _scan_cb(self, msg: LaserScan) -> None:
        with self.state_lock:
            self.latest_scan = msg
            self.latest_scan_received_at = time.monotonic()
            self.latest_scan_sim_pose = self.latest_sim_pose
            self.latest_scan_stamp = self._header_stamp(msg)
            stamp_valid = msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0
            transport_age = max(
                0.0, self.get_clock().now().nanoseconds / 1e9 - self.latest_scan_stamp,
            ) if stamp_valid else 0.0
            self.latest_scan_sample_time = self.latest_scan_received_at - transport_age

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        if yaw is None or not math.isfinite(yaw):
            return
        stamp = self._header_stamp(msg)
        with self.state_lock:
            previous = self.last_accepted_imu
            if previous is not None:
                dt_s = stamp - previous[0]
                if dt_s <= 0.0:
                    return
                yaw_delta = abs(math.atan2(
                    math.sin(yaw - previous[1]),
                    math.cos(yaw - previous[1]),
                ))
                if yaw_delta > math.radians(5.0) + 3.0 * dt_s:
                    return
            self.last_accepted_imu = (stamp, yaw)
            self.latest_imu_yaw = yaw
            self.imu_history.append((stamp, yaw))
            scan_pending = self.latest_scan_stamp > self.last_processed_scan_stamp
        if scan_pending:
            self.work_event.set()

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        odometry = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw,
                float(msg.twist.twist.linear.x),
            )
        with self.state_lock:
            self.latest_odometry = odometry
            self.latest_odometry_speed_mps = odometry[3]
            self.odometry_history.append((self._header_stamp(msg), odometry))
            scan_pending = self.latest_scan_stamp > self.last_processed_scan_stamp
        if scan_pending:
            self.work_event.set()

    def _sim_pose_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            with self.state_lock:
                self.latest_sim_pose = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
                if len(msg.data) >= 4:
                    self.latest_sim_speed_mps = float(msg.data[3])

    def _camera_depth_cb(self, msg: Image) -> None:
        depth = depth_image_m(msg)
        if depth is not None:
            with self.state_lock:
                self.latest_camera_depth = (depth.copy(), self._message_stamp(msg))
            self.camera_event.set()

    def _camera_color_cb(self, msg: Image) -> None:
        with self.state_lock:
            self.latest_camera_color = (msg, self._message_stamp(msg))
        self.camera_event.set()

    def _camera_loop(self) -> None:
        while not self.camera_stop.is_set():
            if not self.camera_event.wait(0.05):
                continue
            self.camera_event.clear()
            if not self.camera_stop.is_set():
                self._process_camera_pair()

    def _process_camera_pair(self) -> None:
        with self.state_lock:
            depth_frame = self.latest_camera_depth
            color_frame = self.latest_camera_color
            if depth_frame is None or color_frame is None:
                return
            depth, depth_stamp = depth_frame
            color, color_stamp = color_frame
            pair_stamp = max(depth_stamp, color_stamp)
            if abs(depth_stamp - color_stamp) > 0.05 or pair_stamp <= self.last_camera_pair_stamp:
                return
            self.last_camera_pair_stamp = pair_stamp
        obstacles = color_obstacles(color, depth)
        observed_at = time.monotonic()
        with self.state_lock:
            self.latest_camera_observation_time = observed_at
            self.latest_color_obstacles = (obstacles, observed_at)
        if obstacles:
            self.work_event.set()

    @staticmethod
    def _header_stamp(msg: Image | Imu | LaserScan | Odometry) -> float:
        stamp = msg.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return value if value > 0.0 else time.monotonic()

    @staticmethod
    def _nearest_sample(history: deque, stamp: float):
        if not history:
            return None
        nearest = min(history, key=lambda sample: abs(sample[0] - stamp))
        return nearest[1] if abs(nearest[0] - stamp) <= 0.20 else None

    @staticmethod
    def _interpolated_yaw(history: deque[tuple[float, float]], stamp: float) -> float | None:
        if not history:
            return None
        if history[-1][0] < stamp:
            return None
        before = history[0]
        for after in history:
            if after[0] >= stamp:
                if after[0] <= before[0]:
                    return after[1]
                fraction = (stamp - before[0]) / (after[0] - before[0])
                delta = math.atan2(
                    math.sin(after[1] - before[1]),
                    math.cos(after[1] - before[1]),
                )
                yaw = before[1] + fraction * delta
                return math.atan2(math.sin(yaw), math.cos(yaw))
            before = after
        return None

    @staticmethod
    def _message_stamp(msg: Image) -> float:
        return Driver._header_stamp(msg)

    def _tick(self) -> None:
        if self.work_busy:
            return
        self.work_event.set()

    def _work_loop(self) -> None:
        while not self.work_stop.is_set():
            if not self.work_event.wait(0.05):
                continue
            self.work_event.clear()
            if self.work_stop.is_set():
                break
            self.work_busy = True
            try:
                self._process_tick()
            finally:
                self.work_busy = False

    def _process_tick(self) -> None:
        tick_start = time.perf_counter()
        now = time.monotonic()
        if self.config is None or self.localizer is None or self.planner is None:
            return
        with self.state_lock:
            scan = self.latest_scan
            sim_pose = self.latest_scan_sim_pose
            scan_stamp = self.latest_scan_stamp
            odometry = self._nearest_sample(self.odometry_history, scan_stamp)
            imu_yaw = self._interpolated_yaw(self.imu_history, scan_stamp)
            imu_stale = not self.imu_history or scan_stamp - self.imu_history[-1][0] > 0.20
            scan_received_at = self.latest_scan_received_at
            scan_sample_time = self.latest_scan_sample_time
            color_obstacles_frame = self.latest_color_obstacles
        if scan is None or now - scan_received_at > 0.30:
            self._log_sensor_wait("scan")
            self._command(0, 0)
            return
        if odometry is None:
            self._log_sensor_wait("odometry")
            self._command(0, 0)
            return
        if imu_yaw is None:
            self._log_sensor_wait("imu")
            if imu_stale:
                self._command(0, 0)
            return
        with self.state_lock:
            if scan_stamp <= self.last_processed_scan_stamp:
                return
            self.last_processed_scan_stamp = scan_stamp
        self.localizer.apply_imu_yaw(imu_yaw)
        self._apply_motion_update(scan_stamp, odometry)

        scan_start = time.perf_counter()
        frame = local_grid_from_scan(
            scan,
            size_m=self.config.local_grid_size_m,
            resolution_m=self.localizer.grid.resolution_m,
            max_range_m=self.config.lidar_max_range_m,
        )
        scan_ms = (time.perf_counter() - scan_start) * 1000.0
        with self.state_lock:
            self.latest_frame = frame
        localize_start = time.perf_counter()
        pose = self.localizer.update_from_sensors(frame)
        localize_ms = (time.perf_counter() - localize_start) * 1000.0
        now = time.monotonic()
        fresh_color_obstacles: tuple[ColorObstacle, ...] = ()
        if (
            color_obstacles_frame is not None
            and color_obstacles_frame[1] > self.last_color_observation_time
            and now - color_obstacles_frame[1] <= self.config.color_obstacle_hold_s
        ):
            fresh_color_obstacles = color_obstacles_frame[0]
            self.last_color_observation_time = color_obstacles_frame[1]
        self.planner.expire_tracked_obstacles(now)
        motor = self.config.drive_motor if self.config.auto_drive_enabled else 0
        scan_age_s = max(0.0, now - scan_sample_time)
        self.last_scan_age_s = scan_age_s
        planning_pose, delay_prefix = self._predict_delayed_pose(
            pose, now, scan_age_s, frame.local_grid,
        )
        self.last_planning_pose = planning_pose
        plan_start = time.perf_counter()
        plan = self.planner.plan(
            pose,
            local_grid=frame.local_grid,
            planning_pose=planning_pose,
            delay_prefix=delay_prefix,
            rear_wall_distance_m=(frame.rear_wall.distance_m if frame.rear_wall.valid else math.inf),
            color_observations=fresh_color_obstacles,
            observation_time=now,
            direction_hint=self.localizer.pending_direction,
        )
        plan_ms = (time.perf_counter() - plan_start) * 1000.0
        with self.state_lock:
            self.latest_plan = plan

        if not self.config.auto_drive_enabled:
            self._command(0, 0)
        else:
            servo = self._servo_for_plan(plan, pose)
            prediction_blocks_command = (
                self.last_prediction_blocked
                and self.last_prediction_block_time_s
                <= self.config.command_delay_s + self.config.actuation_delay_s
            )
            commanded_motor = (
                0
                if self.planner.course_complete or prediction_blocks_command
                else round(abs(motor) * plan.speed_scale) * (1 if plan.motion_direction >= 0 else -1)
            )
            if self.planner.course_complete:
                servo = 0
            self._command(commanded_motor, servo)

        pose_debug_start = time.perf_counter()
        self._log_pose_debug(plan, sim_pose, scan_ms, localize_ms, plan_ms)
        self.last_pose_debug_ms = (time.perf_counter() - pose_debug_start) * 1000.0
        tick_ms = (time.perf_counter() - tick_start) * 1000.0
        self._queue_debug_frame(
            frame,
            plan,
            scan_ms,
            localize_ms,
            plan_ms,
            tick_ms,
        )

    def _log_sensor_wait(self, reason: str) -> None:
        if self.pose_debug_logger is None:
            return
        now = time.monotonic()
        if reason == self.last_sensor_wait_reason and now - self.last_sensor_wait_log_time < 1.0:
            return
        self.last_sensor_wait_reason = reason
        self.last_sensor_wait_log_time = now
        self.pose_debug_logger.write(f"POSE_DEBUG waiting={reason}")

    def _servo_for_plan(self, plan: Plan, pose: PoseEstimate) -> int:
        if self.config is None:
            return 0
        max_servo = self.config.max_servo
        max_angle = max(self.config.max_steering_angle_rad, 0.01)
        return int(clamp(-plan.steering_angle_rad / max_angle * max_servo, -max_servo, max_servo))

    def _command(self, motor: int, servo: int) -> None:
        if self.config is None or self.motor_pub is None or self.servo_pub is None:
            return
        if not self.context.ok():
            return
        max_servo = self.config.max_servo
        motor_output = self._motor_output(motor)
        try:
            self.motor_pub.publish(Int32(data=motor_output))
            clamped_servo = int(clamp(servo, -max_servo, max_servo))
            self.servo_pub.publish(Int32(data=clamped_servo))
        except Exception:
            return
        self.last_motor_command = int(motor)
        self.last_motor_output = motor_output
        now = time.monotonic()
        self.command_history.append((now, int(motor), clamped_servo))
        retain_after = now - max(self.config.command_delay_s * 2.0, 1.0)
        self.command_history = [command for command in self.command_history if command[0] >= retain_after]

    def _motor_output(self, motor: int) -> int:
        assert self.config is not None
        magnitude = abs(int(motor))
        if magnitude == 0:
            return 0
        reference = max(1, self.config.planning_motor_reference)
        deadband = min(max(0, self.config.motor_deadband_command), reference - 1)
        if magnitude >= reference or deadband == 0:
            output = magnitude
        else:
            output = deadband + math.ceil((reference - deadband) * magnitude / reference)
        return output if motor > 0 else -output

    def _motor_speed(self, motor: int) -> float:
        assert self.config is not None
        reference = max(1, self.config.planning_motor_reference)
        return motor * self.config.planning_speed_mps / reference

    def _predict_delayed_pose(
        self,
        pose: PoseEstimate,
        now: float,
        scan_age_s: float = 0.0,
        local_grid=None,
    ) -> tuple[PoseEstimate, tuple[tuple[float, float], ...]]:
        if self.config is None or self.config.command_delay_s <= 1.0e-4:
            return pose, ()
        pose_time = now - max(0.0, scan_age_s)
        end_time = now + self.config.command_delay_s + 0.08
        horizon = end_time - pose_time
        actuation_delay = max(0.0, self.config.actuation_delay_s)
        active_before = pose_time - actuation_delay
        active_motor = 0
        active_servo = 0
        events: list[tuple[float, int, int]] = []
        for sent_at, motor, servo in self.command_history:
            if sent_at <= active_before:
                active_motor, active_servo = motor, servo
            elif sent_at + actuation_delay <= end_time:
                events.append((sent_at + actuation_delay - pose_time, motor, servo))
        x_m, y_m, yaw = pose.x_m, pose.y_m, pose.yaw_rad
        speed_mps = pose.odometry_speed_mps
        prefix: list[tuple[float, float]] = [(x_m, y_m)]
        local_obstacles = (
            set(self.planner._local_obstacle_clusters(local_grid, pose))
            if self.planner is not None and local_grid is not None
            else set()
        )
        self.last_prediction_blocked = False
        self.last_prediction_block_time_s = math.inf
        self.last_prediction_block_reason = "none"
        self.last_prediction_block_point = None
        self.last_prediction_escape_cells = 0
        elapsed = 0.0
        for event_time, motor, servo in (*events, (horizon, active_motor, active_servo)):
            duration = max(0.0, event_time - elapsed)
            x_m, y_m, yaw, speed_mps = self._integrate_control(
                x_m, y_m, yaw, speed_mps, active_motor, active_servo, duration, prefix,
                pose, local_grid, local_obstacles, elapsed,
            )
            elapsed = event_time
            active_motor, active_servo = motor, servo
        if len(prefix) == 1:
            return pose, ()
        predicted_travel_m = sum(
            math.dist(start, end) for start, end in zip(prefix, prefix[1:])
        )
        predicted = PoseEstimate(
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw,
            confidence=pose.confidence,
            alignment_score=pose.alignment_score,
            odometry_motion_m=pose.odometry_motion_m,
            odometry_travel_m=pose.odometry_travel_m + predicted_travel_m,
            odometry_speed_mps=speed_mps,
        )
        return predicted, tuple(prefix)

    def _integrate_control(
        self,
        x_m: float,
        y_m: float,
        yaw: float,
        speed_mps: float,
        motor: int,
        servo: int,
        duration: float,
        prefix: list[tuple[float, float]],
        scan_pose: PoseEstimate,
        local_grid,
        local_obstacles: set[tuple[int, int]],
        elapsed_before_s: float,
    ) -> tuple[float, float, float, float]:
        assert self.config is not None
        target_speed = self._motor_speed(motor)
        if duration <= 1.0e-5:
            return x_m, y_m, yaw, speed_mps
        if abs(speed_mps) < 1.0e-5 and abs(target_speed) < 1.0e-5:
            return x_m, y_m, yaw, 0.0
        distance = max(abs(speed_mps), abs(target_speed)) * duration
        steps = max(1, int(math.ceil(distance / max(self.config.trajectory_step_m, 0.01))))
        dt = duration / steps
        steering = -servo / max(self.config.max_servo, 1) * self.config.max_steering_angle_rad
        for step in range(steps):
            accelerating = speed_mps * target_speed >= 0.0 and abs(target_speed) > abs(speed_mps)
            tau = (
                self.config.planning_accel_tau_s
                if accelerating
                else self.config.planning_decel_tau_s
            )
            alpha = 1.0 - math.exp(-dt / max(tau, 1.0e-3))
            next_speed = speed_mps + (target_speed - speed_mps) * alpha
            speed = 0.5 * (speed_mps + next_speed)
            yaw_delta = speed * dt / max(self.config.wheelbase_m, 0.01) * math.tan(steering)
            travel_yaw = yaw + yaw_delta * 0.5
            next_x = x_m + speed * dt * math.cos(travel_yaw)
            next_y = y_m + speed * dt * math.sin(travel_yaw)
            next_yaw = math.atan2(math.sin(yaw + yaw_delta), math.cos(yaw + yaw_delta))
            if local_grid is not None and not self._prediction_pose_clear(
                next_x, next_y, next_yaw, x_m, y_m, yaw,
                scan_pose, local_grid, local_obstacles,
            ):
                self.last_prediction_blocked = True
                self.last_prediction_block_time_s = min(
                    self.last_prediction_block_time_s,
                    elapsed_before_s + (step + 1) * dt,
                )
                break
            x_m, y_m, yaw = next_x, next_y, next_yaw
            speed_mps = next_speed
            prefix.append((x_m, y_m))
        return x_m, y_m, yaw, speed_mps

    def _prediction_pose_clear(
        self,
        x_m: float,
        y_m: float,
        yaw: float,
        previous_x_m: float,
        previous_y_m: float,
        previous_yaw: float,
        scan_pose: PoseEstimate,
        local_grid,
        local_obstacles: set[tuple[int, int]],
    ) -> bool:
        assert self.config is not None
        if self.planner is not None:
            depth = tracked_obstacle_overlap(self.planner, np.asarray((
                (previous_x_m, previous_y_m, previous_yaw), (x_m, y_m, yaw),
            )))
            if np.any(depth[1] > depth[0] + 1.0e-6):
                self.last_prediction_block_reason = "tracked-obstacle"
                self.last_prediction_block_point = (x_m, y_m)
                return False
        half_l = self.config.robot_length_m * 0.5
        half_w = self.config.robot_width_m * 0.5
        spacing = max(local_grid.resolution_m * 0.5, 0.02)
        longitudinal = np.linspace(-half_l, half_l, max(2, int(math.ceil(2.0 * half_l / spacing)) + 1))
        lateral = np.linspace(-half_w, half_w, max(2, int(math.ceil(2.0 * half_w / spacing)) + 1))
        cy, sy = math.cos(yaw), math.sin(yaw)
        scan_cy, scan_sy = math.cos(scan_pose.yaw_rad), math.sin(scan_pose.yaw_rad)
        temporal_obstacles = (
            self.planner._confirmed_local_obstacles | self.planner._previous_local_obstacles
            if self.planner is not None
            else set()
        )
        active_anchor = None
        if (
            self.planner is not None
            and self.planner._program_action() in ("left", "right")
            and self.planner._obstacle_program_anchor is not None
        ):
            active_anchor = self.planner._obstacle_program_anchor
            active_component = active_obstacle_component_mask(
                self.planner, active_anchor,
            )
            temporal_obstacles = {
                cell for cell in temporal_obstacles
                if not bool(active_component[cell])
            }
            anchor_cell = self.planner.grid.world_to_cell(*active_anchor)
            if anchor_cell is not None:
                temporal_obstacles.add(anchor_cell)

        def escaping_existing_cell(
            obstacle_x: float,
            obstacle_y: float,
            cell_size_m: float,
        ) -> bool:
            dx = obstacle_x - previous_x_m
            dy = obstacle_y - previous_y_m
            previous_cy, previous_sy = math.cos(previous_yaw), math.sin(previous_yaw)
            cell_margin = cell_size_m / math.sqrt(2.0)
            if not (
                abs(dx * previous_cy + dy * previous_sy) <= half_l + cell_margin
                and abs(-dx * previous_sy + dy * previous_cy) <= half_w + cell_margin
            ):
                return False
            previous_distance_sq = dx * dx + dy * dy
            next_dx = obstacle_x - x_m
            next_dy = obstacle_y - y_m
            escaping = next_dx * next_dx + next_dy * next_dy > previous_distance_sq + 1.0e-7
            if escaping:
                self.last_prediction_escape_cells += 1
            return escaping

        for forward in longitudinal:
            for side in lateral:
                gx = x_m + float(forward) * cy - float(side) * sy
                gy = y_m + float(forward) * sy + float(side) * cy
                if self.planner is not None:
                    map_cell = self.planner.grid.world_to_cell(gx, gy)
                    if map_cell is None:
                        self.last_prediction_block_reason = "map-bounds"
                        self.last_prediction_block_point = (gx, gy)
                        return False
                    map_value = Cell(int(self.planner.grid.cells[map_cell]))
                    if map_value == Cell.MAP_WALL:
                        wall_x, wall_y = self.planner.grid.cell_to_world(*map_cell)
                        if escaping_existing_cell(
                            wall_x, wall_y, self.planner.grid.resolution_m,
                        ):
                            continue
                        self.last_prediction_block_reason = "map-wall"
                        self.last_prediction_block_point = (gx, gy)
                        return False
                    if map_cell in temporal_obstacles:
                        obstacle_x, obstacle_y = self.planner.grid.cell_to_world(*map_cell)
                        if escaping_existing_cell(
                            obstacle_x, obstacle_y, self.planner.grid.resolution_m,
                        ):
                            continue
                        self.last_prediction_block_reason = "persisted-obstacle"
                        self.last_prediction_block_point = (gx, gy)
                        return False
                dx, dy = gx - scan_pose.x_m, gy - scan_pose.y_m
                local_x = dx * scan_cy + dy * scan_sy
                local_y = -dx * scan_sy + dy * scan_cy
                cell = local_grid.local_to_cell(local_x, local_y)
                if cell is None:
                    continue
                row, col = cell
                if (row, col) in local_obstacles:
                    local_obstacle_x, local_obstacle_y = local_grid.cell_to_local(row, col)
                    obstacle_x = (
                        scan_pose.x_m
                        + local_obstacle_x * scan_cy
                        - local_obstacle_y * scan_sy
                    )
                    obstacle_y = (
                        scan_pose.y_m
                        + local_obstacle_x * scan_sy
                        + local_obstacle_y * scan_cy
                    )
                    if (
                        active_anchor is not None
                        and math.dist((obstacle_x, obstacle_y), active_anchor)
                        <= ACTIVE_OBSTACLE_CLUSTER_RADIUS_M
                    ):
                        continue
                    if escaping_existing_cell(
                        obstacle_x, obstacle_y, local_grid.resolution_m,
                    ):
                        continue
                    self.last_prediction_block_reason = "scan-obstacle"
                    self.last_prediction_block_point = (gx, gy)
                    return False
        return True

    def _apply_motion_update(
        self,
        sample_stamp: float,
        odometry: tuple[float, float, float, float],
    ) -> float:
        if self.config is None or self.localizer is None:
            return 0.0
        previous_stamp = self.last_motion_stamp
        self.last_motion_stamp = sample_stamp
        dt_s = 0.0 if previous_stamp is None else sample_stamp - previous_stamp
        self.latest_odometry_speed_mps = odometry[3]
        return self.localizer.apply_odometry(odometry[3], dt_s)

    def _queue_debug_frame(
        self,
        frame: SensorFrame,
        plan: Plan,
        scan_ms: float,
        localize_ms: float,
        plan_ms: float,
        tick_ms: float,
    ) -> None:
        if (
            self.config is None
            or self.localizer is None
            or self.planner is None
            or self.debug_window is None
            or not self.config.debug_view_enabled
        ):
            return
        debug_frame = DebugFrame(
            grid=self.localizer.grid,
            pose=self.localizer.pose,
            plan=plan,
            sensor_frame=frame,
            colored_obstacles=self.planner.tracked_obstacle_display(),
            status_lines=(
                f"pending {self.localizer.pending_direction.name}",
                f"score l/r {self.localizer.last_direction_scores[0]:+.1f}/"
                f"{self.localizer.last_direction_scores[1]:+.1f}",
                f"tracked {self.planner.tracked_obstacle_status()}",
                f"laps {self.planner.completed_laps}/{self.planner.target_laps}",
                f"plan {plan.reason}",
                f"ms scan/loc/plan {scan_ms:.1f}/{localize_ms:.1f}/{plan_ms:.1f}",
                f"ms tick/debug {tick_ms:.1f}/{self.last_debug_render_ms:.1f}",
            ),
        )
        with self.debug_lock:
            self.debug_frame = debug_frame

    def _debug_loop(self) -> None:
        while not self.debug_stop.is_set():
            with self.debug_lock:
                frame = self.debug_frame
                self.debug_frame = None
            if frame is None:
                time.sleep(0.005)
                continue
            if self.debug_window is None:
                return
            debug_start = time.perf_counter()
            key = self.debug_window.show(frame)
            self.last_debug_render_ms = (time.perf_counter() - debug_start) * 1000.0
            if key in (ord("r"), ord("R")) and self.localizer is not None:
                self.localizer.reset()
        if self.debug_window is not None:
            self.debug_window.close()

    def stop_workers(self) -> None:
        self.work_stop.set()
        self.work_event.set()
        if self.work_thread is not None:
            self.work_thread.join(timeout=0.5)
        self.camera_stop.set()
        self.camera_event.set()
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=0.5)

    def destroy_node(self) -> None:
        self.stop_workers()
        self.debug_stop.set()
        if self.debug_thread is not None:
            self.debug_thread.join(timeout=0.5)
        if self.pose_debug_logger is not None:
            self.pose_debug_logger.stop()
        super().destroy_node()

    def _log_pose_debug(
        self, plan: Plan, sim_pose: tuple[float, float, float] | None, scan_ms: float, localize_ms: float, plan_ms: float,
    ) -> None:
        if (
            self.config is None
            or self.localizer is None
            or self.planner is None
            or not self.config.pose_debug
        ):
            return
        if self.pose_debug_logger is None:
            return
        pose = self.localizer.pose
        local_counts = np.zeros(6, dtype=np.intp)
        if self.latest_frame is not None:
            local_counts = np.bincount(
                self.latest_frame.local_grid.cells.ravel(), minlength=6,
            )
        global_obstacles = int(np.count_nonzero(self.localizer.grid.cells == int(Cell.UNKNOWN_OBSTRUCTION)))
        confirmed_cells = getattr(self.planner, "_confirmed_local_obstacles", ())
        current_cells = getattr(self.planner, "_previous_local_obstacles", ())
        confirmed_world = sorted(
            (self.localizer.grid.cell_to_world(row, col) for row, col in confirmed_cells),
            key=lambda point: math.hypot(point[0] - pose.x_m, point[1] - pose.y_m),
        )
        nearest_obstacles = "[" + ",".join(f"{x_m:+.2f}:{y_m:+.2f}" for x_m, y_m in confirmed_world[:4]) + "]"
        current_world = sorted(
            (self.localizer.grid.cell_to_world(row, col) for row, col in current_cells),
            key=lambda point: math.hypot(point[0] - pose.x_m, point[1] - pose.y_m),
        )
        current_obstacles = "[" + ",".join(f"{x_m:+.2f}:{y_m:+.2f}" for x_m, y_m in current_world[:4]) + "]"
        with self.state_lock:
            color_obstacles_frame = self.latest_color_obstacles
            camera_observed_at = self.latest_camera_observation_time
        if color_obstacles_frame is None or not color_obstacles_frame[0]:
            age = max(0.0, time.monotonic() - camera_observed_at) if camera_observed_at > 0.0 else math.inf
            camera_status = f"none/{age:.2f}s"
        else:
            obstacles, observed_at = color_obstacles_frame
            observed = ",".join(
                f"{obstacle.label}@{obstacle.depth_m:.2f}/{obstacle.x_norm:+.2f}"
                for obstacle in obstacles[:4]
            )
            camera_status = f"{observed}/{max(0.0, time.monotonic() - observed_at):.2f}s"
        target_status = (
            "none"
            if plan.target is None
            else f"{plan.target[0]:+.2f}:{plan.target[1]:+.2f}"
        )
        odom_base = getattr(self.localizer, "_base_pose", (pose.x_m, pose.y_m, pose.yaw_rad))
        live_correction = getattr(self.localizer, "_correction", (0.0, 0.0))
        line = (
            f"POSE_DEBUG loc=({pose.x_m:+.3f},{pose.y_m:+.3f},"
            f"{math.degrees(pose.yaw_rad):+.1f}deg) "
            f"base=({odom_base[0]:+.3f},{odom_base[1]:+.3f}) "
            f"corr=({live_correction[0]:+.3f},{live_correction[1]:+.3f}) "
            f"pred=({self.last_planning_pose.x_m:+.3f},{self.last_planning_pose.y_m:+.3f},"
            f"{math.degrees(self.last_planning_pose.yaw_rad):+.1f}deg) "
            f"conf={pose.confidence:.2f} dir={self.localizer.grid.direction.name} "
            f"dirscore={self.localizer.last_direction_scores[0]:+.2f}/"
            f"{self.localizer.last_direction_scores[1]:+.2f} "
            f"dirraw={self.localizer.last_direction_raw_scores[0]:+.2f}/"
            f"{self.localizer.last_direction_raw_scores[1]:+.2f} "
            f"dirvote={self.localizer.last_direction_direct.name} "
            f"wall=({pose.wall_pose_correction_x_m:+.2f},{pose.wall_pose_correction_y_m:+.2f}"
            f"/{pose.wall_pose_sources}) "
            f"scan=({pose.scan_motion_correction_x_m:+.2f},{pose.scan_motion_correction_y_m:+.2f}"
            f"/{pose.scan_motion_score:+.2f}) "
            f"cmd={self.last_motor_command:+d}/{self.last_motor_output:+d} "
            f"stamp={self.get_clock().now().nanoseconds / 1e9:.6f} "
            f"scan_stamp={self.last_processed_scan_stamp:.6f} "
            f"age={self.last_scan_age_s:.3f}s "
            f"predblock={int(self.last_prediction_blocked)} "
            f"predhit={self.last_prediction_block_time_s:.3f}s "
            f"predwhy={self.last_prediction_block_reason} "
            f"predat={self._format_prediction_block_point()} "
            f"predescape={self.last_prediction_escape_cells} "
            f"odom={pose.odometry_motion_m:.3f} "
            f"odomv={pose.odometry_speed_mps:+.3f} "
            f"ms={scan_ms:.1f}/{localize_ms:.1f}/{plan_ms:.1f} "
            f"dbgprep={self.last_pose_debug_ms:.1f} "
            f"lst={'/'.join(f'{value:.1f}' for value in getattr(self.localizer, '_profile_localize_ms', ()))} "
            f"sct={'/'.join(f'{value:.1f}' for value in getattr(self.localizer, '_profile_scan_correction_ms', ()))} "
            f"pst={'/'.join(f'{value:.1f}' for value in getattr(self.planner, '_profile_plan_ms', ()))} "
            f"ost={'/'.join(f'{value:.1f}' for value in getattr(self.planner, '_profile_obstacle_ms', ()))} "
            f"gst={'/'.join(f'{value:.1f}' for value in getattr(self.planner, '_profile_graph_ms', ()))} "
            f"hst={'/'.join(f'{value:.1f}' for value in getattr(self.planner, '_profile_heading_ms', ()))} "
            f"plan={plan.reason} motion={'rev' if plan.motion_direction < 0 else 'fwd'} "
            f"corner={self.planner._corner_phase}/"
            f"{self.planner._corner_recovery_attempted_phase} "
            f"laps={self.planner.completed_laps}/{self.planner.target_laps} "
            f"target={target_status} "
            f"graph={'rev' if getattr(self.planner, '_heading_motion_direction', 1) < 0 else 'fwd'} "
            f"entry={int(self.planner._route_entry_recovery)} "
            f"unknown={'backup' if getattr(self.planner, '_uncolored_backup_active', False) else 'clear'} "
            f"unknown_at={getattr(self.planner, '_uncolored_obstacle_debug', 'none')} "
            f"unknown_hold={int(getattr(self.planner, '_uncolored_obstacle_hold', False))} "
            f"lateral={getattr(self.planner, '_lateral_maneuver_phase', 0)} "
            f"uturn={getattr(self.planner, '_obstacle_program_turn_sign', 0):+d}/"
            f"{math.degrees(getattr(self.planner, '_obstacle_program_turn_radians', 0.0)):.1f}/"
            f"{int(getattr(self.planner, '_obstacle_program_stage_target', None) is not None)} "
            f"steer={math.degrees(plan.steering_angle_rad):+.1f} "
            f"edge={math.degrees(getattr(self.planner, '_heading_steering_rad', 0.0)):+.1f} "
            f"pathk={getattr(self.planner, '_path_max_curvature', 0.0):.2f} "
            f"pathclear={getattr(self.planner, '_path_min_clearance', 0.0):.3f} "
            f"search={getattr(self.planner, '_heading_expansions', 0)} "
            f"connected={int(getattr(self.planner, '_graph_search_connected', False))} "
            f"confirmed={len(confirmed_cells)} obs={nearest_obstacles} raw={current_obstacles} "
            f"local={local_counts[2]}/{local_counts[3] + local_counts[4]} "
            f"global={global_obstacles} "
            f"color={self.planner.tracked_obstacle_status() if self.planner is not None else 'none'} "
            f"tracking={self.planner.tracked_obstacle_debug() if self.planner is not None else 'none'} "
            f"action={self.planner._program_action() if self.planner is not None else 'none'} "
            f"camera={camera_status} "
            f"color_assoc={self.planner.color_observation_status() if self.planner is not None else 'none'} "
            f"route={sampled_path(plan.waypoints)} "
            f"path={sampled_path(plan.trajectory)}"
        )
        if sim_pose is not None:
            tx, ty, tyaw = sim_pose
            yaw_err = math.atan2(math.sin(pose.yaw_rad - tyaw), math.cos(pose.yaw_rad - tyaw))
            line += (
                f" true=({tx:+.3f},{ty:+.3f},{math.degrees(tyaw):+.1f}deg)"
                f" truev={self.latest_sim_speed_mps:+.3f}"
                f" err=({pose.x_m - tx:+.3f},{pose.y_m - ty:+.3f},"
                f"{math.degrees(yaw_err):+.1f}deg)"
                f" dist={math.hypot(pose.x_m - tx, pose.y_m - ty):.3f}"
            )
        self.pose_debug_logger.write(line)

    def _format_prediction_block_point(self) -> str:
        point = self.last_prediction_block_point
        return "none" if point is None else f"{point[0]:+.2f}:{point[1]:+.2f}"
