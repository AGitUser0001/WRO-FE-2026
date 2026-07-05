import math
import threading
import time

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Int32

from .config import DriverConfig
from .debug import DebugFrame, DebugWindow
from .grid import Direction
from .localize import Localizer, PoseEstimate
from .motion_feedback import MotionFeedback
from .path_log import sampled_path
from .planner import GridPlanner, Plan
from .pose_log import PoseDebugLogger
from .sensors import SensorFrame, imu_from_msg, local_grid_from_scan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Driver(Node):
    def __init__(self):
        super().__init__("wro_planner")
        self.config: DriverConfig | None = None
        self.localizer: Localizer | None = None
        self.planner: GridPlanner | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_scan_sim_pose: tuple[float, float, float] | None = None
        self.latest_scan_imu = None
        self.latest_imu = None
        self.latest_frame: SensorFrame | None = None
        self.latest_plan: Plan | None = None
        self.latest_sim_pose: tuple[float, float, float] | None = None
        self.debug_window: DebugWindow | None = None
        self.last_debug_render_ms = 0.0
        self.state_lock = threading.Lock()
        self.debug_lock = threading.Lock()
        self.debug_frame: DebugFrame | None = None
        self.debug_thread: threading.Thread | None = None
        self.debug_stop = threading.Event()
        self.work_event = threading.Event()
        self.work_stop = threading.Event()
        self.work_thread: threading.Thread | None = None
        self.work_busy = False
        self.reverse_hold_until = 0.0
        self.last_prediction_time = time.monotonic()
        self.motion_feedback = MotionFeedback()
        self.last_motor_command = 0
        self.last_servo_command = 0
        self.low_confidence_frames = 0
        self.pose_debug_logger: PoseDebugLogger | None = None
        self.motor_pub = None
        self.servo_pub = None

    def configure(
        self,
        config: DriverConfig,
        localizer: Localizer,
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
        self.create_subscription(
            LaserScan,
            config.scan_topic,
            self._scan_cb,
            qos,
        )
        self.create_subscription(
            Imu,
            config.imu_topic,
            self._imu_cb,
            qos,
        )
        self.create_subscription(Float32MultiArray, "/wro_sim/pose", self._sim_pose_cb, qos)
        self.create_timer(1.0 / max(config.tick_rate, 1.0), self._tick)

    def _scan_cb(self, msg: LaserScan) -> None:
        with self.state_lock:
            self.latest_scan = msg
            self.latest_scan_sim_pose = self.latest_sim_pose
            self.latest_scan_imu = self.latest_imu

    def _imu_cb(self, msg: Imu) -> None:
        with self.state_lock:
            self.latest_imu = imu_from_msg(msg)

    def _sim_pose_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            with self.state_lock:
                self.latest_sim_pose = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

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
            imu = self.latest_scan_imu
        if scan is None:
            self._command(0, 0)
            return
        expected_motion_m = self._predict_pose_from_last_command(now)

        scan_start = time.perf_counter()
        frame = local_grid_from_scan(
            scan,
            size_m=self.config.local_grid_size_m,
            resolution_m=self.localizer.grid.resolution_m,
            max_range_m=self.config.lidar_max_range_m,
        )
        scan_ms = (time.perf_counter() - scan_start) * 1000.0
        if imu is not None:
            frame = SensorFrame(
                local_grid=frame.local_grid,
                points=frame.points,
                wall_distances=frame.wall_distances,
                front_wall=frame.front_wall,
                left_wall=frame.left_wall,
                right_wall=frame.right_wall,
                imu=imu,
            )
        with self.state_lock:
            self.latest_frame = frame
        localize_start = time.perf_counter()
        pose = self.localizer.update_from_sensors(frame)
        localize_ms = (time.perf_counter() - localize_start) * 1000.0
        self.motion_feedback.update(
            self.planner,
            self.latest_plan,
            pose,
            expected_motion_m,
            self.last_motor_command,
        )
        motor = self.config.drive_motor if self.config.auto_drive_enabled else 0
        front_too_close = frame.wall_distances.front < self.config.front_backup_distance_m
        held_reverse = now < self.reverse_hold_until
        current_motion_direction = -1 if held_reverse or self.last_motor_command < 0 else 1
        plan_start = time.perf_counter()
        plan = self.planner.plan(
            pose,
            motion_direction=current_motion_direction,
            preferred_direction=self.localizer.pending_direction,
            local_grid=frame.local_grid,
        )
        self.reverse_hold_until = 0.0
        plan_ms = (time.perf_counter() - plan_start) * 1000.0
        with self.state_lock:
            self.latest_plan = plan
        self._log_pose_debug(plan, sim_pose, scan_ms, localize_ms, plan_ms)
        tick_ms = (time.perf_counter() - tick_start) * 1000.0
        self._queue_debug_frame(
            frame,
            plan,
            scan_ms,
            localize_ms,
            plan_ms,
            tick_ms,
            front_too_close,
        )

        if not self.config.auto_drive_enabled:
            self._command(0, 0)
            return
        servo = self._servo_for_plan(plan, pose)
        commanded_motor = abs(motor) * (1 if plan.motion_direction >= 0 else -1)
        self._command(commanded_motor, servo)

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
        try:
            self.motor_pub.publish(Int32(data=int(motor)))
            self.servo_pub.publish(Int32(data=int(clamp(servo, -max_servo, max_servo))))
        except Exception:
            return
        self.last_motor_command = int(motor)
        self.last_servo_command = int(clamp(servo, -max_servo, max_servo))

    def _predict_pose_from_last_command(self, now: float) -> float:
        if self.config is None or self.localizer is None:
            return 0.0
        dt_s = now - self.last_prediction_time
        self.last_prediction_time = now
        self.motion_feedback.begin_prediction(self.localizer.pose)
        if not self.config.auto_drive_enabled:
            return 0.0
        speed_scale = self.config.planning_speed_mps / max(abs(self.config.drive_motor), 1)
        speed_mps = self.last_motor_command * speed_scale
        self.low_confidence_frames = 0
        steering_rad = (
            -self.last_servo_command
            / max(self.config.max_servo, 1)
            * self.config.max_steering_angle_rad
        )
        self.localizer.predict_motion(speed_mps, steering_rad, self.config.wheelbase_m, dt_s)
        self.motion_feedback.finish_prediction(self.localizer.pose)
        return abs(speed_mps) * min(max(dt_s, 0.0), 0.2)

    def _queue_debug_frame(
        self,
        frame: SensorFrame,
        plan: Plan,
        scan_ms: float,
        localize_ms: float,
        plan_ms: float,
        tick_ms: float,
        front_too_close: bool,
    ) -> None:
        if (
            self.config is None
            or self.localizer is None
            or self.debug_window is None
            or not self.config.debug_view_enabled
        ):
            return
        debug_frame = DebugFrame(
            grid=self.localizer.grid,
            pose=self.localizer.pose,
            plan=plan,
            sensor_frame=frame,
            status_lines=(
                f"pending {self.localizer.pending_direction.name}",
                f"score l/r {self.localizer.last_direction_scores[0]:+.1f}/"
                f"{self.localizer.last_direction_scores[1]:+.1f}",
                f"front veto {front_too_close} < {self.config.front_backup_distance_m:.2f}",
                f"rev hold {max(0.0, self.reverse_hold_until - time.monotonic()):.1f}s",
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
        if self.config is None or self.localizer is None or not self.config.pose_debug:
            return
        if self.pose_debug_logger is None:
            return
        pose = self.localizer.pose
        line = (
            f"POSE_DEBUG loc=({pose.x_m:+.3f},{pose.y_m:+.3f},"
            f"{math.degrees(pose.yaw_rad):+.1f}deg) "
            f"conf={pose.confidence:.2f} dir={self.localizer.grid.direction.name} "
            f"wall=({pose.wall_pose_correction_x_m:+.2f},{pose.wall_pose_correction_y_m:+.2f}"
            f"/{pose.wall_pose_sources}) "
            f"ms={scan_ms:.1f}/{localize_ms:.1f}/{plan_ms:.1f} "
            f"plan={plan.reason} motion={'rev' if plan.motion_direction < 0 else 'fwd'} "
            f"path={sampled_path(plan.trajectory)}"
        )
        if sim_pose is not None:
            tx, ty, tyaw = sim_pose
            yaw_err = math.atan2(math.sin(pose.yaw_rad - tyaw), math.cos(pose.yaw_rad - tyaw))
            line += (
                f" true=({tx:+.3f},{ty:+.3f},{math.degrees(tyaw):+.1f}deg)"
                f" err=({pose.x_m - tx:+.3f},{pose.y_m - ty:+.3f},"
                f"{math.degrees(yaw_err):+.1f}deg)"
                f" dist={math.hypot(pose.x_m - tx, pose.y_m - ty):.3f}"
            )
        self.pose_debug_logger.write(line)
