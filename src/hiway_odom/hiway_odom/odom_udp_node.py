#!/usr/bin/env python3
import math
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomUdpNode(Node):

    def __init__(self):
        super().__init__('odom_udp_node')

        # ── Parameters ──
        self.declare_parameter('udp_port', 9999)
        self.declare_parameter('wheelbase', 0.138)           # meters
        # R_center = sqrt(R_outer² - L²) - W/2
        #        ≈ sqrt(30² - 13.8²) - 6.75
        #        ≈ 19.9cm
        self.declare_parameter('max_steering_angle', 0.6066) # 0.6066 radians ≈ 34.8° (arctan(wheelbase/R_center)) (arctan(13.8/19.9)≈34.8°)
        self.declare_parameter('servo_center', 500)
        # pi * wheel_diameter_mm / pulses_per_revolution
        # pulses_per_rev = (40/20) * 11 * 4 * 9.6 = 844.8
        # mm_per_tick = pi * 65 / 844.8 = 0.2418
        self.declare_parameter('mm_per_tick', 3.1416 * 65.0 / 844.8)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_publish_rate', 20.0)

        self.udp_port = self.get_parameter('udp_port').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.servo_center = self.get_parameter('servo_center').value
        self.mm_per_tick = self.get_parameter('mm_per_tick').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.tf_publish_rate = float(self.get_parameter('tf_publish_rate').value)

        # ── State ──
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_ticks = None
        self.last_dist_mm = None  # first packet initializes this
        self.last_time_ms = None
        self.state_lock = threading.Lock()

        # ── Publisher ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.tf_timer = None
        if self.tf_broadcaster is not None and self.tf_publish_rate > 0.0:
            self.tf_timer = self.create_timer(
                1.0 / self.tf_publish_rate,
                self._publish_latest_tf,
            )

        # ── Covariance (same as original ESP32 code) ──
        self.pose_cov = [0.0] * 36
        self.pose_cov[0]  = 0.01   # x
        self.pose_cov[7]  = 0.01   # y
        self.pose_cov[14] = 1e6    # z (not measured)
        self.pose_cov[21] = 1e6    # roll
        self.pose_cov[28] = 1e6    # pitch
        self.pose_cov[35] = 0.03   # yaw

        self.twist_cov = [0.0] * 36
        self.twist_cov[0]  = 0.01  # vx
        self.twist_cov[7]  = 1e6   # vy
        self.twist_cov[14] = 1e6   # vz
        self.twist_cov[21] = 1e6   # roll rate
        self.twist_cov[28] = 1e6   # pitch rate
        self.twist_cov[35] = 0.03  # yaw rate

        # ── UDP receiver thread ──
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(2.0)

        self.running = True
        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        self.get_logger().info(
            f'Odom UDP node listening on port {self.udp_port}, '
            f'wheelbase={self.wheelbase}m, mm_per_tick={self.mm_per_tick:.4f}'
        )

    def _udp_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'UDP recv error: {e}')
                continue

            if len(data) < 12:
                continue

            # Unpack: encoder_ticks(i32), servo_pos(i32), timestamp_ms(u32)
            ticks, servo_pos, time_ms = struct.unpack('<iiI', data[:12])

            # Convert encoder ticks to cumulative distance in mm
            dist_mm = ticks * self.mm_per_tick

            # Initialize on first packet
            if self.last_dist_mm is None:
                self.last_ticks = ticks
                self.last_dist_mm = dist_mm
                self.last_time_ms = time_ms
                continue

            if time_ms < self.last_time_ms:
                self.get_logger().warn(
                    f'ESP32 timestamp went backwards: {self.last_time_ms} -> {time_ms}. '
                    f'Assuming ESP reset/reboot; re-baselining at ticks={ticks}.'
                )
                self.last_ticks = ticks
                self.last_dist_mm = dist_mm
                self.last_time_ms = time_ms
                continue

            if ticks == 0 and self.last_ticks is not None and abs(self.last_ticks) > 50:
                self.get_logger().warn(
                    f'Encoder ticks jumped to zero without timestamp reset: '
                    f'{self.last_ticks} -> {ticks} at t={time_ms}ms. '
                    'Ignoring sample and re-baselining.'
                )
                self.last_ticks = ticks
                self.last_dist_mm = dist_mm
                self.last_time_ms = time_ms
                continue

            # Compute delta distance (meters) and dt (seconds)
            delta_dist = (dist_mm - self.last_dist_mm) / 1000.0
            dt_ms = time_ms - self.last_time_ms
            if dt_ms <= 0:
                dt_ms = 50  # fallback
            dt = dt_ms / 1000.0

            self.last_ticks = ticks
            self.last_dist_mm = dist_mm
            self.last_time_ms = time_ms

            # Steering angle from servo position
            # servo_pos range: -500 to +500, 0=center
            steering = -self.max_steering * (servo_pos / self.servo_center)

            # Ackermann kinematics
            delta_theta = (delta_dist / self.wheelbase) * math.tan(steering)

            with self.state_lock:
                self.x += delta_dist * math.cos(self.theta)
                self.y += delta_dist * math.sin(self.theta)
                self.theta += delta_theta
                x = self.x
                y = self.y
                theta = self.theta

            # Build and publish Odometry message
            now = self.get_clock().now().to_msg()

            msg = Odometry()
            msg.header.stamp = now
            msg.header.frame_id = self.odom_frame
            msg.child_frame_id = self.base_frame

            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0

            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.pose.covariance = self.pose_cov

            msg.twist.twist.linear.x = delta_dist / dt if dt > 0.001 else 0.0
            msg.twist.twist.angular.z = delta_theta / dt if dt > 0.001 else 0.0
            msg.twist.covariance = self.twist_cov

            self.odom_pub.publish(msg)

    def _publish_latest_tf(self):
        if self.tf_broadcaster is None:
            return

        with self.state_lock:
            x = self.x
            y = self.y
            theta = self.theta

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(theta / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
