from __future__ import annotations

import math

import cv2
import numpy as np
from typing import Protocol


Point = tuple[int, int]
Segment = tuple[tuple[float, float], tuple[float, float]]


class RobotView(Protocol):
    x_m: float
    y_m: float
    yaw_rad: float
    motor: int
    servo: int


class SimulatorViewMixin:
    canvas_px: int
    command_timeout_s: float
    drag_aim: tuple[float, float] | None
    dragging: bool
    last_motor_command_time: float
    robot: RobotView
    segments: tuple[Segment, ...]
    ui_scale: float
    window: str
    world_extent_m: float

    def _footprint_corners(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> tuple[tuple[float, float], ...]: ...

    def _lidar_angle_visible(self, local_angle_rad: float) -> bool: ...

    def _pose_is_free(self, x_m: float, y_m: float, yaw_rad: float) -> bool: ...

    def _raycast(self, local_angle_rad: float) -> float: ...

    def _draw(self) -> int:
        image = np.full((self.canvas_px, self.canvas_px, 3), 245, dtype=np.uint8)
        for segment in self.segments:
            cv2.line(
                image,
                self._pix(segment[0][0], segment[0][1]),
                self._pix(segment[1][0], segment[1][1]),
                (35, 35, 35),
                self._thick(3),
            )
        self._draw_lidar(image)
        self._draw_robot(image)
        status = (
            f"motor {self.robot.motor:+d} servo {self.robot.servo:+d} "
            f"pose {self.robot.x_m:+.2f},{self.robot.y_m:+.2f} "
            f"yaw {math.degrees(self.robot.yaw_rad):+.0f}"
        )
        age = self._monotonic() - self.last_motor_command_time
        lines = (
            (status, (20, 20, 20), 34, 0.55),
            (f"cmd age {age:.2f}s timeout {self.command_timeout_s:.1f}s", (80, 80, 80), 70, 0.50),
            ("click to place, drag to aim, r reset, q quit", (80, 80, 80), 106, 0.50),
        )
        for text, color, y_px, scale in lines:
            cv2.putText(
                image,
                text,
                self._pt(18, y_px),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale * self.ui_scale,
                color,
                self._thick(1),
            )
        cv2.imshow(self.window, image)
        return cv2.waitKey(1) & 0xFF

    def _draw_lidar(self, image: np.ndarray) -> None:
        for i in range(0, 361, 4):
            angle = -math.pi + i * (2.0 * math.pi / 360.0)
            if not self._lidar_angle_visible(angle):
                continue
            dist = self._raycast(angle)
            if not math.isfinite(dist):
                continue
            forward = (math.cos(self.robot.yaw_rad), math.sin(self.robot.yaw_rad))
            left = (-math.sin(self.robot.yaw_rad), math.cos(self.robot.yaw_rad))
            wx = self.robot.x_m + dist * (forward[0] * math.cos(angle) + left[0] * math.sin(angle))
            wy = self.robot.y_m + dist * (forward[1] * math.cos(angle) + left[1] * math.sin(angle))
            origin_px = self._pix(self.robot.x_m, self.robot.y_m)
            hit_px = self._pix(wx, wy)
            cv2.line(image, origin_px, hit_px, (185, 205, 220), self._thick(1))
            cv2.circle(image, hit_px, self._thick(2), (0, 110, 255), -1)

    def _draw_robot(self, image: np.ndarray) -> None:
        p = self._pix(self.robot.x_m, self.robot.y_m)
        corners = self._footprint_corners(self.robot.x_m, self.robot.y_m, self.robot.yaw_rad)
        cv2.fillConvexPoly(
            image,
            np.array([self._pix(x, y) for x, y in corners], dtype=np.int32),
            (20, 140, 30),
        )
        nose = (
            self.robot.x_m + 0.25 * math.cos(self.robot.yaw_rad),
            self.robot.y_m + 0.25 * math.sin(self.robot.yaw_rad),
        )
        cv2.arrowedLine(
            image,
            p,
            self._pix(nose[0], nose[1]),
            (0, 0, 220),
            self._thick(2),
            tipLength=0.35,
        )
        if self.dragging and self.drag_aim is not None:
            cv2.line(image, p, self._pix(self.drag_aim[0], self.drag_aim[1]), (220, 80, 0), self._thick(1))

    def _pix(self, x_m: float, y_m: float) -> Point:
        scale = self.canvas_px / (self.world_extent_m * 2.0)
        return (
            int(self.canvas_px * 0.5 + x_m * scale),
            int(self.canvas_px * 0.5 - y_m * scale),
        )

    def _world(self, x_px: int, y_px: int) -> tuple[float, float]:
        scale = self.canvas_px / (self.world_extent_m * 2.0)
        return (
            (x_px - self.canvas_px * 0.5) / scale,
            (self.canvas_px * 0.5 - y_px) / scale,
        )

    def _pt(self, x: int, y: int) -> Point:
        return int(round(x * self.ui_scale)), int(round(y * self.ui_scale))

    def _thick(self, value: int) -> int:
        return max(1, int(round(value * self.ui_scale)))

    def _mouse_cb(self, event: int, x_px: int, y_px: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            wx, wy = self._world(x_px, y_px)
            if self._pose_is_free(wx, wy, self.robot.yaw_rad):
                self.dragging = True
                self.robot.x_m = wx
                self.robot.y_m = wy
                self.drag_aim = (wx, wy)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            wx, wy = self._world(x_px, y_px)
            self.drag_aim = (wx, wy)
            if math.hypot(wx - self.robot.x_m, wy - self.robot.y_m) > 0.03:
                self.robot.yaw_rad = math.atan2(wy - self.robot.y_m, wx - self.robot.x_m)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            self.drag_aim = None

    @staticmethod
    def _monotonic() -> float:
        import time

        return time.monotonic()
