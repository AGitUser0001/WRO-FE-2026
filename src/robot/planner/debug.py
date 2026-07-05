from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np
import numpy.typing as npt

from .grid import Cell, GridMap, LocalGrid
from .localize import PoseEstimate
from .planner import Plan
from .sensors import SensorFrame


Color = tuple[int, int, int]
Point = tuple[int, int]
Image = npt.NDArray[np.uint8]


@dataclass(frozen=True)
class DebugFrame:
    grid: GridMap
    pose: PoseEstimate
    plan: Plan | None = None
    local_grid: LocalGrid | None = None
    sensor_frame: SensorFrame | None = None
    status_lines: tuple[str, ...] = ()


@dataclass(frozen=True)
class DebugStyle:
    map_size_px: int = 720
    panel_width_px: int = 310
    margin_px: int = 34
    background: Color = (245, 245, 245)
    grid_free: Color = (252, 252, 252)
    map_wall: Color = (190, 190, 190)
    wall: Color = (70, 70, 70)
    unknown: Color = (0, 150, 255)
    live_obstacle: Color = (0, 80, 255)
    local_observed: Color = (220, 250, 220)
    local_bounds: Color = (0, 200, 230)
    path: Color = (255, 120, 0)
    trajectory: Color = (230, 40, 180)
    target: Color = (0, 180, 255)
    pose: Color = (30, 130, 30)
    yaw: Color = (20, 20, 220)
    origin: Color = (0, 0, 220)
    text: Color = (25, 25, 25)


class DebugWindow:
    def __init__(
        self,
        name: str = "wro grid planner",
        style: DebugStyle = DebugStyle(),
        robot_length_m: float = 0.25,
        robot_width_m: float = 0.15,
    ):
        self.name = name
        self.style = style
        self.robot_length_m = robot_length_m
        self.robot_width_m = robot_width_m
        self.imshow_failed = False

    def render(self, frame: DebugFrame) -> Image:
        style = self.style
        height = style.map_size_px
        width = style.map_size_px + style.panel_width_px
        image = np.full((height, width, 3), style.background, dtype=np.uint8)
        self._draw_grid(image, frame.grid)
        if frame.local_grid is not None:
            self._draw_local_grid(image, frame.grid, frame.local_grid, frame.pose)
        elif frame.sensor_frame is not None:
            self._draw_local_grid(
                image,
                frame.grid,
                frame.sensor_frame.local_grid,
                frame.pose,
            )
        self._draw_pose(image, frame.grid, frame.pose)
        if frame.plan is not None:
            self._draw_plan(image, frame.grid, frame.plan)
        self._draw_panel(image, frame)
        return image

    def show(self, frame: DebugFrame) -> int:
        image = self.render(frame)
        if self.imshow_failed:
            cv2.imwrite("/tmp/wro_grid_planner_debug.png", image)
            return -1
        try:
            cv2.imshow(self.name, image)
            return cv2.waitKey(1) & 0xFF
        except cv2.error:
            self.imshow_failed = True
            cv2.imwrite("/tmp/wro_grid_planner_debug.png", image)
            return -1

    def close(self) -> None:
        try:
            cv2.destroyWindow(self.name)
        except cv2.error:
            pass

    def _pix(self, grid: GridMap, x_m: float, y_m: float) -> Point:
        style = self.style
        span = grid.spec.half_extent_m * 2.0
        scale = (style.map_size_px - style.margin_px * 2) / max(span, 0.01)
        x_px = int(style.map_size_px * 0.5 + x_m * scale)
        y_px = int(style.map_size_px * 0.5 - y_m * scale)
        return x_px, y_px

    def _draw_grid(self, image: Image, grid: GridMap) -> None:
        style = self.style
        cv2.rectangle(
            image,
            (0, 0),
            (style.map_size_px - 1, style.map_size_px - 1),
            style.grid_free,
            -1,
        )
        cell_px = max(
            1,
            int(
                (style.map_size_px - style.margin_px * 2)
                / max(grid.size_cells - 1, 1)
            ),
        )
        for row, col, value in self._interesting_cells(grid):
            x_m, y_m = grid.cell_to_world(row, col)
            color = self._cell_color(value)
            cx, cy = self._pix(grid, x_m, y_m)
            half = max(1, cell_px // 2)
            cv2.rectangle(
                image,
                (cx - half, cy - half),
                (cx + half, cy + half),
                color,
                -1,
            )
        ox, oy = self._pix(grid, 0.0, 0.0)
        cv2.drawMarker(image, (ox, oy), style.origin, cv2.MARKER_CROSS, 18, 2)

    def _draw_local_grid(
        self,
        image: Image,
        grid: GridMap,
        local: LocalGrid,
        pose: PoseEstimate,
    ) -> None:
        cyaw = math.cos(pose.yaw_rad)
        syaw = math.sin(pose.yaw_rad)
        half = local.spec.half_extent_m
        corners = (
            (half, half),
            (half, -half),
            (-half, -half),
            (-half, half),
        )
        world = tuple(
            (
                pose.x_m + lx * cyaw - ly * syaw,
                pose.y_m + lx * syaw + ly * cyaw,
            )
            for lx, ly in corners
        )
        self._draw_polyline(image, grid, world, self.style.local_bounds, 1, True)
        for row, col, value in local.observed_cells():
            lx, ly = local.cell_to_local(row, col)
            x_m = pose.x_m + lx * cyaw - ly * syaw
            y_m = pose.y_m + lx * syaw + ly * cyaw
            color = self.style.local_observed if value == Cell.FREE else self._cell_color(value)
            cv2.circle(image, self._pix(grid, x_m, y_m), 1, color, -1)

    def _draw_plan(self, image: Image, grid: GridMap, plan: Plan) -> None:
        self._draw_polyline(image, grid, plan.waypoints, self.style.path, 2, False)
        if len(plan.trajectory) == 1:
            px = self._pix(grid, plan.trajectory[0][0], plan.trajectory[0][1])
            cv2.drawMarker(
                image,
                px,
                self.style.trajectory,
                cv2.MARKER_TILTED_CROSS,
                22,
                2,
            )
        else:
            self._draw_polyline(image, grid, plan.trajectory, self.style.trajectory, 4, False)
            for idx, point in enumerate(plan.trajectory):
                if idx % 4 == 0 or idx == len(plan.trajectory) - 1:
                    cv2.circle(image, self._pix(grid, point[0], point[1]), 3, self.style.trajectory, -1)
        if plan.target is not None:
            cv2.circle(image, self._pix(grid, plan.target[0], plan.target[1]), 7, self.style.target, -1)

    def _draw_pose(self, image: Image, grid: GridMap, pose: PoseEstimate) -> None:
        px = self._pix(grid, pose.x_m, pose.y_m)
        footprint = self._robot_footprint(pose)
        cv2.fillConvexPoly(
            image,
            np.array([self._pix(grid, x_m, y_m) for x_m, y_m in footprint], dtype=np.int32),
            self.style.pose,
        )
        self._draw_polyline(image, grid, footprint, (10, 80, 10), 1, True)
        nose = (
            pose.x_m + self.robot_length_m * 0.65 * math.cos(pose.yaw_rad),
            pose.y_m + self.robot_length_m * 0.65 * math.sin(pose.yaw_rad),
        )
        cv2.arrowedLine(image, px, self._pix(grid, nose[0], nose[1]), self.style.yaw, 2, tipLength=0.35)
        radius = int(12 + 18 * max(0.0, min(pose.confidence, 1.0)))
        cv2.circle(image, px, radius, self.style.pose, 1)

    def _robot_footprint(self, pose: PoseEstimate) -> tuple[tuple[float, float], ...]:
        half_l = self.robot_length_m * 0.5
        half_w = self.robot_width_m * 0.5
        local = ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        return tuple(
            (
                pose.x_m + lx * cy - ly * sy,
                pose.y_m + lx * sy + ly * cy,
            )
            for lx, ly in local
        )

    def _draw_panel(self, image: Image, frame: DebugFrame) -> None:
        style = self.style
        x0 = style.map_size_px
        cv2.rectangle(image, (x0, 0), (image.shape[1] - 1, image.shape[0] - 1), (238, 238, 238), -1)
        cv2.line(image, (x0, 0), (x0, image.shape[0] - 1), (180, 180, 180), 1)
        lines = self._status_lines(frame)
        line_step = 20
        for i, line in enumerate(lines):
            cv2.putText(
                image,
                line,
                (x0 + 14, 28 + i * line_step),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                style.text,
                1,
            )
        legend_y = 28 + len(lines) * line_step + 28
        if legend_y < image.shape[0] - 245:
            self._draw_legend(image, x0 + 14, legend_y)

    def _draw_legend(self, image: Image, x: int, y: int) -> None:
        entries = (
            ("map wall", self.style.map_wall),
            ("wall", self.style.wall),
            ("unknown obs", self.style.unknown),
            ("live obs", self.style.live_obstacle),
            ("local observed", self.style.local_observed),
            ("path", self.style.path),
            ("trajectory", self.style.trajectory),
            ("target", self.style.target),
            ("pose/yaw", self.style.pose),
        )
        cv2.putText(image, "legend", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, self.style.text, 1)
        for idx, (label, color) in enumerate(entries):
            yy = y + 26 + idx * 24
            cv2.rectangle(image, (x, yy - 11), (x + 14, yy + 3), color, -1)
            cv2.putText(image, label, (x + 24, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.style.text, 1)

    def _status_lines(self, frame: DebugFrame) -> tuple[str, ...]:
        pose = frame.pose
        plan = frame.plan
        sensor = frame.sensor_frame
        lines = [
            f"pose {pose.x_m:+.2f},{pose.y_m:+.2f}",
            f"yaw {math.degrees(pose.yaw_rad):+.0f} conf {pose.confidence:.2f}",
            f"align {pose.alignment_score:+.2f}",
            f"front move {pose.front_wall_motion_m:+.3f}",
            f"front wall {pose.front_wall_distance_m:.2f} pts {pose.front_wall_points}",
            f"wall corr {pose.wall_pose_correction_x_m:+.2f},{pose.wall_pose_correction_y_m:+.2f}"
            f" n{pose.wall_pose_sources}",
            f"yaw corr {math.degrees(pose.wall_yaw_correction_rad):+.2f} n{pose.wall_yaw_sources}",
            f"dir {frame.grid.direction.name}",
        ]
        if plan is not None:
            move = "rev" if plan.motion_direction < 0 else "fwd"
            traj_m = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(plan.trajectory, plan.trajectory[1:]))
            lines.append(
                f"plan {plan.reason} {move} path {len(plan.waypoints)} traj {len(plan.trajectory)}"
            )
            lines.append(f"traj_m {traj_m:.2f}")
            if len(plan.trajectory) < 2:
                lines.append("trajectory blocked at current pose")
            lines.append(f"steer {math.degrees(plan.steering_angle_rad):+.1f} deg")
        if sensor is not None:
            wall = sensor.wall_distances
            lines.append(f"wall f/l/r {wall.front:.2f}/{wall.left:.2f}/{wall.right:.2f}")
            lines.append(
                f"fit f {sensor.front_wall.distance_m:.2f} "
                f"slope {sensor.front_wall.slope:+.2f}"
            )
            lines.append(
                f"fit l/r {sensor.left_wall.distance_m:.2f}/{sensor.right_wall.distance_m:.2f}"
            )
            lines.append(
                f"slope l/r {sensor.left_wall.slope:+.2f}/{sensor.right_wall.slope:+.2f}"
            )
            lines.append(f"scan pts {len(sensor.points)}")
        lines.extend(frame.status_lines)
        return tuple(lines)

    def _draw_polyline(
        self,
        image: Image,
        grid: GridMap,
        points: tuple[tuple[float, float], ...],
        color: Color,
        thickness: int,
        closed: bool,
    ) -> None:
        if len(points) < 2:
            return
        for a, b in zip(points, points[1:]):
            cv2.line(image, self._pix(grid, a[0], a[1]), self._pix(grid, b[0], b[1]), color, thickness)
        if closed:
            a = points[-1]
            b = points[0]
            cv2.line(image, self._pix(grid, a[0], a[1]), self._pix(grid, b[0], b[1]), color, thickness)

    def _interesting_cells(self, grid: GridMap) -> tuple[tuple[int, int, Cell], ...]:
        cells = []
        snapshot = grid.cells.copy()
        rows, cols = np.nonzero(snapshot)
        for row, col in zip(rows.tolist(), cols.tolist()):
            cells.append((row, col, Cell(int(snapshot[row, col]))))
        return tuple(cells)

    def _cell_color(self, value: Cell) -> Color:
        if value == Cell.MAP_WALL:
            return self.style.map_wall
        if value == Cell.WALL:
            return self.style.wall
        if value == Cell.LIVE_OBSTACLE:
            return self.style.live_obstacle
        if value == Cell.UNKNOWN_OBSTRUCTION:
            return self.style.unknown
        return self.style.grid_free
