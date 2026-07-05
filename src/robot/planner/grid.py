#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

import numpy as np


class Cell(IntEnum):
    FREE = 0
    MAP_WALL = 1
    WALL = 2
    UNKNOWN_OBSTRUCTION = 3
    LIVE_OBSTACLE = 4


class Direction(IntEnum):
    UNKNOWN = 0
    LEFT = -1
    RIGHT = 1


@dataclass(frozen=True)
class GridSpec:
    size_m: float = 8.0
    resolution_m: float = 0.05

    @property
    def cells(self) -> int:
        n = int(math.ceil(self.size_m / self.resolution_m))
        return n if n % 2 == 1 else n + 1

    @property
    def half_extent_m(self) -> float:
        return self.resolution_m * (self.cells - 1) * 0.5


class GridMap:
    def __init__(
        self,
        size_m: float = 8.0,
        resolution_m: float = 0.05,
        corridor_width_m: float = 1.0,
        corridor_length_m: float = 3.0,
    ):
        self.spec = GridSpec(size_m=size_m, resolution_m=resolution_m)
        self.resolution_m = self.spec.resolution_m
        self.size_cells = self.spec.cells
        self.origin_cell = self.size_cells // 2
        self.cells = np.full((self.size_cells, self.size_cells), int(Cell.FREE), dtype=np.uint8)
        self.wall_score = np.zeros_like(self.cells, dtype=np.float32)
        self.unknown_score = np.zeros_like(self.cells, dtype=np.float32)
        self.live_mask = np.zeros_like(self.cells, dtype=np.bool_)
        self.direction = Direction.UNKNOWN
        self.corridor_width_m = corridor_width_m
        self.corridor_length_m = corridor_length_m
        self.map_initial_overlapping_candidates()

    def reset(self) -> None:
        self.cells.fill(int(Cell.FREE))
        self.wall_score.fill(0.0)
        self.unknown_score.fill(0.0)
        self.live_mask.fill(False)
        self.direction = Direction.UNKNOWN
        self.map_initial_overlapping_candidates()

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        col = int(round(x_m / self.resolution_m)) + self.origin_cell
        row = self.origin_cell - int(round(y_m / self.resolution_m))
        if 0 <= row < self.size_cells and 0 <= col < self.size_cells:
            return row, col
        return None

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        x_m = (col - self.origin_cell) * self.resolution_m
        y_m = (self.origin_cell - row) * self.resolution_m
        return x_m, y_m

    def raycast_wall_distance(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        max_distance_m: float = 2.2,
        map_walls_only: bool = False,
    ) -> float | None:
        step = max(self.resolution_m * 0.5, 0.01)
        distance = step
        cx = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        while distance <= max_distance_m:
            idx = self.world_to_cell(x_m + cx * distance, y_m + sy * distance)
            if idx is None:
                return None
            value = Cell(int(self.cells[idx]))
            if value == Cell.MAP_WALL or (not map_walls_only and value == Cell.WALL):
                return distance
            distance += step
        return None

    def map_initial_overlapping_candidates(self) -> None:
        self._draw_direction_box(Direction.LEFT)
        self._draw_direction_box(Direction.RIGHT)
        self._hollow_start_corridor_side_extensions()

    def map_direction(self, direction: Direction | int) -> None:
        direction = Direction(direction)
        if direction == Direction.UNKNOWN:
            return
        self.cells.fill(int(Cell.FREE))
        self.wall_score.fill(0.0)
        self.unknown_score.fill(0.0)
        self.live_mask.fill(False)
        self.direction = direction
        self._draw_direction_box(direction)

    def lock_direction(self, direction: Direction | int) -> None:
        direction = Direction(direction)
        if direction == Direction.UNKNOWN:
            return
        if self.direction == direction:
            return
        self.map_direction(direction)

    def _draw_direction_box(self, direction: Direction) -> None:
        half = 1.5
        corridor_half_w = self.corridor_width_m * 0.5
        if direction == Direction.LEFT:
            x_min = -2.5
            x_max = corridor_half_w
        else:
            x_min = -corridor_half_w
            x_max = 2.5

        self._set_segment((x_min, -half), (x_max, -half), Cell.MAP_WALL)
        self._set_segment((x_min, half), (x_max, half), Cell.MAP_WALL)
        self._set_segment((x_min, -half), (x_min, half), Cell.MAP_WALL)
        self._set_segment((x_max, -half), (x_max, half), Cell.MAP_WALL)

        center_x = (x_min + x_max) * 0.5
        bx0, by0, bx1, by1 = center_x - 0.5, -0.5, center_x + 0.5, 0.5
        self._set_segment((bx0, by0), (bx1, by0), Cell.MAP_WALL)
        self._set_segment((bx1, by0), (bx1, by1), Cell.MAP_WALL)
        self._set_segment((bx1, by1), (bx0, by1), Cell.MAP_WALL)
        self._set_segment((bx0, by1), (bx0, by0), Cell.MAP_WALL)

    def _hollow_start_corridor_side_extensions(self) -> None:
        half_w = self.corridor_width_m * 0.5
        half_h = self.corridor_length_m * 0.5
        side_half_l = self.corridor_width_m * 0.5
        thickness = self.resolution_m * 1.5
        for x in (-half_w, half_w):
            self._clear_rect(x - thickness, side_half_l, x + thickness, half_h - thickness)
            self._clear_rect(x - thickness, -half_h + thickness, x + thickness, -side_half_l)

    def fuse_local(
        self,
        local: "LocalGrid",
        pose_x_m: float,
        pose_y_m: float,
        yaw_rad: float,
        add_alpha: float = 0.08,
        erode_alpha: float = 0.02,
    ) -> None:
        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        add_alpha = max(0.0, min(add_alpha, 1.0))
        erode_alpha = max(0.0, min(erode_alpha, 1.0))
        self.live_mask.fill(False)

        for row, col, value in local.observed_cells():
            lx, ly = local.cell_to_local(row, col)
            gx = pose_x_m + lx * cy - ly * sy
            gy = pose_y_m + lx * sy + ly * cy
            idx = self.world_to_cell(gx, gy)
            if idx is None:
                continue
            if not self._inside_map_bounds(gx, gy):
                if self.cells[idx] != int(Cell.MAP_WALL):
                    self.wall_score[idx] = 0.0
                    self.unknown_score[idx] = 0.0
                    self.cells[idx] = int(Cell.FREE)
                continue
            if value != Cell.FREE:
                self.live_mask[idx] = True
            if value == Cell.MAP_WALL:
                self.wall_score[idx] = 1.0
                self.unknown_score[idx] = self._blend(self.unknown_score[idx], 0.0, erode_alpha)
            elif value == Cell.WALL:
                self.wall_score[idx] = 0.0
                self.unknown_score[idx] = self._blend(self.unknown_score[idx], 0.0, erode_alpha)
            elif value in (Cell.UNKNOWN_OBSTRUCTION, Cell.LIVE_OBSTACLE):
                r0 = max(0, idx[0] - 1); r1 = min(self.size_cells, idx[0] + 2)
                c0 = max(0, idx[1] - 1); c1 = min(self.size_cells, idx[1] + 2)
                patch = self.unknown_score[r0:r1, c0:c1]
                self.unknown_score[r0:r1, c0:c1] = np.maximum(patch, patch * (1.0 - add_alpha * 0.75) + add_alpha * 0.75)
                self.unknown_score[idx] = self._blend(self.unknown_score[idx], 1.0, add_alpha)
                self.wall_score[idx] = self._blend(self.wall_score[idx], 0.0, erode_alpha)
            else:
                self.wall_score[idx] = self._blend(self.wall_score[idx], 0.0, erode_alpha)
                self.unknown_score[idx] = self._blend(self.unknown_score[idx], 0.0, erode_alpha)

        self._refresh_cells_from_scores()

    def nearest_alignment_vector(
        self,
        local: "LocalGrid",
        pose_x_m: float,
        pose_y_m: float,
        yaw_rad: float,
        search_radius_m: float = 0.35,
    ) -> tuple[float, float, float]:
        max_cells = max(1, int(round(search_radius_m / self.resolution_m)))
        occupied = list(local.non_free_cells())
        if not occupied:
            return 0.0, 0.0, 0.0

        cy = math.cos(yaw_rad)
        sy = math.sin(yaw_rad)
        transformed = []
        for row, col, value in occupied:
            if value == Cell.FREE:
                continue
            lx, ly = local.cell_to_local(row, col)
            gx = pose_x_m + lx * cy - ly * sy
            gy = pose_y_m + lx * sy + ly * cy
            idx = self.world_to_cell(gx, gy)
            if idx is not None:
                transformed.append((idx[0], idx[1], value))
        if not transformed:
            return 0.0, 0.0, 0.0

        best = (0, 0, -math.inf)
        for dr in range(-max_cells, max_cells + 1):
            for dc in range(-max_cells, max_cells + 1):
                score = 0.0
                for row, col, value in transformed:
                    rr = row + dr
                    cc = col + dc
                    if not (0 <= rr < self.size_cells and 0 <= cc < self.size_cells):
                        continue
                    global_value = Cell(int(self.cells[rr, cc]))
                    if global_value == Cell.MAP_WALL:
                        score += 2.0 if value in (Cell.MAP_WALL, Cell.WALL) else -1.0
                    elif global_value in (Cell.UNKNOWN_OBSTRUCTION, Cell.LIVE_OBSTACLE):
                        score += 1.0 if value in (Cell.UNKNOWN_OBSTRUCTION, Cell.LIVE_OBSTACLE) else -0.5
                    elif value in (Cell.MAP_WALL, Cell.WALL):
                        score -= 0.25
                if score > best[2]:
                    best = (dr, dc, score)

        dr, dc, score = best
        return dc * self.resolution_m, -dr * self.resolution_m, score / len(transformed)

    def _refresh_cells_from_scores(self) -> None:
        self._clear_dynamic_outside_map_bounds()
        dynamic = self.cells != int(Cell.MAP_WALL)
        wall_mask = dynamic & (self.wall_score >= 0.55)
        obs_mask = dynamic & (self.unknown_score >= 0.55) & ~wall_mask
        free_mask = dynamic & ~wall_mask & ~obs_mask
        self.cells[free_mask] = int(Cell.FREE)
        self.cells[wall_mask] = int(Cell.WALL)
        self.cells[obs_mask] = int(Cell.UNKNOWN_OBSTRUCTION)

    @staticmethod
    def _blend(old: float, new: float, alpha: float) -> float:
        return old * (1.0 - alpha) + new * alpha

    def _inside_map_bounds(self, x_m: float, y_m: float) -> bool:
        if not -1.5 <= y_m <= 1.5:
            return False
        half_w = self.corridor_width_m * 0.5
        if self.direction == Direction.LEFT:
            return -2.5 <= x_m <= half_w
        if self.direction == Direction.RIGHT:
            return -half_w <= x_m <= 2.5
        return -2.5 <= x_m <= 2.5

    def _clear_dynamic_outside_map_bounds(self) -> None:
        rows, cols = np.nonzero((self.wall_score > 0.0) | (self.unknown_score > 0.0))
        for row, col in zip(rows, cols):
            x_m, y_m = self.cell_to_world(int(row), int(col))
            if self._inside_map_bounds(x_m, y_m):
                continue
            self.wall_score[row, col] = 0.0
            self.unknown_score[row, col] = 0.0
            if self.cells[row, col] != int(Cell.MAP_WALL):
                self.cells[row, col] = int(Cell.FREE)

    def _set_segment(
        self,
        a: tuple[float, float],
        b: tuple[float, float],
        value: Cell,
    ) -> None:
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(length / self.resolution_m)))
        for i in range(steps + 1):
            t = i / steps
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            idx = self.world_to_cell(x, y)
            if idx is not None:
                self._write_cell(idx, value)

    def _set_rect(
        self,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        value: Cell,
    ) -> None:
        lo_x, hi_x = sorted((x0, x1))
        lo_y, hi_y = sorted((y0, y1))
        for x in np.arange(lo_x, hi_x + self.resolution_m * 0.5, self.resolution_m):
            for y in np.arange(lo_y, hi_y + self.resolution_m * 0.5, self.resolution_m):
                idx = self.world_to_cell(float(x), float(y))
                if idx is not None:
                    self._write_cell(idx, value)

    def _clear_rect(self, x0: float, y0: float, x1: float, y1: float) -> None:
        lo_x, hi_x = sorted((x0, x1))
        lo_y, hi_y = sorted((y0, y1))
        for x in np.arange(lo_x, hi_x + self.resolution_m * 0.5, self.resolution_m):
            for y in np.arange(lo_y, hi_y + self.resolution_m * 0.5, self.resolution_m):
                idx = self.world_to_cell(float(x), float(y))
                if idx is not None:
                    self.cells[idx] = int(Cell.FREE)
                    self.wall_score[idx] = 0.0
                    self.unknown_score[idx] = 0.0

    def _write_cell(self, idx: tuple[int, int], value: Cell) -> None:
        if self.cells[idx] == int(Cell.MAP_WALL) and value == Cell.WALL:
            self.wall_score[idx] = 1.0
            return
        self.cells[idx] = int(value)
        if value in (Cell.MAP_WALL, Cell.WALL):
            self.wall_score[idx] = 1.0


from .local_grid import LocalGrid  # noqa: E402
