#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import cast

import numpy as np
from scipy.ndimage import find_objects, label, maximum_filter


def inscribed_obstacle_mask(mask: np.ndarray) -> np.ndarray:
    if not bool(mask.any()):
        return mask.copy()
    components, count = cast(
        tuple[np.ndarray, int],
        label(mask, structure=np.ones((3, 3), dtype=np.uint8)),
    )
    result = np.zeros_like(mask, dtype=np.bool_)
    for component, bounds in enumerate(find_objects(components), start=1):
        if bounds is None:
            continue
        row_slice, col_slice = bounds
        component_mask = components[row_slice, col_slice] == component
        rows, cols = np.nonzero(component_mask)
        height, width = component_mask.shape
        aspect = max(height, width) / max(1, min(height, width))
        if height > 6 or width > 6 or aspect > 3.0 or len(rows) > 24:
            continue
        r0 = int(row_slice.start or 0)
        c0 = int(col_slice.start or 0)
        sizes = np.zeros(component_mask.shape, dtype=np.int16)
        for row in range(component_mask.shape[0]):
            for col in range(component_mask.shape[1]):
                if not component_mask[row, col]:
                    continue
                if row == 0 or col == 0:
                    sizes[row, col] = 1
                else:
                    sizes[row, col] = 1 + min(
                        sizes[row - 1, col],
                        sizes[row, col - 1],
                        sizes[row - 1, col - 1],
                    )
        side = int(sizes.max())
        ends = np.argwhere(sizes == side)
        centroid = np.asarray((
            float((rows + r0).mean() - r0),
            float((cols + c0).mean() - c0),
        ))
        centers = ends.astype(np.float32) - (side - 1) * 0.5
        end_row, end_col = ends[int(np.argmin(np.sum((centers - centroid) ** 2, axis=1)))]
        result[
            r0 + int(end_row) - side + 1 : r0 + int(end_row) + 1,
            c0 + int(end_col) - side + 1 : c0 + int(end_col) + 1,
        ] = True
    return result


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
        coordinates = (np.arange(self.size_cells, dtype=np.float32) - self.origin_cell) * self.resolution_m
        self._world_x = np.broadcast_to(coordinates, self.cells.shape)
        self._world_y = np.broadcast_to(-coordinates[:, None], self.cells.shape)
        self.direction = Direction.UNKNOWN
        self._wall_near_direction: Direction | None = None
        self._wall_near_mask: np.ndarray | None = None
        self._physical_inside_direction: Direction | None = None
        self._physical_inside: np.ndarray | None = None
        self._physical_bounds: tuple[int, int, int, int] | None = None
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
        self._refresh_cells_from_scores()

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
        self.unknown_score *= 1.0 - erode_alpha

        local_rows, local_cols = np.nonzero(local.observed)
        if not len(local_rows):
            self._refresh_cells_from_scores()
            return
        local_x = (local.origin_cell - local_rows.astype(np.float64)) * local.resolution_m
        local_y = (local_cols.astype(np.float64) - local.origin_cell) * local.resolution_m
        global_x = pose_x_m + local_x * cy - local_y * sy
        global_y = pose_y_m + local_x * sy + local_y * cy
        cols = np.rint(global_x / self.resolution_m).astype(np.intp) + self.origin_cell
        rows = self.origin_cell - np.rint(global_y / self.resolution_m).astype(np.intp)
        on_grid = (
            (rows >= 0) & (rows < self.size_cells)
            & (cols >= 0) & (cols < self.size_cells)
        )
        local_rows = local_rows[on_grid]
        local_cols = local_cols[on_grid]
        global_x = global_x[on_grid]
        global_y = global_y[on_grid]
        rows = rows[on_grid]
        cols = cols[on_grid]
        values = local.cells[local_rows, local_cols]

        half_w = self.corridor_width_m * 0.5
        if self.direction == Direction.LEFT:
            x_min, x_max = -2.5, half_w
        elif self.direction == Direction.RIGHT:
            x_min, x_max = -half_w, 2.5
        else:
            x_min, x_max = -half_w, half_w
        navigable = (global_x >= x_min) & (global_x <= x_max)
        if self.direction != Direction.UNKNOWN:
            center_x = (x_min + x_max) * 0.5
            navigable &= (global_y >= -1.5) & (global_y <= 1.5)
            navigable &= ~(
                (global_x > center_x - 0.5) & (global_x < center_x + 0.5)
                & (global_y > -0.5) & (global_y < 0.5)
            )

        if (
            self._wall_near_mask is None
            or self._wall_near_direction != self.direction
        ):
            self._wall_near_mask = np.asarray(maximum_filter(
                self.cells == int(Cell.MAP_WALL),
                size=5,
                mode="constant",
                cval=0,
            ), dtype=np.bool_)
            self._wall_near_direction = self.direction
        wall_near = self._wall_near_mask
        flat = rows * self.size_cells + cols
        map_wall = self.cells[rows, cols] == int(Cell.MAP_WALL)
        outside = ~navigable & ~map_wall
        local_wall = navigable & (values == int(Cell.MAP_WALL))
        fitted_wall = navigable & (values == int(Cell.WALL))
        dynamic = navigable & (
            (values == int(Cell.UNKNOWN_OBSTRUCTION))
            | (values == int(Cell.LIVE_OBSTACLE))
        )
        dynamic_clear = dynamic & ~wall_near[rows, cols]
        dynamic_wall = dynamic & ~dynamic_clear
        free = navigable & ~(local_wall | fitted_wall | dynamic)

        np.logical_or.at(self.live_mask.ravel(), flat[dynamic_clear], True)
        self.cells[rows[outside], cols[outside]] = int(Cell.FREE)

        wall_mul = np.ones(len(flat), dtype=np.float32)
        wall_add = np.zeros(len(flat), dtype=np.float32)
        unknown_mul = np.ones(len(flat), dtype=np.float32)
        unknown_add = np.zeros(len(flat), dtype=np.float32)
        erode = np.float32(1.0 - erode_alpha)

        reset_wall = outside | fitted_wall
        wall_mul[reset_wall] = 0.0
        wall_mul[local_wall] = 0.0
        wall_add[local_wall] = 1.0
        wall_mul[dynamic_clear | free] = erode

        unknown_mul[outside | dynamic_wall] = 0.0
        unknown_mul[local_wall | fitted_wall | free] = erode
        unknown_mul[dynamic_clear] = np.float32(1.0 - add_alpha)
        unknown_add[dynamic_clear] = np.float32(add_alpha)

        order = np.argsort(flat, kind="stable")
        sorted_flat = flat[order]
        starts = np.r_[0, np.flatnonzero(np.diff(sorted_flat)) + 1]
        occurrence = np.arange(len(flat)) - np.repeat(starts, np.diff(np.r_[starts, len(flat)]))
        occurrence_in_input_order = np.empty_like(occurrence)
        occurrence_in_input_order[order] = occurrence
        wall_flat = self.wall_score.ravel()
        unknown_flat = self.unknown_score.ravel()
        for rank in range(int(occurrence.max(initial=-1)) + 1):
            selected = occurrence_in_input_order == rank
            indices = flat[selected]
            wall_flat[indices] = wall_flat[indices] * wall_mul[selected] + wall_add[selected]
            unknown_flat[indices] = (
                unknown_flat[indices] * unknown_mul[selected] + unknown_add[selected]
            )

        self._refresh_cells_from_scores()

    def _near_map_wall(self, idx: tuple[int, int], radius: int) -> bool:
        row, col = idx
        r0 = max(0, row - radius); r1 = min(self.size_cells, row + radius + 1)
        c0 = max(0, col - radius); c1 = min(self.size_cells, col + radius + 1)
        return bool((self.cells[r0:r1, c0:c1] == int(Cell.MAP_WALL)).any())

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
        physical_inside, bounds = self._physical_map_region()
        row0, row1, col0, col1 = bounds
        region = (slice(row0, row1), slice(col0, col1))
        local_cells = self.cells[region]
        dynamic = physical_inside[region] & (local_cells != int(Cell.MAP_WALL))
        wall_mask = dynamic & (self.wall_score[region] >= 0.55)
        obs_mask = inscribed_obstacle_mask(
            dynamic & (self.unknown_score[region] >= 0.55) & ~wall_mask,
        )
        free_mask = dynamic & ~wall_mask & ~obs_mask
        local_cells[free_mask] = int(Cell.FREE)
        local_cells[wall_mask] = int(Cell.WALL)
        local_cells[obs_mask] = int(Cell.UNKNOWN_OBSTRUCTION)

    def inside_navigable_bounds(self, x_m: float, y_m: float) -> bool:
        if not -1.5 <= y_m <= 1.5:
            return False
        half_w = self.corridor_width_m * 0.5
        if self.direction == Direction.LEFT:
            inside = -2.5 <= x_m <= half_w
            center_x = (-2.5 + half_w) * 0.5
            return inside and not (center_x - 0.5 < x_m < center_x + 0.5 and -0.5 < y_m < 0.5)
        if self.direction == Direction.RIGHT:
            inside = -half_w <= x_m <= 2.5
            center_x = (-half_w + 2.5) * 0.5
            return inside and not (center_x - 0.5 < x_m < center_x + 0.5 and -0.5 < y_m < 0.5)
        return -half_w <= x_m <= half_w and -1.5 <= y_m <= 1.5

    def _clear_dynamic_outside_map_bounds(self) -> None:
        physical_inside, _bounds = self._physical_map_region()
        outside = ~physical_inside
        self.wall_score[outside] = 0.0
        self.unknown_score[outside] = 0.0

        cleared = (
            (self.cells != int(Cell.MAP_WALL))
            & (self.wall_score <= 0.0)
            & (self.unknown_score <= 0.0)
        )
        self.cells[cleared] = int(Cell.FREE)

    def _physical_map_region(
        self,
    ) -> tuple[np.ndarray, tuple[int, int, int, int]]:
        if (
            self._physical_inside is not None
            and self._physical_bounds is not None
            and self._physical_inside_direction == self.direction
        ):
            return self._physical_inside, self._physical_bounds
        half_w = self.corridor_width_m * 0.5
        y_inside = (self._world_y >= -1.5) & (self._world_y <= 1.5)
        if self.direction == Direction.LEFT:
            x_min, x_max = -2.5, half_w
        elif self.direction == Direction.RIGHT:
            x_min, x_max = -half_w, 2.5
        else:
            x_min, x_max = -half_w, half_w
        physical_inside = y_inside & (self._world_x >= x_min) & (self._world_x <= x_max)
        if self.direction != Direction.UNKNOWN:
            center_x = (x_min + x_max) * 0.5
            physical_inside &= ~(
                (self._world_x > center_x - 0.5) & (self._world_x < center_x + 0.5)
                & (self._world_y > -0.5) & (self._world_y < 0.5)
            )
        rows, cols = np.nonzero(physical_inside)
        bounds = (
            int(rows.min()), int(rows.max()) + 1,
            int(cols.min()), int(cols.max()) + 1,
        )
        self._physical_inside_direction = self.direction
        self._physical_inside = physical_inside
        self._physical_bounds = bounds
        return physical_inside, bounds

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
