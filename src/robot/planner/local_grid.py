from __future__ import annotations

import math
from typing import Iterable, Iterator

import numpy as np

from .grid import Cell, GridSpec


class LocalGrid:
        def __init__(self, size_m: float = 1.5, resolution_m: float = 0.05):
        self.spec = GridSpec(size_m=size_m, resolution_m=resolution_m)
        self.resolution_m = self.spec.resolution_m
        self.size_cells = self.spec.cells
        self.origin_cell = self.size_cells // 2
        self.cells = np.full(
            (self.size_cells, self.size_cells),
            int(Cell.FREE),
            dtype=np.uint8,
        )
        self.observed = np.zeros_like(self.cells, dtype=bool)

    def clear(self) -> None:
        self.cells.fill(int(Cell.FREE))
        self.observed.fill(False)

    def local_to_cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        col = int(round(y_m / self.resolution_m)) + self.origin_cell
        row = self.origin_cell - int(round(x_m / self.resolution_m))
        if 0 <= row < self.size_cells and 0 <= col < self.size_cells:
            return row, col
        return None

    def cell_to_local(self, row: int, col: int) -> tuple[float, float]:
        x_m = (self.origin_cell - row) * self.resolution_m
        y_m = (col - self.origin_cell) * self.resolution_m
        return x_m, y_m

    def mark_scan_points(
        self,
        points: Iterable[tuple[float, float]],
        value: Cell = Cell.LIVE_OBSTACLE,
    ) -> None:
        for x_m, y_m in points:
            idx = self.local_to_cell(x_m, y_m)
            if idx is not None:
                self.cells[idx] = int(value)
                self.observed[idx] = True

    def mark_lidar_points(
        self,
        points: Iterable[tuple[float, float, Cell]],
        free_value: Cell = Cell.FREE,
    ) -> None:
        for x_m, y_m, endpoint_value in points:
            self.mark_free_ray(x_m, y_m, free_value)
            idx = self.local_to_cell(x_m, y_m)
            if idx is not None:
                self.cells[idx] = int(endpoint_value)
                self.observed[idx] = True

    def mark_free_ray(self, x_m: float, y_m: float, value: Cell = Cell.FREE) -> None:
        length = math.hypot(x_m, y_m)
        if length <= 1e-6:
            return
        steps = max(1, int(math.floor(length / self.resolution_m)) - 1)
        for i in range(steps + 1):
            t = i / max(steps + 1, 1)
            idx = self.local_to_cell(x_m * t, y_m * t)
            if idx is not None:
                self.cells[idx] = int(value)
                self.observed[idx] = True

    def mark_wall_line(
        self,
        a: tuple[float, float],
        b: tuple[float, float],
        value: Cell = Cell.WALL,
    ) -> None:
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(length / self.resolution_m)))
        for i in range(steps + 1):
            t = i / steps
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            idx = self.local_to_cell(x, y)
            if idx is not None:
                self.cells[idx] = int(value)
                self.observed[idx] = True

    def non_free_cells(self) -> Iterator[tuple[int, int, Cell]]:
        rows, cols = np.nonzero(self.cells)
        for row, col in zip(rows.tolist(), cols.tolist()):
            value = Cell(int(self.cells[row, col]))
            if value != Cell.FREE:
                yield row, col, value

    def observed_cells(self) -> Iterator[tuple[int, int, Cell]]:
        rows, cols = np.nonzero(self.observed)
        for row, col in zip(rows.tolist(), cols.tolist()):
            yield row, col, Cell(int(self.cells[row, col]))

    def copy_cells(self) -> np.ndarray:
        return self.cells.copy()
