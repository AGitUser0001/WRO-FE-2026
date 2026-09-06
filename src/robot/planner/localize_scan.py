from __future__ import annotations

import math
from typing import cast

import numpy as np
from scipy.ndimage import distance_transform_edt, find_objects, label, map_coordinates

from .grid import Cell, GridMap
from .sensors import SensorFrame


_MAP_DISTANCE_CACHE: dict[
    tuple[bytes, tuple[int, int], float],
    tuple[np.ndarray, np.ndarray],
] = {}


def scan_world_walls(
    frame: SensorFrame,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    min_span_m: float = 0.20,
) -> tuple[tuple[tuple[float, float], ...], tuple[int, ...]]:
    wall_mask = frame.local_grid.cells == int(Cell.WALL)
    components, _count = cast(
        tuple[np.ndarray, int],
        label(wall_mask, structure=np.ones((3, 3), dtype=np.uint8)),
    )
    points: list[tuple[float, float]] = []
    normal_axes: list[int] = []
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    for component, bounds in enumerate(find_objects(components), start=1):
        if bounds is None:
            continue
        row_slice, col_slice = bounds
        rows, cols = np.nonzero(components[row_slice, col_slice] == component)
        rows += int(row_slice.start or 0)
        cols += int(col_slice.start or 0)
        if len(rows) < 6:
            continue
        local = np.column_stack(
            (
                (frame.local_grid.origin_cell - rows) * frame.local_grid.resolution_m,
                (cols - frame.local_grid.origin_cell) * frame.local_grid.resolution_m,
            )
        ).astype(np.float32, copy=False)
        centered = local - local.mean(axis=0)
        covariance = centered.T @ centered
        xx = float(covariance[0, 0])
        xy = float(covariance[0, 1])
        yy = float(covariance[1, 1])
        radius = math.hypot((xx - yy) * 0.5, xy)
        largest = (xx + yy) * 0.5 + radius
        smallest = (xx + yy) * 0.5 - radius
        if largest < max(smallest * 8.0, 1.0e-5):
            continue
        angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
        tangent = np.asarray((math.cos(angle), math.sin(angle)), dtype=np.float32)
        projection = centered @ tangent
        if float(projection.max() - projection.min()) < min_span_m:
            continue
        world_tx = float(tangent[0]) * cy - float(tangent[1]) * sy
        world_ty = float(tangent[0]) * sy + float(tangent[1]) * cy
        if abs(world_ty) >= 0.85:
            normal_axis = 0
        elif abs(world_tx) >= 0.85:
            normal_axis = 1
        else:
            continue
        local_x = (
            frame.local_grid.origin_cell - rows.astype(np.float64)
        ) * frame.local_grid.resolution_m
        local_y = (
            cols.astype(np.float64) - frame.local_grid.origin_cell
        ) * frame.local_grid.resolution_m
        world_x = x_m + local_x * cy - local_y * sy
        world_y = y_m + local_x * sy + local_y * cy
        points.extend(zip(world_x.tolist(), world_y.tolist()))
        normal_axes.extend([normal_axis] * len(rows))
    return tuple(points), tuple(normal_axes)

def best_map_shift(
    points: tuple[tuple[float, float], ...],
    normal_axes: tuple[int, ...],
    grid: GridMap,
    candidate_offsets: tuple[tuple[float, float], ...],
    offset_penalty: float = 0.35,
    prior_offset: tuple[float, float] = (0.0, 0.0),
    prior_penalty: float = 0.0,
) -> tuple[float, float, float, float] | None:
    if len(points) < 6 or len(normal_axes) != len(points) or not candidate_offsets:
        return None
    wall_mask = grid.cells == int(Cell.MAP_WALL)
    if not bool(wall_mask.any()):
        return None
    cache_key = (wall_mask.tobytes(), wall_mask.shape, grid.resolution_m)
    distance_fields = _MAP_DISTANCE_CACHE.get(cache_key)
    if distance_fields is None:
        vertical = wall_mask & (
            np.roll(wall_mask, 1, axis=0) | np.roll(wall_mask, -1, axis=0)
        )
        horizontal = wall_mask & (
            np.roll(wall_mask, 1, axis=1) | np.roll(wall_mask, -1, axis=1)
        )
        distance_fields = (
            np.asarray(distance_transform_edt(~vertical), dtype=np.float32)
            * grid.resolution_m,
            np.asarray(distance_transform_edt(~horizontal), dtype=np.float32)
            * grid.resolution_m,
        )
        _MAP_DISTANCE_CACHE[cache_key] = distance_fields
        if len(_MAP_DISTANCE_CACHE) > 4:
            _MAP_DISTANCE_CACHE.pop(next(iter(_MAP_DISTANCE_CACHE)))
    current = np.asarray(points, dtype=np.float32)
    offsets = np.asarray(candidate_offsets, dtype=np.float32)
    shifted = np.asarray(current[None, :, :] + offsets[:, None, :], dtype=np.float32)
    cols = shifted[:, :, 0] / grid.resolution_m + grid.origin_cell
    rows = grid.origin_cell - shifted[:, :, 1] / grid.resolution_m
    axes = np.asarray(normal_axes, dtype=np.intp)
    distances = np.empty(rows.shape, dtype=np.float32)
    for axis, field in enumerate(distance_fields):
        selected = axes == axis
        if not bool(selected.any()):
            continue
        coordinates = np.asarray((rows[:, selected].ravel(), cols[:, selected].ravel()))
        distances[:, selected] = np.asarray(
            map_coordinates(
                field,
                coordinates,
                order=1,
                mode="constant",
                cval=0.30,
                prefilter=False,
            ),
            dtype=np.float32,
        ).reshape(len(offsets), int(selected.sum()))
    sigma = max(grid.resolution_m * 1.2, 0.05)
    scores = np.exp(-((distances / sigma) ** 2)).mean(axis=1)
    scores -= np.hypot(offsets[:, 0], offsets[:, 1]) * max(0.0, offset_penalty)
    prior_distance = np.hypot(
        offsets[:, 0] - prior_offset[0],
        offsets[:, 1] - prior_offset[1],
    )
    scores -= prior_distance * max(0.0, prior_penalty)
    prior_idx = int(np.argmin(prior_distance))
    best_idx = int(np.argmax(scores))
    return (
        float(offsets[best_idx, 0]),
        float(offsets[best_idx, 1]),
        float(scores[best_idx]),
        float(scores[prior_idx]),
    )
