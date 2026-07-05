from __future__ import annotations

import math

from .grid import Cell, Direction, GridMap, LocalGrid
from .localize_types import PoseEstimate
from .sensors import SensorFrame


class DirectionLockMixin:
    grid: GridMap
    pose: PoseEstimate
    pending_direction: Direction
    pending_direction_frames: int
    pending_unknown_frames: int
    last_direction_scores: tuple[float, float]
    direction_evidence: dict[Direction, float]
    direction_hypotheses: dict[Direction, GridMap]
    direction_lock_confirm_frames: int
    direction_lock_min_score_delta: float
    front_wall_motion_min_points: int

    def lock_direction(self, direction: Direction | int) -> None:
        candidate = Direction(direction)
        if candidate == Direction.UNKNOWN or self.grid.direction == candidate:
            return
        hypothesis = self.direction_hypotheses.get(candidate)
        if hypothesis is None:
            hypothesis = self._make_direction_hypothesis(candidate)
        self.grid.cells[:, :] = hypothesis.cells
        self.grid.wall_score[:, :] = hypothesis.wall_score
        self.grid.unknown_score[:, :] = hypothesis.unknown_score
        self.grid.direction = candidate
        self.pending_direction = candidate
        self.pending_direction_frames = 0
        half_w = self.grid.corridor_width_m * 0.5
        if abs(self.pose.x_m) <= half_w + 0.25:
            self.pose.x_m = max(-half_w, min(half_w, self.pose.x_m))

    def _update_direction_lock(self, frame: SensorFrame) -> None:
        if self.grid.direction != Direction.UNKNOWN:
            self._maybe_relock_direction(frame)
            return
        candidate = self._corner_direction_candidate(frame)
        if candidate == Direction.UNKNOWN:
            self._note_unknown_direction()
            return
        self.pending_unknown_frames = 0
        left_score, right_score = self.last_direction_scores
        if abs(left_score - right_score) < self.direction_lock_min_score_delta:
            self._note_unknown_direction()
            return
        if candidate != self.pending_direction:
            self.pending_direction = candidate
            self.pending_direction_frames = 1
            return
        self.pending_direction_frames += 1
        if self.pending_direction_frames >= self.direction_lock_confirm_frames:
            self.lock_direction(candidate)

    def _maybe_relock_direction(self, frame: SensorFrame) -> None:
        current = self.grid.direction
        other = Direction.LEFT if current == Direction.RIGHT else Direction.RIGHT
        current_score = self._score_direction_hypothesis(frame.local_grid, current)
        other_score = self._score_direction_hypothesis(frame.local_grid, other)
        self.last_direction_scores = (
            other_score if other == Direction.LEFT else current_score,
            other_score if other == Direction.RIGHT else current_score,
        )
        if not math.isfinite(other_score) or other_score < current_score + 0.45 or other_score < 0.20:
            self.pending_direction_frames = 0
            return
        if self.pending_direction != other:
            self.pending_direction = other
            self.pending_direction_frames = 1
            return
        self.pending_direction_frames += 1
        if self.pending_direction_frames >= self.direction_lock_confirm_frames:
            self.lock_direction(other)

    def _note_unknown_direction(self) -> None:
        self.pending_unknown_frames += 1
        if self.pending_unknown_frames >= 3:
            self.pending_direction = Direction.UNKNOWN
            self.pending_direction_frames = 0

    def _corner_direction_candidate(self, frame: SensorFrame) -> Direction:
        direct = self._direct_extension_direction(frame.local_grid)
        if direct != Direction.UNKNOWN:
            self._update_direction_evidence(direct, 1.0)
            other = Direction.RIGHT if direct == Direction.LEFT else Direction.LEFT
            self._update_direction_evidence(other, -0.2)
        left_raw = self._score_direction_hypothesis(frame.local_grid, Direction.LEFT)
        right_raw = self._score_direction_hypothesis(frame.local_grid, Direction.RIGHT)
        self._update_direction_evidence(Direction.LEFT, left_raw)
        self._update_direction_evidence(Direction.RIGHT, right_raw)
        left_score = self.direction_evidence[Direction.LEFT]
        right_score = self.direction_evidence[Direction.RIGHT]
        self.last_direction_scores = (left_score, right_score)
        if not math.isfinite(left_raw) and not math.isfinite(right_raw):
            return Direction.UNKNOWN
        if abs(left_score - right_score) < self.direction_lock_min_score_delta:
            return Direction.UNKNOWN
        best_score = max(left_score, right_score)
        if best_score < 0.25:
            return Direction.UNKNOWN
        return Direction.LEFT if left_score > right_score else Direction.RIGHT

    def _direct_extension_direction(self, local: LocalGrid) -> Direction:
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        corridor_half_w = self.grid.corridor_width_m * 0.5
        corridor_half_l = self.grid.corridor_width_m * 0.5
        margin = 0.08
        left_count = 0
        right_count = 0
        for row, col, value in local.observed_cells():
            if value != Cell.WALL:
                continue
            lx, ly = local.cell_to_local(row, col)
            gx = self.pose.x_m + lx * cy - ly * sy
            gy = self.pose.y_m + lx * sy + ly * cy
            if abs(gy) <= corridor_half_l + margin:
                continue
            if gx < -corridor_half_w - margin:
                left_count += 1
            elif gx > corridor_half_w + margin:
                right_count += 1
        min_count = max(3, self.front_wall_motion_min_points // 2)
        if left_count >= min_count and left_count >= right_count + 2:
            return Direction.LEFT
        if right_count >= min_count and right_count >= left_count + 2:
            return Direction.RIGHT
        return Direction.UNKNOWN

    def _update_direction_evidence(self, direction: Direction, raw_score: float) -> None:
        old = self.direction_evidence[direction]
        if not math.isfinite(raw_score):
            self.direction_evidence[direction] = old * 0.85
            return
        clipped = max(-0.4, min(raw_score, 1.0))
        self.direction_evidence[direction] = old * 0.72 + clipped * 0.28

    def _make_direction_hypothesis(self, direction: Direction) -> GridMap:
        hypothesis = GridMap(
            size_m=self.grid.spec.size_m,
            resolution_m=self.grid.resolution_m,
            corridor_width_m=self.grid.corridor_width_m,
            corridor_length_m=self.grid.corridor_length_m,
        )
        hypothesis.lock_direction(direction)
        return hypothesis

    def _score_direction_hypothesis(self, local: LocalGrid, direction: Direction) -> float:
        hypothesis = self.direction_hypotheses[direction]
        cy = math.cos(self.pose.yaw_rad)
        sy = math.sin(self.pose.yaw_rad)
        score = 0.0
        count = 0
        for row, col, value in local.observed_cells():
            if value != Cell.WALL:
                continue
            lx, ly = local.cell_to_local(row, col)
            gx = self.pose.x_m + lx * cy - ly * sy
            gy = self.pose.y_m + lx * sy + ly * cy
            idx = hypothesis.world_to_cell(gx, gy)
            if idx is None:
                count += 1
                continue
            score += self._extension_wall_match_score(hypothesis, idx[0], idx[1])
            count += 1
        if count < self.front_wall_motion_min_points:
            return -math.inf
        return score / count

    def _extension_wall_match_score(self, grid: GridMap, row: int, col: int) -> float:
        best = -0.2
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                rr = row + dr
                cc = col + dc
                if not (0 <= rr < grid.size_cells and 0 <= cc < grid.size_cells):
                    continue
                expected = Cell(int(grid.cells[rr, cc]))
                if expected in (Cell.MAP_WALL, Cell.WALL):
                    best = max(best, 1.0 - 0.16 * math.hypot(dr, dc))
        return best
