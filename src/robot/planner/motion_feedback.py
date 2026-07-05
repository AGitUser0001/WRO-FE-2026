from __future__ import annotations

import math
from typing import Protocol

from .localize_types import PoseEstimate
from .planner import Plan


class PlannerFeedback(Protocol):
    def note_stalled_trajectory(self, trajectory: tuple[tuple[float, float], ...]) -> None:
        ...


class MotionFeedback:
    def __init__(self) -> None:
        self.start: tuple[float, float] | None = None
        self.delta = (0.0, 0.0)
        self.stall_frames = 0
        self.block_cooldown = 0

    def begin_prediction(self, pose: PoseEstimate) -> None:
        self.start = (pose.x_m, pose.y_m)
        self.delta = (0.0, 0.0)

    def finish_prediction(self, pose: PoseEstimate) -> None:
        if self.start is None:
            return
        self.delta = (pose.x_m - self.start[0], pose.y_m - self.start[1])

    def update(
        self,
        planner: PlannerFeedback | None,
        plan: Plan | None,
        pose: PoseEstimate,
        expected_motion_m: float,
        motor_command: int,
    ) -> None:
        if planner is None or plan is None or self.start is None:
            return
        dx, dy = self.delta
        predicted = math.hypot(dx, dy)
        if expected_motion_m < 0.012 or predicted < 1.0e-4 or abs(motor_command) < 1:
            self.stall_frames = max(0, self.stall_frames - 1)
            return
        actual_dx = pose.x_m - self.start[0]
        actual_dy = pose.y_m - self.start[1]
        progress = (actual_dx * dx + actual_dy * dy) / predicted
        if progress < expected_motion_m * 0.25:
            self.stall_frames += 1
        else:
            self.stall_frames = max(0, self.stall_frames - 2)
        if self.block_cooldown > 0:
            self.block_cooldown -= 1
            return
        if self.stall_frames >= 4:
            planner.note_stalled_trajectory(plan.trajectory)
            self.block_cooldown = 5
            self.stall_frames = 2
