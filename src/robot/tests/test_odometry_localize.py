from __future__ import annotations

import math
import unittest
from unittest.mock import Mock, patch

import numpy as np

from planner.grid import Direction, GridMap, LocalGrid
from planner.odometry_localize import OdometryLocalizer
from planner.sensors import SensorFrame, WallDistances


class OdometryLocalizerTest(unittest.TestCase):
    def test_twist_integrates_forward_along_imu_heading(self) -> None:
        localizer = OdometryLocalizer(GridMap())
        localizer.apply_odometry(0.20, 0.0)

        motion_m = localizer.apply_odometry(0.20, 0.5)

        self.assertAlmostEqual(motion_m, 0.10)
        self.assertAlmostEqual(localizer.pose.x_m, 0.0, places=6)
        self.assertAlmostEqual(localizer.pose.y_m, 0.10, places=6)

    def test_twist_integrates_reverse_opposite_imu_heading(self) -> None:
        localizer = OdometryLocalizer(GridMap())
        localizer.apply_odometry(-0.20, 0.0)

        motion_m = localizer.apply_odometry(-0.20, 0.5)

        self.assertAlmostEqual(motion_m, 0.10)
        self.assertAlmostEqual(localizer.pose.x_m, 0.0, places=6)
        self.assertAlmostEqual(localizer.pose.y_m, -0.10, places=6)

    def test_twist_uses_midpoint_imu_heading(self) -> None:
        localizer = OdometryLocalizer(GridMap(), start_yaw_rad=0.0)
        localizer.apply_odometry(0.20, 0.0)
        localizer.pose.yaw_rad = math.pi / 2.0

        localizer.apply_odometry(0.20, 0.5)

        expected = 0.10 / math.sqrt(2.0)
        self.assertAlmostEqual(localizer.pose.x_m, expected, places=6)
        self.assertAlmostEqual(localizer.pose.y_m, expected, places=6)

    def test_vectorized_correction_bounds_match_scalar_reference(self) -> None:
        rng = np.random.default_rng(17)
        offsets = [
            (float(x), float(y))
            for x, y in rng.uniform(-0.35, 0.35, size=(300, 2))
        ]
        for direction in Direction:
            grid = GridMap()
            if direction != Direction.UNKNOWN:
                grid.lock_direction(direction)
            localizer = OdometryLocalizer(grid)
            for _ in range(20):
                localizer._base_pose = (
                    float(rng.uniform(-2.5, 2.5)),
                    float(rng.uniform(-1.5, 1.5)),
                    0.0,
                )
                localizer.pose.yaw_rad = float(rng.uniform(-math.pi, math.pi))
                expected = tuple(
                    (x, y)
                    for x, y in offsets
                    if math.hypot(x, y) <= localizer.correction_max_m + 1.0e-9
                    and localizer._footprint_inside_navigable_bounds(
                        localizer._base_pose[0] + x,
                        localizer._base_pose[1] + y,
                    )
                )
                self.assertEqual(localizer._valid_correction_offsets(offsets), expected)

    def test_direction_candidate_survives_temporary_wall_dropout(self) -> None:
        localizer = OdometryLocalizer(GridMap())
        localizer.direction_evidence[Direction.LEFT] = 0.90
        localizer.direction_evidence[Direction.RIGHT] = 0.76
        empty = np.empty(0, dtype=np.float32)

        with patch.object(
            localizer,
            "_direction_wall_points",
            return_value=(empty, empty),
        ):
            candidate = localizer._corner_direction_candidate(Mock())

        self.assertEqual(candidate, Direction.LEFT)
        self.assertEqual(localizer.last_direction_scores, (0.90, 0.76))

    def test_wall_ends_can_correct_translation_along_wall_tangent(self) -> None:
        grid = GridMap()
        grid.lock_direction(Direction.RIGHT)
        localizer = OdometryLocalizer(grid)
        localizer.pose.odometry_motion_m = 0.02
        frame = SensorFrame(LocalGrid(), (), WallDistances())
        candidates: list[tuple[tuple[float, float], ...]] = []

        def best_shift(
            _points: object,
            _normal_axes: object,
            _grid: object,
            offsets: tuple[tuple[float, float], ...],
            **_kwargs: object,
        ) -> tuple[float, float, float, float]:
            candidates.append(offsets)
            return 0.0, 0.0, 0.10, 0.0

        horizontal_wall_points = tuple((float(index), 0.0) for index in range(6))
        with (
            patch(
                "planner.odometry_localize.scan_world_walls",
                return_value=(horizontal_wall_points, (1,) * 6),
            ),
            patch(
                "planner.odometry_localize.best_map_shift",
                side_effect=best_shift,
            ),
        ):
            localizer._apply_scan_correction(frame)

        self.assertEqual(len(candidates), 2)
        self.assertGreater(len({x_m for x_m, _y_m in candidates[0]}), 1)


if __name__ == "__main__":
    unittest.main()
