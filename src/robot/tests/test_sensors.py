from __future__ import annotations

import math
import unittest
from unittest.mock import patch

import numpy as np
from sensor_msgs.msg import LaserScan

from planner.grid import Cell, LocalGrid
from planner.sensors import (
    FrontWallEstimate, ScanPoint, SideWallEstimate, _supported_endpoints,
    local_grid_from_scan, mark_wall_runs, scan_points_from_msg,
)


def scalar_scan_points(
    msg: LaserScan,
    max_range_m: float = 2.2,
    min_range_m: float = 0.05,
    stride: int = 1,
) -> tuple[ScanPoint, ...]:
    low = max(msg.range_min, min_range_m)
    high = min(msg.range_max, max_range_m)
    result = []
    for index in range(0, len(msg.ranges), max(1, stride)):
        distance = float(msg.ranges[index])
        if not math.isfinite(distance) or distance < low or distance > high:
            continue
        angle = msg.angle_min + index * msg.angle_increment
        result.append(ScanPoint(distance * math.cos(angle), distance * math.sin(angle), distance, angle))
    return tuple(result)


def scalar_supported_endpoints(local, points, size_m):
    cells = {}
    for point in points:
        if abs(point.x_m) > size_m * 0.5 or abs(point.y_m) > size_m * 0.5:
            continue
        cell = local.local_to_cell(point.x_m, point.y_m)
        if cell is None:
            continue
        sx, sy, count = cells.get(cell, (0.0, 0.0, 0))
        cells[cell] = sx + point.x_m, sy + point.y_m, count + 1
    output = []
    for (row, col), (sx, sy, count) in cells.items():
        support = sum(cells.get((row + dr, col + dc), (0, 0, 0))[2]
                      for dr in (-1, 0, 1) for dc in (-1, 0, 1))
        if support >= 2:
            output.append((sx / count, sy / count, Cell.UNKNOWN_OBSTRUCTION))
    return tuple(output)


def scalar_wall_runs(local, points, front, left, right):
    for valid, matches in (
        (front.valid, lambda p: p.x_m >= 0 and abs(p.x_m - (front.distance_m + front.slope * p.y_m)) <= .10),
        (left.valid, lambda p: p.y_m >= 0 and abs(p.y_m - (left.distance_m + left.slope * p.x_m)) <= .10),
        (right.valid, lambda p: p.y_m <= 0 and abs(p.y_m - (-right.distance_m + right.slope * p.x_m)) <= .10),
    ):
        if not valid:
            continue
        run = []
        gap = 0
        for point in (*points, None, None, None):
            if point is not None and matches(point):
                run.append(point)
                gap = 0
                continue
            gap += 1
            if gap <= 2:
                continue
            if len(run) >= 6 and math.hypot(run[-1].x_m - run[0].x_m, run[-1].y_m - run[0].y_m) >= .22:
                local.mark_scan_points(((p.x_m, p.y_m) for p in run), Cell.WALL)
            run = []
            gap = 0


class ScanPointsTest(unittest.TestCase):
    def make_scan(self) -> LaserScan:
        msg = LaserScan()
        msg.angle_min = -math.pi
        msg.angle_increment = 0.001
        msg.range_min = 0.1
        msg.range_max = 3.0
        rng = np.random.default_rng(42)
        ranges = rng.uniform(0.05, 3.1, 6284).astype(np.float32)
        ranges[::31] = np.inf
        ranges[::57] = np.nan
        ranges[::83] = -np.inf
        msg.ranges = ranges.tolist()
        return msg

    def test_vectorized_conversion_matches_scalar_filter_and_order(self) -> None:
        msg = self.make_scan()
        for stride in (0, 1, 3, 9000):
            with self.subTest(stride=stride):
                actual = scan_points_from_msg(msg, stride=stride)
                expected = scalar_scan_points(msg, stride=stride)
                self.assertEqual(len(actual), len(expected))
                for point, reference in zip(actual, expected):
                    self.assertEqual(point.range_m, reference.range_m)
                    self.assertEqual(point.angle_rad, reference.angle_rad)
                    self.assertAlmostEqual(point.x_m, reference.x_m, places=12)
                    self.assertAlmostEqual(point.y_m, reference.y_m, places=12)

    def test_empty_scan_and_range_boundaries(self) -> None:
        msg = self.make_scan()
        msg.ranges = []
        self.assertEqual(scan_points_from_msg(msg), ())
        msg.range_min = 0.125
        msg.range_max = 2.0
        msg.ranges = [0.0625, 0.125, 1.0, 2.0, 2.5]
        self.assertEqual([point.range_m for point in scan_points_from_msg(msg)], [0.125, 1.0, 2.0])

    def test_grid_classification_matches_scalar_conversion(self) -> None:
        msg = self.make_scan()
        actual = local_grid_from_scan(msg, size_m=2.4)
        with patch("planner.sensors.scan_points_from_msg", side_effect=scalar_scan_points), patch(
            "planner.sensors._supported_endpoints", side_effect=scalar_supported_endpoints,
        ), patch(
            "planner.sensors.mark_wall_runs", side_effect=scalar_wall_runs,
        ):
            expected = local_grid_from_scan(msg, size_m=2.4)
        np.testing.assert_array_equal(actual.local_grid.cells, expected.local_grid.cells)
        np.testing.assert_array_equal(actual.local_grid.observed, expected.local_grid.observed)

    def test_endpoint_support_counts_neighbors_and_preserves_centroids(self) -> None:
        local = LocalGrid(size_m=2.4)
        points = tuple(ScanPoint(x, y, math.hypot(x, y), math.atan2(y, x)) for x, y in (
            (0.30, 0.10), (0.31, 0.11), (0.35, 0.10),
            (-1.20, 0.0), (-1.20, 0.05), (0.8, 0.8), (1.3, 1.3),
        ))
        actual = sorted(_supported_endpoints(local, points, 2.4))
        expected = sorted(scalar_supported_endpoints(local, points, 2.4))
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
        self.assertEqual(_supported_endpoints(local, (), 2.4), ())
        self.assertEqual(_supported_endpoints(local, points[-1:], 2.4), ())

    def test_wall_runs_keep_point_count_span_and_gap_thresholds(self) -> None:
        wall = FrontWallEstimate(distance_m=0.5, valid=True)
        for count, span, gap, expected in (
            (5, .30, 0, False), (6, .219, 0, False), (6, .22, 0, True),
            (6, .30, 1, True), (6, .30, 2, True), (6, .30, 3, False),
        ):
            with self.subTest(count=count, span=span, gap=gap):
                xy = [(0.5, float(y)) for y in np.linspace(0, span, count)]
                xy[count // 2:count // 2] = [(0.0, -0.5)] * gap
                points = tuple(ScanPoint(x, y, math.hypot(x, y), math.atan2(y, x)) for x, y in xy)
                local = LocalGrid()
                mark_wall_runs(local, points, wall, SideWallEstimate(), SideWallEstimate())
                self.assertEqual(bool((local.cells == int(Cell.WALL)).any()), expected)

    def test_wall_runs_match_scalar_with_both_sides_and_overlapping_lines(self) -> None:
        rng = np.random.default_rng(41)
        for count in (0, 5, 6, 800):
            for _ in range(10):
                xy = rng.uniform(-1.4, 1.4, (count, 2))
                if count:
                    xy[:count // 2, 0] = 0.5 + rng.uniform(-.12, .12, count // 2)
                points = tuple(ScanPoint(float(x), float(y), math.hypot(x, y), math.atan2(y, x)) for x, y in xy)
                estimates = (
                    FrontWallEstimate(distance_m=.5, slope=.05, valid=True),
                    SideWallEstimate(distance_m=.5, slope=.10, valid=True),
                    SideWallEstimate(distance_m=.5, slope=-.10, valid=True),
                )
                actual, expected = LocalGrid(), LocalGrid()
                mark_wall_runs(actual, points, *estimates)
                scalar_wall_runs(expected, points, *estimates)
                np.testing.assert_array_equal(actual.cells, expected.cells)
                np.testing.assert_array_equal(actual.observed, expected.observed)


if __name__ == "__main__":
    unittest.main()
