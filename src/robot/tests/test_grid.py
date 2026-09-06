from __future__ import annotations

import unittest

import numpy as np

from planner.grid import Cell, Direction, GridMap, inscribed_obstacle_mask


class GridMapTest(unittest.TestCase):
    def test_cropped_score_refresh_matches_full_grid_reference(self) -> None:
        grid = GridMap()
        grid.lock_direction(Direction.RIGHT)
        rng = np.random.default_rng(42)
        dynamic = grid.cells != int(Cell.MAP_WALL)
        grid.wall_score[dynamic] = rng.random(np.count_nonzero(dynamic), dtype=np.float32)
        grid.unknown_score[dynamic] = rng.random(np.count_nonzero(dynamic), dtype=np.float32)

        expected_cells = grid.cells.copy()
        expected_wall = grid.wall_score.copy()
        expected_unknown = grid.unknown_score.copy()
        physical_inside = (
            (grid._world_y >= -1.5)
            & (grid._world_y <= 1.5)
            & (grid._world_x >= -0.5)
            & (grid._world_x <= 2.5)
            & ~(
                (grid._world_x > 0.5)
                & (grid._world_x < 1.5)
                & (grid._world_y > -0.5)
                & (grid._world_y < 0.5)
            )
        )
        expected_wall[~physical_inside] = 0.0
        expected_unknown[~physical_inside] = 0.0
        cleared = (
            (expected_cells != int(Cell.MAP_WALL))
            & (expected_wall <= 0.0)
            & (expected_unknown <= 0.0)
        )
        expected_cells[cleared] = int(Cell.FREE)
        refreshable = expected_cells != int(Cell.MAP_WALL)
        wall_mask = refreshable & (expected_wall >= 0.55)
        obstacle_mask = inscribed_obstacle_mask(
            refreshable & (expected_unknown >= 0.55) & ~wall_mask,
        )
        expected_cells[refreshable & ~wall_mask & ~obstacle_mask] = int(Cell.FREE)
        expected_cells[wall_mask] = int(Cell.WALL)
        expected_cells[obstacle_mask] = int(Cell.UNKNOWN_OBSTRUCTION)

        grid._refresh_cells_from_scores()

        np.testing.assert_array_equal(grid.cells, expected_cells)
        np.testing.assert_array_equal(grid.wall_score, expected_wall)
        np.testing.assert_array_equal(grid.unknown_score, expected_unknown)

    def test_physical_region_cache_tracks_direction(self) -> None:
        grid = GridMap()
        unknown_mask, unknown_bounds = grid._physical_map_region()

        grid.lock_direction(Direction.RIGHT)
        right_mask, right_bounds = grid._physical_map_region()

        self.assertFalse(np.array_equal(unknown_mask, right_mask))
        self.assertNotEqual(unknown_bounds, right_bounds)
        right_cell = grid.world_to_cell(2.0, 1.0)
        left_cell = grid.world_to_cell(-2.0, 1.0)
        assert right_cell is not None and left_cell is not None
        self.assertTrue(right_mask[right_cell])
        self.assertFalse(right_mask[left_cell])


if __name__ == "__main__":
    unittest.main()
