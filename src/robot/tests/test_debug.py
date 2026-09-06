from __future__ import annotations

import unittest

import numpy as np

from planner.debug import DebugFrame, DebugStyle, DebugWindow
from planner.grid import Direction, GridMap
from planner.localize_types import PoseEstimate


class DebugWindowTest(unittest.TestCase):
    def test_tracked_obstacles_render_with_associated_color(self) -> None:
        grid = GridMap()
        grid.lock_direction(Direction.RIGHT)
        window = DebugWindow(style=DebugStyle(map_size_px=300, panel_width_px=100, margin_px=20))
        obstacles = (
            ("red", (0.75, 0.75)),
            ("green", (1.00, 0.75)),
            ("yellow", (1.25, 0.75)),
        )

        image = window.render(DebugFrame(
            grid=grid,
            pose=PoseEstimate(x_m=0.0, y_m=0.0),
            colored_obstacles=obstacles,
        ))

        for label, anchor in obstacles:
            x_px, y_px = window._pix(grid, *anchor)
            np.testing.assert_array_equal(
                image[y_px, x_px],
                np.asarray(window._obstacle_colors[label], dtype=np.uint8),
            )


if __name__ == "__main__":
    unittest.main()
