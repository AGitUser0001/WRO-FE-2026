from __future__ import annotations

import unittest

import numpy as np
from sensor_msgs.msg import Image

from planner.camera_color import color_obstacles


def _color_message(image: np.ndarray) -> Image:
    message = Image()
    message.height, message.width = image.shape[:2]
    message.encoding = "bgr8"
    message.step = message.width * 3
    message.data = image.tobytes()
    return message


class ColorObstacleTest(unittest.TestCase):
    def test_accepts_partially_visible_obstacle_face(self) -> None:
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        image[70:87, 70:81] = (0, 0, 255)
        depth = np.full((120, 160), np.nan, dtype=np.float32)
        depth[70:87, 70:81] = 1.0

        obstacles = color_obstacles(_color_message(image), depth)

        self.assertEqual([obstacle.label for obstacle in obstacles], ["red"])

    def test_rejects_small_color_speck(self) -> None:
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        image[70:76, 70:76] = (0, 255, 0)
        depth = np.full((120, 160), np.nan, dtype=np.float32)
        depth[70:76, 70:76] = 1.0

        self.assertEqual(color_obstacles(_color_message(image), depth), ())

    def test_rejects_horizontal_colored_floor_stripe(self) -> None:
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        image[80:90, 50:100] = (55, 205, 242)
        depth = np.full((120, 160), np.nan, dtype=np.float32)
        depth[80:90, 50:100] = 0.7

        self.assertEqual(color_obstacles(_color_message(image), depth), ())

    def test_white_balance_rejects_warm_neutral_patch(self) -> None:
        image = np.full((120, 160, 3), (10, 40, 45), dtype=np.uint8)
        image[60:110, 70:90] = (20, 100, 110)
        depth = np.full((120, 160), np.nan, dtype=np.float32)
        depth[60:110, 70:90] = 1.0

        self.assertEqual(color_obstacles(_color_message(image), depth), ())

    def test_rejects_colored_patch_above_course_surface(self) -> None:
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        image[38:49, 70:78] = (55, 205, 242)
        depth = np.full((120, 160), np.nan, dtype=np.float32)
        depth[38:49, 70:78] = 1.5

        self.assertEqual(color_obstacles(_color_message(image), depth), ())

    def test_accepts_upright_yellow_obstacle_after_white_balance(self) -> None:
        image = np.full((120, 160, 3), 100, dtype=np.uint8)
        image[65:95, 70:86] = (55, 205, 242)
        depth = np.full((120, 160), np.nan, dtype=np.float32)
        depth[65:95, 70:86] = 1.0

        obstacles = color_obstacles(_color_message(image), depth)

        self.assertEqual([obstacle.label for obstacle in obstacles], ["yellow"])


if __name__ == "__main__":
    unittest.main()
