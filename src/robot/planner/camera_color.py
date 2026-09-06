from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from sensor_msgs.msg import Image

from .obstacle_policy import obstacle_actions


OBSTACLE_COLORS_BGR: dict[str, tuple[int, int, int]] = {
    "red": (0, 0, 220),
    "green": (0, 180, 0),
    "yellow": (55, 205, 242),
}


@dataclass(frozen=True)
class ColorObstacle:
    label: str
    actions: tuple[str, ...]
    depth_m: float
    x_norm: float


def depth_image_m(msg: Image) -> np.ndarray | None:
    if msg.height <= 0 or msg.width <= 0:
        return None
    if msg.encoding in ("32FC1", "32FC"):
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    if msg.encoding in ("16UC1", "mono16"):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
    return None


def color_obstacles(color_msg: Image, depth_m: np.ndarray) -> tuple[ColorObstacle, ...]:
    color = _bgr_image(color_msg)
    if color is None or depth_m.shape != color.shape[:2]:
        return ()
    channel_means = np.asarray(cv2.mean(color)[:3], dtype=np.float32)
    if float(np.min(channel_means)) >= 5.0:
        neutral = float(np.mean(channel_means))
        gains = np.clip(neutral / channel_means, 0.5, 2.5)
        color = np.clip(
            color.astype(np.float32) * gains.reshape(1, 1, 3),
            0.0,
            255.0,
        ).astype(np.uint8)
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    configured_actions = obstacle_actions()
    masks = (
        (
            "red",
            cv2.bitwise_or(
                cv2.inRange(hsv, np.array((0, 95, 70), dtype=np.uint8), np.array((10, 255, 255), dtype=np.uint8)),
                cv2.inRange(hsv, np.array((170, 95, 70), dtype=np.uint8), np.array((180, 255, 255), dtype=np.uint8)),
            ),
        ),
        (
            "green",
            cv2.inRange(
                hsv,
                np.array((38, 65, 55), dtype=np.uint8),
                np.array((92, 255, 255), dtype=np.uint8),
            ),
        ),
        (
            "yellow",
            cv2.inRange(
                hsv,
                np.array((18, 80, 90), dtype=np.uint8),
                np.array((36, 255, 255), dtype=np.uint8),
            ),
        ),
    )
    candidates: list[ColorObstacle] = []
    kernel = np.ones((3, 3), dtype=np.uint8)
    for label, mask in masks:
        actions = configured_actions.get(label)
        if actions is None:
            continue
        mask[: int(color.shape[0] * 0.30)] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            if cv2.contourArea(contour) < 20 or height < 4 or width < 3:
                continue
            component = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(component, [contour], -1, (255,), -1)
            for rows, cols, depth in _depth_surfaces(component, depth_m):
                surface_width = int(cols.max() - cols.min() + 1)
                surface_height = int(rows.max() - rows.min() + 1)
                physical_width = surface_width * depth / 554.25469
                physical_height = surface_height * depth / 554.25469
                if (
                    int(rows.max()) < int(color.shape[0] * 0.43)
                    or surface_height < 0.75 * surface_width
                    or max(physical_width, physical_height) < 0.025
                    or min(physical_width, physical_height) < 0.012
                    or physical_width * physical_height < 0.00040
                ):
                    continue
                candidates.append(
                    ColorObstacle(
                        label=label,
                        actions=actions,
                        depth_m=depth,
                        x_norm=((float(np.median(cols)) + 0.5) / max(color.shape[1], 1)) - 0.5,
                    )
                )
    return tuple(sorted(candidates, key=lambda candidate: candidate.depth_m))


def _depth_surfaces(component: np.ndarray, depth_m: np.ndarray) -> tuple[tuple[np.ndarray, np.ndarray, float], ...]:
    valid = (component != 0) & np.isfinite(depth_m) & (depth_m >= 0.10) & (depth_m <= 2.50)
    rows, cols = np.nonzero(valid)
    if len(rows) < 20:
        return ()
    values = depth_m[rows, cols]
    order = np.argsort(values)
    boundaries = np.nonzero(np.diff(values[order]) > 0.08)[0] + 1
    surfaces: list[tuple[np.ndarray, np.ndarray, float]] = []
    for group in np.split(order, boundaries):
        if len(group) < 20:
            continue
        group_mask = np.zeros(component.shape, dtype=np.uint8)
        group_mask[rows[group], cols[group]] = 1
        count, labels = cv2.connectedComponents(group_mask, connectivity=8)
        for index in range(1, count):
            surface_rows, surface_cols = np.nonzero(labels == index)
            if len(surface_rows) < 20:
                continue
            surfaces.append((surface_rows, surface_cols, float(np.median(depth_m[surface_rows, surface_cols]))))
    return tuple(surfaces)


def _bgr_image(msg: Image) -> np.ndarray | None:
    if msg.height <= 0 or msg.width <= 0:
        return None
    if msg.encoding not in ("bgr8", "rgb8"):
        return None
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
    image = image.reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image
