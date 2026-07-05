from __future__ import annotations

import math


def sampled_path(points: tuple[tuple[float, float], ...], count: int = 6) -> str:
    if not points:
        return "[]"
    count = max(2, min(count, len(points)))
    last = len(points) - 1
    indices = tuple(round(last * i / (count - 1)) for i in range(count))
    samples = ",".join(f"{points[i][0]:+.2f}:{points[i][1]:+.2f}" for i in indices)
    return f"[{samples}]"


def path_change(a: tuple[tuple[float, float], ...], b: tuple[tuple[float, float], ...]) -> float:
    count = min(len(a), len(b), 12)
    if count <= 1:
        return 0.0
    return sum(math.hypot(pa[0] - pb[0], pa[1] - pb[1]) for pa, pb in zip(a[:count], b[:count])) / count
