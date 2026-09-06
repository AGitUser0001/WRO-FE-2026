#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np


POINT_RE = re.compile(r"([+-][0-9.]+):([+-][0-9.]+)")


def shape_score(points: list[tuple[float, float]]) -> tuple[float, float]:
    segments = [
        (b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
        if math.dist(a, b) > 1.0e-4
    ]
    if len(segments) < 2:
        return 1.0, 1.0
    direction_dots = [
        (a[0] * b[0] + a[1] * b[1]) / (math.hypot(*a) * math.hypot(*b))
        for a, b in zip(segments, segments[1:])
    ]
    length = sum(math.hypot(*segment) for segment in segments)
    displacement = max(math.dist(points[0], points[-1]), 1.0e-4)
    return min(direction_dots), length / displacement


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--field", choices=("route", "path"), default="route")
    parser.add_argument("--limit", type=int, default=30)
    args = parser.parse_args()
    field_re = re.compile(rf" {args.field}=\[([^]]*)\]")
    ranked: list[tuple[float, float, int, str]] = []
    for line_number, line in enumerate(args.log.read_text(encoding="utf-8").splitlines(), 1):
        match = field_re.search(line)
        if match is None:
            continue
        points = [(float(x), float(y)) for x, y in POINT_RE.findall(match.group(1))]
        if len(points) < 3:
            continue
        min_dot, stretch = shape_score(points)
        ranked.append((min_dot, -stretch, line_number, match.group(1)))
    for min_dot, negative_stretch, line_number, path in sorted(ranked)[: args.limit]:
        print(
            f"line={line_number} min_dot={min_dot:+.2f} "
            f"stretch={-negative_stretch:.2f} path=[{path}]"
        )
    if ranked:
        dots = np.asarray([item[0] for item in ranked])
        stretches = np.asarray([-item[1] for item in ranked])
        print(
            f"summary frames={len(ranked)} reverse={np.mean(dots < 0.0):.1%} "
            f"strong_reverse={np.mean(dots < -0.25):.1%} "
            f"stretch50={np.percentile(stretches, 50):.2f} "
            f"stretch95={np.percentile(stretches, 95):.2f}"
        )


if __name__ == "__main__":
    main()
