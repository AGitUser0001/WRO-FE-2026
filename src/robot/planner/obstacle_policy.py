from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path


ObstacleActions = dict[str, tuple[str, ...]]
_KNOWN_ACTIONS = frozenset(("left", "right", "u_turn"))
ACTIVE_OBSTACLE_CLUSTER_RADIUS_M = 0.30
TRACKED_OBSTACLE_ASSOCIATION_M = 0.28


def _policy_path() -> Path:
    source_path = Path(__file__).resolve().parents[1] / "config" / "obstacle_navigation.json"
    if source_path.is_file():
        return source_path
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("robot")) / "config" / "obstacle_navigation.json"


@lru_cache(maxsize=1)
def obstacle_actions() -> ObstacleActions:
    raw = json.loads(_policy_path().read_text(encoding="ascii"))
    policy = {
        str(color): tuple(str(action) for action in actions)
        for color, actions in dict(raw).items()
    }
    for color, actions in policy.items():
        if not actions:
            raise ValueError(f"configured obstacle color {color!r} has no actions")
        unknown = set(actions) - _KNOWN_ACTIONS
        if unknown:
            raise ValueError(
                f"configured obstacle color {color!r} has unknown actions: {sorted(unknown)}"
            )
    return policy
