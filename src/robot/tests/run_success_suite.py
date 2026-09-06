#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
import ctypes
import json
import math
import os
import re
import selectors
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path


START_RE = re.compile(r"SIM_START .*expected_direction=(LEFT|RIGHT)")
OBSTACLES_RE = re.compile(r"SIM_OBSTACLES count=(\d+)")
OBSTACLE_RE = re.compile(
    r"SIM_OBSTACLE color=(\w+) x=([+-][\d.]+) y=([+-][\d.]+)"
)
STAMP_RE = re.compile(r"\[INFO\] \[([\d.]+)\].*SIM_COLLISION")
POSE_RE = re.compile(
    r"POSE_DEBUG loc=\(([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)deg\).*?dir=(\w+).*?"
    r"ms=([\d.]+)/([\d.]+)/([\d.]+).*?"
    r"target=(none|[+-][\d.]+:[+-][\d.]+).*?"
    r"connected=(\d).*?"
    r"dist=([\d.]+)"
)
YAW_ERROR_RE = re.compile(r"err=\([^,]+,[^,]+,([+-][\d.]+)deg\)")
LAPS_RE = re.compile(r"\blaps=(\d+)/(\d+)")
COLOR_RE = re.compile(r"\bcolor=([^ ]+)")
TRUE_POSITION_RE = re.compile(r"\btrue=\(([+-][\d.]+),([+-][\d.]+),")
PLAN_MOTION_RE = re.compile(r"\bplan=([^ ]+) motion=(fwd|rev)")
PATH_QUALITY_RE = re.compile(r"\bpathk=([\d.]+) pathclear=([\d.]+)")
TRACKING_RE = re.compile(r"\btracking=([^ ]+)")
TRACKED_ITEM_RE = re.compile(
    r"(\w+)@([+-][\d.]+):([+-][\d.]+)/[^;]+"
)


def _prepare_child_process() -> None:
    os.setsid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGINT) != 0:
        os._exit(127)
    if os.getppid() == 1:
        os.kill(os.getpid(), signal.SIGINT)


def _stop_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    for stop_signal, timeout_s in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0)):
        try:
            os.killpg(process.pid, stop_signal)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=3.0)


def _interrupt(_signum: int, _frame: object) -> None:
    raise KeyboardInterrupt


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.inf
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


@dataclass
class RunResult:
    seed: int
    passed: bool
    reason: str
    expected_direction: str
    locked_direction: str
    collisions: int
    expected_obstacles: int
    tracked_obstacles: int
    color_association_errors: int
    minimum_lap_tracked_obstacles: int
    target_changes: int
    route_motion_switches: int
    max_target_dwell_frames: int
    pose_frames: int
    connected_fraction: float
    localization_p95_m: float
    localization_max_m: float
    yaw_error_p95_deg: float
    lap_return_max_longitudinal_m: float
    lap_return_max_lateral_m: float
    plan_p50_ms: float
    plan_p95_ms: float
    path_curvature_p95_inv_m: float
    path_curvature_violation_fraction: float
    path_clearance_p05_m: float
    wall_time_s: float
    log_path: str


class RunMetrics:
    def __init__(self, seed: int, laps: int) -> None:
        self.seed = seed
        self.required_target_changes = max(1, laps) * 4
        self.expected_direction = ""
        self.expected_obstacles = 0
        self.locked_direction = ""
        self.collisions = 0
        self.last_collision_stamp = -math.inf
        self.targets: list[str] = []
        self.route_motion_switches = 0
        self.last_route_motion = ""
        self.target_dwell_frames = 0
        self.max_target_dwell_frames = 0
        self.last_target = ""
        self.target_progress_position: tuple[float, float] | None = None
        self.distances: list[float] = []
        self.yaw_errors: list[float] = []
        self.plan_ms: list[float] = []
        self.path_curvatures: list[float] = []
        self.path_clearances: list[float] = []
        self.connected: list[int] = []
        self.completed_laps = 0
        self.start_pose: tuple[float, float, float] | None = None
        self.lap_return_longitudinal: list[float] = []
        self.lap_return_lateral: list[float] = []
        self.tracked_obstacles = 0
        self.expected_obstacle_manifest: list[tuple[str, float, float]] = []
        self.unexpected_obstacle_colors: set[str] = set()
        self.color_association_errors: set[tuple[str, str, int, int]] = set()
        self.lap_tracked_obstacles: list[int] = []

    def consume(self, line: str) -> None:
        start = START_RE.search(line)
        if start:
            self.expected_direction = start.group(1)
        obstacles = OBSTACLES_RE.search(line)
        if obstacles:
            self.expected_obstacles = int(obstacles.group(1))
        obstacle = OBSTACLE_RE.search(line)
        if obstacle:
            label_name, x_m, y_m = obstacle.groups()
            if label_name not in ("red", "green"):
                self.unexpected_obstacle_colors.add(label_name)
            self.expected_obstacle_manifest.append(
                (label_name, float(x_m), float(y_m)),
            )
        if "SIM_COLLISION" in line:
            stamp = STAMP_RE.search(line)
            collision_stamp = float(stamp.group(1)) if stamp else self.last_collision_stamp + 3.0
            if collision_stamp - self.last_collision_stamp > 2.0:
                self.collisions += 1
            self.last_collision_stamp = collision_stamp
        pose = POSE_RE.search(line)
        if not pose:
            return
        x_m, y_m, yaw_deg, direction, _scan_ms, _localize_ms, plan_ms, target, connected, distance = pose.groups()
        position = (float(x_m), float(y_m))
        true_position = TRUE_POSITION_RE.search(line)
        physical_position = (
            (float(true_position.group(1)), float(true_position.group(2)))
            if true_position is not None
            else position
        )
        if self.start_pose is None:
            self.start_pose = (*physical_position, math.radians(float(yaw_deg)))
        if direction != "UNKNOWN" and not self.locked_direction:
            self.locked_direction = direction
        if target != "none" and (not self.targets or target != self.targets[-1]):
            self.targets.append(target)
        plan_motion = PLAN_MOTION_RE.search(line)
        if plan_motion:
            reason, motion = plan_motion.groups()
            if reason in ("route", "corridor"):
                progressed = (
                    self.target_progress_position is not None
                    and math.hypot(
                        physical_position[0] - self.target_progress_position[0],
                        physical_position[1] - self.target_progress_position[1],
                    ) >= 0.05
                )
                if target == self.last_target and not progressed:
                    self.target_dwell_frames += 1
                else:
                    self.last_target = target
                    self.target_dwell_frames = 1
                    self.target_progress_position = physical_position
                self.max_target_dwell_frames = max(
                    self.max_target_dwell_frames,
                    self.target_dwell_frames,
                )
            else:
                self.last_target = ""
                self.target_dwell_frames = 0
                self.target_progress_position = None
            if reason == "route":
                if " unknown=backup " in line:
                    self.last_route_motion = ""
                else:
                    if self.last_route_motion and motion != self.last_route_motion:
                        self.route_motion_switches += 1
                    self.last_route_motion = motion
            else:
                self.last_route_motion = ""
        self.plan_ms.append(float(plan_ms))
        path_quality = PATH_QUALITY_RE.search(line)
        if (
            path_quality
            and plan_motion
            and plan_motion.group(1) in ("route", "corridor")
        ):
            curvature, clearance = path_quality.groups()
            self.path_curvatures.append(float(curvature))
            self.path_clearances.append(float(clearance))
        self.connected.append(int(connected))
        self.distances.append(float(distance))
        color = COLOR_RE.search(line)
        current_tracked = 0
        if color and color.group(1) != "none":
            current_tracked = len(color.group(1).split(","))
            self.tracked_obstacles = max(self.tracked_obstacles, current_tracked)
        tracking = TRACKING_RE.search(line)
        if tracking and self.expected_obstacle_manifest:
            for tracked in TRACKED_ITEM_RE.finditer(tracking.group(1)):
                label_name, x_m, y_m = tracked.groups()
                x_value, y_value = float(x_m), float(y_m)
                expected_label, expected_x, expected_y = min(
                    self.expected_obstacle_manifest,
                    key=lambda item: math.hypot(x_value - item[1], y_value - item[2]),
                )
                if (
                    math.hypot(x_value - expected_x, y_value - expected_y) <= 0.30
                    and label_name != expected_label
                ):
                    self.color_association_errors.add((
                        label_name,
                        expected_label,
                        int(round(expected_x * 100)),
                        int(round(expected_y * 100)),
                    ))
        laps = LAPS_RE.search(line)
        if laps:
            completed = int(laps.group(1))
            if completed > self.completed_laps:
                start_x, start_y, start_yaw = self.start_pose or (0.0, 0.0, 0.0)
                dx, dy = physical_position[0] - start_x, physical_position[1] - start_y
                return_lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
                count = completed - self.completed_laps
                self.lap_return_longitudinal.extend([0.0] * count)
                self.lap_return_lateral.extend(
                    [abs(return_lateral)] * count
                )
                self.lap_tracked_obstacles.extend(
                    [current_tracked] * count
                )
                self.completed_laps = completed
        yaw = YAW_ERROR_RE.search(line)
        if yaw:
            self.yaw_errors.append(abs(float(yaw.group(1))))

    def result(self, elapsed_s: float, log_path: Path) -> RunResult:
        failures: list[str] = []
        target_changes = max(0, len(self.targets) - 1)
        connected_fraction = statistics.fmean(self.connected) if self.connected else 0.0
        localization_p95 = percentile(self.distances, 0.95)
        yaw_p95 = percentile(self.yaw_errors, 0.95)
        if self.collisions:
            failures.append(f"{self.collisions} collision(s)")
        if self.unexpected_obstacle_colors:
            failures.append(
                "unexpected simulator obstacle color(s): "
                + ", ".join(sorted(self.unexpected_obstacle_colors))
            )
        if self.color_association_errors:
            failures.append(
                f"{len(self.color_association_errors)} wrong color association(s)"
            )
        if self.expected_obstacles <= 0:
            failures.append("no obstacle manifest")
        elif self.tracked_obstacles < self.expected_obstacles:
            failures.append(
                f"tracked {self.tracked_obstacles}/{self.expected_obstacles} obstacles"
            )
        elif self.tracked_obstacles > self.expected_obstacles:
            failures.append(
                f"over-associated {self.tracked_obstacles}/{self.expected_obstacles} obstacles"
            )
        elif (
            self.lap_tracked_obstacles
            and min(self.lap_tracked_obstacles) < self.expected_obstacles
        ):
            failures.append(
                "lap completed before all obstacle colors were known"
            )
        if not self.locked_direction:
            failures.append("no direction lock")
        elif self.expected_direction and self.locked_direction != self.expected_direction:
            failures.append(f"wrong direction {self.locked_direction}")
        required_laps = self.required_target_changes // 4
        if self.completed_laps < required_laps:
            failures.append(
                f"incomplete run ({self.completed_laps}/{required_laps} laps)"
            )
        elif any(distance > 0.15 for distance in self.lap_return_longitudinal):
            failures.append("lap completed away from start line")
        elif any(distance > 0.50 for distance in self.lap_return_lateral):
            failures.append("lap completed outside start section")
        if len(self.distances) < 20:
            failures.append("insufficient pose telemetry")
        elif localization_p95 > 0.15:
            failures.append(f"localization p95 {localization_p95:.3f}m")
        if connected_fraction < 0.90:
            failures.append(f"route connected {connected_fraction:.1%}")
        if self.route_motion_switches > 20 * required_laps:
            failures.append(
                f"route motion switched {self.route_motion_switches} times"
            )
        if self.max_target_dwell_frames > 180:
            failures.append(
                f"target stalled for {self.max_target_dwell_frames} frames"
            )
        return RunResult(
            seed=self.seed,
            passed=not failures,
            reason="; ".join(failures) if failures else f"complete {self.required_target_changes // 4} laps",
            expected_direction=self.expected_direction,
            locked_direction=self.locked_direction,
            collisions=self.collisions,
            expected_obstacles=self.expected_obstacles,
            tracked_obstacles=self.tracked_obstacles,
            color_association_errors=len(self.color_association_errors),
            minimum_lap_tracked_obstacles=min(self.lap_tracked_obstacles, default=0),
            target_changes=target_changes,
            route_motion_switches=self.route_motion_switches,
            max_target_dwell_frames=self.max_target_dwell_frames,
            pose_frames=len(self.distances),
            connected_fraction=connected_fraction,
            localization_p95_m=localization_p95,
            localization_max_m=max(self.distances, default=math.inf),
            yaw_error_p95_deg=yaw_p95,
            lap_return_max_longitudinal_m=max(self.lap_return_longitudinal, default=math.inf),
            lap_return_max_lateral_m=max(self.lap_return_lateral, default=math.inf),
            plan_p50_ms=percentile(self.plan_ms, 0.50),
            plan_p95_ms=percentile(self.plan_ms, 0.95),
            path_curvature_p95_inv_m=percentile(self.path_curvatures, 0.95),
            path_curvature_violation_fraction=(
                sum(value > 3.22 for value in self.path_curvatures)
                / len(self.path_curvatures)
                if self.path_curvatures else 0.0
            ),
            path_clearance_p05_m=percentile(self.path_clearances, 0.05),
            wall_time_s=elapsed_s,
            log_path=str(log_path),
        )


def run_seed(
    repo: Path,
    seed: int,
    duration_s: float,
    output_dir: Path,
    show_views: bool,
    laps: int,
    run_number: int = 0,
    drive_motor: int = 150,
) -> RunResult:
    suffix = f"-run-{run_number}" if run_number else ""
    log_path = output_dir / f"seed-{seed}{suffix}.log"
    view_value = "true" if show_views else "false"
    command = (
        "source install/setup.bash && "
        "exec ros2 launch robot planner.launch.py "
        f"auto_drive_enabled:=true drive_motor:={drive_motor} "
        f"debug_view_enabled:={view_value} sim_view_enabled:={view_value} "
        "sim:=true pose_debug:=true "
        f"course_laps:={laps} "
        f"simulator_seed:={seed}"
    )
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    process = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=repo,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=_prepare_child_process,
    )
    metrics = RunMetrics(seed, laps)
    started = time.monotonic()
    with log_path.open("w", encoding="utf-8", buffering=1) as log:
        assert process.stdout is not None
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ)
        try:
            while time.monotonic() - started < duration_s:
                remaining_s = duration_s - (time.monotonic() - started)
                if not selector.select(timeout=max(0.0, min(remaining_s, 0.1))):
                    if process.poll() is not None:
                        break
                    continue
                line = process.stdout.readline()
                if not line:
                    if process.poll() is not None:
                        break
                    continue
                log.write(line)
                metrics.consume(line)
                if metrics.completed_laps >= laps:
                    break
        finally:
            selector.close()
            _stop_process_group(process)
    return metrics.result(time.monotonic() - started, log_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run seeded simulated lap regressions.")
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--drive-motor", type=int, default=150)
    parser.add_argument("--output", type=Path, default=Path("/tmp/robot-success-suite"))
    parser.add_argument(
        "--headless",
        action="store_true",
        help="disable planner debug and simulator windows for unattended batches",
    )
    return parser.parse_args()


def main() -> int:
    signal.signal(signal.SIGTERM, _interrupt)
    signal.signal(signal.SIGHUP, _interrupt)
    args = parse_args()
    repo = Path(__file__).resolve().parents[2]
    seeds = [int(value) for value in args.seeds.split(",") if value.strip()]
    seed_counts = Counter(seeds)
    seen: Counter[int] = Counter()
    args.output.mkdir(parents=True, exist_ok=True)
    results: list[RunResult] = []
    for seed in seeds:
        seen[seed] += 1
        print(f"[seed {seed}] running for {args.duration:.0f}s", flush=True)
        result = run_seed(
            repo,
            seed,
            args.duration,
            args.output,
            show_views=not args.headless,
            laps=args.laps,
            run_number=seen[seed] if seed_counts[seed] > 1 else 0,
            drive_motor=args.drive_motor,
        )
        results.append(result)
        status = "PASS" if result.passed else "FAIL"
        print(
            f"[seed {seed}] {status}: {result.reason}; "
            f"loc95={result.localization_p95_m:.3f}m "
            f"connected={result.connected_fraction:.1%} "
            f"plan95={result.plan_p95_ms:.1f}ms "
            f"pathk95={result.path_curvature_p95_inv_m:.1f}/m "
            f"overk={result.path_curvature_violation_fraction:.1%} "
            f"clear05={result.path_clearance_p05_m:.2f}m "
            f"tracked={result.tracked_obstacles}/{result.expected_obstacles}",
            flush=True,
        )
    summary_path = args.output / "summary.json"
    summary_path.write_text(
        json.dumps([asdict(result) for result in results], indent=2) + "\n",
        encoding="utf-8",
    )
    passed = sum(result.passed for result in results)
    print(f"success={passed}/{len(results)} ({passed / len(results):.0%}) summary={summary_path}")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
