#!/usr/bin/env python3

from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time

import Jetson.GPIO as GPIO


BUTTON_PIN = int(os.environ.get("WRO_BUTTON_PIN", "29"))
DEBOUNCE_MS = int(os.environ.get("WRO_BUTTON_DEBOUNCE_MS", "250"))
MICRO_ROS_PORT = os.environ.get("MICRO_ROS_PORT", "8888")
DRIVE_MOTOR = os.environ.get("WRO_DRIVE_MOTOR", "100")
DEBUG_VIEW = os.environ.get("WRO_DEBUG_VIEW", "false")


class RobotAutostart:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._agent: subprocess.Popen[bytes] | None = None
        self._planner: subprocess.Popen[bytes] | None = None
        self._ready_sound_sent = False

    @staticmethod
    def _start(command: list[str]) -> subprocess.Popen[bytes]:
        return subprocess.Popen(command, start_new_session=True)

    @staticmethod
    def _stop_process(process: subprocess.Popen[bytes] | None) -> None:
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()

    def start_agent(self) -> None:
        self._agent = self._start(
            ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "udp4", "--port", MICRO_ROS_PORT]
        )
        logging.info("Started micro-ROS agent on UDP port %s (pid=%s).", MICRO_ROS_PORT, self._agent.pid)

    def on_press(self, _channel: int) -> None:
        with self._lock:
            if self._planner is not None and self._planner.poll() is None:
                logging.info("Button press ignored: planner is already running.")
                return
            self._planner = self._start(
                [
                    "ros2",
                    "launch",
                    "robot",
                    "planner.launch.py",
                    "start_micro_ros:=false",
                    "auto_drive_enabled:=true",
                    f"drive_motor:={DRIVE_MOTOR}",
                    f"debug_view_enabled:={DEBUG_VIEW}",
                    "sim:=false",
                ]
            )
            logging.info("Button press started the planner (pid=%s).", self._planner.pid)

    def _try_ready_sound(self) -> None:
        if self._ready_sound_sent:
            return
        result = subprocess.run(
            ["ros2", "topic", "info", "/microROS/audio_play"],
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )
        subscription_count = 0
        for line in result.stdout.splitlines():
            label, separator, value = line.strip().partition(":")
            if separator and label == "Subscription count":
                try:
                    subscription_count = int(value.strip())
                except ValueError:
                    pass
                break
        if subscription_count < 1:
            return
        subprocess.run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/microROS/audio_play",
                "std_msgs/msg/Int32MultiArray",
                "{data: [660, 500]}",
            ],
            check=False,
            timeout=5,
        )
        self._ready_sound_sent = True
        logging.info("ESP32 connected; played the ready sound.")

    def run(self) -> None:
        self.start_agent()
        GPIO.setwarnings(True)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BUTTON_PIN, GPIO.IN)
        GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=self.on_press, bouncetime=DEBOUNCE_MS)
        logging.info("Waiting for an active-low button press on physical pin %s.", BUTTON_PIN)
        try:
            while not self._stop.wait(1.0):
                if self._agent is None or self._agent.poll() is not None:
                    logging.warning("micro-ROS agent exited; restarting it.")
                    time.sleep(1.0)
                    self.start_agent()
                    self._ready_sound_sent = False
                try:
                    self._try_ready_sound()
                except (OSError, subprocess.SubprocessError) as error:
                    logging.debug("Ready-sound check failed: %s", error)
                with self._lock:
                    if self._planner is not None and self._planner.poll() is not None:
                        logging.info("Planner exited with status %s.", self._planner.returncode)
                        self._planner = None
        finally:
            GPIO.remove_event_detect(BUTTON_PIN)
            GPIO.cleanup(BUTTON_PIN)
            self._stop_process(self._planner)
            self._stop_process(self._agent)

    def stop(self, _signum: int, _frame: object) -> None:
        self._stop.set()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    app = RobotAutostart()
    signal.signal(signal.SIGTERM, app.stop)
    signal.signal(signal.SIGINT, app.stop)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
