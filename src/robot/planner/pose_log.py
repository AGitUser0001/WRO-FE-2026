from __future__ import annotations

import queue
import threading


class PoseDebugLogger:
    def __init__(self) -> None:
        self.queue: queue.Queue[str] = queue.Queue(maxsize=8)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def write(self, line: str) -> None:
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(line)
            except queue.Full:
                pass

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=0.5)

    def _loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                line = self.queue.get(timeout=0.05)
            except queue.Empty:
                continue
            print(line, flush=True)
