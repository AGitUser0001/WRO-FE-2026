from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


CellIndex = tuple[int, int]
Key = tuple[float, float]


@dataclass
class DStarLite:
    shape: tuple[int, int]
    goal: CellIndex
    cost: np.ndarray
    g: np.ndarray = field(init=False)
    rhs: np.ndarray = field(init=False)
    queue: list[tuple[float, float, CellIndex]] = field(default_factory=list)
    queued: dict[CellIndex, Key] = field(default_factory=dict)
    reset_count: int = 0
    repair_count: int = 0
    last_changed_cells: int = 0
    last_expansions: int = 0
    max_expansions: int = 500

    def __post_init__(self) -> None:
        self.cost = self.cost.astype(np.float32, copy=True)
        self.g = np.full(self.shape, math.inf, dtype=np.float32)
        self.rhs = np.full(self.shape, math.inf, dtype=np.float32)
        self._reset_distances()

    def reset(self, goal: CellIndex, cost: np.ndarray) -> None:
        self.reset_count += 1
        self.goal = goal
        self.cost = cost.astype(np.float32, copy=True)
        self.g.fill(math.inf)
        self.rhs.fill(math.inf)
        self.queue.clear()
        self.queued.clear()
        self._reset_distances()

    def update_cost(self, cost: np.ndarray, start: CellIndex) -> None:
        changed = np.argwhere(np.abs(cost - self.cost) > 5.0)
        self.last_changed_cells = int(len(changed))
        if changed.size == 0:
            return
        if len(changed) > 64:
            self.cost = cost.astype(np.float32, copy=True)
            self.reset_count += 1
            self._reset_distances()
            return
        self.repair_count += 1
        self.cost = cost.astype(np.float32, copy=True)
        touched: set[CellIndex] = set()
        for row, col in changed.tolist():
            cell = (int(row), int(col))
            touched.add(cell)
            touched.update(self._neighbors(cell))
        for cell in touched:
            self._update_vertex(cell, start)

    def path(self, start: CellIndex) -> tuple[CellIndex, ...]:
        if not self._compute_shortest_path(start):
            self._reset_distances()
        if not math.isfinite(float(self.g[start])) and not math.isfinite(float(self.rhs[start])):
            return (start,)
        out = [start]
        current = start
        seen = {start}
        limit = self.shape[0] * self.shape[1]
        while current != self.goal and len(out) < limit:
            nxt = self._best_successor(current)
            if nxt is None or nxt in seen:
                break
            out.append(nxt)
            seen.add(nxt)
            current = nxt
        return tuple(out)

    def _reset_distances(self) -> None:
        graph = _reverse_grid_graph(self.cost)
        goal_flat = int(np.ravel_multi_index(self.goal, self.shape))
        dist = dijkstra(graph, directed=True, indices=goal_flat)
        values = np.asarray(dist, dtype=np.float32).reshape(self.shape)
        self.g[:, :] = values
        self.rhs[:, :] = values
        self.g[self.goal] = 0.0
        self.rhs[self.goal] = 0.0
        self.queue.clear()
        self.queued.clear()

    def _compute_shortest_path(self, start: CellIndex) -> bool:
        self.last_expansions = 0
        while self.queue:
            self.last_expansions += 1
            if self.last_expansions > self.max_expansions:
                return False
            top = self.queue[0]
            start_key = self._key(start, start)
            if (top[0], top[1]) >= start_key and self._consistent(start):
                return True
            _, _, u = heapq.heappop(self.queue)
            old_key = (top[0], top[1])
            if self.queued.get(u) != old_key:
                continue
            self.queued.pop(u, None)
            new_key = self._key(u, start)
            if old_key < new_key:
                self._push(u, new_key)
            elif self._consistent(u):
                continue
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for pred in self._neighbors(u):
                    self._update_vertex(pred, start)
            else:
                self.g[u] = math.inf
                self._update_vertex(u, start)
                for pred in self._neighbors(u):
                    self._update_vertex(pred, start)
        return True

    def _update_vertex(self, cell: CellIndex, start: CellIndex) -> None:
        if cell != self.goal:
            self.rhs[cell] = min(
                (self._edge_cost(cell, nxt) + float(self.g[nxt]) for nxt in self._neighbors(cell)),
                default=math.inf,
            )
        if not self._consistent(cell):
            self._push(cell, self._key(cell, start))
        else:
            self.queued.pop(cell, None)

    def _best_successor(self, cell: CellIndex) -> CellIndex | None:
        best: tuple[float, CellIndex] | None = None
        for nxt in self._neighbors(cell):
            score = self._edge_cost(cell, nxt) + float(self.g[nxt])
            if not math.isfinite(score):
                continue
            if best is None or score < best[0]:
                best = (score, nxt)
        return None if best is None else best[1]

    def _key(self, cell: CellIndex, start: CellIndex) -> Key:
        value = min(float(self.g[cell]), float(self.rhs[cell]))
        return value + self._heuristic(start, cell), value

    def _consistent(self, cell: CellIndex) -> bool:
        return abs(float(self.g[cell]) - float(self.rhs[cell])) <= 1.0e-5

    def _push(self, cell: CellIndex, key: Key) -> None:
        self.queued[cell] = key
        heapq.heappush(self.queue, (key[0], key[1], cell))

    def _edge_cost(self, a: CellIndex, b: CellIndex) -> float:
        value = float(self.cost[b])
        if value >= 1.0e8:
            return math.inf
        return value * math.hypot(a[0] - b[0], a[1] - b[1])

    def _neighbors(self, cell: CellIndex) -> tuple[CellIndex, ...]:
        row, col = cell
        out: list[CellIndex] = []
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            rr = row + dr
            cc = col + dc
            if 0 <= rr < self.shape[0] and 0 <= cc < self.shape[1]:
                out.append((rr, cc))
        return tuple(out)

    @staticmethod
    def _heuristic(a: CellIndex, b: CellIndex) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


def _reverse_grid_graph(cost: np.ndarray) -> csr_matrix:
    h, w = cost.shape
    ids = np.arange(h * w, dtype=np.int32).reshape(h, w)
    rows: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    data: list[np.ndarray] = []
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        sr = slice(max(0, -dr), min(h, h - dr))
        sc = slice(max(0, -dc), min(w, w - dc))
        tr = slice(max(0, dr), min(h, h + dr))
        tc = slice(max(0, dc), min(w, w + dc))
        edge_cost = cost[tr, tc].ravel() * math.hypot(dr, dc)
        edge_cost = np.where(edge_cost >= 1.0e8, math.inf, edge_cost).astype(np.float32)
        rows.append(ids[tr, tc].ravel())
        cols.append(ids[sr, sc].ravel())
        data.append(edge_cost)
    return csr_matrix((np.concatenate(data), (np.concatenate(rows), np.concatenate(cols))), shape=(h * w, h * w))
