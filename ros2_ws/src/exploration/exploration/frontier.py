"""Frontier detection on a 2D occupancy grid (plan §30). Pure numpy; Phase 8 V3 wires it to /slam/map."""
from __future__ import annotations

import numpy as np

FREE, UNKNOWN, OCCUPIED = 0, -1, 100


def frontier_cells(grid: np.ndarray, free_max: int = 49) -> np.ndarray:
    """Return (row, col) of free cells that touch an unknown cell (4-neighbourhood)."""
    free = (grid >= 0) & (grid <= free_max)
    unknown = grid == UNKNOWN
    pad = np.pad(unknown, 1, constant_values=False)
    touch = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
    return np.argwhere(free & touch)


def cluster_frontiers(cells: np.ndarray, min_size: int = 5):
    """Greedy 8-neighbour clustering; returns list of (centroid_row, centroid_col, size)."""
    if len(cells) == 0:
        return []
    cell_set = {tuple(c) for c in cells}
    seen, clusters = set(), []
    for c in map(tuple, cells):
        if c in seen:
            continue
        stack, members = [c], []
        seen.add(c)
        while stack:
            r, q = stack.pop()
            members.append((r, q))
            for dr in (-1, 0, 1):
                for dq in (-1, 0, 1):
                    n = (r + dr, q + dq)
                    if n in cell_set and n not in seen:
                        seen.add(n)
                        stack.append(n)
        if len(members) >= min_size:
            m = np.array(members)
            clusters.append((float(m[:, 0].mean()), float(m[:, 1].mean()), len(members)))
    return clusters
