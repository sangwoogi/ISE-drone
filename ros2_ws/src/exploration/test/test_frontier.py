import numpy as np

from exploration.frontier import FREE, OCCUPIED, UNKNOWN, cluster_frontiers, frontier_cells


def test_frontier_between_free_and_unknown():
    g = np.full((10, 10), UNKNOWN, dtype=np.int16)
    g[:, :5] = FREE
    g[0, :] = OCCUPIED
    cells = frontier_cells(g)
    assert set(cells[:, 1]) == {4}
    assert len(cells) == 9
    clusters = cluster_frontiers(cells, min_size=3)
    assert len(clusters) == 1 and clusters[0][2] == 9
