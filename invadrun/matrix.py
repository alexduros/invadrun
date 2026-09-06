"""Street-network distance matrix between invaders.

The old approach ran one point-to-point Dijkstra per pair: ~650k searches
for 1,100 targets. Here every target is a *source* of a single-source
Dijkstra on a CSR graph (scipy, C implementation), so the whole matrix
costs ~1,100 searches, run in chunks to bound memory.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from scipy.sparse.csgraph import dijkstra

from . import paths
from .graph import Network


def compute(net: Network, node_idx: np.ndarray, chunk: int = 64, log=print) -> np.ndarray:
    """Return an (n, n) matrix of walking metres between snapped nodes."""
    uniq, inverse = np.unique(node_idx, return_inverse=True)
    n_nodes = net.A.shape[0]
    dist_u = np.empty((len(uniq), len(uniq)), dtype=np.float64)
    t0 = time.time()
    for start in range(0, len(uniq), chunk):
        src = uniq[start : start + chunk]
        d = dijkstra(net.A, directed=True, indices=src)  # (len(src), n_nodes)
        dist_u[start : start + len(src), :] = d[:, uniq]
        done = start + len(src)
        log(f"  dijkstra {done}/{len(uniq)} sources  ({time.time() - t0:.0f}s, graph {n_nodes} nodes)")
    dist = dist_u[np.ix_(inverse, inverse)]
    np.fill_diagonal(dist, 0.0)
    unreachable = np.isinf(dist).sum()
    if unreachable:
        log(f"  warning: {unreachable} unreachable pairs, set to 10x the longest reachable leg")
        dist[np.isinf(dist)] = 10 * dist[np.isfinite(dist)].max()
    return dist


def save(path: Path, ids: list[str], dist: np.ndarray, node_idx: np.ndarray, snap_m: np.ndarray, nodes: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, ids=np.array(ids), dist=dist, node_idx=node_idx, snap_m=snap_m, node_osmid=nodes[node_idx])


def load(path: Path = paths.MATRIX) -> dict:
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}
