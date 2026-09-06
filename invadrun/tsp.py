"""Order the invaders: one open Hamiltonian path, solved with OR-Tools.

An FKT attempt starts and ends wherever is fastest, so the default is a
free-start / free-end path. A dummy node at distance 0 from everything
turns that into the closed tour the routing solver expects.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


@dataclass
class Solution:
    order: list[int]     # indices into the matrix, in visiting order
    length_m: float
    objective: int
    seconds: float


def solve(
    dist: np.ndarray,
    start: int | None = None,
    end: int | None = None,
    time_limit_s: int = 120,
    log_search: bool = False,
    seed: int | None = None,
) -> Solution:
    n = len(dist)
    matrix = np.rint(dist).astype(np.int64)
    dummy = None
    if start is None or end is None:
        dummy = n
        padded = np.zeros((n + 1, n + 1), dtype=np.int64)
        padded[:n, :n] = matrix
        matrix = padded
    s = dummy if start is None else start
    e = dummy if end is None else end
    if s == e:
        manager = pywrapcp.RoutingIndexManager(len(matrix), 1, s)
    else:
        manager = pywrapcp.RoutingIndexManager(len(matrix), 1, [s], [e])
    routing = pywrapcp.RoutingModel(manager)
    flat = matrix.tolist()

    def cost(i: int, j: int) -> int:
        return flat[manager.IndexToNode(i)][manager.IndexToNode(j)]

    transit = routing.RegisterTransitCallback(cost)
    routing.SetArcCostEvaluatorOfAllVehicles(transit)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = int(time_limit_s)
    params.log_search = log_search
    if seed is not None:
        params.use_full_propagation = False

    import time

    t0 = time.time()
    assignment = routing.SolveWithParameters(params)
    if assignment is None:
        raise RuntimeError("OR-Tools found no solution")

    order: list[int] = []
    idx = routing.Start(0)
    while not routing.IsEnd(idx):
        node = manager.IndexToNode(idx)
        if node != dummy:
            order.append(node)
        idx = assignment.Value(routing.NextVar(idx))
    node = manager.IndexToNode(idx)
    if node != dummy and node not in order:
        order.append(node)

    length = float(sum(dist[a, b] for a, b in zip(order, order[1:])))
    return Solution(order=order, length_m=length, objective=int(assignment.ObjectiveValue()), seconds=time.time() - t0)


def two_opt_check(dist: np.ndarray, order: list[int]) -> float:
    """Length of the path (sanity helper for tests)."""
    return float(sum(dist[a, b] for a, b in zip(order, order[1:])))
