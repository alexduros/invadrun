import numpy as np

from invadrun.tsp import solve


def _grid(n=12, seed=1):
    rng = np.random.default_rng(seed)
    pts = rng.uniform(0, 1000, size=(n, 2))
    return pts, np.linalg.norm(pts[:, None] - pts[None], axis=-1)


def test_free_endpoints_visits_everything_once():
    _, d = _grid()
    sol = solve(d, time_limit_s=1)
    assert sorted(sol.order) == list(range(len(d)))
    assert sol.length_m > 0


def test_fixed_start_and_end():
    _, d = _grid()
    sol = solve(d, start=3, end=7, time_limit_s=1)
    assert sol.order[0] == 3 and sol.order[-1] == 7
    assert sorted(sol.order) == list(range(len(d)))


def test_fixed_start_free_end():
    _, d = _grid()
    sol = solve(d, start=5, time_limit_s=1)
    assert sol.order[0] == 5 and sorted(sol.order) == list(range(len(d)))


def test_open_path_never_longer_than_closed_tour():
    _, d = _grid(n=15)
    open_path = solve(d, time_limit_s=2)
    closed = solve(d, start=0, end=0, time_limit_s=2)
    assert open_path.length_m <= closed.length_m + d[closed.order[-1], 0] + 1e-6
