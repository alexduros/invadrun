"""OpenStreetMap walking network: download, cache, sparse matrix, snapping."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import networkx as nx
import numpy as np
import osmnx as ox
from scipy.sparse import csr_array
from scipy.spatial import cKDTree
from shapely.geometry import MultiPoint, Polygon

from . import paths

R_EARTH = 6_371_008.8


def _configure() -> None:
    ox.settings.use_cache = True
    ox.settings.log_console = False
    ox.settings.cache_folder = str(paths.CACHE / "osmnx")


def load_or_download(polygon: Polygon, path: Path = paths.GRAPH, buffer_deg: float = 0.004) -> nx.MultiDiGraph:
    """Walkable street graph covering ``polygon`` (+ ~400 m buffer)."""
    _configure()
    if path.exists():
        return ox.load_graphml(path)
    area = polygon.buffer(buffer_deg)
    G = ox.graph_from_polygon(area, network_type="walk", simplify=True, retain_all=False, truncate_by_edge=True)
    path.parent.mkdir(parents=True, exist_ok=True)
    ox.save_graphml(G, path)
    return G


def coverage_polygon(lats, lons, context_city: Polygon | None) -> Polygon:
    """Area the graph must cover: the city, enlarged to hold every target."""
    hull = MultiPoint(list(zip(lons, lats))).convex_hull.buffer(0.01)
    if context_city is None:
        return hull
    return context_city.union(hull) if not context_city.contains(hull) else context_city


@dataclass
class Network:
    """Directed simple graph as CSR (min length over parallel edges) + node coordinates."""

    D: nx.DiGraph
    nodes: np.ndarray        # osmid per row/col
    A: csr_array             # metres
    xy: np.ndarray           # local metric coords (n, 2) for KD-tree
    lat0: float

    @classmethod
    def from_multidigraph(cls, G: nx.MultiDiGraph) -> "Network":
        D = ox.convert.to_digraph(G, weight="length")
        nodes = np.fromiter(D.nodes, dtype=np.int64, count=D.number_of_nodes())
        A = csr_array(nx.to_scipy_sparse_array(D, nodelist=nodes.tolist(), weight="length", format="csr"))
        lat = np.array([D.nodes[n]["y"] for n in nodes])
        lon = np.array([D.nodes[n]["x"] for n in nodes])
        lat0 = float(lat.mean())
        return cls(D=D, nodes=nodes, A=A, xy=project(lat, lon, lat0), lat0=lat0)

    def snap(self, lats, lons) -> tuple[np.ndarray, np.ndarray]:
        """Nearest graph node index for each point, and distance in metres."""
        tree = cKDTree(self.xy)
        d, idx = tree.query(project(np.asarray(lats), np.asarray(lons), self.lat0))
        return idx.astype(np.int64), d


def project(lat, lon, lat0: float) -> np.ndarray:
    """Equirectangular projection to metres; plenty accurate over one city."""
    k = math.pi / 180 * R_EARTH
    x = (np.asarray(lon)) * k * math.cos(math.radians(lat0))
    y = (np.asarray(lat)) * k
    return np.column_stack([x, y])


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))
