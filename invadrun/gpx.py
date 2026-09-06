"""Turn an ordered list of invaders into street geometry and GPX files."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import gpxpy
import gpxpy.gpx
import networkx as nx

from .data import Invader
from .graph import Network, haversine_m


@dataclass
class Leg:
    """Street path from one invader to the next."""

    length_m: float
    coords: list[tuple[float, float]] = field(default_factory=list)  # (lat, lon)


def leg_geometry(net: Network, u: int, v: int) -> Leg:
    """Shortest walking path between two OSM node ids, with edge geometry."""
    D = net.D
    if u == v:
        return Leg(0.0, [(D.nodes[u]["y"], D.nodes[u]["x"])])
    length, path = nx.bidirectional_dijkstra(D, u, v, weight="length")
    coords: list[tuple[float, float]] = []
    for a, b in zip(path, path[1:]):
        data = D[a][b]
        geom = data.get("geometry")
        if geom is not None:
            pts = [(y, x) for x, y in geom.coords]
            # edge geometry may be stored in the opposite direction
            if haversine_m(pts[0][0], pts[0][1], D.nodes[a]["y"], D.nodes[a]["x"]) > haversine_m(
                pts[-1][0], pts[-1][1], D.nodes[a]["y"], D.nodes[a]["x"]
            ):
                pts.reverse()
        else:
            pts = [(D.nodes[a]["y"], D.nodes[a]["x"]), (D.nodes[b]["y"], D.nodes[b]["x"])]
        if coords and coords[-1] == pts[0]:
            pts = pts[1:]
        coords.extend(pts)
    return Leg(float(length), coords)


def build_legs(net: Network, node_osmids: list[int], order: list[int]) -> list[Leg]:
    """One leg per consecutive pair in ``order`` (len(order) - 1 legs)."""
    return [leg_geometry(net, int(node_osmids[a]), int(node_osmids[b])) for a, b in zip(order, order[1:])]


def write_gpx(
    path: Path,
    name: str,
    invaders: list[Invader],
    legs: list[Leg],
    cum_km: list[float],
    description: str = "",
) -> Path:
    """``invaders`` are in visiting order; ``legs[i]`` joins invaders i and i+1."""
    gpx = gpxpy.gpx.GPX()
    gpx.name = name
    gpx.description = description
    gpx.creator = "invadrun"

    for i, inv in enumerate(invaders):
        gpx.waypoints.append(
            gpxpy.gpx.GPXWaypoint(
                latitude=inv.lat,
                longitude=inv.lon,
                name=inv.label,
                description=inv.address,
                comment=f"#{i + 1} · {cum_km[i]:.1f} km",
                symbol="Flag, Blue",
            )
        )

    track = gpxpy.gpx.GPXTrack(name=name)
    segment = gpxpy.gpx.GPXTrackSegment()
    last = None
    for leg in legs:
        for lat, lon in leg.coords:
            if last == (lat, lon):
                continue
            segment.points.append(gpxpy.gpx.GPXTrackPoint(latitude=lat, longitude=lon))
            last = (lat, lon)
    track.segments.append(segment)
    gpx.tracks.append(track)

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(gpx.to_xml(), encoding="utf-8")
    return path
