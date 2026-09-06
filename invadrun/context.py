"""Fetch light-weight map context (city limits, arrondissements, Seine).

Stored once in ``data/context.geojson`` so pages can draw a recognisable
Paris without any tile server, and so the ``in_paris`` flag is reproducible.
"""

from __future__ import annotations

import json
from pathlib import Path

import osmnx as ox
from shapely.geometry import mapping
from shapely.ops import linemerge, unary_union

from . import paths


def fetch(out: Path = paths.CONTEXT, simplify_m: float = 15.0) -> Path:
    ox.settings.use_cache = True
    ox.settings.log_console = False
    ox.settings.cache_folder = str(paths.CACHE / "osmnx")
    tol = simplify_m / 111_000  # degrees, good enough for drawing

    features = []
    city = ox.geocode_to_gdf("Paris, France").geometry.iloc[0]
    features.append(_feature("city", city.simplify(tol), {"name": "Paris"}))

    arr = ox.features_from_polygon(city.buffer(0.002), tags={"admin_level": "9", "boundary": "administrative"})
    arr = arr[arr.geometry.geom_type.isin(["Polygon", "MultiPolygon"])]
    for _, row in arr.iterrows():
        name = str(row.get("name", ""))
        if "Arrondissement" not in name or not name.startswith("Paris"):
            continue
        num = int("".join(ch for ch in name.split()[1] if ch.isdigit()) or 0)
        features.append(_feature("arrondissement", row.geometry.simplify(tol), {"name": name, "number": num}))

    seine = ox.features_from_polygon(city.buffer(0.01), tags={"waterway": "river"})
    seine = seine[seine.get("name", "").astype(str).str.contains("Seine|Marne", regex=True)]
    lines = [g for g in seine.geometry if g.geom_type in ("LineString", "MultiLineString")]
    if lines:
        merged = linemerge(unary_union(lines))
        features.append(_feature("river", merged.simplify(tol), {"name": "La Seine"}))

    water = ox.features_from_polygon(city, tags={"natural": "water", "water": "river"})
    water = water[water.geometry.geom_type.isin(["Polygon", "MultiPolygon"])]
    if len(water):
        poly = unary_union(list(water.geometry)).simplify(tol)
        features.append(_feature("water", poly, {"name": "Seine (surface)"}))

    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"type": "FeatureCollection", "features": features}, fh, ensure_ascii=False)
    return out


def _feature(kind: str, geom, props: dict) -> dict:
    return {
        "type": "Feature",
        "geometry": json.loads(json.dumps(mapping(geom))),
        "properties": {"kind": kind, **props},
    }
