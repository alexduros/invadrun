"""FKT preparation plan: stages, cue sheet, time estimates, page rendering."""

from __future__ import annotations

import json
import re
import shutil
from datetime import date
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, mapping, shape

from . import __version__, paths
from .data import Invader
from .gpx import Leg


def parse_pace(text: str) -> float:
    """``"6:30"`` (min:sec per km) -> seconds per km."""
    m = re.fullmatch(r"\s*(\d{1,2})(?::(\d{2}))?\s*", text)
    if not m:
        raise ValueError(f"pace must look like 6:30, got {text!r}")
    return int(m.group(1)) * 60 + int(m.group(2) or 0)


def fmt_duration(seconds: float) -> str:
    h, rem = divmod(int(round(seconds)), 3600)
    m = rem // 60
    return f"{h}h{m:02d}" if h else f"{m} min"


def split_stages(cum_km: list[float], stage_km: float) -> list[tuple[int, int]]:
    """Cut the visiting order into stages of roughly ``stage_km`` each.

    Returns (first, last) indices into the order; consecutive stages share
    their boundary invader (you resume from where you stopped).
    """
    total = cum_km[-1]
    k = max(1, int(round(total / stage_km)))
    target = total / k
    cuts = [0]
    arr = np.asarray(cum_km)
    for j in range(1, k):
        i = int(np.abs(arr - j * target).argmin())
        if i > cuts[-1]:
            cuts.append(i)
    cuts.append(len(cum_km) - 1)
    return [(cuts[i], cuts[i + 1]) for i in range(len(cuts) - 1)]


def build_plan(
    ordered: list[Invader],
    legs: list[Leg],
    snap_m: list[float],
    *,
    scope: str,
    stage_km: float,
    pace: str,
    flash_seconds: int,
    skipped: dict,
    solver: dict,
    context: dict,
    graph_info: dict | None = None,
) -> dict:
    n = len(ordered)
    leg_km = [0.0] + [leg.length_m / 1000 for leg in legs]
    cum_km = list(np.cumsum(leg_km))
    total_km = cum_km[-1]
    pace_s = parse_pace(pace)
    stages_idx = split_stages(cum_km, stage_km)

    # polyline with stage offsets
    poly: list[list[float]] = []
    leg_offsets: list[int] = []
    for leg in legs:
        leg_offsets.append(len(poly))
        pts = [[round(lat, 5), round(lon, 5)] for lat, lon in leg.coords]
        if poly and pts and poly[-1] == pts[0]:
            pts = pts[1:]
        poly.extend(pts)
    leg_offsets.append(len(poly))

    stage_of = [0] * n
    stages = []
    for s, (a, b) in enumerate(stages_idx, start=1):
        for i in range(a if s == 1 else a + 1, b + 1):
            stage_of[i] = s
        walls = b - a + (1 if s == 1 else 0)
        codes = sum(len(ordered[i].codes) for i in range(a if s == 1 else a + 1, b + 1))
        km = cum_km[b] - cum_km[a]
        run_s = km * pace_s
        flash_s = walls * flash_seconds
        stages.append(
            {
                "n": s,
                "start": _stop(ordered[a], a, cum_km[a]),
                "end": _stop(ordered[b], b, cum_km[b]),
                "km": round(km, 2),
                "cum_km_start": round(cum_km[a], 2),
                "cum_km_end": round(cum_km[b], 2),
                "walls": walls,
                "codes": codes,
                "run_s": int(run_s),
                "flash_s": int(flash_s),
                "est_s": int(run_s + flash_s),
                "poly_start": leg_offsets[a],
                "poly_end": leg_offsets[b],
                "arrondissements": sorted({ordered[i].extra.get("arrondissement") for i in range(a, b + 1)} - {None}),
            }
        )
    stage_of[0] = 1

    route = [
        {
            "i": i + 1,
            "id": inv.id,
            "label": inv.label,
            "codes": inv.codes,
            "address": inv.address,
            "arr": inv.extra.get("arrondissement"),
            "lat": round(inv.lat, 6),
            "lon": round(inv.lon, 6),
            "leg_km": round(leg_km[i], 3),
            "cum_km": round(cum_km[i], 2),
            "stage": stage_of[i],
            "snap_m": int(round(snap_m[i])),
            "status": inv.extra.get("status", "unknown"),
            "points": inv.extra.get("points"),
            "invaders": inv.extra.get("invaders", [{"code": c, "status": "unknown"} for c in inv.codes]),
        }
        for i, inv in enumerate(ordered)
    ]

    run_total = total_km * pace_s
    flash_total = n * flash_seconds
    per_arr: dict[int, int] = {}
    for inv in ordered:
        a = inv.extra.get("arrondissement")
        if a:
            per_arr[a] = per_arr.get(a, 0) + 1

    return {
        "meta": {
            "generated": date.today().isoformat(),
            "version": __version__,
            "scope": scope,
            "walls": n,
            "codes": sum(len(i.codes) for i in ordered),
            "total_km": round(total_km, 1),
            "longest_leg_km": round(max(leg_km), 2),
            "mean_leg_m": int(round(1000 * total_km / max(1, n - 1))),
            "stage_km": stage_km,
            "stages": len(stages),
            "pace": pace,
            "pace_s": pace_s,
            "flash_seconds": flash_seconds,
            "run_s": int(run_total),
            "flash_s": int(flash_total),
            "est_s": int(run_total + flash_total),
            "start": _stop(ordered[0], 0, 0.0),
            "end": _stop(ordered[-1], n - 1, total_km),
            "solver": solver,
            "graph": graph_info or {},
            "per_arrondissement": dict(sorted(per_arr.items())),
            "points_total": sum(inv.extra.get("points") or 0 for inv in ordered),
            "status_counts": {k: sum(1 for inv in ordered if inv.extra.get("status", "unknown") == k) for k in sorted({inv.extra.get("status", "unknown") for inv in ordered})},
            "snap_over_50m": int(sum(1 for s in snap_m if s > 50)),
        },
        "stages": stages,
        "route": route,
        "polyline": poly,
        "skipped": skipped,
        "context": _context_geojson(context),
    }


def _stop(inv: Invader, idx: int, cum_km: float) -> dict:
    return {
        "i": idx + 1,
        "id": inv.id,
        "label": inv.label,
        "address": inv.address,
        "arr": inv.extra.get("arrondissement"),
        "lat": round(inv.lat, 6),
        "lon": round(inv.lon, 6),
        "cum_km": round(cum_km, 2),
    }


def _context_geojson(context: dict, precision: int = 4, simplify_deg: float = 0.0004) -> dict:
    feats = []
    for kind, items in context.items():
        for props, geom in items:
            g = geom.simplify(simplify_deg, preserve_topology=True)
            gj = mapping(g)
            feats.append({"type": "Feature", "properties": {"kind": kind, **props}, "geometry": _round_geom(gj, precision)})
    return {"type": "FeatureCollection", "features": feats}


def _round_geom(gj: dict, p: int) -> dict:
    def rec(c):
        if isinstance(c, (int, float)):
            return round(c, p)
        return [rec(x) for x in c]

    return {"type": gj["type"], "coordinates": rec(gj["coordinates"])}


def simplify_polyline(coords: list[list[float]], tolerance_m: float = 2.0) -> list[list[float]]:
    if len(coords) < 3:
        return coords
    line = LineString([(lon, lat) for lat, lon in coords]).simplify(tolerance_m / 111_000, preserve_topology=False)
    return [[round(lat, 5), round(lon, 5)] for lon, lat in line.coords]


# --------------------------------------------------------------------------- rendering


def render_pages(plan: dict, docs: Path = paths.DOCS, templates: Path = paths.TEMPLATES) -> list[Path]:
    """Inline the plan into each template (self-contained pages, artifact-ready)."""
    docs.mkdir(parents=True, exist_ok=True)
    out = []
    payload = json.dumps(plan, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    summary = json.dumps({"meta": plan["meta"], "stages": plan["stages"], "skipped": plan["skipped"]}, ensure_ascii=False).replace("</", "<\\/")
    partials = templates / "partials"
    for tpl in sorted(templates.glob("*.html")):
        html = tpl.read_text(encoding="utf-8")
        for name in set(re.findall(r"\{\{INCLUDE:([\w.-]+)\}\}", html)):
            html = html.replace("{{INCLUDE:%s}}" % name, (partials / name).read_text(encoding="utf-8"))
        html = html.replace("{{PLAN_JSON}}", payload).replace("{{SUMMARY_JSON}}", summary)
        target = docs / tpl.name
        target.write_text(html, encoding="utf-8")
        out.append(target)
    static = templates / "static"
    if static.exists():
        shutil.copytree(static, docs / "static", dirs_exist_ok=True)
    return out
