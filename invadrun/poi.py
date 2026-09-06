"""POI files for Organic Maps (and any GPX/KML reader).

One placemark per *place*: walls closer than ``radius_m`` are merged, and a
wall already carries every code painted on it. The description is HTML with
the invader-spotter picture, close-up, points, status and Instagram tag for
each invader, plus the stop number, stage and kilometre on the route.
Organic Maps renders HTML descriptions and reads ``osmand:color`` on
waypoints and ``#placemark-<colour>`` style urls in KML.
"""

from __future__ import annotations

import html
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
from scipy.spatial import cKDTree

from . import paths
from .graph import project

STAGE_HEX = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
STAGE_OM = ["blue", "orange", "teal", "yellow", "pink", "green", "purple", "red"]
SPOTTER_CREDIT = "Pictures and statuses: invader-spotter.art"


def stage_hex(n: int) -> str:
    return STAGE_HEX[(n - 1) % len(STAGE_HEX)]


def stage_om(n: int) -> str:
    return STAGE_OM[(n - 1) % len(STAGE_OM)]


def group_places(route: list[dict], radius_m: float = 25.0) -> list[list[dict]]:
    """Union-find on walls within ``radius_m``; groups ordered by first stop."""
    lat = np.array([r["lat"] for r in route])
    lon = np.array([r["lon"] for r in route])
    xy = project(lat, lon, float(lat.mean()))
    parent = list(range(len(route)))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b in cKDTree(xy).query_pairs(radius_m):
        parent[find(a)] = find(b)
    groups: dict[int, list[dict]] = {}
    for i, r in enumerate(route):
        groups.setdefault(find(i), []).append(r)
    out = [sorted(g, key=lambda r: r["i"]) for g in groups.values()]
    return sorted(out, key=lambda g: g[0]["i"])


def place_name(group: list[dict]) -> str:
    codes = [c for r in group for c in r["codes"]]
    label = " · ".join(codes) if len(codes) <= 3 else f"{codes[0]} +{len(codes) - 1}"
    stops = f"#{group[0]['i']}" if len(group) == 1 else f"#{group[0]['i']}–{group[-1]['i']}"
    return f"{label} ({stops})"


def place_center(group: list[dict]) -> tuple[float, float]:
    return (sum(r["lat"] for r in group) / len(group), sum(r["lon"] for r in group) / len(group))


STATUS_LABEL = {"ok": "OK", "damaged": "damaged", "very_damaged": "badly damaged", "destroyed": "destroyed", "hidden": "hidden", "not_visible": "not visible", "unknown": "status unknown"}


def describe_html(group: list[dict], total: int) -> str:
    h = html.escape
    parts = []
    for r in group:
        arr = "" if not r.get("arr") else (" · 1er" if r["arr"] == 1 else f" · {r['arr']}e")
        parts.append(f"<p><b>{h(r['address'])}</b>{arr}<br/>Stop #{r['i']} of {total} · km {r['cum_km']:.1f} · stage {r['stage']}</p>")
        for d in r.get("invaders", []):
            meta = []
            if d.get("points"):
                meta.append(f"{d['points']} pts")
            meta.append(STATUS_LABEL.get(d.get("status", "unknown"), d.get("status", "?")) + (f", {h(d['status_date'])}" if d.get("status_date") else ""))
            if d.get("installed"):
                meta.append(f"installed {h(d['installed'])}")
            pics = ""
            if d.get("photo"):
                img = f'<img src="{h(d["photo"])}" alt="{h(d["code"])}" style="max-width:100%;width:300px"/>'
                pics += f'<a href="{h(d.get("photo_full") or d["photo"])}">{img}</a> '
            if d.get("closeup"):
                pics += f'<img src="{h(d["closeup"])}" alt="{h(d["code"])} close-up" width="80"/>'
            insta = f' · <a href="{h(d["instagram"])}">Instagram</a>' if d.get("instagram") else ""
            parts.append(f"<p><b>{h(d['code'])}</b> · {' · '.join(meta)}{insta}<br/>{pics}</p>")
    parts.append(f"<p><small>{SPOTTER_CREDIT} · route: invadrun</small></p>")
    return "".join(parts)


def describe_text(group: list[dict]) -> str:
    codes = ", ".join(c for r in group for c in r["codes"])
    return f"{group[0]['address']} · {codes} · km {group[0]['cum_km']:.1f}"


def write_gpx(path: Path, plan: dict, groups: list[list[dict]], with_track: bool = True) -> Path:
    total = len(plan["route"])
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<gpx version="1.1" creator="invadrun" xmlns="http://www.topografix.com/GPX/1/1" xmlns:osmand="https://osmand.net" '
           'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">',
           f'<metadata><name>Invadrun Paris · {total} walls</name><desc>{escape(SPOTTER_CREDIT)}</desc></metadata>']
    for g in groups:
        lat, lon = place_center(g)
        out.append(
            f'<wpt lat="{lat:.6f}" lon="{lon:.6f}"><name>{escape(place_name(g))}</name>'
            f'<cmt>{escape(describe_text(g))}</cmt><desc>{escape(describe_html(g, total))}</desc>'
            f'<sym>Flag, Blue</sym><type>Stage {g[0]["stage"]}</type>'
            f'<extensions><osmand:color>{stage_hex(g[0]["stage"])}</osmand:color></extensions></wpt>'
        )
    if with_track:
        poly = plan["polyline"]
        for s in plan["stages"]:
            pts = poly[s["poly_start"] : s["poly_end"] + 1]
            seg = "".join(f'<trkpt lat="{la:.5f}" lon="{lo:.5f}"/>' for la, lo in pts)
            out.append(
                f'<trk><name>Stage {s["n"]} · {s["km"]:.1f} km · {s["walls"]} walls</name>'
                f'<extensions><osmand:color>{stage_hex(s["n"])}</osmand:color></extensions><trkseg>{seg}</trkseg></trk>'
            )
    out.append("</gpx>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(out), encoding="utf-8")
    return path


def _kml_color(hex_rgb: str, alpha: str = "cc") -> str:
    r, g, b = hex_rgb[1:3], hex_rgb[3:5], hex_rgb[5:7]
    return f"{alpha}{b}{g}{r}"


def write_kml(path: Path, plan: dict, groups: list[list[dict]], with_track: bool = True) -> Path:
    total = len(plan["route"])
    out = ['<?xml version="1.0" encoding="UTF-8"?>', '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
           f"<name>Invadrun Paris · {total} walls</name><description>{escape(SPOTTER_CREDIT)}</description><visibility>1</visibility>"]
    for n in range(1, len(plan["stages"]) + 1):
        c = stage_om(n)
        out.append(f'<Style id="placemark-{c}"><IconStyle><Icon><href>https://omaps.app/placemarks/placemark-{c}.png</href></Icon></IconStyle></Style>')
        out.append(f'<Style id="track-{n}"><LineStyle><color>{_kml_color(stage_hex(n))}</color><width>4</width></LineStyle></Style>')
    for g in groups:
        lat, lon = place_center(g)
        out.append(
            f"<Placemark><name>{escape(place_name(g))}</name><description><![CDATA[{describe_html(g, total)}]]></description>"
            f'<styleUrl>#placemark-{stage_om(g[0]["stage"])}</styleUrl><Point><coordinates>{lon:.6f},{lat:.6f}</coordinates></Point></Placemark>'
        )
    if with_track:
        poly = plan["polyline"]
        for s in plan["stages"]:
            pts = poly[s["poly_start"] : s["poly_end"] + 1]
            coords = " ".join(f"{lo:.5f},{la:.5f}" for la, lo in pts)
            out.append(
                f'<Placemark><name>Stage {s["n"]} · {s["km"]:.1f} km · {s["walls"]} walls</name><styleUrl>#track-{s["n"]}</styleUrl>'
                f"<LineString><tessellate>1</tessellate><coordinates>{coords}</coordinates></LineString></Placemark>"
            )
    out.append("</Document></kml>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(out), encoding="utf-8")
    return path


def write_all(plan: dict, out_dir: Path = paths.DOCS_DATA, radius_m: float = 25.0) -> dict:
    groups = group_places(plan["route"], radius_m)
    gpx = write_gpx(out_dir / "invadrun-poi.gpx", plan, groups)
    kml = write_kml(out_dir / "invadrun-poi.kml", plan, groups)
    return {"places": len(groups), "walls": len(plan["route"]), "grouped": sum(1 for g in groups if len(g) > 1), "gpx": gpx, "kml": kml}
