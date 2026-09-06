"""Load, clean and normalise the invader dataset.

The raw file is a uMap export of https://www.invader-spotter.art locations.
Its names look like ``Space Invader PA_0035 & PA_0482, 24 rue PavÃ©e``:
UTF-8 text that was decoded as Latin-1, several codes per wall with
inconsistent spelling (``PA__0290``, ``& 573``), and a comma-separated
address. This module turns that into a tidy GeoJSON with one feature per
wall and clean, comparable invader codes.
"""

from __future__ import annotations

import json
import re
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable

from shapely.geometry import Point, shape

from . import paths

# A wall carries one code, or a list that always ends with "& <code>":
#   "PA_0035 & PA_0482", "PA_0109, 587 & 1081", "PA_0973 PA_0974 & PA_0975".
# Requiring the trailing "&" keeps "PA_0914, 103,5 rue Baudin" as one code.
_TOKEN = r"(?:[A-Z]{2,4}_*\d{1,4}|\d{1,4})"
_FIRST = r"[A-Z]{2,4}_*\d{1,4}"
_CODES = rf"{_FIRST}(?:(?:\s*,\s*|\s+){_TOKEN})*\s*&\s*{_TOKEN}|{_FIRST}"
NAME_RE = re.compile(rf"^\s*space\s+invader\s*,?\s*(?P<codes>{_CODES})\s*,\s*(?P<address>.+?)\s*$", re.I)
CODE_RE = re.compile(r"(?P<prefix>[A-Z]{2,4})?_*(?P<num>\d{1,4})")


@dataclass
class Invader:
    id: str                 # canonical code of the wall (first code), e.g. PA_0290
    codes: list[str]        # every code painted on that wall
    city: str               # code prefix: PA (Paris), VRS (Versailles) ...
    address: str
    lat: float
    lon: float
    in_paris: bool = True
    excluded: bool = False
    exclude_reason: str = ""
    raw_name: str = ""
    extra: dict = field(default_factory=dict)

    @property
    def label(self) -> str:
        return " / ".join(self.codes)


def fix_mojibake(text: str) -> str:
    """Undo UTF-8 bytes that were decoded as cp1252/Latin-1 (``Ã©`` -> ``é``, ``Ã‰`` -> ``É``)."""
    for enc in ("cp1252", "latin-1"):
        try:
            return text.encode(enc).decode("utf-8")
        except (UnicodeEncodeError, UnicodeDecodeError):
            continue
    return text


def parse_codes(raw: str) -> list[str]:
    """``"PA__0290 & 532"`` -> ``["PA_0290", "PA_0532"]``."""
    codes: list[str] = []
    prefix = None
    for m in CODE_RE.finditer(raw):
        prefix = m.group("prefix") or prefix
        if prefix is None:
            continue
        num = m.group("num")
        width = 4 if prefix == "PA" else max(len(num), 2)
        code = f"{prefix}_{num.zfill(width)}"
        if code not in codes:
            codes.append(code)
    return codes


def parse_name(name: str) -> tuple[list[str], str]:
    name = fix_mojibake(name).replace("\xa0", " ").strip()
    m = NAME_RE.match(name)
    if not m:
        raise ValueError(f"unrecognised invader name: {name!r}")
    codes = parse_codes(m.group("codes"))
    if not codes:
        raise ValueError(f"no code found in: {name!r}")
    address = re.sub(r"\s+", " ", m.group("address")).strip()
    return codes, address


def load_raw(path: Path = paths.RAW_UMAP) -> list[Invader]:
    with open(path, encoding="utf-8") as fh:
        payload = json.load(fh)
    out: list[Invader] = []
    for feat in payload["features"]:
        lon, lat = feat["geometry"]["coordinates"][:2]
        raw = feat["properties"]["name"]
        codes, address = parse_name(raw)
        out.append(
            Invader(
                id=codes[0],
                codes=codes,
                city=codes[0].split("_")[0],
                address=address,
                lat=float(lat),
                lon=float(lon),
                raw_name=fix_mojibake(raw),
            )
        )
    return out


def load_exclusions(path: Path = paths.EXCLUSIONS) -> dict[str, str]:
    """``PA_0123  # destroyed 2023`` per line -> {code: reason}."""
    if not path.exists():
        return {}
    out: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        code, _, reason = line.partition("#")
        out[code.strip().upper()] = reason.strip()
    return out


def load_context(path: Path = paths.CONTEXT) -> dict:
    """Return {kind: [shapely geometries]} from data/context.geojson."""
    if not path.exists():
        return {}
    with open(path, encoding="utf-8") as fh:
        gj = json.load(fh)
    ctx: dict[str, list] = {}
    for feat in gj["features"]:
        ctx.setdefault(feat["properties"]["kind"], []).append(
            (feat["properties"], shape(feat["geometry"]))
        )
    return ctx


def tag_in_paris(invaders: Iterable[Invader], context: dict) -> None:
    boundary = [g for _, g in context.get("city", [])]
    arrondissements = context.get("arrondissement", [])
    for inv in invaders:
        pt = Point(inv.lon, inv.lat)
        inv.in_paris = any(g.contains(pt) for g in boundary) if boundary else _bbox_paris(inv)
        for props, geom in arrondissements:
            if geom.contains(pt):
                inv.extra["arrondissement"] = props.get("number")
                break


def _bbox_paris(inv: Invader) -> bool:
    return 2.224 <= inv.lon <= 2.470 and 48.815 <= inv.lat <= 48.903


# Wall status = best status among its codes. Anything in UNFLASHABLE cannot be
# flashed today and is left out of the route unless --keep-destroyed is given.
STATUS_RANK = ["ok", "damaged", "very_damaged", "unknown", "hidden", "not_visible", "destroyed"]
UNFLASHABLE = {"destroyed", "hidden", "not_visible"}


def merge_spotter(invaders: Iterable[Invader], spotter: dict[str, dict]) -> None:
    """Attach invader-spotter status, points, dates and pictures to each wall."""
    for inv in invaders:
        details = []
        for code in inv.codes:
            rec = spotter.get(code)
            details.append({"code": code, **({k: rec[k] for k in ("status", "status_text", "status_date", "points", "installed", "photo", "photo_full", "closeup", "instagram")} if rec else {"status": "unknown"})})
        inv.extra["invaders"] = details
        statuses = [d.get("status", "unknown") for d in details]
        inv.extra["status"] = min(statuses, key=lambda st: STATUS_RANK.index(st) if st in STATUS_RANK else len(STATUS_RANK))
        pts = [d.get("points") for d in details if d.get("points")]
        inv.extra["points"] = sum(pts) if pts else None
        if not inv.extra.get("arrondissement"):
            arr = next((spotter[c]["arrondissement"] for c in inv.codes if c in spotter and spotter[c].get("arrondissement")), None)
            if arr and inv.in_paris:
                inv.extra["arrondissement"] = arr


def clean(raw: Path = paths.RAW_UMAP, out: Path = paths.INVADERS) -> list[Invader]:
    from . import spotter as spotter_mod

    invaders = load_raw(raw)
    tag_in_paris(invaders, load_context())
    merge_spotter(invaders, spotter_mod.load())
    exclusions = load_exclusions()
    for inv in invaders:
        hit = [c for c in inv.codes if c in exclusions]
        if hit and len(hit) == len(inv.codes):
            inv.excluded = True
            inv.exclude_reason = exclusions[hit[0]] or "excluded"
    invaders.sort(key=lambda i: (i.city != "PA", i.id))
    save(invaders, out)
    return invaders


def save(invaders: list[Invader], out: Path = paths.INVADERS) -> None:
    features = []
    for inv in invaders:
        props = asdict(inv)
        props.pop("lat"), props.pop("lon")
        extra = props.pop("extra")
        props.update(extra)
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [round(inv.lon, 6), round(inv.lat, 6)]},
                "properties": props,
            }
        )
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"type": "FeatureCollection", "features": features}, fh, ensure_ascii=False, indent=1)


def load(path: Path = paths.INVADERS) -> list[Invader]:
    with open(path, encoding="utf-8") as fh:
        gj = json.load(fh)
    out = []
    for f in gj["features"]:
        p = dict(f["properties"])
        lon, lat = f["geometry"]["coordinates"]
        known = {k: p.pop(k) for k in list(p) if k in Invader.__dataclass_fields__}
        out.append(Invader(lat=lat, lon=lon, extra=p, **known))
    return out


def select(invaders: list[Invader], scope: str = "paris", keep_destroyed: bool = False) -> list[Invader]:
    """Targets for routing. ``paris`` = inside the city limits, ``all`` = every PA_ code.

    Walls whose every code is destroyed/hidden (per invader-spotter) are dropped
    unless ``keep_destroyed``.
    """
    keep = [i for i in invaders if not i.excluded and i.city == "PA"]
    if scope == "paris":
        keep = [i for i in keep if i.in_paris]
    elif scope != "all":
        raise ValueError(f"unknown scope {scope!r} (expected 'paris' or 'all')")
    if not keep_destroyed:
        keep = [i for i in keep if i.extra.get("status", "unknown") not in UNFLASHABLE]
    return keep


def summary(invaders: list[Invader]) -> dict:
    pa = [i for i in invaders if i.city == "PA"]
    return {
        "walls": len(invaders),
        "codes": sum(len(i.codes) for i in invaders),
        "paris_walls": len(pa),
        "paris_codes": sum(len(i.codes) for i in pa),
        "inside_city_limits": sum(1 for i in pa if i.in_paris),
        "outside_city_limits": sum(1 for i in pa if not i.in_paris),
        "excluded": sum(1 for i in invaders if i.excluded),
        "unflashable_in_city": sum(1 for i in pa if i.in_paris and i.extra.get("status") in UNFLASHABLE),
        "status_in_city": dict(sorted(Counter(i.extra.get("status", "unknown") for i in pa if i.in_paris).items())),
        "points_in_city": sum(i.extra.get("points") or 0 for i in pa if i.in_paris and i.extra.get("status") not in UNFLASHABLE),
        "other_cities": sorted({i.city for i in invaders if i.city != "PA"}),
    }
