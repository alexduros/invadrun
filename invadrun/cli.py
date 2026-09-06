"""Command line: ``invadrun <step>``. Run ``invadrun all`` for the full pipeline."""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from . import __version__, paths


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


# ------------------------------------------------------------------ steps


def cmd_context(args) -> None:
    from . import context

    t = time.time()
    out = context.fetch()
    log(f"context -> {out.relative_to(paths.ROOT)} ({time.time() - t:.0f}s)")


def cmd_clean(args) -> None:
    from . import data

    inv = data.clean()
    log(f"clean -> {paths.INVADERS.relative_to(paths.ROOT)}")
    print(json.dumps(data.summary(inv), indent=1, ensure_ascii=False))


def _targets(scope: str, keep_destroyed: bool = False):
    from . import data

    return data.select(data.load(), scope=scope, keep_destroyed=keep_destroyed)


def cmd_spotter(args) -> None:
    from . import spotter

    if args.refresh or not any(spotter.PAGES.glob("PA_lst_p*.html")):
        n = spotter.fetch(log=log)
        log(f"spotter: fetched {n} listing pages")
    data = spotter.parse_cached()
    spotter.save(data)
    from collections import Counter

    log(f"spotter -> {spotter.SPOTTER.relative_to(paths.ROOT)}: {len(data)} invaders, {dict(Counter(v['status'] for v in data.values()))}")


def cmd_poi(args) -> None:
    from . import poi

    plan = json.loads((paths.DOCS_DATA / "plan.json").read_text(encoding="utf-8"))
    info = poi.write_all(plan, radius_m=args.radius)
    log(f"poi -> {info['gpx'].name}, {info['kml'].name}: {info['places']} places for {info['walls']} walls ({info['grouped']} places hold several walls)")


def _graph(scope: str):
    from . import data, graph

    ctx = data.load_context()
    city = ctx["city"][0][1] if ctx.get("city") else None
    inv = _targets(scope, True)
    poly = graph.coverage_polygon([i.lat for i in inv], [i.lon for i in inv], city)
    t = time.time()
    G = graph.load_or_download(poly)
    log(f"graph: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges ({time.time() - t:.0f}s)")
    return G


def cmd_graph(args) -> None:
    _graph(args.scope)


def cmd_matrix(args) -> None:
    from . import graph, matrix

    inv = _targets(args.scope, args.keep_destroyed)
    G = _graph(args.scope)
    net = graph.Network.from_multidigraph(G)
    node_idx, snap_m = net.snap([i.lat for i in inv], [i.lon for i in inv])
    log(f"snap: median {np.median(snap_m):.0f} m, max {snap_m.max():.0f} m, {int((snap_m > 100).sum())} over 100 m")
    dist = matrix.compute(net, node_idx, chunk=args.chunk, log=log)
    matrix.save(paths.MATRIX, [i.id for i in inv], dist, node_idx, snap_m, net.nodes)
    log(f"matrix -> {paths.MATRIX.relative_to(paths.ROOT)} ({len(inv)}x{len(inv)})")


def cmd_solve(args) -> None:
    from . import matrix, tsp

    m = matrix.load()
    ids = list(m["ids"])
    inv = {i.id: i for i in _targets(args.scope, args.keep_destroyed)}
    if set(ids) != set(inv):
        sys.exit("matrix and target set differ: run `invadrun matrix` again (same --scope/--keep-destroyed)")

    def idx(code: str | None):
        if code is None:
            return None
        code = code.upper()
        if code not in ids:
            sys.exit(f"{code} is not a routed target")
        return ids.index(code)

    log(f"solve: {len(ids)} targets, {args.time_limit}s budget, start={args.start or 'free'} end={args.end or 'free'}")
    sol = tsp.solve(m["dist"], start=idx(args.start), end=idx(args.end), time_limit_s=args.time_limit, log_search=args.log_search)
    route = {
        "scope": args.scope,
        "ids": [ids[i] for i in sol.order],
        "length_m": round(sol.length_m),
        "solver": {
            "engine": "ortools " + _ortools_version(),
            "strategy": "PATH_CHEAPEST_ARC + GUIDED_LOCAL_SEARCH",
            "time_limit_s": args.time_limit,
            "seconds": round(sol.seconds),
            "start": args.start,
            "end": args.end,
        },
    }
    paths.ROUTE.parent.mkdir(parents=True, exist_ok=True)
    paths.ROUTE.write_text(json.dumps(route, indent=1))
    log(f"solve -> {paths.ROUTE.relative_to(paths.ROOT)}: {sol.length_m / 1000:.1f} km, {ids[sol.order[0]]} -> {ids[sol.order[-1]]}")


def cmd_export(args) -> None:
    from . import data, gpx, graph, matrix, plan

    m = matrix.load()
    route = json.loads(paths.ROUTE.read_text())
    all_inv = data.load()
    by_id = {i.id: i for i in all_inv}
    ordered = [by_id[c] for c in route["ids"]]
    ids = list(m["ids"])
    pos = [ids.index(c) for c in route["ids"]]
    node_osmids = [int(m["node_osmid"][p]) for p in pos]
    snap_m = [float(m["snap_m"][p]) for p in pos]

    G = _graph(route["scope"])
    net = graph.Network.from_multidigraph(G)
    t = time.time()
    legs = gpx.build_legs(net, node_osmids, list(range(len(ordered))))
    total = sum(l.length_m for l in legs) / 1000
    log(f"geometry: {len(legs)} legs, {total:.1f} km, {sum(len(l.coords) for l in legs)} points ({time.time() - t:.0f}s)")

    targets = set(route["ids"])
    skipped = {
        "outside_city": [_brief(i) for i in all_inv if i.city == "PA" and not i.excluded and i.id not in targets and not i.in_paris],
        "unflashable": [_brief(i) | {"status": i.extra.get("status"), "status_date": next((d.get("status_date") for d in inv_details(i) if d.get("status_date")), "")} for i in all_inv if i.city == "PA" and i.in_paris and not i.excluded and i.id not in targets and i.extra.get("status") in data.UNFLASHABLE],
        "excluded": [_brief(i) | {"reason": i.exclude_reason} for i in all_inv if i.excluded],
        "other_cities": {c: sum(1 for i in all_inv if i.city == c) for c in sorted({i.city for i in all_inv if i.city != "PA"})},
    }
    p = plan.build_plan(
        ordered,
        legs,
        snap_m,
        scope=route["scope"],
        stage_km=args.stage_km,
        pace=args.pace,
        flash_seconds=args.flash_seconds,
        skipped=skipped,
        solver=route["solver"],
        context=data.load_context(),
        graph_info={"nodes": G.number_of_nodes(), "edges": G.number_of_edges(), "network": "walk", "source": "OpenStreetMap"},
    )
    paths.DOCS_DATA.mkdir(parents=True, exist_ok=True)
    (paths.DOCS_DATA / "plan.json").write_text(json.dumps(p, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    cum = [r["cum_km"] for r in p["route"]]
    gpx.write_gpx(paths.DOCS_DATA / "invadrun.gpx", "Invadrun Paris", ordered, legs, cum, description=f"{total:.1f} km, {len(ordered)} walls")
    for s in p["stages"]:
        a, b = s["start"]["i"] - 1, s["end"]["i"] - 1
        gpx.write_gpx(
            paths.DOCS_DATA / f"stage_{s['n']:02d}.gpx",
            f"Invadrun stage {s['n']}",
            ordered[a : b + 1],
            legs[a:b],
            cum[a : b + 1],
            description=f"{s['km']:.1f} km, {s['walls']} walls",
        )
    for stale in paths.DOCS_DATA.glob("stage_*.gpx"):
        if int(stale.stem.split("_")[1]) > len(p["stages"]):
            stale.unlink()
    log(f"export -> docs/data/plan.json, invadrun.gpx, {len(p['stages'])} stage GPX")
    from . import poi

    info = poi.write_all(p)
    log(f"poi -> invadrun-poi.gpx/.kml: {info['places']} places for {info['walls']} walls")
    pages = plan.render_pages(p)
    log("render -> " + ", ".join(pg.relative_to(paths.ROOT).as_posix() for pg in pages))


def cmd_render(args) -> None:
    from . import plan

    p = json.loads((paths.DOCS_DATA / "plan.json").read_text(encoding="utf-8"))
    pages = plan.render_pages(p)
    log("render -> " + ", ".join(pg.relative_to(paths.ROOT).as_posix() for pg in pages))


def cmd_all(args) -> None:
    if not paths.CONTEXT.exists():
        cmd_context(args)
    from . import spotter

    if not spotter.SPOTTER.exists():
        args.refresh = False
        cmd_spotter(args)
    cmd_clean(args)
    cmd_matrix(args)
    cmd_solve(args)
    cmd_export(args)


def _brief(i) -> dict:
    return {"id": i.id, "label": i.label, "address": i.address, "lat": round(i.lat, 6), "lon": round(i.lon, 6)}


def inv_details(i) -> list[dict]:
    return i.extra.get("invaders", [])


def _ortools_version() -> str:
    try:
        import ortools

        return ortools.__version__
    except Exception:  # pragma: no cover
        return "?"


# ------------------------------------------------------------------ parser


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="invadrun", description=__doc__)
    ap.add_argument("--version", action="version", version=__version__)
    ap.add_argument("--scope", choices=["paris", "all"], default="paris", help="paris = inside city limits (default); all = every PA_ code")
    ap.add_argument("--keep-destroyed", action="store_true", help="route walls invader-spotter reports as destroyed or hidden too")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("context", help="fetch Paris limits, arrondissements and Seine from OSM").set_defaults(fn=cmd_context)
    p = sub.add_parser("spotter", help="statuses, points and pictures from invader-spotter.art -> data/spotter.json")
    p.add_argument("--refresh", action="store_true", help="re-download the listing pages (polite, ~2 min)")
    p.set_defaults(fn=cmd_spotter)
    sub.add_parser("clean", help="normalise the raw uMap export into data/invaders.geojson (merges spotter.json)").set_defaults(fn=cmd_clean)
    sub.add_parser("graph", help="download/cache the OSM walking network").set_defaults(fn=cmd_graph)

    p = sub.add_parser("matrix", help="street distance matrix between targets")
    p.add_argument("--chunk", type=int, default=64, help="Dijkstra sources per batch (memory/speed trade-off)")
    p.set_defaults(fn=cmd_matrix)

    p = sub.add_parser("solve", help="order the targets (open TSP path)")
    p.add_argument("--start", help="invader code to start from (default: solver picks)")
    p.add_argument("--end", help="invader code to finish at (default: solver picks)")
    p.add_argument("--time-limit", type=int, default=120, help="seconds of local search (default 120)")
    p.add_argument("--log-search", action="store_true")
    p.set_defaults(fn=cmd_solve)

    p = sub.add_parser("export", help="GPX files, plan.json and HTML pages into docs/")
    _export_args(p)
    p.set_defaults(fn=cmd_export)

    sub.add_parser("render", help="re-render HTML pages from docs/data/plan.json").set_defaults(fn=cmd_render)

    p = sub.add_parser("poi", help="Organic Maps POI files (GPX + KML) from docs/data/plan.json")
    p.add_argument("--radius", type=float, default=25.0, help="merge walls closer than this many metres (default 25)")
    p.set_defaults(fn=cmd_poi)

    p = sub.add_parser("all", help="clean + matrix + solve + export")
    p.add_argument("--chunk", type=int, default=64)
    p.add_argument("--start")
    p.add_argument("--end")
    p.add_argument("--time-limit", type=int, default=120)
    p.add_argument("--log-search", action="store_true")
    _export_args(p)
    p.set_defaults(fn=cmd_all)

    args = ap.parse_args(argv)
    args.fn(args)


def _export_args(p) -> None:
    p.add_argument("--stage-km", type=float, default=40.0, help="target length of one stage (default 40)")
    p.add_argument("--pace", default="6:30", help="running pace min:sec per km (default 6:30)")
    p.add_argument("--flash-seconds", type=int, default=45, help="stop time per wall to flash it (default 45)")


if __name__ == "__main__":
    main()
