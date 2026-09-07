# invadrun

The shortest walk past every Space Invader in Paris, as a GPX track and a
stage-by-stage route book for a fastest-known-time attempt.

- **Route book** (`docs/plan.html`): stages, cue sheets, time budget, downloads.
- **Map viewer** (`docs/viewer.html`): the route and every wall on OpenStreetMap tiles, with pictures.
- **Project page** (`docs/index.html`): what it is, how it is built, sources.
- **Files** (`docs/data/`): `invadrun.gpx` and `stage_NN.gpx` for a watch,
  `invadrun-poi.kml` / `invadrun-poi.gpx` for Organic Maps (one pin per place,
  pictures, points, status), `plan.json` with everything.

Current route (scope `paris`, snapshot of invader-spotter from September 2024):

| walls | invaders | distance | stages | estimate at 6:30/km + 45 s per wall |
|------:|---------:|---------:|-------:|------------------------------------:|
| 1,135 | 1,178    | 276 km   | 7 × ~40 km | 44 h (30 h running, 14 h at the walls) |

Start and finish are chosen by the solver. Numbers change with the solver
time budget, the exclusions file and the scope; see below.

## How it works

```
data/raw/invaders_umap.json   uMap export of invader-spotter.art locations (1,264 points)
data/spotter.json             invadrun spotter: status, points, dates, pictures per code (invader-spotter.art)
        │  invadrun clean      fix encoding, parse "PA_0035 & PA_0482", tag city limits + arrondissement,
        │                      merge statuses (destroyed/hidden walls leave the route)
        ▼
data/invaders.geojson         one feature per wall, clean codes
        │  invadrun matrix     OSM walking graph (osmnx, cached) → snap walls → scipy Dijkstra per wall
        ▼
cache/matrix.npz              1,135 × 1,135 metres along footpaths (~10 s)
        │  invadrun solve      OR-Tools open TSP path, free or fixed start/end, guided local search
        ▼
cache/route.json              visiting order
        │  invadrun export     shortest path per leg → GPX (full + per stage), Organic Maps POIs, plan.json, HTML
        ▼
docs/                         static site (GitHub Pages ready)
```

The distance matrix is the expensive part in principle. Instead of one
point-to-point search per pair (~640,000 for 1,135 walls) each wall is the
source of a single Dijkstra on a sparse CSR graph, so the whole matrix takes
seconds and the solver gets exact street distances.

## Setup

Needs Python 3.11+ and [uv](https://docs.astral.sh/uv/).

```sh
uv sync                 # creates .venv with osmnx, OR-Tools, scipy, gpxpy
uv run pytest           # a few tests on the data cleaning and the solver
```

## Run

```sh
uv run invadrun all --time-limit 240      # whole pipeline, ~6 min on a laptop
```

Or step by step:

```sh
uv run invadrun context                   # once: Paris limits, arrondissements, Seine (data/context.geojson)
uv run invadrun spotter [--refresh]       # statuses/points/pictures from invader-spotter.art (~2 min, polite)
uv run invadrun clean                     # data/invaders.geojson + a summary
uv run invadrun matrix                    # downloads the graph on first run (~1 min, 140 MB in cache/)
uv run invadrun solve --time-limit 600    # more time = shorter route; --start PA_0041 --end PA_1000 to pin ends
uv run invadrun export --stage-km 50 --pace 6:00 --flash-seconds 30
uv run invadrun render                    # re-render HTML from docs/data/plan.json only
uv run invadrun poi --radius 25           # rebuild the Organic Maps files only
make serve                                # http://localhost:8000
```

Options:

- `--scope paris` (default) routes the walls inside the city limits;
  `--scope all` routes every `PA_` code, suburbs and airport included
  (much larger graph download).
- Walls whose every code is reported *destroyed* or *hidden* by
  invader-spotter are left out; `--keep-destroyed` routes them anyway.
- `data/exclusions.txt`: one code per line to skip for any other reason.
  Re-run from `clean`.
- Stage boundaries fall on a wall; consecutive stages share it, so splitting
  costs no distance. Estimates count running at the given pace plus a fixed
  stop per wall, nothing else.

## Layout

```
invadrun/        package: data, context, graph, matrix, tsp, gpx, plan, cli
invadrun/templates/   HTML templates (+ partials) rendered into docs/
data/            raw export, cleaned GeoJSON, map context, exclusions
docs/            generated site: index, plan, viewer, data/*.gpx, data/plan.json
cache/           git-ignored: OSM graph, distance matrix, route
tests/
```

## Organic Maps

`docs/data/invadrun-poi.kml` (or the `.gpx` twin) is made for
[Organic Maps](https://organicmaps.app/): open the file on the phone and choose
Organic Maps, or *Bookmarks & Tracks → Import*. You get one bookmark list with:

- one pin per **place**: walls closer than 25 m are merged, and a wall already
  holds every code painted on it (e.g. `PA_0290 · PA_0532 (#12)`); the number is
  the stop on the route, the colour is the stage;
- a description with, for each invader, its **picture and close-up**, points,
  last known status and date, installation date and Instagram tag, plus the
  stop number, stage and kilometre;
- the route itself as one track per stage.

Pictures are loaded from invader-spotter.art when the pin is opened, so they
need a connection the first time. Credit and thanks to the spotters.

## Deploy

The site is <https://invadrun.duros.fr/>: the `docs/` folder served by GitHub
Pages (branch `main`, path `/docs`, custom domain in `docs/CNAME`, HTTPS
enforced). `alexduros.github.io/invadrun/` redirects there. Regenerate with
`uv run invadrun export` (or `render`), commit `docs/`, push; Pages rebuilds
in about a minute.

DNS for `duros.fr` is hosted at Vercel: `invadrun  CNAME  alexduros.github.io.`
If the domain ever needs re-binding:

```sh
gh api -X PUT repos/alexduros/invadrun/pages -f cname=invadrun.duros.fr   # writes docs/CNAME
gh api -X PUT repos/alexduros/invadrun/pages -F https_enforced=true       # once the certificate is issued
```

Optional hardening: GitHub → Settings → Pages → *Add a verified domain* for
`duros.fr`, so no other account can bind a Pages site to it.

## Sources

- Wall locations: uMap export of <https://www.invader-spotter.art/villes.php>.
  Statuses, points, dates and pictures: same site, read by `invadrun spotter`
  (keep it rare: one polite pass through the listing). Invaders newer than the
  location layer cannot be routed until someone maps them.
- Streets: © OpenStreetMap contributors, ODbL, via [osmnx](https://osmnx.readthedocs.io/).
- Solver: [OR-Tools](https://developers.google.com/optimization) routing library.

Code is released under the Unlicense (see `LICENSE`). The art belongs to Invader.
