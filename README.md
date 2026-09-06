# invadrun

The shortest walk past every Space Invader in Paris, as a GPX track and a
stage-by-stage route book for a fastest-known-time attempt.

- **Route book** (`docs/plan.html`): stages, cue sheets, time budget, downloads.
- **Map viewer** (`docs/viewer.html`): the route and every wall on OpenStreetMap tiles.
- **Project page** (`docs/index.html`): what it is, how it is built, sources.

Current route (scope `paris`, snapshot of invader-spotter from September 2024):

| walls | invaders | distance | stages | estimate at 6:30/km + 45 s per wall |
|------:|---------:|---------:|-------:|------------------------------------:|
| 1,135 | 1,178    | 276 km   | 7 × ~40 km | 44 h (30 h running, 14 h at the walls) |

Start and finish are chosen by the solver. Numbers change with the solver
time budget, the exclusions file and the scope; see below.

## How it works

```
data/raw/invaders_umap.json   uMap export of invader-spotter.art (1,264 points)
        │  invadrun clean      fix encoding, parse "PA_0035 & PA_0482", tag city limits + arrondissement
        ▼
data/invaders.geojson         one feature per wall, clean codes
        │  invadrun matrix     OSM walking graph (osmnx, cached) → snap walls → scipy Dijkstra per wall
        ▼
cache/matrix.npz              1,135 × 1,135 metres along footpaths (~10 s)
        │  invadrun solve      OR-Tools open TSP path, free or fixed start/end, guided local search
        ▼
cache/route.json              visiting order
        │  invadrun export     shortest path per leg → GPX (full + per stage), plan.json, HTML pages
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
uv run invadrun clean                     # data/invaders.geojson + a summary
uv run invadrun matrix                    # downloads the graph on first run (~1 min, 140 MB in cache/)
uv run invadrun solve --time-limit 600    # more time = shorter route; --start PA_0041 --end PA_1000 to pin ends
uv run invadrun export --stage-km 50 --pace 6:00 --flash-seconds 30
uv run invadrun render                    # re-render HTML from docs/data/plan.json only
make serve                                # http://localhost:8000
```

Options:

- `--scope paris` (default) routes the walls inside the city limits;
  `--scope all` routes every `PA_` code, suburbs and airport included
  (much larger graph download).
- `data/exclusions.txt`: one code per line to skip (destroyed, covered,
  unreachable). Re-run from `clean`.
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

## Deploy

`docs/` is served by GitHub Pages (branch `main`, path `/docs`):
<https://alexduros.github.io/invadrun/>. Regenerate with `uv run invadrun export`
(or `render`), commit `docs/`, push; Pages rebuilds in about a minute.

Custom domain **invadrun.duros.fr** (DNS for `duros.fr` is hosted at Vercel):

1. Vercel → Domains → `duros.fr` → DNS records, add
   `invadrun  CNAME  alexduros.github.io.` (TTL 60). Today the name falls
   through to a wildcard that answers with a Vercel 404, so a specific record
   is required.
2. Once `dig +short CNAME invadrun.duros.fr` returns `alexduros.github.io.`,
   point Pages at the domain (this also commits `docs/CNAME`):

   ```sh
   gh api -X PUT repos/alexduros/invadrun/pages -f cname=invadrun.duros.fr
   ```

   From then on `alexduros.github.io/invadrun/` redirects to the domain.
3. GitHub issues a Let's Encrypt certificate a few minutes later; then enforce HTTPS:

   ```sh
   gh api -X PUT repos/alexduros/invadrun/pages -F https_enforced=true
   ```

4. Optional: GitHub → Settings → Pages → *Add a verified domain* for
   `duros.fr`, so nobody else can bind a Pages site to it.

## Sources

- Wall locations: <https://www.invader-spotter.art/villes.php> (uMap export).
  Statuses change often; check before an attempt.
- Streets: © OpenStreetMap contributors, ODbL, via [osmnx](https://osmnx.readthedocs.io/).
- Solver: [OR-Tools](https://developers.google.com/optimization) routing library.

Code is released under the Unlicense (see `LICENSE`). The art belongs to Invader.
