# invadrun — see README.md
UV ?= uv

.PHONY: setup test clean-data matrix solve export render all serve

setup:            ## create .venv and install (needs uv: https://docs.astral.sh/uv/)
	$(UV) sync

test:
	$(UV) run pytest -q

clean-data:       ## raw uMap export -> data/invaders.geojson
	$(UV) run invadrun clean

matrix:           ## download OSM walk graph (once) + street distance matrix
	$(UV) run invadrun matrix

solve:            ## order the invaders (TIME=seconds of local search)
	$(UV) run invadrun solve --time-limit $(or $(TIME),120)

export:           ## GPX + plan.json + HTML pages into docs/
	$(UV) run invadrun export

render:           ## re-render HTML from docs/data/plan.json (no compute)
	$(UV) run invadrun render

all: clean-data matrix solve export

serve:            ## preview docs/ at http://localhost:8000
	cd docs && python3 -m http.server 8000
