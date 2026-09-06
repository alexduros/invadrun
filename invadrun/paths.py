"""Project layout. Everything is relative to the repository root."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
RAW_UMAP = DATA / "raw" / "invaders_umap.json"
INVADERS = DATA / "invaders.geojson"
CONTEXT = DATA / "context.geojson"
EXCLUSIONS = DATA / "exclusions.txt"
CACHE = ROOT / "cache"
GRAPH = CACHE / "paris_walk.graphml"
MATRIX = CACHE / "matrix.npz"
ROUTE = CACHE / "route.json"
DOCS = ROOT / "docs"
DOCS_DATA = DOCS / "data"
TEMPLATES = Path(__file__).resolve().parent / "templates"
