"""invader-spotter.art listing: fetch (with a session cookie) and parse.

The site has no API and its listing is a POST form, so we page through
``listing.php`` once, cache the HTML under ``cache/spotter/`` and keep a
small JSON snapshot in ``data/spotter.json``: status, points, dates and
picture URLs per invader code. Please keep requests rare and polite.
"""

from __future__ import annotations

import html
import json
import re
import time
from datetime import date
from pathlib import Path

from . import paths

BASE = "https://www.invader-spotter.art/"
UA = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/128.0 Safari/537.36 invadrun/0.2"
SPOTTER = paths.DATA / "spotter.json"
PAGES = paths.CACHE / "spotter"

ENTRY_RE = re.compile(
    r'<img src="grosplan/(?P<city>[A-Z]+)/(?P<code>[A-Z]+_\d+)-grosplan\.png"[^>]*>.*?'
    r"<b>(?P<short>[A-Z]+_\d+)(?:\s*\[(?P<pts>\d+)\s*pts?\])?</b>"
    r"(?P<body>.*?)</font></td>(?P<rest>.*?)</tr>\s*<tr>",
    re.S,
)
PHOTO_RE = re.compile(r"href='(photos/[A-Z]+/[^']+)'[^>]*>\s*<img src='(images/[A-Z]+/[^']+)'")
STATUS_RE = re.compile(r"Dernier &eacute;tat connu\s*:\s*(?:<img src='nav/(?P<icon>[^']+)'[^>]*>)?\s*(?P<text>[^<]*)<br/>")
STATUS_DATE_RE = re.compile(r"Date et source\s*:\s*([^<]*)<")
POSE_RE = re.compile(r"Date de pose\s*:\s*([^<]*)<")
ARR_RE = re.compile(r"lienv\(\"[A-Z]+\",\"(\d+)\"\)")
INSTA_RE = re.compile(r"href='(https://www\.instagram\.com/explore/tags/[^']+)'")

# Icon -> status, used only when the text next to it is empty. The site reuses
# the "destroyed" icon for "Très dégradé", so the text is the primary source.
STATUS_KEY = {
    "spot_invader_ok.png": "ok",
    "spot_invader_degraded.png": "damaged",
    "spot_invader_destroyed.png": "destroyed",
    "spot_invader_neutre.png": "unknown",
}


def clean_text(s: str) -> str:
    return re.sub(r"\s+", " ", html.unescape(s)).strip()


def status_key(icon: str | None, text: str) -> str:
    t = clean_text(text).lower()
    if t:
        if "détruit" in t or "detruit" in t or "disparu" in t:
            return "destroyed"
        if "très dégradé" in t or "tres degrade" in t:
            return "very_damaged"
        if "dégradé" in t or "degrade" in t or "abîmé" in t:
            return "damaged"
        if "caché" in t or "cache" in t or "recouvert" in t or "non visible" in t:
            return "hidden"
        if t.startswith("ok") or "réactivé" in t or "reactive" in t:
            return "ok"
    if icon and icon in STATUS_KEY:
        return STATUS_KEY[icon]
    return t or "unknown"


def parse_page(text: str) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for m in ENTRY_RE.finditer(text):
        code = m.group("code")
        num = code.split("_")[1].zfill(4)
        code = f"{m.group('city')}_{num}"
        body, rest = m.group("body"), m.group("rest")
        st = STATUS_RE.search(body)
        photo = PHOTO_RE.search(rest)
        arr = ARR_RE.search(body)
        rec = {
            "points": int(m.group("pts")) if m.group("pts") else None,
            "status": status_key(st.group("icon") if st else None, st.group("text") if st else ""),
            "status_text": clean_text(st.group("text")) if st else "",
            "status_date": clean_text(STATUS_DATE_RE.search(body).group(1)) if STATUS_DATE_RE.search(body) else "",
            "installed": clean_text(POSE_RE.search(body).group(1)) if POSE_RE.search(body) else "",
            "arrondissement": int(arr.group(1)) if arr else None,
            "closeup": BASE + f"grosplan/{m.group('city')}/{m.group('code')}-grosplan.png",
            "photo": BASE + photo.group(2) if photo else None,
            "photo_full": BASE + photo.group(1) if photo else None,
            "instagram": INSTA_RE.search(body).group(1) if INSTA_RE.search(body) else None,
        }
        out[code] = rec
    return out


def parse_cached(pages_dir: Path = PAGES) -> dict[str, dict]:
    data: dict[str, dict] = {}
    for f in sorted(pages_dir.glob("PA_lst_p*.html")):
        data.update(parse_page(f.read_text(encoding="utf-8", errors="replace")))
    return data


def fetch(city: str = "PA", pages_dir: Path = PAGES, delay_s: float = 1.5, log=print) -> int:
    """Download every listing page for ``city`` into ``pages_dir``."""
    import urllib.request
    from http.cookiejar import CookieJar

    jar = CookieJar()
    opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar))
    opener.addheaders = [("User-Agent", UA), ("Referer", BASE + "villes.php")]
    opener.open(BASE + "villes.php", timeout=60).read()
    pages_dir.mkdir(parents=True, exist_ok=True)
    n = 0
    for page in range(1, 200):
        form = urllib.parse.urlencode({"ville": city, "arron": "00", "mode": "lst", "rang": "10", "siid": "oui", "etat": "oui", "page": page}).encode()
        text = opener.open(BASE + "listing.php", data=form, timeout=90).read().decode("utf-8", errors="replace")
        if "grosplan.png" not in text:
            break
        (pages_dir / f"{city}_lst_p{page:02d}.html").write_text(text, encoding="utf-8")
        n += 1
        log(f"  page {page}: {text.count('grosplan.png')} entries")
        time.sleep(delay_s)
    return n


def save(data: dict[str, dict], out: Path = SPOTTER) -> None:
    payload = {"source": BASE + "villes.php", "fetched": date.today().isoformat(), "count": len(data), "invaders": dict(sorted(data.items()))}
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=0), encoding="utf-8")


def load(path: Path = SPOTTER) -> dict[str, dict]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))["invaders"]
