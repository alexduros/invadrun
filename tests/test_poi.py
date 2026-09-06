import xml.etree.ElementTree as ET

from invadrun.poi import describe_html, group_places, place_name, write_gpx, write_kml


def _row(i, code, lat, lon, stage=1, **extra):
    return {"i": i, "id": code, "label": code, "codes": [code], "address": f"{i} rue Test", "arr": 11, "lat": lat, "lon": lon,
            "cum_km": i * 0.3, "stage": stage, "points": 20, "status": "ok",
            "invaders": [{"code": code, "status": "ok", "points": 20, "photo": "https://x/p.jpg", "closeup": "https://x/c.png"}], **extra}


ROUTE = [
    _row(1, "PA_0001", 48.8600, 2.3500),
    _row(2, "PA_0002", 48.86005, 2.35008),   # ~10 m from #1
    _row(3, "PA_0003", 48.8700, 2.3600, stage=2),
]
PLAN = {"route": ROUTE, "stages": [{"n": 1, "km": 1, "walls": 2, "poly_start": 0, "poly_end": 1}, {"n": 2, "km": 1, "walls": 1, "poly_start": 1, "poly_end": 2}],
        "polyline": [[48.86, 2.35], [48.865, 2.355], [48.87, 2.36]]}


def test_groups_nearby_walls_and_keeps_route_order():
    groups = group_places(ROUTE, radius_m=25)
    assert [[r["id"] for r in g] for g in groups] == [["PA_0001", "PA_0002"], ["PA_0003"]]
    assert place_name(groups[0]) == "PA_0001 · PA_0002 (#1–2)"
    assert place_name(groups[1]) == "PA_0003 (#3)"


def test_description_carries_pictures_and_route_position():
    html = describe_html(group_places(ROUTE, 25)[0], total=3)
    assert "https://x/p.jpg" in html and "https://x/c.png" in html
    assert "Stop #1 of 3" in html and "20 pts" in html


def test_gpx_and_kml_are_well_formed(tmp_path):
    groups = group_places(ROUTE, 25)
    gpx = ET.parse(write_gpx(tmp_path / "poi.gpx", PLAN, groups)).getroot()
    ns = {"g": "http://www.topografix.com/GPX/1/1", "osmand": "https://osmand.net"}
    wpts = gpx.findall("g:wpt", ns)
    assert len(wpts) == 2 and len(gpx.findall("g:trk", ns)) == 2
    assert wpts[0].find("g:extensions/osmand:color", ns).text == "#2a78d6"
    kml = ET.parse(write_kml(tmp_path / "poi.kml", PLAN, groups)).getroot()
    k = {"k": "http://www.opengis.net/kml/2.2"}
    marks = kml.findall("k:Document/k:Placemark", k)
    assert len(marks) == 4  # 2 places + 2 stage tracks
    assert marks[0].find("k:styleUrl", k).text == "#placemark-blue"
