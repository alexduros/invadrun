from invadrun.spotter import parse_page, status_key

SNIPPET = """<tr class="haut">
<td align="left" rowspan="2" width="300"><img src="grosplan/PA/PA_0002-grosplan.png" title="---" width="100"><br/><font class="normal"><b>PA_02 [10 pts]</b><br/>Date de pose : 12/03/1998<br/>(<a href='javascript:lienv("PA","04");'>Paris - 4&egrave;me arrondissement</a>)<br/>Dernier &eacute;tat connu : <img src='nav/spot_invader_ok.png' class='banniere' width=25> OK<br/>Date et source : juin 2019 (spott)<br/>Instagram: <a href='https://www.instagram.com/explore/tags/pa_02/' target='_blank'>hashtag #PA_02</a></font></td>
<td width=300 align=left><div id='image2'><a class='chocolat-image' href='photos/PA/PA_0002-juin2019.jpg' title='PA_0002-juin2019'><img src='images/PA/PA_0002-juin2019.jpg' width=300 class='imagelien'></a></div></td>
</tr>
<tr>
<td>legende</td></tr>
<tr class="haut">
<td align="left" rowspan="2" width="300"><img src="grosplan/PA/PA_0001-grosplan.png" title="---" width="100"><br/><font class="normal"><b>PA_01 [10 pts]</b><br/>Date de pose : 15/01/1998<br/>(<a href='javascript:lienv("PA","11");'>Paris - 11&egrave;me arrondissement</a>)<br/>Dernier &eacute;tat connu : <img src='nav/spot_invader_detruit.png' class='banniere' width=25> D&eacute;truit !<br/>Date et source : mai 2025<br/></font></td>
<td width=300 align=left>&nbsp;</td>
</tr>
<tr>
<td>x</td></tr>
"""


def test_parse_page_extracts_status_points_pictures():
    d = parse_page(SNIPPET)
    assert set(d) == {"PA_0002", "PA_0001"}
    ok = d["PA_0002"]
    assert ok["points"] == 10 and ok["status"] == "ok" and ok["arrondissement"] == 4
    assert ok["installed"] == "12/03/1998" and ok["status_date"] == "juin 2019 (spott)"
    assert ok["photo"].endswith("images/PA/PA_0002-juin2019.jpg") and ok["photo_full"].endswith("photos/PA/PA_0002-juin2019.jpg")
    assert ok["closeup"].endswith("grosplan/PA/PA_0002-grosplan.png")
    assert ok["instagram"] == "https://www.instagram.com/explore/tags/pa_02/"
    gone = d["PA_0001"]
    assert gone["status"] == "destroyed" and gone["status_text"] == "Détruit !" and gone["photo"] is None


def test_status_key_falls_back_to_text():
    assert status_key(None, "Un peu d&eacute;grad&eacute;") == "damaged"
    assert status_key(None, "Non visible") == "hidden"
    assert status_key("spot_invader_destroyed.png", "") == "destroyed"
    assert status_key("spot_invader_destroyed.png", "Tr&egrave;s d&eacute;grad&eacute;") == "very_damaged"
