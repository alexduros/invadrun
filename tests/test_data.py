from invadrun.data import fix_mojibake, parse_codes, parse_name


def test_fix_mojibake_single_and_double():
    assert fix_mojibake("Sainte-GeneviÃ¨ve") == "Sainte-Geneviève"
    assert fix_mojibake("rue de l'Ã‰pine") == "rue de l'Épine"
    assert fix_mojibake("carrÃ© Ã\xa0 la MarÃ©e") == "carré à la Marée"
    assert fix_mojibake("plain ascii") == "plain ascii"


def test_parse_codes_normalises_prefix_and_padding():
    assert parse_codes("PA__0290 & PA_0532") == ["PA_0290", "PA_0532"]
    assert parse_codes("PA_0432 & 573") == ["PA_0432", "PA_0573"]
    assert parse_codes("PA_0973 PA_0974 & PA_0975") == ["PA_0973", "PA_0974", "PA_0975"]
    assert parse_codes("VRS_004") == ["VRS_004"]


def test_parse_name_splits_codes_from_address():
    assert parse_name("Space Invader PA_0002, 45 rue Quincampoix") == (["PA_0002"], "45 rue Quincampoix")
    assert parse_name("Space Invader, PA_0227, 15 rue Martel") == (["PA_0227"], "15 rue Martel")
    assert parse_name("Space invader PA_0851, 14 Bis rue de la Grande ChaumiÃ¨re") == (
        ["PA_0851"],
        "14 Bis rue de la Grande Chaumière",
    )


def test_parse_name_keeps_numeric_addresses_out_of_codes():
    assert parse_name("Space Invader PA_0914, 103,5 rue Baudin") == (["PA_0914"], "103,5 rue Baudin")
    assert parse_name("Space Invader PA_0215 & PA_1056, 0 rue Montgallet") == (["PA_0215", "PA_1056"], "0 rue Montgallet")


def test_parse_name_handles_long_comma_lists():
    codes, address = parse_name(
        "Space Invader PA_0778, 0779, 0782, 0783, 0784, 0786, 0787, 0788, 0789 & 0790, périphérique le long des immeubles"
    )
    assert codes == [f"PA_{n:04d}" for n in (778, 779, 782, 783, 784, 786, 787, 788, 789, 790)]
    assert address == "périphérique le long des immeubles"
    assert parse_name("Space Invader PA_0066, PA_0067 & PA_0068, 63 rue des Trois Frères")[0] == ["PA_0066", "PA_0067", "PA_0068"]
