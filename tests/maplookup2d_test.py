# pylint: disable-all


from utils.mapLookup2d import MapLookup2D


def test_nopoints():
    mut = MapLookup2D([])
    assert mut.lookup(5) == 0
    assert mut.lookup(-3.2) == 0


def test_onepoint():
    mut = MapLookup2D([(3, 4)])
    assert mut.lookup(0) == 4
    assert mut.lookup(-2.1) == 4
    assert mut.lookup(9999) == 4


def test_linear():
    mut = MapLookup2D(
        [
            (1, 1),
            (10, 10),
            (100, 100),
            (500, 500),
            (3, 3),
        ]
    )

    for testVal in range(1, 500):
        assert abs(mut.lookup(testVal) - testVal) < 0.00001


def test_onpoints():
    mut = MapLookup2D(
        [
            (1, 4),
            (3, 2),
            (5, 2),
            (6, 3),
            (9, 0),
        ]
    )

    assert mut.lookup(0) == 4
    assert mut.lookup(1) == 4
    assert mut.lookup(3) == 2
    assert mut.lookup(5) == 2
    assert mut.lookup(6) == 3
    assert mut.lookup(9) == 0
    assert mut.lookup(9.1) == 0
