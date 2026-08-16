"""Host regression tests for the CommandStation signal aspect-map contract."""

MAIN = 0o421
ALT = 0o431


def pin_map(aspect, state, has_amber=True):
    if not has_amber and state == "amber":
        return ((aspect >> 6) & 0o7) | (aspect & 0o7)
    return {"red": aspect >> 6, "amber": (aspect >> 3) & 0o7, "green": aspect & 0o7}[state]


def output(aspect, state, active_high, has_amber=True):
    mask = pin_map(aspect, state, has_amber)
    values = [(mask & bit) != 0 for bit in (0o4, 0o2, 0o1)]
    return values if active_high else [not value for value in values]


def test_main_map_preserves_legacy_low_active_signal():
    assert output(MAIN, "red", False) == [False, True, True]
    assert output(MAIN, "amber", False) == [True, False, True]
    assert output(MAIN, "green", False) == [True, True, False]


def test_high_active_signal_is_the_polarity_inverse():
    assert output(MAIN, "red", True) == [True, False, False]


def test_alternate_map_can_light_red_and_green_for_amber():
    assert output(ALT, "amber", True) == [False, True, True]


def test_missing_amber_pin_synthesizes_red_and_green():
    assert output(MAIN, "amber", True, has_amber=False) == [True, False, True]


if __name__ == "__main__":
    tests = [value for name, value in globals().items() if name.startswith("test_")]
    for test in tests:
        test()
    print(f"{len(tests)} signal aspect tests passed")
