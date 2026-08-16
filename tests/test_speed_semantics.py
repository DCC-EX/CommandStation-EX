from pathlib import Path

ROOT = Path(__file__).parents[1]

def output_speed(commanded, limit=127):
    return limit if commanded > 1 and commanded > limit else commanded

def test_limit_preserves_command_and_clamps_output():
    assert output_speed(80, 30) == 30

def test_stop_and_emergency_stop_are_not_clamped():
    assert output_speed(0, 30) == 0
    assert output_speed(1, 30) == 1

def test_unlimited_default_is_backward_compatible():
    for speed in (0, 1, 2, 30, 127):
        assert output_speed(speed) == speed

def test_exrail_surface_and_guard_exist():
    base = (ROOT / "EXRAIL2MacroBase.h").read_text()
    macros = (ROOT / "EXRAILMacros.h").read_text()
    asserts = (ROOT / "EXRAILAsserts.h").read_text()
    assert "#define SPEED_LIMIT(speed)" in base
    assert "#define CLEAR_SPEED_LIMIT" in base
    assert "OPCODE_SPEED_LIMIT" in macros
    assert "OPCODE_CLEAR_SPEED_LIMIT" in macros
    assert "speed>=2 && speed<128" in asserts

def test_loco_slot_adds_documented_limit_state():
    slot = (ROOT / "LocoSlot.h").read_text()
    assert "byte speedLimit;" in slot
    assert "127 means unlimited" in slot
