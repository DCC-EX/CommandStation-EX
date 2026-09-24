"""Host-side structural checks for the EXRAIL E-stop predicate."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(name):
    return (ROOT / name).read_text(encoding="utf-8")


def test_if_estop_paused_is_wired_once():
    assert read("EXRAILMacros.h").count("#define IF_ESTOP_PAUSED") == 1
    assert read("EXRAIL2.h").count("OPCODE_IF_ESTOP_PAUSED") == 1
    assert read("EXRAIL2MacroReset.h").count("#undef IF_ESTOP_PAUSED") == 1
    assert read("EXRAIL2MacroBase.h").count("#define IF_ESTOP_PAUSED") == 1


def test_predicate_is_fail_safe_and_three_bytes():
    assert "skipIf=!DCC::isEstopLocked();" in read("EXRAIL2.cpp")
    assert "ESTOP_PAUSE" in read("EXRAILTest.h")
    assert "ESTOP_RESUME" in read("EXRAILTest.h")
    assert "IF_ESTOP_PAUSED" in read("EXRAILTest.h")
    assert "#define IF_ESTOP_PAUSED OPCODE_IF_ESTOP_PAUSED,0,0," in read(
        "EXRAILMacros.h"
    )
