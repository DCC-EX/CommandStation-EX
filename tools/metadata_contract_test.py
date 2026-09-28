"""Regression checks for the CommandStation host metadata contract."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DCC = (ROOT / "DCC.cpp").read_text(encoding="utf-8")
PARSER = (ROOT / "DCCEXParser.cpp").read_text(encoding="utf-8")
DOC = (ROOT / "CommandStation-Metadata.md").read_text(encoding="utf-8")


assert DCC.count("void DCC::printMetadata(Print *stream)") == 1
assert DCC.count("printMetadata(&USB_SERIAL)") == 1
assert PARSER.count("case 'i': // PRODUCT/DEVICE/RELEASE METADATA <i>") == 1
assert PARSER.count("DCC::printMetadata(stream);") == 2
assert "side-effect-free `<i>` command" in DOC
assert "<iDCC-EX V-" in DCC
