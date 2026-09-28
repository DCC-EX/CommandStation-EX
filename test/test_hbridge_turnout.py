import pathlib
import unittest


class HBridgeOutputModel:
    MAX_PULSE_MS = 500

    def __init__(self, pulse_ms):
        self.pulse_ms = min(pulse_ms, self.MAX_PULSE_MS)
        self.outputs = {"throw": 0, "close": 0}
        self.deadline = None

    def command(self, closed, now_ms):
        self.outputs = {"throw": 0, "close": 0}
        self.outputs["close" if closed else "throw"] = 1
        self.deadline = now_ms + self.pulse_ms

    def loop(self, now_ms):
        if self.deadline is not None and now_ms >= self.deadline:
            self.outputs = {"throw": 0, "close": 0}
            self.deadline = None


class HBridgeTurnoutTests(unittest.TestCase):
    def test_throw_only_and_pulse_end(self):
        model = HBridgeOutputModel(100)
        model.command(False, 10)
        self.assertEqual(model.outputs, {"throw": 1, "close": 0})
        model.loop(109)
        self.assertEqual(model.outputs, {"throw": 1, "close": 0})
        model.loop(110)
        self.assertEqual(model.outputs, {"throw": 0, "close": 0})

    def test_close_only(self):
        model = HBridgeOutputModel(100)
        model.command(True, 10)
        self.assertEqual(model.outputs, {"throw": 0, "close": 1})
        model.loop(110)
        self.assertEqual(model.outputs, {"throw": 0, "close": 0})

    def test_pulse_cap_and_repeated_command(self):
        model = HBridgeOutputModel(900)
        model.command(False, 0)
        model.command(True, 20)
        self.assertEqual(model.outputs, {"throw": 0, "close": 1})
        model.loop(519)
        self.assertEqual(model.outputs, {"throw": 0, "close": 1})
        model.loop(520)
        self.assertEqual(model.outputs, {"throw": 0, "close": 0})


class SourceContractTests(unittest.TestCase):
    def test_non_blocking_and_standard_types(self):
        root = pathlib.Path(__file__).parents[1]
        turnouts = (root / "Turnouts.cpp").read_text()
        pin = (root / "IO_HBridgePin.h").read_text()
        types = (root / "Turnouts.h").read_text()
        self.assertIn("HBridgePin::create", turnouts)
        self.assertIn("IODevice::write(inactiveVpin, LOW)", turnouts)
        self.assertIn("delayUntil", pin)
        self.assertNotIn("delay(", turnouts)
        for name in ("TURNOUT_DCC", "TURNOUT_SERVO", "TURNOUT_VPIN"):
            self.assertIn(name, types)


if __name__ == "__main__":
    unittest.main()
