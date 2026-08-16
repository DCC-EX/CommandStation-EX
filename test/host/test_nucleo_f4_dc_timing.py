"""Host-only regression checks for the Nucleo-F4 DC timing boundary.

These checks intentionally inspect source contracts; the PWM phase result still
requires the bench procedure in Release_Notes/NucleoF4_DC_Timing.md.
"""

from pathlib import Path
import unittest


ROOT = Path(__file__).parents[2]


class NucleoF4DCTimingContractTests(unittest.TestCase):
    def test_frequency_is_configured_before_duty(self):
        source = (ROOT / "MotorDriver.cpp").read_text()
        block = source[source.index("void MotorDriver::setDCSignal"):]
        self.assertLess(
            block.index("DCCEXanalogWriteFrequency(brakePin, f)"),
            block.index("DCCEXanalogWrite(brakePin, brake, invertBrake)"),
        )

    def test_timer_registers_are_variant_gated(self):
        source = (ROOT / "DCCTimerSTM32.cpp").read_text()
        for timer in ("TIM1", "TIM2", "TIM3", "TIM4", "TIM9"):
            self.assertIn(f"#if defined({timer})", source)

    def test_no_track_manager_power_replay_for_timing(self):
        source = (ROOT / "TrackManager.cpp").read_text()
        self.assertNotIn("re-initialize DC mode timer settings", source)


if __name__ == "__main__":
    unittest.main()
