"""Source-level protocol regression checks for the identity/power contract.

These checks intentionally avoid Arduino dependencies and run in CI before a
board build is available.
"""
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class IdentityPowerContractTest(unittest.TestCase):
    def test_identity_is_an_additive_explicit_query(self):
        parser = (ROOT / "DCCEXParser.cpp").read_text(encoding="utf-8")
        self.assertIn("case 'i': // DEVICE IDENTITY <i>", parser)
        self.assertIn('if (params != 0) break;', parser)
        self.assertIn('F("<iID %02X%02X%02X%02X%02X%02X>\\n")', parser)

    def test_status_remains_power_and_object_status(self):
        parser = (ROOT / "DCCEXParser.cpp").read_text(encoding="utf-8")
        status = parser[parser.index("case 's': // STATUS <s>"):]
        self.assertIn("CommandDistributor::broadcastPower();", status)
        self.assertIn("Turnout::printAll(stream);", status)
        self.assertIn("Sensor::printAll(stream);", status)

    def test_join_has_an_explicit_power_broadcast(self):
        track = (ROOT / "TrackManager.cpp").read_text(encoding="utf-8")
        join = track[track.index("void TrackManager::setJoin(bool joined)"):]
        self.assertIn("CommandDistributor::broadcastPower();", join)

    def test_watchdog_contract_is_connection_local_and_fail_safe(self):
        distributor = (ROOT / "CommandDistributor.cpp").read_text(encoding="utf-8")
        self.assertIn('buffer[2]==\' \' && buffer[3]==\'W\'', distributor)
        self.assertIn('F("<# W %d>\\n")', distributor)
        self.assertIn('DCC::estopAll();', distributor)
        self.assertIn('F("<# TIMEOUT %d>\\n")', distributor)
        self.assertIn('watchdogEnabled[clientId]=false;', distributor)


if __name__ == "__main__":
    unittest.main()
