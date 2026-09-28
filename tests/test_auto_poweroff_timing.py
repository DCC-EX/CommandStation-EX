"""Host timing and source-hook tests for the AutoPowerOff policy contract."""

from pathlib import Path

root = Path(__file__).resolve().parents[1]
track_manager = (root / "TrackManager.cpp").read_text()
parser = (root / "DCCEXParser.cpp").read_text()
config = (root / "config.example.h").read_text()
assert "defined(AUTO_POWER_OFF_PIN) && defined(AUTO_POWER_OFF_TIMEOUT_MS)" in track_manager
assert "digitalWrite(AUTO_POWER_OFF_PIN, HIGH)" in track_manager
assert "digitalWrite(AUTO_POWER_OFF_PIN, LOW)" in track_manager
assert parser.count("TrackManager::autoPowerOffActivity();") == 1
assert "#define AUTO_POWER_OFF_PIN" in config
assert "#define AUTO_POWER_OFF_TIMEOUT_MS" in config

UINT32 = 1 << 32

class Timer:
    def __init__(self, timeout, callback):
        self.timeout = timeout
        self.callback = callback
        self.last = 0
        self.armed = False

    def begin(self, now):
        self.last, self.armed = now & 0xffffffff, self.timeout != 0

    def activity(self, now):
        self.last, self.armed = now & 0xffffffff, self.timeout != 0

    def loop(self, now):
        elapsed = ((now & 0xffffffff) - self.last) & 0xffffffff
        if not self.armed or elapsed < self.timeout:
            return False
        self.armed = False
        self.callback()
        return True

calls = []
disabled = Timer(0, lambda: calls.append("disabled"))
disabled.begin(1000)
assert not disabled.loop(1000000) and not calls

timer = Timer(1000, lambda: calls.append("off"))
timer.begin(1000)
assert not timer.loop(1999)
assert timer.loop(2000) and calls == ["off"]
assert not timer.loop(3000)
timer.activity(4000)
assert not timer.loop(4999)
assert timer.loop(5000) and calls == ["off", "off"]

wrapped = Timer(100, lambda: calls.append("wrap"))
wrapped.begin(UINT32 - 50)
assert not wrapped.loop(49)
assert wrapped.loop(50) and calls[-1] == "wrap"

print("auto-poweroff timing tests passed")
