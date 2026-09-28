"""Host-side regression checks for the platform-independent S88 HAL contract.

The embedded project is compiled by PlatformIO in CI.  These checks run
without an Arduino toolchain and protect the review-sensitive properties that
can be verified from source: no timer-specific implementation, bounded packed
state, compile-time configuration checks, and coverage of the supported CI
targets.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise AssertionError(message)


source = (ROOT / "IO_S88.h").read_text(encoding="utf-8")
device = (ROOT / "IODevice.cpp").read_text(encoding="utf-8")
platformio = (ROOT / "platformio.ini").read_text(encoding="utf-8")
workflow = (ROOT / ".github" / "workflows" / "main.yml").read_text(encoding="utf-8")
hardware = (ROOT / "S88.md").read_text(encoding="utf-8")
exrail = (ROOT / "EXRAILMacros.h").read_text(encoding="utf-8")

# The old contribution used an AVR Timer 5 ISR.  The HAL driver must remain
# valid on megaAVR, Teensy, and other Arduino architectures.
for forbidden in ("ISR(", "TCCR5", "TIMSK5", "OCR5A", "delayMicroseconds("):
    require(forbidden not in source, f"S88 driver contains platform-specific {forbidden}")

require("class S88 : public IODevice" in source, "S88 HAL class is missing")
require("_hasCallback = true" in source, "S88 input callbacks are not enabled")
require("IONotifyCallback::invokeAll" in source, "S88 changes are not published")
require("static constexpr uint16_t stateBytes" in source, "S88 RAM sizing helper is missing")
require("MAX_PINS = 255" in source, "S88 VPIN bound is not explicit")
require("MAX_STATE_BYTES = 32" in source, "S88 packed-state bound is not explicit")


def state_bytes(n_pins):
    return (n_pins + 7) // 8


require(state_bytes(128) * 2 == 32, "128-input S88 RAM expectation changed")
require(state_bytes(255) * 2 == 64, "maximum S88 RAM expectation changed")

require("S88_NUM_PINS" in device, "config.h S88 path is missing")
for assertion in (
    "S88_NUM_PINS <= S88::MAX_PINS",
    "S88::stateBytes(S88_NUM_PINS) <= S88::MAX_STATE_BYTES",
    "S88_CLOCK_PERIOD_MICROS >= S88::MIN_CLOCK_PERIOD_MICROS",
    "S88_CLOCK_PIN != S88_LOAD_PIN",
    "S88_CLOCK_PIN < NUM_DIGITAL_PINS",
):
    require(assertion in device, f"compile-time S88 check is missing: {assertion}")

require("[env:mega2560-s88]" in platformio, "Mega S88 compile target is missing")
require("[env:Teensy3_2-s88]" in platformio, "Teensy S88 compile target is missing")
require("#define HAL(haltype,params...)  haltype::create(params);" in exrail,
        "EX-RAIL HAL expansion is not available")
require("HAL(S88," in hardware, "S88 EX-RAIL usage is not documented")
require("platformio run -e mega2560-s88" in workflow, "Mega S88 CI step is missing")
require("platformio run -e Teensy3_2-s88" in workflow, "Teensy S88 CI step is missing")
for marker in ("CLOCK", "LOAD", "RESET", "DATA", "S88_INVERT_DATA", "S88_DATA_PULLUP"):
    require(marker in hardware, f"hardware criteria omit {marker}")

print("S88 host regression checks passed")
