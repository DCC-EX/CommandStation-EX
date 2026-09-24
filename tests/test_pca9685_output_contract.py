"""Source-level regression checks for the PCA9685 output contract."""
from pathlib import Path

ROOT = Path(__file__).parents[1]
DRIVER = (ROOT / "IO_PCA9685.cpp").read_text(encoding="utf-8")
PWM_DRIVER = (ROOT / "IO_PCA9685pwm.h").read_text(encoding="utf-8")
DOC = (ROOT / "docs" / "PCA9685_OUTPUTS.md").read_text(encoding="utf-8")


def test_digital_levels_are_full_on_and_off():
    assert "PCA9685_OUTPUT_ON = 4095" in DRIVER
    assert "PCA9685_OUTPUT_OFF = 0" in DRIVER
    assert "value ? PCA9685_OUTPUT_ON : PCA9685_OUTPUT_OFF" in DRIVER


def test_startup_clears_every_configured_channel():
    assert "for (int pin = 0; pin < _nPins; pin++) writeDevice(pin, PCA9685_OUTPUT_OFF);" in DRIVER
    assert "for (int pin = 0; pin < _nPins; pin++) writeDevice(pin, 0);" in PWM_DRIVER


def test_pwm_driver_implements_level_write():
    assert "void _write(VPIN vpin, int value) override" in PWM_DRIVER
    assert "_writeAnalogue(vpin, value ? 4095 : 0, 0, 0);" in PWM_DRIVER


def test_contract_rejects_implicit_solenoid_timing():
    assert "does not create a pulse" in DOC
    assert "external driver" in DOC
    assert "electrical safety" in DOC
