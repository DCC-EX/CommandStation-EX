"""Host-side regression checks for the ESP32 ADC attenuation contract."""

from pathlib import Path

ROOT = Path(__file__).parents[1]


def test_esp32_configures_attenuation_and_12_bit_raw_range():
    source = (ROOT / "DCCTimerESP.cpp").read_text(encoding="utf-8")
    assert "adc1_config_width(ADC_WIDTH_BIT_12)" in source
    assert "ADC_ATTEN_11db" in source
    assert "return 4095;" in source


def test_motor_driver_keeps_calibration_in_sense_factor():
    source = (ROOT / "MotorDriver.cpp").read_text(encoding="utf-8")
    assert "raw * senseFactorInternal / senseScale" in source
    assert "mA * senseScale / senseFactorInternal" in source
    assert "ADC_ATTEN" not in source


def test_bench_criteria_are_documented():
    document = (ROOT / "docs" / "ADC_Attenuation_Bench.md").read_text(encoding="utf-8")
    for required in ("zero current", "three measured loads", "raw code", "trip point"):
        assert required in document
