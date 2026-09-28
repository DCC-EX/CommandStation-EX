from pathlib import Path
import unittest


GPIO_BASE = Path(__file__).resolve().parents[1] / "IO_GPIOBase.h"


def gpio_pin_mask(width, pin):
    return (1 << pin) & ((1 << width) - 1)


class GPIOBaseMaskTest(unittest.TestCase):
    def test_production_uses_template_width(self):
        source = GPIO_BASE.read_text(encoding="utf-8")
        self.assertIn("T mask = T(1) << pin;", source)

    def test_16_pin_boundary(self):
        self.assertEqual(gpio_pin_mask(16, 15), 0x8000)

    def test_32_pin_boundary(self):
        self.assertEqual(gpio_pin_mask(32, 31), 0x80000000)

    def test_64_pin_boundary(self):
        self.assertEqual(gpio_pin_mask(64, 63), 0x8000000000000000)


if __name__ == "__main__":
    unittest.main()
