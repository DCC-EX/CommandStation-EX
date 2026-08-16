# Platform support and build matrix

This table describes the platforms that are currently configured and covered by
the repository's automated compile checks. A successful compile is build
coverage only; it is not a claim that every board variant has been tested on
hardware.

| Platform | PlatformIO environment | Board | Arduino / core constraint | CI coverage | Status |
| --- | --- | --- | --- | --- | --- |
| AVR | `mega2560` | `megaatmega2560` | Arduino AVR platform selected by PlatformIO | Yes | Supported baseline |
| ESP32 | `ESP32` | `esp32dev` | `espressif32 @ 6.7.0` (Arduino 2.0.16, ESP-IDF 4.4.7) | Yes | Supported baseline |
| ESP32-C6 | — | — | No configured environment or validated core/board combination | No | Not supported/validated |
| RP2040 | — | — | No configured environment in `master` | No | Not supported/validated |

The ESP32 pin is intentional. Issue [#411](https://github.com/DCC-EX/CommandStation-EX/issues/411)
reports source incompatibilities with Arduino-ESP32 3.0.2, so upgrading the
core must be treated as a compatibility change and verified separately. Issue
[#496](https://github.com/DCC-EX/CommandStation-EX/issues/496) requests ESP32-C6
support, but this repository does not yet have the environment and build
evidence needed to advertise it.

Pull request [#416](https://github.com/DCC-EX/CommandStation-EX/pull/416) is a
separate, open early RP2040 implementation and is not replaced or duplicated by
this matrix.
