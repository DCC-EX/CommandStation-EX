# ESP32 Ethernet support

ESP32 builds can use a W5500 Ethernet module as their network interface. At startup, CommandStation checks for Ethernet hardware and waits up to five seconds for a cable link. If Ethernet is unavailable, has no link, or cannot obtain an IP address, it falls back to Wi-Fi.

The default SPI pin assignment is CS 5, SCK 18, MISO 16, MOSI 17, with reset on GPIO 4. These pins can be overridden with `ETHERNET_CS_PIN`, `ETHERNET_SCK_PIN`, `ETHERNET_MISO_PIN`, `ETHERNET_MOSI_PIN`, and `ETHERNET_RST_PIN`. Set `ETHERNET_RST_PIN` to `-1` if the module has no reset connection.

Ethernet uses DHCP by default. Define `IP_ADDRESS` in the build configuration to use a static IP instead. The assigned IP is shown on the display and registered with the network services, including mDNS.

The default network hostname is `DCC-EX`. Define `DCCHOSTNAME` as a build flag to override it, for example `-DDCCHOSTNAME=\"dccex-test\"`. When set, this build hostname overrides the hostname saved in preferences.

While running on Ethernet, CommandStation monitors the physical link. Link-down indications shorter than 500 ms are ignored; after a sustained outage, the link must remain up for one second before the Ethernet TCP/UDP, multicast, mDNS, and OTA services are restarted.

ESP32 Ethernet OTA is available through the `ESP32-OTA` and `ESP32-NODE-OTA` PlatformIO environments. The default upload hostname is `dcc-ex`. OTA authentication is configured with `OTA_AUTH`; change the example password in `config.h` to a private value and keep the PlatformIO upload authentication setting in sync.
