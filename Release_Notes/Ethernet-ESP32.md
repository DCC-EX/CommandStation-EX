# ESP32 Ethernet support

ESP32 builds can use a W5500 Ethernet module as their network interface. At startup, CommandStation checks for Ethernet hardware and waits up to five seconds for a cable link. If Ethernet is unavailable, has no link, or cannot obtain an IP address, it falls back to Wi-Fi.

The default SPI pin assignment is CS 5, SCK 18, MISO 16, MOSI 17, with reset on GPIO 4. These pins can be overridden with `ETHERNET_CS_PIN`, `ETHERNET_SCK_PIN`, `ETHERNET_MISO_PIN`, `ETHERNET_MOSI_PIN`, and `ETHERNET_RST_PIN`. Set `ETHERNET_RST_PIN` to `-1` if the module has no reset connection.

Ethernet uses DHCP by default. Define `IP_ADDRESS` in the build configuration to use a static IP instead. The assigned IP is shown on the display and registered with the network services, including mDNS.

The default network hostname is `DCC-EX`. Define `DCCHOSTNAME` as a build flag to override it, for example `-DDCCHOSTNAME=\"dccex-test\"`. When set, this build hostname overrides the hostname saved in preferences.

While running on Ethernet, CommandStation monitors the physical link. Link-down indications shorter than 500 ms are ignored. On confirmed link loss, CommandStation emergency-stops all locos and closes the throttle and node multicast listeners. After the link has been up for one second, it reopens those listeners so the W5500 sends fresh IGMP joins without first sending leaves on the restored link. The UDP transmit socket, mDNS, and OTA are also restarted.

To check multicast recovery, operate a point from the mimic before unplugging the server's Ethernet cable, then again after `Ethernet service recovery complete`. The server should log `Node in:` for the command (with node diagnostics enabled), and the point should move without rebooting either device. The socket recovery log confirms that the listeners opened, not that multicast packets reached the server.

For W5500 receive troubleshooting, temporarily add `-DETHERNET_W5500_RX_DIAG_PORT=2561` to the server's PlatformIO build flags (use `IP_PORT + 1` if the base port is customized). `W5500 socket` logs the hardware socket configuration each time the listener opens. `W5500 RX` reports queued hardware bytes before parsing and the UDP payload bytes returned by the parser. The hardware count includes an 8-byte UDP receive header per packet and can be zero if the packet arrives between the sample and parsing. Empty polls are not logged. Remove the flag after testing.

ESP32 Ethernet OTA is available through PlatformIO's `espota` upload protocol. Configure the upload hostname and authentication flag in your PlatformIO environment. Set `OTA_AUTH` to a private value in `config.h` and keep the upload authentication setting in sync.
