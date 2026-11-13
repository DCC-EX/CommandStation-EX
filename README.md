# What is DCC-EX?
DCC-EX is a team of dedicated enthusiasts producing open source DCC & DC solutions for you to run your complete model railroad layout. Our easy to use, do-it-yourself, and free open source products run on off-the-shelf Arduino technology and are supported by numerous third party hardware and apps like JMRI, Engine Driver, wiThrottle, Rocrail and more. 

Currently, our products include the following:

* [EX-CommandStation](https://github.com/DCC-EX/CommandStation-EX/releases)
* [EX-WebThrottle](https://github.com/DCC-EX/exWebThrottle)
* [EX-Installer](https://github.com/DCC-EX/EX-Installer)
* [EX-MotoShield8874](https://dcc-ex.com/reference/hardware/motorboards/ex-motor-shield-8874.html#gsc.tab=0)
* [EX-DCCInspector](https://github.com/DCC-EX/DCCInspector-EX)
* [EX-Toolbox](https://github.com/DCC-EX/EX-Toolbox)
* [EX-Turntable](https://github.com/DCC-EX/EX-Turntable)
* [EX-IOExpander](https://github.com/DCC-EX/EX-IOExpander)
* [EX-FastClock](https://github.com/DCC-EX/EX-FastClock)
* [DCCEXProtocol](https://github.com/DCC-EX/DCCEXProtocol)

Details of these projects can be found on [our web site](https://dcc-ex.com/).

# What’s in this Repository?

This repository, CommandStation-EX, contains a complete DCC-EX *EX-CommmandStation* sketch designed for compiling and uploading into an Arduino Uno, Mega, or Nano.

To utilize this sketch, you can use the following: 

1. (recommended for all levels of user) our [automated installer](https://github.com/DCC-EX/EX-Installer)
2. (intermediate) download the latest version from the [releases page](https://github.com/DCC-EX/CommandStation-EX/releases)
3. (advanced) use git clone on this repository 

Refer to [our web site](https://https://dcc-ex.com/ex-commandstation/get-started/index.html#/) for the hardware required for this project.

**We seriously recommend using the EX-Installer**, however if you choose not to use the installer... 

* Open the file ``CommandStation-EX.ino`` in the Arduino IDE or Visual Studio Code (VSC). Please do not rename the folder containing the sketch code, nor add any files in that folder. The Arduino IDE relies on the structure and name of the folder to properly display and compile the code. 
* Rename or copy ``config.example.h`` to ``config.h``. 
* You must edit ``config.h`` according to the help texts in ``config.h``.

## ESP32 network OTA updates

The ESP32 network OTA service listens on UDP port 3232 over the selected WiFi or
Ethernet interface. When `OTA_AUTH` is configured, authentication uses the
ESP32 Arduino Core 3.3.1+ PBKDF2-HMAC-SHA256 protocol (10,000 iterations).
Use a current `espota.py` uploader and supply the plaintext `OTA_AUTH` password
with `-a`; do not use its `--md5-target` password-hash option. The firmware image
checksum still uses MD5, independently of authentication.

The device connects back to the uploader's TCP listening port, so that port must
be reachable through the host firewall. Uploading over USB may be necessary once
to replace firmware with the older authentication reply-port bug. Older
MD5-only uploaders cannot authenticate with the updated service.

# More information
You can learn more at the [DCC-EX website](https://dcc-ex.com/)
