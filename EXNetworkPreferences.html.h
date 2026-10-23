/*
 *  © 2026 Chris Harlow
 *
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */


// This file referes to NVS values for network preferences
// using the enumeration values from EXNetworkPreferences.h

const char EXNetworkPreferences_html[]=R"???(
Host Name  <nvsinput nvs=hostName length=100 />)???"

#ifdef ARDUINO_ARCH_ESP32
R"???(
<p/> WiFi Connection details<p/>

Station Mode: Connected via a router<p/>
SSID      <nvsinput nvs=ssidSTA length=100 /><br>
Password  <nvsinput nvs=passwordSTA length=100 />
<p/>
If Station Mode SSID is empty, or fails to connect, the device will revert to Access Point (AP) mode
<p  />
AP SSID      <nvsinput nvs=ssidAP length=100 /><br>
AP Password  <nvsinput nvs=passwordAP length=100 /><br>
AP Channel   <nvsinput nvs=channelAP min="0" max="13" /><br>
AP Hidden    <nvsinput nvs=hiddenAP min="0" max="1" /> <br>
<p/>
If AP SSID and Password are not given DCC-EX will create a default Access Point based on the device's mac address. This will be shown on the OLED screen. If however an AP password is given, it will not appear.<br>
<p/>
WARNING: If you set the AP password and forget it, you will have to use the USB serial console to reset it.
)???"
#endif
;
