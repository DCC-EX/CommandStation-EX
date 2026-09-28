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
// Authors note, I hope to make this name rather than number based in the future by passing the enum to js and haveing the js code resolve the actual NVS numbers.

const char EXNetworkPreferences_html[]=R"???(
Host Name  <nvsinput nvs=32005 length=100 />)???"

#ifdef ARDUINO_ARCH_ESP32
R"???(
<p/> WiFi Connection details<p/>

Station Mode: Connected via a router<p/>
SSID      <nvsinput nvs=32001 length=100 /><br>
Password  <nvsinput nvs=32002 length=100 />
<p/>
If Station Mode SSID is empty, or fails to connect, the device will revert to Access Point (AP) mode
<p  />
AP SSID      <nvsinput nvs=32003 length=100 /><br>
AP Password  <nvsinput nvs=32004 length=100 /><br>
AP Channel   <nvsinput nvs=32007 min="0" max="13" /><br>
AP Hidden    <nvsinput nvs=32008 min="0" max="1" /> <br>
<p/>
If AP SSID and Password are not given DCC-EX will create a default Access Point based on the device's mac address. This will be shown on the OLED screen. If however an AP password is given, it will not appear.<br>
<p/>
WARNING: If you set the AP password and forget it, you will have to use the USB serial console to reset it.
)???"
#endif
;
