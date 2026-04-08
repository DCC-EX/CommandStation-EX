#ifdef ARDUINO_ARCH_ESP32
#include "WifiPreferences.h"
#include <Preferences.h>
Preferences preferences;

void WifiPreferences::load() {
  preferences.begin("DCCEX-WIFI", true); // read only
  preferences.getString("ssid", ssid, sizeof(ssid));
  preferences.getString("password", password, sizeof(password));
  channel = preferences.getUChar("channel", 0);
  forceAP = preferences.getBool("forceAP", false);
  preferences.end();
}
void WifiPreferences::save(const char *_ssid, const char *_password,  byte _channel, bool _forceAP) {
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.putString("ssid", _ssid);
  preferences.putString("password", _password);
  preferences.putUChar("channel", _channel);
  preferences.putBool("forceAP", _forceAP);
  preferences.end();
  load(); // reload to update static variables
}
void WifiPreferences::clear() {
  preferences.begin("DCCEX-WIFI", false); // read/write
  preferences.clear();
  preferences.end();
  load(); // reload to update static variables
}
const char *WifiPreferences::getSSID() {
  return ssid;
}
const char *WifiPreferences::getPassword() {
  return password;
}
byte WifiPreferences::getChannel() {
  return channel;
}
bool WifiPreferences::getForceAP() {
  return forceAP    ;
}
char WifiPreferences::ssid[32] ="";   
char WifiPreferences::password[32] ="";
byte WifiPreferences::channel = 0;
bool WifiPreferences::forceAP = false;
#endif //ARDUINO_ARCH_ESP32
