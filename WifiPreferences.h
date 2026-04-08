#ifndef WifiPreferences_h
#define WifiPreferences_h
#include <Arduino.h>
class WifiPreferences {
public:
    static void load();
    static void save(const char *_ssid, const char *_password, byte _channel, bool _forceAP);
    static void clear();
    static const char *getSSID();
    static const char *getPassword();
    static byte getChannel();
    static bool getForceAP();
private:
    static char ssid[32];
    static char password[32];
    static byte channel;
    static bool forceAP;
};
#endif //WifiPreferences_h
