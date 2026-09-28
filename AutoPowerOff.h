#ifndef AutoPowerOff_h
#define AutoPowerOff_h
#include <stdint.h>
class AutoPowerOff {
  public:
    typedef void (*PowerOffCallback)();
    AutoPowerOff(uint32_t timeout, PowerOffCallback callback)
      : timeoutMs(timeout), callback(callback), lastActivity(0), armed(false) {}
    void begin(uint32_t now) { lastActivity=now; armed=timeoutMs != 0; }
    void activity(uint32_t now) { lastActivity=now; armed=timeoutMs != 0; }
    bool loop(uint32_t now) {
      if (!armed || (uint32_t)(now-lastActivity) < timeoutMs) return false;
      armed=false; if (callback) callback(); return true;
    }
  private:
    uint32_t timeoutMs, lastActivity;
    PowerOffCallback callback;
    bool armed;
};
#endif
