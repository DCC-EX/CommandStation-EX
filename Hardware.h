#ifndef Hardware_h
#define Hardware_h
// Virtualised hardware Interface
class Hardware {
  public:
    static void init();
    static void setPower(bool isMainTrack, bool on);
    static void setSignal(bool isMainTrack, bool high);
    static int  getCurrentMilliamps(bool isMainTrack);
    static void setBrake(bool isMainTrack, bool on);
    static void setCallback(int duration,  void (*isr)());
    static void setSingleCallback(int duration,  void (*isr)());
    static void resetSingleCallback(int duration);
};
#endif
