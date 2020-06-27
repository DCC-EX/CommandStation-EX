#ifndef WiThrottle_h
#define WiTHrottle_h


struct MYLOCO {
    char throttle;
    int cab;
};

class WiThrottle {
  public:  
    static void loop();
    void parse(Print & stream, char * cmd);
    static WiThrottle* getThrottle(Print & stream, int wifiClient); 

  private: 
    WiThrottle(Print & stream, int wifiClientId);
    ~WiThrottle();
   
      static const int MAX_MY_LOCO=10;
      static const int HEARTBEAT_TIMEOUT=10;
      static WiThrottle* firstThrottle;
      static int getInt(char * cmd);
      static int getLocoId(char * cmd);
      
      WiThrottle* nextThrottle;
      int clientid;
       
      MYLOCO myLocos[MAX_MY_LOCO];   
      bool heartBeatEnable;
      unsigned long heartBeat;
      
     void multithrottle(Print & stream, char* cmd);
     void locoAction(Print & stream, char* aval, char throttleChar, int cab);
     void accessory(Print & stream, char* cmd);
     void checkHeartbeat();  
};
#endif
