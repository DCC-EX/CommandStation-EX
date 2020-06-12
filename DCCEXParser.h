#ifndef DCCEXParser_h
#define DCCEXParser_h
#include <Arduino.h>
struct DCCEXParser
{
   DCCEXParser();
   void loop(Stream & pstream);
   void flush();
   private:
  
    static const int MAX_PARAMS=10;  // longest command sent in
    static const int MAX_BUFFER=50;  // longest command sent in
     byte  bufferLength=0;
     bool  inCommandPayload=false;
     char  buffer[MAX_BUFFER+2]; 
    int splitValues( int result[MAX_PARAMS]);
    void parse(Print & stream, const char * command);
     
     bool parseT(Print & stream, int params, int p[]);
     bool parseZ(Print & stream, int params, int p[]);
     bool parseS(Print & stream,  int params, int p[]);

    
    static bool stashBusy;
    static Print & stashStream;
    static int stashP[MAX_PARAMS];
    static bool stashCallback(Print & stream, int p[MAX_PARAMS]);
    static void callback_W(int result);
    static void callback_B(int result);        
    static void callback_R(int result);
  
};

#define BOARD_NAME "not yet configured"
#endif
