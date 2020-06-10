#ifndef DCCEXParser_h
#define DCCEXParser_h
struct DCCEXParser
{
   DCCEXParser(Stream & myStream);
   void loop();
   private:
  
    static const int MAX_PARAMS=10;  // longest command sent in
    static const int MAX_BUFFER=50;  // longest command sent in
        
     Stream & stream;
     byte  bufferLength=0;
     bool  inCommandPayload=false;
     char  buffer[MAX_BUFFER]; 
     void parse(const char * command);
     int splitValues( int result[MAX_PARAMS]);
   
     bool parseT(int params, int p[]);
     bool parseZ(int params, int p[]);
     bool parseS( int params, int p[]);

    
    static bool stashBusy;
    static Stream & stashStream;
    static int stashP[MAX_PARAMS];
    static bool stashCallback(Stream & stream, int p[MAX_PARAMS]);
    static void callback_W(int result);
    static void callback_B(int result);        
    static void callback_R(int result);
  
};

#define BOARD_NAME "not yet configured"
#endif
