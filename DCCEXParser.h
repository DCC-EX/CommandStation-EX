#ifndef DCCEXParser_h
#define DCCEXParser_h
struct DCCEXParser
{
    static void parse(Stream & stream,const char * command);
   
   private:

    static const int MAX_PARAMS=10;  // longest command sent in
        
    static bool parseT(Stream & stream, int params, int p[]);
    static bool parseZ(Stream & stream, int params, int p[]);
    static bool parseS(Stream & stream, int params, int p[]);
    
    static int stashP[MAX_PARAMS];
    static bool stashBusy;
    static Stream & stashStream;
    static bool stashCallback(Stream & stream, int p[MAX_PARAMS]);
    static void callback_W(int result);
    static void callback_B(int result);        
    static void callback_R(int result);
  
};

#define BOARD_NAME "not yet configured"
#endif
