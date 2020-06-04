#ifndef DCCEXParser_h
#define DCCEXParser_h
struct DCCEXParser
{
    static void parse(Stream & stream,const char * command);
   
   private:
        
    static bool parseT(Stream & stream, int params, int p[]);
    static bool parseZ(Stream & stream, int params, int p[]);
    static bool parseS(Stream & stream, int params, int p[]);

    static const int MAX_PARAMS=10;  // longest command sent in  
};

#define BOARD_NAME "not yet configured"
#endif
