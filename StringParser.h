#ifndef StringParser_h
#define StringParser_h
#include <Arduino.h>
class StringParser
{
  public:
    static void loop(Stream & stream, void (*callback)(Stream & stream, const char * data) );
    static int parse(const char * com, int result[], byte maxResults);
    static void print( const __FlashStringHelper* input...);
    static void send(Stream & serial, const __FlashStringHelper* input...);
 private:
    static void send(Stream & serial, const __FlashStringHelper* input,va_list args);

    static byte bufferLength;
    static bool inCommandPayload;
    static const byte MAX_BUFFER=100;
    static char buffer[MAX_BUFFER];
};
#endif
