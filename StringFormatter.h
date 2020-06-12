#ifndef StringFormatter_h
#define StringFormatter_h
#include <Arduino.h>
class StringFormatter
{
  public:
    static int parse(const char * com, int result[], byte maxResults);
    static void print( const __FlashStringHelper* input...);
    static void send(Print & serial, const __FlashStringHelper* input...);
 private:
    static void send(Print & serial, const __FlashStringHelper* input,va_list args);

   
};
#endif
