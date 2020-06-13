#include "StringFormatter.h"
#include <stdarg.h>


void StringFormatter::print( const __FlashStringHelper* input...) {
  va_list args;
  va_start(args, input);
  send(Serial,input,args);
}

void StringFormatter::send(Print & stream, const __FlashStringHelper* input...) {
  va_list args;
  va_start(args, input);
  send(stream,input,args);
}

void StringFormatter::send(Print & stream,const __FlashStringHelper* format, va_list args) {
    
  // thanks to Jan Turoň  https://arduino.stackexchange.com/questions/56517/formatting-strings-in-arduino-for-output

  char* flash=(char*)format;
  for(int i=0; ; ++i) {
    char c=pgm_read_byte_near(flash+i);
    if (c=='\0') return;
    if(c!='%') { stream.print(c); continue; }
    i++;
    c=pgm_read_byte_near(flash+i);
    switch(c) {
      case '%': stream.print('%'); break;
      case 'c': stream.print((char) va_arg(args, int)); break;
      case 's': stream.print(va_arg(args, char*)); break;
      case 'S': stream.print((const __FlashStringHelper*)va_arg(args, char*)); break;
      case 'd': stream.print(va_arg(args, int), DEC); break;
      case 'b': stream.print(va_arg(args, int), BIN); break;
      case 'o': stream.print(va_arg(args, int), OCT); break;
      case 'x': stream.print(va_arg(args, int), HEX); break;
      case 'f': stream.print(va_arg(args, double), 2); break;
    }
  }
  va_end(args);
}
