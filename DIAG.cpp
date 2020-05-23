#include <arduino.h>
#include <stdarg.h>

void Serialprint(const __FlashStringHelper* input...) {
  // thanks to Jan Turoň  https://arduino.stackexchange.com/questions/56517/formatting-strings-in-arduino-for-output

  va_list args;
  va_start(args, input);
  char* flash=(char*)input;
  for(int i=0; ; ++i) {
    char c=pgm_read_byte_near(flash+i);
    if (c=='\0') return;
    if(c!='%') { Serial.print(c); continue; }
    i++;
    c=pgm_read_byte_near(flash+i);
    switch(c) {
      case '%': Serial.print('%'); break;
      case 's': Serial.print(va_arg(args, char*)); break;
      case 'd': Serial.print(va_arg(args, int), DEC); break;
      case 'b': Serial.print(va_arg(args, int), BIN); break;
      case 'o': Serial.print(va_arg(args, int), OCT); break;
      case 'x': Serial.print(va_arg(args, int), HEX); break;
      case 'f': Serial.print(va_arg(args, double), 2); break;
    }
  }
  va_end(args);
}
