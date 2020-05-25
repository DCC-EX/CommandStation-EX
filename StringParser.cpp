#include "StringParser.h"
#include <stdarg.h>

byte  StringParser::bufferLength=0;
bool  StringParser::inCommandPayload=false;
char  StringParser::buffer[MAX_BUFFER];

void StringParser::loop(Stream & stream, void (*callback)(Stream & stream, const char * data) ) {
  while(stream.available()) {
    if (bufferLength==MAX_BUFFER) {
       bufferLength=0;
      inCommandPayload=false;
    }
    char ch = stream.read();
    if (ch == '<') {
      inCommandPayload = true;
      bufferLength=0;
      buffer[0]='\0';
    } 
    else if (ch == '>') {
      buffer[bufferLength]='\0';
      (*callback)(Serial, buffer);
      inCommandPayload = false;
    } else if(inCommandPayload) {
      buffer[bufferLength++]= ch;
    }
  }
  }
int StringParser::parse(const char * com, int result[], byte maxResults) {
  byte state=1;
  byte parameterCount=0;
  int runningValue=0;
  const char * remainingCmd=com;  // skips the opcode
  bool signNegative=false;
  
  // clear all parameters in case not enough found
  for (int i=0;i<maxResults;i++) result[i]=0;
  
  while(parameterCount<maxResults) {
      char hot=*remainingCmd;
      
       switch (state) {
    
            case 1: // skipping spaces before a param
               if (hot==' ') break;
               if (hot == '\0' || hot=='>') return parameterCount;
               state=2;
               continue;
               
            case 2: // checking sign
               signNegative=false;
               runningValue=0;
               state=3; 
               if (hot!='-') continue; 
               signNegative=true;
               break; 
            case 3: // building a parameter   
               if (hot>='0' && hot<='9') {
                   runningValue=10*runningValue+(hot-'0');
                   break;
               }
               result[parameterCount] = runningValue * (signNegative ?-1:1);
               parameterCount++;
               state=1; 
               continue;
         }   
         remainingCmd++;
      }
      return parameterCount;
}



void StringParser::print( const __FlashStringHelper* input...) {
  va_list args;
  va_start(args, input);
  send(Serial,input,args);
}

void StringParser::send(Stream & stream, const __FlashStringHelper* input...) {
  va_list args;
  va_start(args, input);
  send(stream,input,args);
}

void StringParser::send(Stream & stream,const __FlashStringHelper* format, va_list args) {
    

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
      case 's': stream.print(va_arg(args, char*)); break;
      case 'd': stream.print(va_arg(args, int), DEC); break;
      case 'b': stream.print(va_arg(args, int), BIN); break;
      case 'o': stream.print(va_arg(args, int), OCT); break;
      case 'x': stream.print(va_arg(args, int), HEX); break;
      case 'f': stream.print(va_arg(args, double), 2); break;
    }
  }
  va_end(args);
}
