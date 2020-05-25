#include "StringParser.h"

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
