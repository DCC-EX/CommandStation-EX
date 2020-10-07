#ifndef CommandDistributor_h
#define CommandDistributor_h
#include "DCCEXParser.h"

typedef void (*HTTP_CALLBACK)(Print *stream, byte *cmd);

class CommandDistributor {

public :
  static void setHTTPCallback(HTTP_CALLBACK callback);
  static bool parse(byte clientId,byte* buffer, Print * streamer);


private:
   static HTTP_CALLBACK httpCallback;
   static bool isHTTP(byte * buffer);
   static DCCEXParser * parser;
};

#endif
