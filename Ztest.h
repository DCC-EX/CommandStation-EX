#ifndef ztest_h
#define ztest_h
#include "RingStream.h"

class Ztest {
  public:
    static void parse(const FSH * command, const FSH * expect, bool (*test)() );
    
    private:
     static RingStream * ring;
};

#endif
