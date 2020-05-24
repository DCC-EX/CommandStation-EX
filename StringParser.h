#ifndef CommParser_h
#define CommParser_h

#include <Arduino.h>

const int MAX_PARAMS=8;
struct StringParser
{
    static void init();
    static void parse(const char *);

    private:
    static int parse(const char *, byte);
    static int p[MAX_PARAMS];
};

#endif
