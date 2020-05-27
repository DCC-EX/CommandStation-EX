#ifndef DIAG_h
#define DIAG_h

#include "StringParser.h"
#ifndef DIAG_ENABLED
 #define DIAG_ENABLED true
#endif
#define DIAG if (DIAG_ENABLED) StringParser::print
#endif
