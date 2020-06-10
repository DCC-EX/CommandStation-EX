#ifndef DIAG_h
#define DIAG_h

#include "StringFormatter.h"
#ifndef DIAG_ENABLED
 #define DIAG_ENABLED true
#endif
#define DIAG if (DIAG_ENABLED) StringFormatter::print
#endif
