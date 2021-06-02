#include "StringLogger.h"

// DIAG.h the #define DIAG points to here ... 
// EthernetSetup , Wifisetup, etc can register a function to be called allowing the channel
// to publish the diag info to 
// serial is default end enabled all the time

StringLogger StringLogger::singleton;          // static instantiation;

void StringLogger::addDiagWriter(DiagWriter l) {
    if ( registered == MAXWRITERS ) {
        Serial.println("Error: Max amount of writers exceeded.");
        return;
    }
    writers[registered] = l;
    registered++;
}


void StringLogger::diag(const FSH *input,...)
{ 

    char b1[128];

    va_list args;
    va_start(args, input);

    int len = 0;
    len += sprintf(&b1[len], "<* ");
    len += vsprintf_P(&b1[len], (const char *)input, args); 
    len += sprintf(&b1[len], " *>\n");

    // allways print to Serial 
    Serial.print(b1);

    // callback the other registered diag writers 
    for (size_t i = 0; i < (size_t) registered; i++)
    {
        writers[i](b1, len);   
    }

    va_end(args);

}