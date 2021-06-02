#ifndef StringLogger_h
#define StringLogger_h

#include "MemStream.h"
#include "RingStream.h"
#include "StringFormatter.h"
#include <stdarg.h>

// stream for diagnostics in addition to serial you should be able to configure
// additional (besides Serial) 'outputs' to e.g. Ethernet or MQTT or WIFI etc ...
// Serial outpout is managed in StringFormatter on top for formatting the message
// which gets printed char by char

#define MAXWRITERS 10

typedef void (*DiagWriter)(const char *msg, const int length);

class StringLogger
{

private:
    // Methods
    StringLogger() = default;
    StringLogger(const StringLogger &);            // non construction-copyable
    StringLogger &operator=(const StringLogger &); // non copyable

    // Members
    static StringLogger singleton; // unique instance of the MQTTInterface object
    DiagWriter writers[MAXWRITERS];
    int registered = 0; // number of registered writers ( Serial is not counted as always used )

public:
    // Methods
    static StringLogger &get() noexcept
    { // return a reference to the unique instance
        return singleton;
    }
    void diag(const FSH *input...);
    void addDiagWriter(DiagWriter l);
    ~StringLogger() = default;

    // Members
};

#endif