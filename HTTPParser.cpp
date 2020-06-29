#include "HTTPParser.h"
#include "StringFormatter.h"

void HTTPParser::parse(Print & stream, byte * cmd) {
     (void)cmd;  // Avoid compiler warning because this example doesnt use this parameter
      
     // BEWARE   - As soon as you start responding, the cmd buffer is trashed!
     // You must get everything you need from it before using StringFormatter::send!
       
     StringFormatter::send(stream,F("HTTP/1.1 200 OK\nContent-Type: text/html\nConnnection: close\n\n"));
     StringFormatter::send(stream,F("<html><body>This is <b>not</b> a web server.<br/></body></html>"));    
}
