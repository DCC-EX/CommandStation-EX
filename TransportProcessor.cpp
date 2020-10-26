/*
 *  © 2020, Gregor Baues. All rights reserved.
 *  
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "DIAG.h"

#include "NetworkInterface.h"
#include "HttpRequest.h"
#include "TransportProcessor.h"

#ifdef DCCEX_ENABLED

#include "DCCEXParser.h"
#include "MemStream.h"

DCCEXParser ethParser;

#endif

HttpRequest httpReq;
uint16_t _rseq[MAX_SOCK_NUM] = {0};
uint16_t _sseq[MAX_SOCK_NUM] = {0};

char protocolName[4][11] = {"JMRI", "WITHROTTLE", "HTTP", "UNKNOWN"}; // change for Progmem
bool diagNetwork = false;
uint8_t diagNetworkClient = 0;

#ifdef DCCEX_ENABLED
/**
 * @brief Sending a reply by using the StringFormatter (this will result in every byte send individually which may/will create an important Network overhead).
 * Here we hook back into the DCC code for actually processing the command using a DCCParser. Alternatively we could use MemeStream in order to build the entiere reply
 * before ending it.
 * 
 * @param stream    Actually the Client to whom to send the reply. As Clients implement Print this is working
 * @param t         TransportProcessor used for accessing the buffers to be send
 * @param blocking  if set to true will instruct the DCC code to not use the async callback functions
 */

void sendToDCC(Connection *c, TransportProcessor* t, bool blocking)
{
    static MemStream *streamer = new MemStream((byte *)command, MAX_ETH_BUFFER, MAX_ETH_BUFFER, true);

    DIAG(F("DCC parsing:            [%e]\n"), command);
    // as we use buffer for recv and send we have to reset the write position
    streamer->setBufferContentPosition(0, 0);

    ethParser.parse(streamer, (byte *)t->command, true); // set to true to that the execution in DCC is sync

    if (streamer->available() == 0)
    {
        DIAG(F("No response\n"));
    }
    else
    {
        command[streamer->available()] = '\0'; // mark end of buffer, so it can be used as a string later
        DIAG(F("Response: %s\n"), t->command);
        if (c->client->connected())
        {
            c->client->write((byte *)t->command, streamer->available());
        }
    }
}
#else
/**
 * @brief Sending a reply without going through the StringFormatter. Sends the repy in one go
 * 
 * @param client  Client who send the command to which the reply shall be send
 * @param command Command initaliy recieved to be echoed back 
 */
void sendReply(Connection *c, TransportProcessor *t)
{
    byte reply[MAX_ETH_BUFFER];
    byte *response;
    char *number;
    char *command = t->command;
    char seqNumber[6];
    int i = 0;

    memset(reply, 0, MAX_ETH_BUFFER); // reset reply

    // This expects messages to be send with a trailing sequence number <R 1 1 1:0>
    // as of my stress test program to verify the arrival of messages

    number = strrchr(command, ':'); // replace the int after the last ':' if number != 0
    if (number != 0)
    {
        while (&command[i] != number)
        { // copy command into the reply upto the last ':'
            reply[i] = command[i];
            i++;
        }
        strcat((char *)reply, ":");
        itoa(_sseq[c->id], seqNumber, 10);
        strcat((char *)reply, seqNumber);
        strcat((char *)reply, ">");
        response = reply;
    } else {
        response = (byte *)command;
    }

    DIAG(F("Response:               [%e]"), (char *)response);
    if (c->client->connected())
    {
        c->client->write(response, strlen((char *)response));
        _sseq[c->id]++;
        DIAG(F(" send\n"));
    }
};
#endif

/**
 * @brief creates a HttpRequest object for the user callback. Some conditions apply esp reagrding the length of the items in the Request
 * can be found in @file HttpRequest.h 
 *  
 * @param client Client object from whom we receievd the data
 * @param c id of the Client object
 */
void httpProcessor(Connection *c, TransportProcessor *t)
{

    if (httpReq.callback == 0)
        return; // no callback i.e. nothing to do
    /**
     * @todo look for jmri formatted uris and execute those if there is no callback. If no command found ignore and 
     * ev. send a 401 error back
     */
    uint8_t i, l = 0;
    ParsedRequest preq;
    l = strlen((char *)t->buffer);
    for (i = 0; i < l; i++)
    {
        httpReq.parseRequest((char)t->buffer[i]);
    }
    if (httpReq.endOfRequest())
    {
        preq = httpReq.getParsedRequest();
        httpReq.callback(&preq, c->client);
        httpReq.resetRequest();
    } // else do nothing and continue with the next packet
}

/**
 * @brief Set the App Protocol. The detection id done upon the very first message recieved. The client will then be bound to that protocol. Its very brittle 
 * as e.g. The N message as first message for WiThrottle is not a requirement by the protocol; If any client talking Withrottle doesn't implement this the detection 
 * will default to JMRI. For HTTP we base this only on a subset of th HTTP verbs which can be used.
 * 
 * @param a First character of the recieved buffer upon first connection
 * @param b Second character of the recieved buffer upon first connection
 * @return appProtocol 
 */
appProtocol setAppProtocol(char a, char b, Connection *c)
{
    appProtocol p;
    switch (a)
    {
    case 'G': // GET
    case 'C': // CONNECT
    case 'O': // OPTIONS
    case 'T': // TRACE
    {
        p = HTTP;
        break;
    }
    case 'D': // DELETE or D plux hex value
    {
        if (b == 'E')
        {
            p = HTTP;
        }
        else
        {
            p = WITHROTTLE;
        }
        break;
    }
    case 'P':
    {
        if (b == 'T' || b == 'R')
        {
            p = WITHROTTLE;
        }
        else
        {
            p = HTTP; // PUT / PATCH / POST
        }
        break;
    }
    case 'H':
    {
        if (b == 'U')
        {
            p = WITHROTTLE;
        }
        else
        {
            p = HTTP; // HEAD
        }
        break;
    }
    case 'M':
    case '*':
    case 'R':
    case 'Q': // That doesn't make sense as it's the Q or close on app level
    case 'N':
    {
        p = WITHROTTLE;
        break;
    }
    case '<':
    {
        p = DCCEX;
        break;
    }
    case '#':
    {
        p = DCCEX;
        DIAG(F("\nDiagnostics routed to network client\n"));
        StringFormatter::setDiagOut(c);
        diagNetwork = true;
        diagNetworkClient = c->id;
        break;
    }
    default:
    {
        // here we don't know
        p = UNKNOWN_PROTOCOL;
        break;
    }
    }
    DIAG(F("\nClient speaks:          [%s]\n"), protocolName[p]);
    return p;
}

/**
 * @brief Parses the buffer to extract commands to be executed
 * 
 */
void processStream(Connection *c, TransportProcessor *t)
{
    uint8_t i, j, k, l = 0;
    uint8_t *_buffer = t->buffer;

    DIAG(F("\nBuffer:                 [%e]\n"), _buffer);
    memset(t->command, 0, MAX_JMRI_CMD); // clear out the command

    // copy overflow into the command
    if ((i = strlen(c->overflow)) != 0)
    {
        // DIAG(F("\nCopy overflow to command: %e"), c->overflow);
        strncpy(t->command, c->overflow, i);
        k = i;
    }
    // reset the overflow
    memset(c->overflow, 0, MAX_OVERFLOW);

    // check if there is again an overflow and copy if needed
    if ((i = strlen((char *)_buffer)) == MAX_ETH_BUFFER - 1)
    {
        // DIAG(F("\nPossible overflow situation detected: %d "), i);
        j = i;
        while (_buffer[i] != c->delimiter)
        {
            i--;
        }
        i++; // start of the buffer to copy
        l = i;
        k = j - i; // length to copy

        for (j = 0; j < k; j++, i++)
        {
            c->overflow[j] = _buffer[i];
            // DIAG(F("\n%d %d %d %c"),k,j,i, buffer[i]); // c->overflow[j]);
        }
        _buffer[l] = '\0'; // terminate buffer just after the last '>'
        // DIAG(F("\nNew buffer: [%s] New overflow: [%s]\n"), (char*) buffer, c->overflow );
    }
    // breakup the buffer using its changed length
    i = 0;
    k = strlen(t->command); // current length of the command buffer telling us where to start copy in
    l = strlen((char *)_buffer);
    // DIAG(F("\nCommand buffer: [%s]:[%d:%d:%d]\n"), command, i, l, k );
    while (i < l)
    {
        // DIAG(F("\nl: %d k: %d , i: %d"), l, k, i);
        t->command[k] = _buffer[i];
        if (_buffer[i] == c->delimiter)
        { // closing bracket need to fix if there is none before an opening bracket ?

            t->command[k + 1] = '\0';

            DIAG(F("Command:                [%d:%e]\n"), _rseq[c->id], t->command);
#ifdef DCCEX_ENABLED
            sendToDCC(c, command, true);
#else
            sendReply(c, t);
#endif
            _rseq[c->id]++;
            j = 0;
            k = 0;
        }
        else
        {
            k++;
        }
        i++;
    }
}

void echoProcessor(Connection *c, TransportProcessor *t)
{
    byte reply[MAX_ETH_BUFFER];

    memset(reply, 0, MAX_ETH_BUFFER);
    sprintf((char *)reply, "ERROR: malformed content in [%s]", t->buffer);
    if (c->client->connected())
    {
        c->client->write(reply, strlen((char *)reply));
        _sseq[c->id]++;
    }
}
void jmriProcessor(Connection *c, TransportProcessor *t)
{
    DIAG(F("Processing JMRI ... \n"));
    processStream(c, t);
}
void withrottleProcessor(Connection *c, TransportProcessor *t)
{
    processStream(c, t);
}

/**
 * @brief Reads what is available on the incomming TCP stream and hands it over to the protocol handler.
 * 
 * @param c    Pointer to the connection struct contining relevant information handling the data from that connection
 */

void TransportProcessor::readStream(Connection *c)
{

    // read bytes from a client
    int count = c->client->read(buffer, MAX_ETH_BUFFER - 1); // count is the amount of data ready for reading, -1 if there is no data, 0 is the connection has been closed
    buffer[count] = 0;

    // figure out which protocol

    if (!c->isProtocolDefined)
    {
        c->p = setAppProtocol(buffer[0], buffer[1], c);
        c->isProtocolDefined = true;
        switch (c->p)
        {
        case N_DIAG:
        case DCCEX:
        {
            c->delimiter = '>';
            c->appProtocolHandler = (appProtocolCallback)jmriProcessor;
            break;
        }
        case WITHROTTLE:
        {
            c->delimiter = '\n';
            c->appProtocolHandler = (appProtocolCallback)withrottleProcessor;
            break;
        }
        case HTTP:
        {
            c->appProtocolHandler = (appProtocolCallback)httpProcessor;
            httpReq.callback = nwi->getHttpCallback();
            break;
        }
        case UNKNOWN_PROTOCOL:
        {
            DIAG(F("Requests will not be handeled and packet echoed back\n"));
            c->appProtocolHandler = (appProtocolCallback)echoProcessor;
            break;
        }
        }
    }

    IPAddress remote = c->client->remoteIP();
    buffer[count] = '\0'; // terminate the string properly
    DIAG(F("\nReceived packet of size:[%d] from [%d.%d.%d.%d]\n"), count, remote[0], remote[1], remote[2], remote[3]);
    DIAG(F("Client #:               [%d]\n"), c->id);
    DIAG(F("Packet:                 [%e]\n"), buffer);

    // chop the buffer into CS / WiThrottle commands || assemble command across buffer read boundaries
    c->appProtocolHandler(c, this);
}

/**
 * @brief Sending a reply by using the StringFormatter (this will result in every byte send individually which may/will create an important Network overhead).
 * Here we hook back into the DCC code for actually processing the command using a DCCParser. Alternatively we could use MemeStream in order to build the entiere reply
 * before ending it (cf. Scratch pad below)
 * 
 * @param stream    Actually the Client to whom to send the reply. As Clients implement Print this is working
 * @param command   The reply to be send ( echo as in sendReply() )
 * @param blocking  if set to true will instruct the DCC code to not use the async callback functions
 */
void parse(Print *stream, byte *command, bool blocking)
{
    DIAG(F("DCC parsing:            [%e]\n"), command);
    // echo back (as mock parser )
    StringFormatter::send(stream, F("reply to: %s"), command);
}