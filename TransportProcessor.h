/*
 * © 2020 Gregor Baues. All rights reserved.
 *  
 * This is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 * 
 * See the GNU General Public License for more details <https://www.gnu.org/licenses/>
 */

#ifndef TransportProcessor_h
#define TransportProcessor_h

#include <Arduino.h>
#include <Ethernet.h>
#include <WiFiEspAT.h>

#include "Transport.h"
#include "NetworkConfig.h"
#include "NetworkInterface.h"

#ifdef DCCEX_ENABLED
#include "RingStream.h"
#endif

class TransportProcessor
{
private:
#ifdef DCCEX_ENABLED
    void sendToDCC(Connection *c, TransportProcessor* t, bool blocking);
#endif
    

public:
    UDP *udp;                                 // need to carry the single UDP server instance over to the processor for sending packest
    NetworkInterface *nwi;
    uint8_t buffer[MAX_ETH_BUFFER];
    char command[MAX_JMRI_CMD];

    void readStream(Connection *c, bool read); // process incomming packets and processes them; if read = false the buffer has already been filled 

    TransportProcessor(){};
    ~TransportProcessor(){};
};

#endif // !Transport_h