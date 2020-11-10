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

#ifndef NetworkSetup_h
#define NetworkSetup_h

#include <Ethernet.h>

#include "NetworkConfig.h"
#include "NetworkInterface.h"

class NetworkSetup
{
private:
public:
    IPAddress       dnsip;
    IPAddress       ip;
    uint8_t         mac[6] = MAC_ADDRESS;
    uint8_t         maxConnections;
    bool            connected;                  // semantics is that the server has successfullt started  or not; client connections will be started in the Transport object
    protocolType    protocol;
    uint16_t        port = LISTEN_PORT;         // Default port

    NetworkSetup();
    ~NetworkSetup();
};

#endif