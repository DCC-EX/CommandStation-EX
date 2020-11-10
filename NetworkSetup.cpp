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

#include <Arduino.h>
#include "ArduinoUniqueID.h"
#include "NetworkSetup.h"
#include "NetworkDiag.h"

bool NetworkSetup::deviceIdSet = false;
bool NetworkSetup::macAddressSet = false;
char NetworkSetup::_deviceId[MAXDEVICEID] = {0};
uint8_t NetworkSetup::mac[6] = MAC_ADDRESS;       // default MacAddress
uint8_t NetworkSetup::apWifiMacAddress[6] = MAC_ADDRESS;       // default MacAddress
uint8_t NetworkSetup::stWifiMacAddress[6] = MAC_ADDRESS;       // default MacAddress

static void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
        buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
    }
    buffer[len * 2] = '\0';
}

void NetworkSetup::setDeviceId()
{
    array_to_string(UniqueID, UniqueIDsize, _deviceId);
    DBG(F("Unique device ID: %s\n"), _deviceId);
    deviceIdSet = true;
}

void NetworkSetup::printMacAddress(uint8_t a[]) {
    INFO(F("MAC Address: [%x:%x:%x:%x:%x:%x]"),a[0],a[1],a[2],a[3],a[4],a[5]);
}

/**
 * @brief generates Mac Addresses for Ethernet and WiFi
 * 
 */
void NetworkSetup::genMacAddress() {

    if (!deviceIdSet) NetworkSetup::setDeviceId();
    if (!macAddressSet) {
        for (byte i = 0; i < 6; i++)
        {
            mac[i] = UniqueID[i];
        }; 
    }
    macAddressSet = true;
}

NetworkSetup::NetworkSetup() {}
NetworkSetup::~NetworkSetup() {}

    
    // WiFi.apMacAddress(apWifiMacAddress);
    // WiFi.macAddress(stWifiMacAddress);