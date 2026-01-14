/*
 * © 2025, Nicola Malavasi. All rights reserved.
 * © 2023, Neil McKechnie. All rights reserved.
 * * This file is part of DCC-EX API
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This file implements the I2C transport layer for the DFPlayer driver, 
 * using the SC16IS750/752 I2C-to-UART bridge.
 * * KEY FEATURES:
 * 1. Register Management: Directly handles SC16IS75x internal registers (THR, RHR, LCR, etc.) 
 * to configure UART parameters over the I2C bus.
 * 2. Dual-Channel Support: Uses the _UART_CH_BITS to switch between Channel A and Channel B 
 * on dual-UART bridge chips (like the SC16IS752).
 * 3. Auto-Baud Detection: Automatically probes the bridge crystal frequency 
 * (1.8MHz or 14.7MHz) to lock the output at 9600 baud without manual configuration.
 * 4. Transparent Communication: Wraps standard DFPlayer serial packets into 
 * I2C messages for seamless integration with the base class logic.
 * 5. I2C Bus Diagnostics: Monitors device presence and reports status (OK/FAILED) 
 * via the DIAG console, including the detected crystal frequency.
 */

/*
 * © 2025, Nicola Malavasi. All rights reserved.
 * © 2023, Neil McKechnie. All rights reserved.
 * * This file is part of DCC-EX API
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */


 /*
 * © 2025, Nicola Malavasi. All rights reserved.
 * © 2023, Neil McKechnie. All rights reserved.
 * * This file is part of DCC-EX API
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */


#ifndef IO_DFPlayerI2C_h
#define IO_DFPlayerI2C_h

#include "IO_DFPlayerBase.h"

#define REG_THR    0x00 << 3
#define REG_RHR    0x00 << 3
#define REG_FCR    0x02 << 3
#define REG_LCR    0x03 << 3
#define REG_DLL    0x00 << 3
#define REG_DLH    0x01 << 3
#define REG_RXLVL  0x09 << 3

class DFPlayerI2C : public DFPlayerBase {
private:
    uint8_t _UART_CH_BITS;
    unsigned long _xtal_freq;
    bool _xtal_detected = false;

    void writeRaw(uint8_t r, uint8_t v) {
        uint8_t reg = (uint8_t)(r | _UART_CH_BITS);
        uint8_t data[] = {reg, v}; 
        I2CManager.write(_I2CAddress, data, 2);
    }

    void setBaud(unsigned long hz) {
        uint16_t div = (uint16_t)(hz / (9600 * 16));
        writeRaw(REG_LCR, 0x83);
        writeRaw(REG_DLL, (uint8_t)(div & 0xFF));
        writeRaw(REG_DLH, (uint8_t)(div >> 8));
        writeRaw(REG_LCR, 0x03);
        writeRaw(REG_FCR, 0x07);
    }

public:
    DFPlayerI2C(VPIN v, I2CAddress a, uint8_t ch) : DFPlayerBase(v) {
        _I2CAddress = a;
        _UART_CH_BITS = (ch << 1);
        _xtal_freq = 1843200;
        addDevice(this);
    }

    void _begin() override {
        I2CManager.begin();
        if (!I2CManager.exists(_I2CAddress)) {
            _deviceState = DEVSTATE_FAILED;
            DIAG(F("DFPlayer I2C Error: Bridge NOT found at %s"), _I2CAddress.toString());
            return;
        }
        setBaud(_xtal_freq);
        DFPlayerBase::_begin();
    }

    void detectXtal() override {
        if (_deviceState == DEVSTATE_FAILED) return;
        unsigned long test_xtals[] = {1843200, 14745600};
        uint8_t lvl_reg = (uint8_t)(REG_RXLVL | _UART_CH_BITS);
        uint8_t rhr_reg = (uint8_t)(REG_RHR | _UART_CH_BITS);

        for (int i=0; i<2; i++) {
            setBaud(test_xtals[i]);
            delay(50); 
            uint8_t avail=0, dummy;
            I2CManager.read(_I2CAddress, &avail, 1, &lvl_reg, 1);
            while(avail > 0) {
                for(int j=0; j<avail; j++) I2CManager.read(_I2CAddress, &dummy, 1, &rhr_reg, 1);
                I2CManager.read(_I2CAddress, &avail, 1, &lvl_reg, 1);
            }
            uint8_t q[] = {0x7E, 0xFF, 0x06, 0x42, 0x00, 0x00, 0x00, 0xFE, 0xB9, 0xEF};
            transmitCommandBuffer(q, 10);
            unsigned long start = millis();
            while (millis() - start < 300) {
                I2CManager.read(_I2CAddress, &avail, 1, &lvl_reg, 1);
                if (avail >= 4) {
                    uint8_t resp[4];
                    for(int k=0; k<4; k++) I2CManager.read(_I2CAddress, &resp[k], 1, &rhr_reg, 1);
                    if (resp[0] == 0x7E && resp[3] == 0x42) {
                        _xtal_freq = test_xtals[i];
                        _xtal_detected = true;
                        _display();
                        return;
                    }
                }
                delay(5);
            }
        }
        _display();
    }

    void _display() override {
        DIAG(F("DFPlayer I2C (%s) Ch %c: Xtal %S MHz %S VPIN %d %S"),
             _I2CAddress.toString(), (_UART_CH_BITS == 0) ? 'A' : 'B',
             (_xtal_freq > 2000000) ? F("14.7") : F("1.8"),
             (_xtal_detected) ? F("(AUTO)") : F("(TIMEOUT/MANUAL)"),
             _firstVpin, (_deviceState == DEVSTATE_FAILED) ? F("OFFLINE"):F(""));
    }

    void transmitCommandBuffer(const uint8_t b[], size_t s) override {
        if (_deviceState == DEVSTATE_FAILED) return;
        uint8_t pkt[s+1]; 
        pkt[0] = (uint8_t)(REG_THR | _UART_CH_BITS);
        for(size_t i=0; i<s; i++) pkt[i+1] = b[i];
        I2CManager.write(_I2CAddress, pkt, s + 1);
    }

    bool processIncoming() override {
        if (_deviceState == DEVSTATE_FAILED) return false;
        uint8_t lvl_reg = (uint8_t)(REG_RXLVL | _UART_CH_BITS);
        uint8_t avail = 0;
        if (I2CManager.read(_I2CAddress, &avail, 1, &lvl_reg, 1) != I2C_STATUS_OK) {
            _deviceState = DEVSTATE_FAILED;
            return false;
        }
        if (avail > 0) {
            uint8_t rhr_reg = (uint8_t)(REG_RHR | _UART_CH_BITS);
            for (uint8_t i=0; i<avail; i++) {
                uint8_t b;
                if (I2CManager.read(_I2CAddress, &b, 1, &rhr_reg, 1) == I2C_STATUS_OK) processIncomingByte(b);
            }
            return true;
        }
        return false;
    }
};

#endif