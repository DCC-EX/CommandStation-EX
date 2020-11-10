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

#ifndef NetworkDiag_h
#define NetworkDiag_h

#include <Arduino.h>
#include "freeMemory.h"
#include "StringFormatter.h"

#include "TransportProcessor.h"

#define EH_DW(code) \
    do              \
    {               \
        code        \
    } while (0) //wraps in a do while(0) so that the syntax is correct.


// enable/ disbale the loglevel at runtime everything up to the compile time level will be shown the rest not !
#define EH_IFLL(LL,code) if(_nLogLevel >= LL){code} 
#define EH_IFIL(IL,code) if(_nInfoLevel >= IL){code} 

#ifndef DEBUG
#define DEBUG
#ifndef LOGLEVEL
#define LOGLEVEL 4             // compile time level by default up to error can be overridden at compiletime with a -D flag in build_flags (PIO) extra.comiler.flags on Arduino IDE
#endif
#endif

class NetworkDiag: public StringFormatter {

public:
    
    static void setDiagOut(Connection *c) {
      if ( c->client->connected() ) {
        diagSerial = c->client;
      }
    }
    
    static void resetDiagOut() {
      diagSerial = &Serial;
    }

};

#define NDIAG NetworkDiag::diag

extern int _dMem;
extern int _cMem;
extern byte _nLogLevel;         // runtime level set by <D NET #level >; by default set to 0 i.e. silent excpet for network startup at level 3 to get the init mesages shown
extern byte _nInfoLevel;        // runtime lvel of the details of the messages displayed - TBD
extern byte _dOutput;           // where the diag messages shall be send; 1 = Serial, 2 = future CLI on port 23 - TBD

#define LOGV_DEBUG 5
#define LOGV_TRACE 4
#define LOGV_ERROR 3
#define LOGV_WARN 2
#define LOGV_INFO 1
#define LOGV_SILENT 0       

#ifdef LOGLEVEL
#if LOGLEVEL == LOGV_SILENT
#define INFO(message...)
#define WARN(message...)
#define ERR(message...)
#define TRC(message...)
#define DBG(message...)
#endif
#if LOGLEVEL == LOGV_INFO 
#define INFO(message...) EH_DW(EH_IFLL(LOGV_INFO, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[INF]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define WARN(message...)
#define ERR(message...)
#define TRC(message...)
#define DBG(message...)
#endif
#if LOGLEVEL == LOGV_WARN
#define INFO(message...) EH_DW(EH_IFLL(LOGV_INFO, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[INF]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); )) 
#define WARN(message...) EH_DW(EH_IFLL(LOGV_ERROR, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[WRN]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define ERR(message...) 
#define TRC(message...)
#define DBG(message...)
#endif
#if LOGLEVEL == LOGV_ERROR
#define INFO(message...) EH_DW(EH_IFLL(LOGV_INFO, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[INF]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); )) 
#define WARN(message...) EH_DW(EH_IFLL(LOGV_WARN, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[WRN]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define ERR(message...)  EH_DW(EH_IFLL(LOGV_ERROR, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[ERR]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define TRC(message...)
#define DBG(message...)
#endif
#if LOGLEVEL == LOGV_TRACE
#define INFO(message...) EH_DW(EH_IFLL(LOGV_INFO, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[INF]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); )) 
#define WARN(message...) EH_DW(EH_IFLL(LOGV_WARN, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[WRN]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define ERR(message...)  EH_DW(EH_IFLL(LOGV_ERROR, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[ERR]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define TRC(message...)  EH_DW(EH_IFLL(LOGV_TRACE, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[TRC]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define DBG(message...)
#endif
#if LOGLEVEL >= LOGV_DEBUG
#define INFO(message...) EH_DW(EH_IFLL(LOGV_INFO, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[INF]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); )) 
#define WARN(message...) EH_DW(EH_IFLL(LOGV_WARN, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[WRN]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define ERR(message...)  EH_DW(EH_IFLL(LOGV_ERROR, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[ERR]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define TRC(message...)  EH_DW(EH_IFLL(LOGV_TRACE, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[TRC]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#define DBG(message...)  EH_DW(EH_IFLL(LOGV_DEBUG, _cMem = freeMemory(); _dMem = _cMem - _dMem; NDIAG(F("::[DBG]:%d:%d:%s:%d : "), _cMem, _dMem, __FILE__, __LINE__); _dMem = _cMem; NDIAG(message); NDIAG(F("\n")); ))
#endif

#endif

#endif