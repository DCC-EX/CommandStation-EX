/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of CommandStation-EX
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
#ifndef RMFT2_H
#define RMFT2_H
#include "FSH.h"
#include "IODevice.h"
   
// The following are the operation codes (or instructions) for a kind of virtual machine.
// Each instruction is normally 2 bytes long with an operation code followed by a parameter.
// In cases where more than one parameter is required, the first parameter is followed by one  
// or more OPCODE_PAD instructions with the subsequent parameters. This wastes a byte but makes 
// searching easier as a parameter can never be confused with an opcode. 
// 
enum OPCODE : byte {OPCODE_THROW,OPCODE_CLOSE,
             OPCODE_FWD,OPCODE_REV,OPCODE_SPEED,OPCODE_INVERT_DIRECTION,
             OPCODE_RESERVE,OPCODE_FREE,
             OPCODE_AT,OPCODE_AFTER,
             OPCODE_LATCH,OPCODE_UNLATCH,OPCODE_SET,OPCODE_RESET,
             OPCODE_IF,OPCODE_IFNOT,OPCODE_ENDIF,OPCODE_IFRANDOM,OPCODE_IFRESERVE,
             OPCODE_DELAY,OPCODE_DELAYMINS,OPCODE_RANDWAIT,
             OPCODE_FON,OPCODE_FOFF,
             OPCODE_RED,OPCODE_GREEN,OPCODE_AMBER,
             OPCODE_SERVO,OPCODE_SIGNAL,OPCODE_TURNOUT,
             OPCODE_PAD,OPCODE_FOLLOW,OPCODE_CALL,OPCODE_RETURN,
             OPCODE_JOIN,OPCODE_UNJOIN,OPCODE_READ_LOCO1,OPCODE_READ_LOCO2,OPCODE_POM,
             OPCODE_START,OPCODE_SETLOCO,
             OPCODE_PAUSE, OPCODE_RESUME,
             OPCODE_ONCLOSE, OPCODE_ONTHROW, OPCODE_SERVOTURNOUT, OPCODE_PINTURNOUT,  
             OPCODE_ROUTE,OPCODE_AUTOMATION,OPCODE_SEQUENCE,OPCODE_ENDTASK,OPCODE_ENDEXRAIL
             };


 
  // Flag bits for status of hardware and TPL
  static const byte SECTION_FLAG = 0x01;
  static const byte LATCH_FLAG = 0x02;

  static const byte  MAX_STACK_DEPTH=4;
 
   static const short MAX_FLAGS=256;
  #define FLAGOVERFLOW(x) x>=MAX_FLAGS

 class RMFT2 {
   public:
    static void begin();
    static void loop();
    RMFT2(int progCounter);
    RMFT2(int route, uint16_t cab);
    ~RMFT2();
    static void readLocoCallback(int cv);
    static void emitWithrottleRouteList(Print* stream); 
    static void createNewTask(int route, uint16_t cab);
    static void turnoutEvent(VPIN id, bool thrown);  
private: 
    static void ComandFilter(Print * stream, byte & opcode, byte & paramCount, int p[]);
    static bool parseSlash(Print * stream, byte & paramCount, int p[]) ;
    static void streamFlags(Print* stream);
    static void setFlag(VPIN id,byte onMask, byte OffMask=0);
    static byte getFlag(VPIN id,byte mask);   
    static int locateRouteStart(int16_t _route);
    static int progtrackLocoId;
    static void doSignal(VPIN id,bool red, bool amber, bool green); 

    static RMFT2 * loopTask;
    static RMFT2 * pausingTask;
    void delayMe(long millisecs);
    void driveLoco(byte speedo);
    bool readSensor(int16_t sensorId);
    bool skipIfBlock();
    bool readLoco();
    void showManual();
    void showProg(bool progOn);
    bool doManual();
    void loop2();
    void kill(const FSH * reason=NULL,int operand=0);          
    
   static bool diag;
   static const  FLASH  byte RouteCode[];
   static const  FLASH char RouteDescription[];
   static byte flags[MAX_FLAGS];
 
 // Local variables - exist for each instance/task 
    RMFT2 *next;   // loop chain 
    int progCounter;    // Byte offset of next route opcode in ROUTES table
    unsigned long delayStart; // Used by opcodes that must be recalled before completing
    unsigned long waitAfter; // Used by OPCODE_AFTER
    unsigned long  delayTime;
    int loco;
    bool forward;
    bool invert;
    int speedo;
    byte stackDepth;
    int callStack[MAX_STACK_DEPTH];
};
#endif
