/*
 *  © 2021 Neil McKechnie
 *  © 2020-2022 Chris Harlow
 *  © 2022-2023 Colin Murdoch
 *  © 2023 Harald Barth
 *  All rights reserved.
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
#ifndef EXRAIL2_H
#define EXRAIL2_H
#include "FSH.h"
#include "IODevice.h"
#include "Turnouts.h"
#include "Turntables.h"
   
// The following are the operation codes (or instructions) for a kind of virtual machine.
// Each instruction is normally 3 bytes long with an operation code followed by a parameter.
// In cases where more than one parameter is required, the first parameter is followed by one  
// or more OPCODE_PAD instructions with the subsequent parameters. This wastes a byte but makes 
// searching easier as a parameter can never be confused with an opcode. 
// 
enum OPCODE : byte {OPCODE_THROW,OPCODE_CLOSE,
             OPCODE_FWD,OPCODE_REV,OPCODE_SPEED,OPCODE_INVERT_DIRECTION,
             OPCODE_RESERVE,OPCODE_FREE,
             OPCODE_AT,OPCODE_AFTER,
             OPCODE_AFTEROVERLOAD,OPCODE_AUTOSTART,
             OPCODE_ATGTE,OPCODE_ATLT,
             OPCODE_ATTIMEOUT1,OPCODE_ATTIMEOUT2,
             OPCODE_LATCH,OPCODE_UNLATCH,OPCODE_SET,OPCODE_RESET,
             OPCODE_ENDIF,OPCODE_ELSE,
             OPCODE_DELAY,OPCODE_DELAYMINS,OPCODE_DELAYMS,OPCODE_RANDWAIT,
             OPCODE_FON,OPCODE_FOFF,OPCODE_XFON,OPCODE_XFOFF,
             OPCODE_RED,OPCODE_GREEN,OPCODE_AMBER,OPCODE_DRIVE,
             OPCODE_SERVO,OPCODE_SIGNAL,OPCODE_TURNOUT,OPCODE_WAITFOR,
             OPCODE_PAD,OPCODE_FOLLOW,OPCODE_CALL,OPCODE_RETURN,
#ifndef DISABLE_PROG
             OPCODE_JOIN,OPCODE_UNJOIN,OPCODE_READ_LOCO1,OPCODE_READ_LOCO2,
#endif
             OPCODE_POM,
             OPCODE_START,OPCODE_SETLOCO,OPCODE_SENDLOCO,OPCODE_FORGET,
             OPCODE_PAUSE, OPCODE_RESUME,OPCODE_POWEROFF,OPCODE_POWERON,
             OPCODE_ONCLOSE, OPCODE_ONTHROW, OPCODE_SERVOTURNOUT, OPCODE_PINTURNOUT,
             OPCODE_PRINT,OPCODE_DCCACTIVATE,OPCODE_ASPECT,
             OPCODE_ONACTIVATE,OPCODE_ONDEACTIVATE,
             OPCODE_ROSTER,OPCODE_KILLALL,
             OPCODE_ROUTE,OPCODE_AUTOMATION,OPCODE_SEQUENCE,
             OPCODE_ENDTASK,OPCODE_ENDEXRAIL,
             OPCODE_SET_TRACK,OPCODE_SET_POWER,
             OPCODE_ONRED,OPCODE_ONAMBER,OPCODE_ONGREEN,
             OPCODE_ONCHANGE,
             OPCODE_ONCLOCKTIME,
             OPCODE_ONTIME,
             OPCODE_TTADDPOSITION,OPCODE_DCCTURNTABLE,OPCODE_EXTTTURNTABLE,
             OPCODE_ONROTATE,OPCODE_ROTATE,OPCODE_WAITFORTT,
             OPCODE_LCC,OPCODE_LCCX,OPCODE_ONLCC,
             OPCODE_ONOVERLOAD,
             OPCODE_ROUTE_ACTIVE,OPCODE_ROUTE_INACTIVE,OPCODE_ROUTE_HIDDEN,
             OPCODE_ROUTE_DISABLED,
             OPCODE_STASH,OPCODE_CLEAR_STASH,OPCODE_CLEAR_ALL_STASH,OPCODE_PICKUP_STASH,

             // OPcodes below this point are skip-nesting IF operations
             // placed here so that they may be skipped as a group
             // see skipIfBlock()
            IF_TYPE_OPCODES, // do not move this... 
             OPCODE_IFRED,OPCODE_IFAMBER,OPCODE_IFGREEN,
             OPCODE_IFGTE,OPCODE_IFLT,
             OPCODE_IFTIMEOUT,
             OPCODE_IF,OPCODE_IFNOT,
             OPCODE_IFRANDOM,OPCODE_IFRESERVE,
             OPCODE_IFCLOSED,OPCODE_IFTHROWN,
             OPCODE_IFRE,
             OPCODE_IFLOCO,
             OPCODE_IFTTPOSITION
             };

// Ensure thrunge_lcd is put last as there may be more than one display, 
// sequentially numbered from thrunge_lcd.
enum thrunger: byte {
  thrunge_print, thrunge_broadcast, thrunge_withrottle,
  thrunge_serial,thrunge_parse,
  thrunge_serial1, thrunge_serial2, thrunge_serial3,
  thrunge_serial4, thrunge_serial5, thrunge_serial6,
  thrunge_lcn, 
  thrunge_lcd,  // Must be last!!
  };

  // Flag bits for compile time features.
  static const byte FEATURE_SIGNAL= 0x80;
  static const byte FEATURE_LCC   = 0x40;
  static const byte FEATURE_ROSTER= 0x20;
  static const byte FEATURE_ROUTESTATE= 0x10;
  static const byte FEATURE_STASH = 0x08;
  
 
  // Flag bits for status of hardware and TPL
  static const byte SECTION_FLAG = 0x80;
  static const byte LATCH_FLAG   = 0x40;
  static const byte TASK_FLAG    = 0x20;
  static const byte SPARE_FLAG   = 0x10;
  static const byte SIGNAL_MASK  = 0x0C;
  static const byte SIGNAL_RED   = 0x08;
  static const byte SIGNAL_AMBER = 0x0C;
  static const byte SIGNAL_GREEN = 0x04;

  static const byte  MAX_STACK_DEPTH=4;
 
   static const short MAX_FLAGS=256;
  #define FLAGOVERFLOW(x) x>=MAX_FLAGS

class LookList {
  public: 
    LookList(int16_t size);
    void chain(LookList* chainTo);
    void add(int16_t lookup, int16_t result);
    int16_t find(int16_t value); // finds result value
    int16_t findPosition(int16_t value); // finds index 
    int16_t size();
    void stream(Print * _stream); 
    void handleEvent(const FSH* reason,int16_t id);

  private:
     int16_t m_size;
     int16_t m_loaded;
     int16_t * m_lookupArray;
     int16_t * m_resultArray;
     LookList* m_chain;     
};

 class RMFT2 {
   public:
    static void begin();
    static void loop();
    RMFT2(int progCounter);
    RMFT2(int route, uint16_t cab);
    ~RMFT2();
    static void readLocoCallback(int16_t cv);
    static void createNewTask(int route, uint16_t cab);
    static void turnoutEvent(int16_t id, bool closed);  
    static void activateEvent(int16_t addr, bool active);
    static void changeEvent(int16_t id, bool change);
    static void clockEvent(int16_t clocktime, bool change);
    static void rotateEvent(int16_t id, bool change);
    static void powerEvent(int16_t track, bool overload);
    static bool signalAspectEvent(int16_t address, byte aspect );    
    static const int16_t SERVO_SIGNAL_FLAG=0x4000;
    static const int16_t ACTIVE_HIGH_SIGNAL_FLAG=0x2000;
    static const int16_t DCC_SIGNAL_FLAG=0x1000;
    static const int16_t DCCX_SIGNAL_FLAG=0x3000;
    static const int16_t SIGNAL_ID_MASK=0x0FFF;
 // Throttle Info Access functions built by exrail macros 
  static const byte rosterNameCount;
  static const int16_t HIGHFLASH routeIdList[];
  static const int16_t HIGHFLASH automationIdList[];
  static const int16_t HIGHFLASH rosterIdList[];
  static const FSH *  getRouteDescription(int16_t id);
  static char   getRouteType(int16_t id);
  static const FSH *  getTurnoutDescription(int16_t id);
  static const FSH *  getRosterName(int16_t id);
  static const FSH *  getRosterFunctions(int16_t id);
  static const FSH *  getTurntableDescription(int16_t id);
  static const FSH *  getTurntablePositionDescription(int16_t turntableId, uint8_t positionId);
  static void startNonRecursiveTask(const FSH* reason, int16_t id,int pc);

private: 
    static void ComandFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[]);
    static bool parseSlash(Print * stream, byte & paramCount, int16_t p[]) ;
    static void streamFlags(Print* stream);
    static bool setFlag(VPIN id,byte onMask, byte OffMask=0);
    static bool getFlag(VPIN id,byte mask); 
    static int16_t progtrackLocoId;
    static void doSignal(int16_t id,char rag); 
    static bool isSignal(int16_t id,char rag); 
    static int16_t getSignalSlot(int16_t id);
    static void setTurnoutHiddenState(Turnout * t);
    #ifndef IO_NO_HAL
    static void setTurntableHiddenState(Turntable * tto);
    #endif
    static LookList* LookListLoader(OPCODE op1,
                      OPCODE op2=OPCODE_ENDEXRAIL,OPCODE op3=OPCODE_ENDEXRAIL);
    static uint16_t getOperand(int progCounter,byte n);
    static RMFT2 * loopTask;
    static RMFT2 * pausingTask;
    void delayMe(long millisecs);
    void driveLoco(byte speedo);
    bool readSensor(uint16_t sensorId);
    bool skipIfBlock();
    bool readLoco();
    void loop2();
    void kill(const FSH * reason=NULL,int operand=0);          
    void printMessage(uint16_t id);  // Built by RMFTMacros.h
    void printMessage2(const FSH * msg);
    void thrungeString(uint32_t strfar, thrunger mode, byte id=0);
    uint16_t getOperand(byte n); 
    
   static bool diag;
   static const  HIGHFLASH3  byte RouteCode[];
   static const  HIGHFLASH  int16_t SignalDefinitions[];
   static byte flags[MAX_FLAGS];
   static Print * LCCSerial;
   static LookList * routeLookup;
   static LookList * onThrowLookup;
   static LookList * onCloseLookup;
   static LookList * onActivateLookup;
   static LookList * onDeactivateLookup;
   static LookList * onRedLookup;
   static LookList * onAmberLookup;
   static LookList * onGreenLookup;
   static LookList * onChangeLookup;
   static LookList * onClockLookup;
#ifndef IO_NO_HAL
   static LookList * onRotateLookup;
#endif
   static LookList * onOverloadLookup;
   
   static const int countLCCLookup;
   static int onLCCLookup[];
   static const byte compileFeatures;
   static void manageRouteState(uint16_t id, byte state);
   static void manageRouteCaption(uint16_t id, const FSH* caption);
   static byte * routeStateArray;
   static const FSH ** routeCaptionArray;
   static int16_t * stashArray;
   static int16_t maxStashId;
    
  // Local variables - exist for each instance/task 
    RMFT2 *next;   // loop chain 
    int progCounter;    // Byte offset of next route opcode in ROUTES table
    unsigned long delayStart; // Used by opcodes that must be recalled before completing
    unsigned long  delayTime;
    union {
      unsigned long waitAfter; // Used by OPCODE_AFTER
      unsigned long timeoutStart; // Used by OPCODE_ATTIMEOUT
    };
    bool timeoutFlag;
    byte  taskId;
    
    uint16_t loco;
    bool forward;
    bool invert;
    byte speedo;
    int onEventStartPosition;
    byte stackDepth;
    int callStack[MAX_STACK_DEPTH];
};

#define GET_OPCODE GETHIGHFLASH(RMFT2::RouteCode,progCounter)
#define SKIPOP progCounter+=3

// IO_I2CDFPlayer commands and values
enum  : uint8_t{
    DF_PLAY          = 0x0F,
    DF_VOL           = 0x06,
    DF_FOLDER        = 0x2B, // Not a DFPlayer command, used to set folder nr where audio file is
    DF_REPEATPLAY    = 0x08,
    DF_STOPPLAY      = 0x16,
    DF_EQ            = 0x07, // Set equaliser, require parameter NORMAL, POP, ROCK, JAZZ, CLASSIC or BASS
    DF_RESET         = 0x0C,
    DF_DACON         = 0x1A,
    DF_SETAM         = 0x2A, // Set audio mixer 1 or 2 for this DFPLayer   
    DF_NORMAL        = 0x00, // Equalizer parameters
    DF_POP           = 0x01,
    DF_ROCK          = 0x02,
    DF_JAZZ          = 0x03,
    DF_CLASSIC       = 0x04,
    DF_BASS          = 0x05,
  };

#endif
