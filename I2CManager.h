/*
 *  © 2023, Neil McKechnie. All rights reserved.
 *  © 2022 Paul M Antoine
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

#ifndef I2CMANAGER_H
#define I2CMANAGER_H

#include <inttypes.h>
#include "FSH.h"
#include "defines.h"
#include "DIAG.h"

/* 
 * Manager for I2C communications.  For portability, it allows use 
 * of the Wire class, but also has a native implementation for AVR
 * which supports non-blocking queued I/O requests.
 * 
 * Helps to avoid calling Wire.begin() multiple times (which is not
 * entirely benign as it reinitialises).
 * 
 * Also helps to avoid the Wire clock from being set, by another device
 * driver, to a speed which is higher than a device supports.
 * 
 * Thirdly, it provides a convenient way to check whether there is a 
 * device on a particular I2C address.
 * 
 * Non-blocking requests are issued by creating an I2C Request Block
 * (I2CRB) which is then added to the I2C manager's queue.  The 
 * application refers to this block to check for completion of the
 * operation, and for reading completion status.
 * 
 * Examples:
 *  I2CRB rb;
 *  uint8_t status = I2CManager.write(address, buffer, sizeof(buffer), &rb);
 *  ...
 *  if (!rb.isBusy()) {
 *    status = rb.status;
 *    // Repeat write
 *    I2CManager.queueRequest(&rb);
 *    ...
 *    status = rb.wait(); // Wait for completion and read status
 *  }
 *  ...
 *  I2CRB rb2;
 *  outbuffer[0] = 12;  // Register number in I2C device to be read
 *  rb2.setRequestParams(address, inBuffer, 1, outBuffer, 1);
 *  status = I2CManager.queueRequest(&rb2);
 *  if (status == I2C_STATUS_OK) { 
 *    status = rb2.wait();
 *    if (status == I2C_STATUS_OK) {
 *      registerValue = inBuffer[0];
 *    }
 *  }
 *  ...
 *  
 * Synchronous (blocking) calls are also possible, e.g. 
 *  status = I2CManager.write(address, buffer, sizeof(buffer));
 * 
 * When using non-blocking requests, neither the I2CRB nor the input or output
 * buffers should be modified until the I2CRB is complete (not busy).
 * 
 * Timeout monitoring is possible, but requires that the following call is made
 * reasonably frequently in the program's loop() function:
 *  I2CManager.loop();
 * So that the application doesn't need to do this explicitly, this call is performed
 * from the I2CRB::isBusy() or I2CRB::wait() functions.
 * 
 */

/* 
 *  I2C Multiplexer (e.g. TCA9547, TCA9548)
 * 
 *  A multiplexer offers a way of extending the address range of I2C devices.  For example, GPIO extenders use address range 0x20-0x27
 *  to are limited to 8 on a bus.  By adding a multiplexer, the limit becomes 8 for each of the multiplexer's 8 sub-buses, i.e. 64.
 *  And a single I2C bus can have up to 8 multiplexers, giving up to 64 sub-buses and, in theory, up to 512 I/O extenders; that's 
 *  as many as 8192 input/output pins!
 *  Secondly, the capacitance of the bus is an electrical limiting factor of the length of the bus, speed and number of devices.
 *  The multiplexer isolates each sub-bus from the others, and so reduces the capacitance of the bus.  For example, with one 
 *  multiplexer and 64 GPIO extenders, only 9 devices are connected to the bus at any time (multiplexer plus 8 extenders). 
 *  Thirdly, the multiplexer offers the ability to use mixed-speed devices more effectively, by allowing high-speed devices to be
 *  put on a different bus to low-speed devices, enabling the software to switch the I2C speed on-the-fly between I2C transactions.
 * 
 * 
 *  Non-interrupting I2C:
 * 
 *  Non-blocking I2C may be operated without interrupts (undefine I2C_USE_INTERRUPTS).  Instead, the I2C state
 *  machine handler, currently invoked from the interrupt service routine, is invoked from the loop() function.
 *  The speed at which I2C operations can be performed then becomes highly dependent on the frequency that 
 *  the loop() function is called, and may be adequate under some circumstances.  
 *  The advantage of NOT using interrupts is that the impact of I2C upon the DCC waveform (when accurate timing mode isn't in use)
 *  becomes almost zero.
 * 
 */

// Maximum number of retries on an I2C operation.
// A value of zero will disable retries.
// Maximum value is 254 (unsigned byte counter)
// Note that timeout failures are not retried, but any timeout
// configured applies to each try separately.
#define MAX_I2C_RETRIES 2

// Add following line to config.h to enable Wire library instead of native I2C drivers
//#define I2C_USE_WIRE

// Add following line to config.h to disable the use of interrupts by the native I2C drivers.
//#define I2C_NO_INTERRUPTS

// Default to use interrupts within the native I2C drivers.
#ifndef I2C_NO_INTERRUPTS
#define I2C_USE_INTERRUPTS
#endif

// I2C Extended Address support I2C Multiplexers and allows various properties to be 
// associated with an I2C address such as the MUX and SubBus.  In the future, this
// may be extended to include multiple buses, and other features. 
// Uncomment to enable extended address.
//

//#define I2C_EXTENDED_ADDRESS

/////////////////////////////////////////////////////////////////////////////////////
// Extended I2C Address type to facilitate extended I2C addresses including
// I2C multiplexer support.
/////////////////////////////////////////////////////////////////////////////////////

// Currently only one bus supported, and one instance of I2CManager to handle it.
enum I2CBus : uint8_t {
    I2CBus_0 = 0,
};

// Currently I2CAddress supports one I2C bus, with up to eight
// multipexers (MUX) attached.  Each MUX can have up to eight sub-buses.
enum I2CMux : uint8_t {
  I2CMux_0 = 0,
  I2CMux_1 = 1,
  I2CMux_2 = 2,
  I2CMux_3 = 3,
  I2CMux_4 = 4,
  I2CMux_5 = 5,
  I2CMux_6 = 6,
  I2CMux_7 = 7,
  I2CMux_None = 255,   // Address doesn't need mux switching
};
  
enum I2CSubBus : uint8_t {
  SubBus_0 = 0,        // Enable individual sub-buses...
  SubBus_1 = 1,
#if !defined(I2CMUX_PCA9542)
  SubBus_2 = 2,
  SubBus_3 = 3,
#if !defined(I2CMUX_PCA9544)
  SubBus_4 = 4,
  SubBus_5 = 5,
  SubBus_6 = 6,
  SubBus_7 = 7,
#endif
#endif
  SubBus_No,           // Number of subbuses (highest + 1)
  SubBus_None = 254,   // Disable all sub-buses on selected mux
  SubBus_All = 255,    // Enable all sub-buses (not supported by some multiplexers)
};

// Type to hold I2C address
#if defined(I2C_EXTENDED_ADDRESS)

// First MUX address (they range between 0x70-0x77).
#define I2C_MUX_BASE_ADDRESS 0x70

// Currently I2C address supports one I2C bus, with up to eight
// multiplexers (MUX) attached.  Each MUX can have up to eight sub-buses.
// This structure could be extended in the future (if there is a need) 
// to support 10-bit I2C addresses, different I2C clock speed for each
// sub-bus, multiple I2C buses, and other features not yet thought of.
struct I2CAddress {
private:
  // Fields
  I2CBus _busNumber;
  I2CMux _muxNumber;
  I2CSubBus _subBus;
  uint8_t _deviceAddress;
  static char addressBuffer[];
public:
  // Constructors
  // For I2CAddress "{I2CBus_0, Mux_0, SubBus_0, 0x23}" syntax.
  I2CAddress(const I2CBus busNumber, const I2CMux muxNumber, const I2CSubBus subBus, const uint8_t deviceAddress) {
    _busNumber = busNumber;
    _muxNumber = muxNumber;
    _subBus = subBus;
    _deviceAddress = deviceAddress;
  }

  // Basic constructor
  I2CAddress() : I2CAddress(I2CMux_None, SubBus_None, 0) {}

  // For I2CAddress "{Mux_0, SubBus_0, 0x23}" syntax.
  I2CAddress(const I2CMux muxNumber, const I2CSubBus subBus, const uint8_t deviceAddress) :
    I2CAddress(I2CBus_0, muxNumber, subBus, deviceAddress) {}

  // For I2CAddress in form "{SubBus_0, 0x23}" - assume Mux0 (0x70)
  I2CAddress(I2CSubBus subBus, uint8_t deviceAddress) : 
    I2CAddress(I2CMux_0, subBus, deviceAddress) {}

  // Conversion from uint8_t to I2CAddress
  // For I2CAddress in form "0x23"
  // (device assumed to be on the main I2C bus).
  I2CAddress(const uint8_t deviceAddress) : 
    I2CAddress(I2CMux_None, SubBus_None, deviceAddress) {}
    
  // Conversion from uint8_t to I2CAddress
  // For I2CAddress in form "{I2CBus_1, 0x23}"
  // (device not connected via multiplexer).
  I2CAddress(const I2CBus bus, const uint8_t deviceAddress) : 
    I2CAddress(bus, I2CMux_None, SubBus_None, deviceAddress) {}

  // For I2CAddress in form "{I2CMux_0, SubBus_0}" (mux selector)
  I2CAddress(const I2CMux muxNumber, const I2CSubBus subBus) :
    I2CAddress(muxNumber, subBus, 0x00) {}

  // For I2CAddress in form "{i2cAddress, deviceAddress}"
  // where deviceAddress is to be on the same subbus as i2cAddress.
  I2CAddress(I2CAddress firstAddress, uint8_t newDeviceAddress) :
    I2CAddress(firstAddress._muxNumber, firstAddress._subBus, newDeviceAddress) {}

  // Conversion operator from I2CAddress to uint8_t
  // For "uint8_t address = i2cAddress;" syntax
  // (device assumed to be on the main I2C bus or on a currently selected subbus.
  operator uint8_t () const { return _deviceAddress; }

  // Conversion from I2CAddress to char* (uses static storage so only 
  // one conversion can be done at a time).  So don't call it twice in a
  // single DIAG statement for example.
  const char* toString() { 
    char *ptr = addressBuffer;
    if (_muxNumber != I2CMux_None) {
      strcpy_P(ptr, (const char*)F("{I2CMux_"));
      ptr += 8;
      *ptr++ = '0' + _muxNumber;
      strcpy_P(ptr, (const char*)F(",Subbus_"));
      ptr += 8;
      if (_subBus == SubBus_None) {
        strcpy_P(ptr, (const char*)F("None"));
        ptr += 4;
      } else if (_subBus == SubBus_All) {
        strcpy_P(ptr, (const char*)F("All"));
        ptr += 3;
      } else 
        *ptr++ = '0' + _subBus;
      *ptr++ = ',';
    }
    toHex(_deviceAddress, ptr);
    ptr += 4;
    if (_muxNumber != I2CMux_None)
      *ptr++ = '}';
    *ptr = 0; // terminate string
    return addressBuffer;
  }

  // Comparison operator
  int operator == (I2CAddress &a) const {
    if (_deviceAddress != a._deviceAddress) 
      return false; // Different device address so no match
    if (_muxNumber == I2CMux_None || a._muxNumber == I2CMux_None)
      return true;  // Same device address, one or other on main bus
    if (_subBus == SubBus_None || a._subBus == SubBus_None) 
      return true;  // Same device address, one or other on main bus
    if (_muxNumber != a._muxNumber) 
      return false; // Connected to a subbus on a different mux
    if (_subBus != a._subBus)
      return false;  // different subbus
    return true;  // Same address on same mux and same subbus
  }
  // Field accessors
  I2CMux muxNumber() { return _muxNumber; }
  I2CSubBus subBus() { return _subBus; }
  uint8_t deviceAddress() { return _deviceAddress; }

private:
  // Helper function for converting byte to four-character hex string (e.g. 0x23).
  void toHex(const uint8_t value, char *buffer);
};

#else
struct I2CAddress {
private:
  uint8_t _deviceAddress;
  static char addressBuffer[];
public:
  // Constructors
  I2CAddress(const uint8_t deviceAddress) {
    _deviceAddress = deviceAddress;
  }
  I2CAddress(I2CMux, I2CSubBus, const uint8_t deviceAddress) {
    addressWarning();
    _deviceAddress = deviceAddress;
  }
  I2CAddress(I2CSubBus, const uint8_t deviceAddress) {
    addressWarning();
    _deviceAddress = deviceAddress;
  }

  // Basic constructor
  I2CAddress() : I2CAddress(0) {}

  // Conversion operator from I2CAddress to uint8_t
  // For "uint8_t address = i2cAddress;" syntax
  operator uint8_t () const { return _deviceAddress; }

  // Conversion from I2CAddress to char* (uses static storage so only 
  // one conversion can be done at a time).  So don't call it twice in a
  // single DIAG statement for example.
  const char* toString () { 
    char *ptr = addressBuffer;
    // Just display hex value, two digits.
    toHex(_deviceAddress, ptr);
    ptr += 4;
    *ptr = 0; // terminate string
    return addressBuffer;
  }

  // Comparison operator
  int operator == (I2CAddress &a) const {
    if (_deviceAddress != a._deviceAddress) 
      return false; // Different device address so no match
    return true;  // Same address on same mux and same subbus
  }
private:
  // Helper function for converting byte to four-character hex string (e.g. 0x23).
  void toHex(const uint8_t value, char *buffer);
  void addressWarning() {
    if (!_addressWarningDone) {
      DIAG(F("WARNIING: Extended I2C address used but not supported in this configuration"));
      _addressWarningDone = true;
    }
  }    
  static bool _addressWarningDone;
};
#endif // I2C_EXTENDED_ADDRESS


// Status codes for I2CRB structures.
enum : uint8_t {
  // Codes used by Wire and by native drivers
  I2C_STATUS_OK=0,
  I2C_STATUS_TRUNCATED=1,
  I2C_STATUS_NEGATIVE_ACKNOWLEDGE=2,
  I2C_STATUS_TRANSMIT_ERROR=3,
  I2C_STATUS_TIMEOUT=5,
  // Code used by Wire only
  I2C_STATUS_OTHER_TWI_ERROR=4, // catch-all error
  // Codes used by native drivers only
  I2C_STATUS_ARBITRATION_LOST=6,
  I2C_STATUS_BUS_ERROR=7,
  I2C_STATUS_UNEXPECTED_ERROR=8,
  I2C_STATUS_PENDING=253,
};

// Status codes for the state machine (not returned to caller).
enum : uint8_t {
  I2C_STATE_ACTIVE=253,
  I2C_STATE_FREE=254,
  I2C_STATE_CLOSING=255,
  I2C_STATE_COMPLETED=252,
};

typedef enum : uint8_t
{
  OPERATION_READ = 1,
  OPERATION_REQUEST = 2,
  OPERATION_SEND = 3,
  OPERATION_SEND_P = 4,
  OPERATION_NORETRY = 0x80,  // OR with operation to suppress retries.
  OPERATION_MASK = 0x7f,  // mask for extracting the operation code
} OperationEnum;


// Default I2C frequency
#ifndef I2C_FREQ
#define I2C_FREQ    400000L
#endif

// Class defining a request context for an I2C operation.
class I2CRB {
public:
  volatile uint8_t status; // Completion status, or pending flag (updated from IRC)
  volatile uint8_t nBytes; // Number of bytes read (updated from IRC)

  inline I2CRB() { status = I2C_STATUS_OK; };
  uint8_t wait();
  bool isBusy();

  void setReadParams(I2CAddress i2cAddress, uint8_t *readBuffer, uint8_t readLen);
  void setRequestParams(I2CAddress i2cAddress, uint8_t *readBuffer, uint8_t readLen, const uint8_t *writeBuffer, uint8_t writeLen);
  void setWriteParams(I2CAddress i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen);
  void suppressRetries(bool suppress);

  uint8_t writeLen;
  uint8_t readLen;
  uint8_t operation;
  I2CAddress i2cAddress;
  uint8_t *readBuffer;
  const uint8_t *writeBuffer;
#if !defined(I2C_USE_WIRE)
  I2CRB *nextRequest;  // Used by non-blocking devices for I2CRB queue management.
#endif
};

// I2C Manager
class I2CManagerClass {
public:

  // If not already initialised, initialise I2C (wire).
  void begin(void);
  // Set clock speed to the lowest requested one.
  void setClock(uint32_t speed);
  // Force clock speed 
  void forceClock(uint32_t speed);
  // setTimeout sets the timout value for I2C transactions (milliseconds).
  void setTimeout(unsigned long);
  // Check if specified I2C address is responding.
  uint8_t checkAddress(I2CAddress address);
  inline bool exists(I2CAddress address) {
    return checkAddress(address)==I2C_STATUS_OK;
  }
  // Select/deselect Mux Sub-Bus (if using legacy addresses, just checks address)
  // E.g. muxSelectSubBus({I2CMux_0, SubBus_3});
  uint8_t muxSelectSubBus(I2CAddress address) {
    return checkAddress(address);
  }
  // Write a complete transmission to I2C from an array in RAM
  uint8_t write(I2CAddress address, const uint8_t buffer[], uint8_t size);
  uint8_t write(I2CAddress address, const uint8_t buffer[], uint8_t size, I2CRB *rb);
  // Write a complete transmission to I2C from an array in Flash
  uint8_t write_P(I2CAddress address, const uint8_t buffer[], uint8_t size);
  uint8_t write_P(I2CAddress address, const uint8_t buffer[], uint8_t size, I2CRB *rb);
  // Write a transmission to I2C from a list of bytes.
  uint8_t write(I2CAddress address, uint8_t nBytes, ...);
  // Write a command from an array in RAM and read response
  uint8_t read(I2CAddress address, uint8_t readBuffer[], uint8_t readSize, 
    const uint8_t writeBuffer[]=NULL, uint8_t writeSize=0);
  uint8_t read(I2CAddress address, uint8_t readBuffer[], uint8_t readSize, 
    const uint8_t writeBuffer[], uint8_t writeSize, I2CRB *rb);
  // Write a command from an arbitrary list of bytes and read response
  uint8_t read(I2CAddress address, uint8_t readBuffer[], uint8_t readSize, 
    uint8_t writeSize, ...);
  void queueRequest(I2CRB *req);

  // Function to abort long-running operations.
  void checkForTimeout();

  // Loop method
  void loop();

  // Expand error codes into text.  Note that they are in flash so 
  // need to be printed using FSH.
  static const FSH *getErrorMessage(uint8_t status);

private:
  bool _beginCompleted = false;
  bool _clockSpeedFixed = false;
  uint8_t retryCounter;  // Count of retries
  // Clock speed must be no higher than 400kHz on AVR. Higher is possible on 4809, SAMD
  // and STM32 but most popular I2C devices are 400kHz so in practice the higher speeds
  // will not be useful.  The speed can be overridden by I2CManager::forceClock().
  uint32_t _clockSpeed = I2C_FREQ;  
  // Default timeout 100ms on I2C request block completion.
  // A full 32-byte transmission takes about 8ms at 100kHz,
  // so this value allows lots of headroom.  
  // It can be modified by calling I2CManager.setTimeout() function.
  // When retries are enabled, the timeout applies to each
  // try, and failure from timeout does not get retried.
  // A value of 0 means disable timeout monitoring.
  uint32_t _timeout = 100000UL;
    
  // Finish off request block by waiting for completion and posting status.
  uint8_t finishRB(I2CRB *rb, uint8_t status);

  void _initialise();
  void _setClock(unsigned long);

#if defined(I2C_EXTENDED_ADDRESS)
// Count of I2C multiplexers found when initialising.  If there is only one
// MUX then the subbus does not need de-selecting after use; however, if there
// are two or more, then the subbus must be deselected to avoid multiple
// sub-bus legs on different multiplexers being accessible simultaneously.
private:
  uint8_t _muxCount = 0;
public:
  uint8_t getMuxCount() { return _muxCount; }
#endif

#if !defined(I2C_USE_WIRE)
    // I2CRB structs are queued on the following two links.
    // If there are no requests, both are NULL.
    // If there is only one request, then queueHead and queueTail both point to it.
    // Otherwise, queueHead is the pointer to the first request in the queue and
    // queueTail is the pointer to the last request in the queue.
    // Within the queue, each request's nextRequest field points to the 
    // next request, or NULL.
    // Mark volatile as they are updated by IRC and read/written elsewhere.
private:
    I2CRB * volatile queueHead = NULL;
    I2CRB * volatile queueTail = NULL;

    // State is set to I2C_STATE_FREE when the interrupt handler has finished
    // the current request and is ready to complete.
    uint8_t state = I2C_STATE_FREE;

    // CompletionStatus may be set by the interrupt handler at any time but is
    // not written to the I2CRB until the state is I2C_STATE_FREE.
    uint8_t completionStatus = I2C_STATUS_OK;
    uint8_t overallStatus = I2C_STATUS_OK;

    I2CRB * currentRequest = NULL;
    uint8_t txCount = 0;
    uint8_t rxCount = 0;
    uint8_t bytesToSend = 0;
    uint8_t bytesToReceive = 0;
    uint8_t operation = 0;
    uint32_t startTime = 0;
    uint8_t muxPhase = 0;
    uint8_t muxAddress = 0;
    uint8_t muxData[1];
    uint8_t deviceAddress;
    const uint8_t *sendBuffer;
    uint8_t *receiveBuffer;
    uint8_t transactionState = 0;
  
    volatile uint32_t pendingClockSpeed = 0;

    void startTransaction();
    
    // Low-level hardware manipulation functions.
    void I2C_init();
    void I2C_setClock(unsigned long i2cClockSpeed);
    void I2C_handleInterrupt();
    void I2C_sendStart();
    void I2C_sendStop();
    void I2C_close();
    
  public:
    // handleInterrupt needs to be public to be called from the ISR function!
    void handleInterrupt();
#endif


};

// Pointer to class instance (Note: if there is more than one bus, each will have
// its own instance of I2CManager, selected by the queueRequest function from
// the I2CBus field within the request block's I2CAddress).
extern I2CManagerClass I2CManager;


#endif
