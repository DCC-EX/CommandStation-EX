/*
 *  © 2021, Neil McKechnie. All rights reserved.
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

/* 
 * Manager for I2C communications.  For portability, it allows use 
 * of the Wire class, but also has a native implementation for AVR
 * which supports non-blocking queued I/O requests.
 * 
 * Helps to avoid calling Wire.begin() multiple times (which is not)
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
 * 
 */

/* 
 * Future enhancement possibility:
 *
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
 *  Changes required:  Increase the size of the I2CAddress field in the IODevice class from uint8_t to uint16_t.
 *  The most significant byte would contain a '1' bit flag, the multiplexer number (0-7) and bus number (0-7).  Then, when performing
 *  an I2C operation, the I2CManager would check this byte and, if zero, do what it currently does.  If the byte is non-zero, then
 *  that means the device is connected via a multiplexer so the I2C transaction should be preceded by a select command issued to the
 *  relevant multiplexer.
 * 
 *  Non-interrupting I2C:
 * 
 *  I2C may be operated without interrupts (undefine I2C_USE_INTERRUPTS).  Instead, the I2C state
 *  machine handler, currently invoked from the interrupt service routine, is invoked from the loop() function.
 *  The speed at which I2C operations can be performed then becomes highly dependent on the frequency that 
 *  the loop() function is called, and may be adequate under some circumstances.  
 *  The advantage of NOT using interrupts is that the impact of I2C upon the DCC waveform (when accurate timing mode isn't in use)
 *  becomes almost zero.
 * 
 */

// Uncomment following line to enable Wire library instead of native I2C drivers
//#define I2C_USE_WIRE

// Uncomment following line to disable the use of interrupts by the native I2C drivers.
//#define I2C_NO_INTERRUPTS

// Default to use interrupts within the native I2C drivers.
#ifndef I2C_NO_INTERRUPTS
#define I2C_USE_INTERRUPTS
#endif

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
};

typedef enum : uint8_t
{
  OPERATION_READ = 1,
  OPERATION_REQUEST = 2,
  OPERATION_SEND = 3,
  OPERATION_SEND_P = 4,
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

  void setReadParams(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen);
  void setRequestParams(uint8_t i2cAddress, uint8_t *readBuffer, uint8_t readLen, const uint8_t *writeBuffer, uint8_t writeLen);
  void setWriteParams(uint8_t i2cAddress, const uint8_t *writeBuffer, uint8_t writeLen);

  uint8_t writeLen;
  uint8_t readLen;
  uint8_t operation;
  uint8_t i2cAddress;
  uint8_t *readBuffer;
  const uint8_t *writeBuffer;
#if !defined(I2C_USE_WIRE)
  I2CRB *nextRequest;
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
  // Check if specified I2C address is responding.
  uint8_t checkAddress(uint8_t address);
  inline bool exists(uint8_t address) {
    return checkAddress(address)==I2C_STATUS_OK;
  }
  // Write a complete transmission to I2C from an array in RAM
  uint8_t write(uint8_t address, const uint8_t buffer[], uint8_t size);
  uint8_t write(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb);
  // Write a complete transmission to I2C from an array in Flash
  uint8_t write_P(uint8_t address, const uint8_t buffer[], uint8_t size);
  uint8_t write_P(uint8_t address, const uint8_t buffer[], uint8_t size, I2CRB *rb);
  // Write a transmission to I2C from a list of bytes.
  uint8_t write(uint8_t address, uint8_t nBytes, ...);
  // Write a command from an array in RAM and read response
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
    const uint8_t writeBuffer[]=NULL, uint8_t writeSize=0);
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
    const uint8_t writeBuffer[], uint8_t writeSize, I2CRB *rb);
  // Write a command from an arbitrary list of bytes and read response
  uint8_t read(uint8_t address, uint8_t readBuffer[], uint8_t readSize, 
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
  uint32_t _clockSpeed = 400000L;  // 400kHz max on Arduino.

  // Finish off request block by waiting for completion and posting status.
  uint8_t finishRB(I2CRB *rb, uint8_t status);

  void _initialise();
  void _setClock(unsigned long);

#if !defined(I2C_USE_WIRE)
    // I2CRB structs are queued on the following two links.
    // If there are no requests, both are NULL.
    // If there is only one request, then queueHead and queueTail both point to it.
    // Otherwise, queueHead is the pointer to the first request in the queue and
    // queueTail is the pointer to the last request in the queue.
    // Within the queue, each request's nextRequest field points to the 
    // next request, or NULL.
    // Mark volatile as they are updated by IRC and read/written elsewhere.
    static I2CRB * volatile queueHead;
    static I2CRB * volatile queueTail;
    static volatile uint8_t state;

    static I2CRB * volatile currentRequest;
    static volatile uint8_t txCount;
    static volatile uint8_t rxCount;
    static volatile uint8_t bytesToSend;
    static volatile uint8_t bytesToReceive;
    static volatile uint8_t operation;
    static volatile unsigned long startTime;

    static unsigned long timeout; // Transaction timeout in microseconds.  0=disabled.
    
    void startTransaction();
    
    // Low-level hardware manipulation functions.
    static void I2C_init();
    static void I2C_setClock(unsigned long i2cClockSpeed);
    static void I2C_handleInterrupt();
    static void I2C_sendStart();
    static void I2C_sendStop();
    static void I2C_close();
    
  public:
    // setTimeout sets the timout value for I2C transactions.
    // TODO: Get I2C timeout working before uncommenting the code below.
    void setTimeout(unsigned long value) { (void)value; /* timeout = value; */ };

    // handleInterrupt needs to be public to be called from the ISR function!
    static void handleInterrupt();
#endif


};

extern I2CManagerClass I2CManager;

#endif
