#include <DIO2.h>
// This hardware configuration would normally be setup in a .h file elsewhere
const byte MAIN_POWER_PIN = 3;
const byte MAIN_SIGNAL_PIN = 12;
const byte MAIN_SENSE_PIN = A0;

const byte PROG_POWER_PIN = 11;
const byte PROG_SIGNAL_PIN = 13;
const byte PROG_SENSE_PIN = A1;

const int  POWER_SAMPLE_MAX = 300;
const int  POWER_SAMPLE_ON_WAIT = 100;
const int  POWER_SAMPLE_OFF_WAIT = 1000;
const int  POWER_SAMPLE_OVERLOAD_WAIT = 4000;


// ACK current analogRead values (vary depending on motor shield and cpu voltage)
const int   ACK_TIMEOUT = 25 ;  // millis getAck is prepared to wait for a signal
const int   ACK_MAX_NOT_PULSE = 10 ;  // current below which this is NOT a pulse any more
const int   ACK_MIN_PULSE = 50 ;   // current above which a pulse is recognised

const int   PREAMBLE_BITS_MAIN = 20;
const int   PREAMBLE_BITS_PROG = 22;

// Railcom settings
const bool RAILCOM_CUTOUT = true;
const byte RAILCOM_PREAMBLES_BEFORE_CUTOUT = 1; // how far into the preamble do we cutout
const byte RAILCOM_PREAMBLES_SKIPPED_IN_CUTOUT = 5;
const byte RAILCOM_BRAKE_PIN = 9;



const byte   MAX_PACKET_SIZE = 12;
// NOTE: static functions are used for the overall controller, then
// one instance is created for each track.


enum class POWERMODE { OFF, ON, OVERLOAD };
const byte idleMessage[] = {0xFF, 0x00};
const byte resetMessage[] = {0x00, 0x00};
const byte idlePacket[] = {0xFF, 0x00, 0xFF};
const byte resetPacket[] = {0x00, 0x00, 0x00};

class DCCWaveform {
  public:
    DCCWaveform(byte powerPinNo, byte directionPinNo, byte sensePinNo, byte preambleBits, bool isMain);
    static void begin();
    static void loop();
    static DCCWaveform  mainTrack;
    static DCCWaveform  progTrack;

    void beginTrack();
    void setPowerMode(POWERMODE);
    POWERMODE getPowerMode();
    void checkPowerOverload();
    void schedulePacket(const byte buffer[], byte byteCount, byte repeats);
    volatile bool packetPending;
    bool  startAckProcess();
    bool  getAck();


  private:

    static void interruptHandler();
    bool interrupt1();
    void interrupt2();

    bool isMainTrack;

    // Transmission controller
    byte transmitPacket[MAX_PACKET_SIZE];  // packet being transmitted
    byte transmitLength;
    byte transmitRepeats;      // remaining repeats of transmission
    byte remainingPreambles;
    byte requiredPreambles;
    bool currentBit;           // bit to be transmitted
    byte bits_sent;           // 0-8 (yes 9 bits) sent for current byte
    byte bytes_sent;          // number of bytes sent from transmitPacket
    byte state;               // wave generator state machine
    void checkRailcom();

    byte pendingPacket[MAX_PACKET_SIZE];
    byte pendingLength;
    byte pendingRepeats;

    // Hardware fast pins
    GPIO_pin_t directionPin;
    GPIO_pin_t powerPin;
    GPIO_pin_t railcomBrakePin;

    // current sampling
    POWERMODE powerMode;
    byte sensePin;
    unsigned long nextSampleDue;


};
