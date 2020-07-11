#ifndef DCCWaveform_h
#define DCCWaveform_h


const int  POWER_SAMPLE_MAX = 1000;       // XXX only until correct short detection on prog rail is implemented
const int  POWER_SAMPLE_ON_WAIT = 100;
const int  POWER_SAMPLE_OFF_WAIT = 1000;
const int  POWER_SAMPLE_OVERLOAD_WAIT = 20;


// ACK current analogRead values (vary depending on motor shield and cpu voltage)
const int   ACK_BASELINE_SAMPLES = 250 ;  // current samples before sending ACKable request
const int   ACK_TIMEOUT = 25 ;  // millis getAck is prepared to wait for a signal
const int   ACK_MIN_PULSE = 60 ;   // current above baseline which a pulse is recognised

const int   PREAMBLE_BITS_MAIN = 20;
const int   PREAMBLE_BITS_PROG = 22;



const byte   MAX_PACKET_SIZE = 12;
// NOTE: static functions are used for the overall controller, then
// one instance is created for each track.


enum class POWERMODE { OFF, ON, OVERLOAD };

const byte idlePacket[] = {0xFF, 0x00, 0xFF};
const byte resetPacket[] = {0x00, 0x00, 0x00};

class DCCWaveform {
  public:
    DCCWaveform( byte preambleBits, bool isMain);
    static void begin();
    static void loop();
    static DCCWaveform  mainTrack;
    static DCCWaveform  progTrack;

    void beginTrack();
    void setPowerMode(POWERMODE);
    POWERMODE getPowerMode();
    void checkPowerOverload();
    int  getLastCurrent();
    void schedulePacket(const byte buffer[], byte byteCount, byte repeats);
    volatile bool packetPending;
    volatile byte sentResetsSincePacket;
    void setAckBaseline(bool debug);  //prog track only
    void setAckPending(bool debug);  //prog track only
    byte getAck(bool debug);               //prog track only 0=NACK, 1=ACK 2=keep waiting
    
  private:

    static void interruptHandler();
    bool interrupt1();
    void interrupt2();
    void checkAck();
    
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

    byte pendingPacket[MAX_PACKET_SIZE];
    byte pendingLength;
    byte pendingRepeats;
    int lastCurrent;

    
    // current sampling
    POWERMODE powerMode;
    unsigned long lastSampleTaken;
    unsigned int sampleDelay;
    unsigned long power_sample_overload_wait = POWER_SAMPLE_OVERLOAD_WAIT;
    unsigned int power_good_counter = 0;

    // ACK management (Prog track only)  
    bool ackPending;    
    bool ackDetected;   
    int  ackThreshold; 
    int ackMaxCurrent;
    unsigned long ackCheckStart; // millis
    unsigned int ackCheckDuration; // millis       
    
    unsigned int ackPulseDuration;  // micros
    unsigned long ackPulseStart; // micros
           
};
#endif
