#ifdef EXTT_HOST_TEST
#include <Arduino.h>
#include <unity.h>

#define iodevice_h
#define I2CMANAGER_H
#define DIAG_h
#define TURNTABLES_H
#define CommandDistributor_h
#define DIAG(...) ((void)0)

using VPIN = uint16_t;
constexpr uint8_t I2C_STATUS_OK = 0;
constexpr uint8_t I2C_STATUS_TIMEOUT = 5;
constexpr uint8_t DEVSTATE_FAILED = 1;

struct I2CAddress {
  const char *toString() const { return "test"; }
};

class IODevice {
public:
  virtual ~IODevice() = default;
  void poll(unsigned long now) { _loop(now); }
  void write(int value, uint8_t activity) { _writeAnalogue(_firstVpin, value, activity, 0); }
  int read() { return _read(_firstVpin); }
  unsigned long nextPoll = 0;

protected:
  static bool checkNoOverlap(VPIN, int, I2CAddress) { return true; }
  static void addDevice(IODevice *) {}
  void delayUntil(unsigned long time) { nextPoll = time; }
  virtual void _begin() = 0;
  virtual void _loop(unsigned long) = 0;
  virtual int _read(VPIN) = 0;
  virtual void _writeAnalogue(VPIN, int, uint8_t, uint16_t) = 0;
  virtual void _display() = 0;
  VPIN _firstVpin = 0;
  int _nPins = 0;
  I2CAddress _I2CAddress;
  uint8_t _deviceState = 0;
};

struct FakeI2CManager {
  uint8_t status = I2C_STATUS_OK;
  uint8_t value = 0;
  void begin() {}
  bool exists(I2CAddress) { return true; }
  uint8_t read(I2CAddress, uint8_t *buffer, uint8_t) {
    buffer[0] = value;
    return status;
  }
  void write(I2CAddress, uint8_t, int, int, int) {}
  const char *getErrorMessage(uint8_t) { return "test error"; }
} I2CManager;

struct Turntable {
  static int changes;
  static bool moving;
  static uint16_t getByVpin(VPIN) { return 1; }
  static void setMoving(uint16_t, bool state) { moving = state; ++changes; }
};
int Turntable::changes = 0;
bool Turntable::moving = false;

#include "../../IO_EXTurntable.cpp"

void setUp() {
  I2CManager.status = I2C_STATUS_OK;
  I2CManager.value = 0;
  Turntable::changes = 0;
  Turntable::moving = false;
}

void tearDown() {}

void test_failed_read_keeps_moving_until_a_successful_read() {
  EXTurntable turntable(600, 1, I2CAddress());
  turntable.write(100, Turn);
  TEST_ASSERT_TRUE(Turntable::moving);
  TEST_ASSERT_EQUAL_INT(1, Turntable::changes);

  I2CManager.status = I2C_STATUS_TIMEOUT;
  turntable.poll(1000);
  TEST_ASSERT_EQUAL_INT(1, turntable.read());
  TEST_ASSERT_EQUAL_INT(1, Turntable::changes);
  TEST_ASSERT_EQUAL_UINT32(101000, turntable.nextPoll);

  I2CManager.status = I2C_STATUS_OK;
  turntable.poll(101000);
  TEST_ASSERT_FALSE(Turntable::moving);
  TEST_ASSERT_EQUAL_INT(0, turntable.read());
  TEST_ASSERT_EQUAL_INT(2, Turntable::changes);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_failed_read_keeps_moving_until_a_successful_read);
  return UNITY_END();
}
#endif
