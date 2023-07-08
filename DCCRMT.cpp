/*
 *  © 2021-2022, Harald Barth.
 *  
 *  This file is part of DCC-EX
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

#if defined(ARDUINO_ARCH_ESP32)
#include "defines.h"
#include "DIAG.h"
#include "DCCRMT.h"
#include "DCCTimer.h"
#include "DCCWaveform.h" // for MAX_PACKET_SIZE
#include "soc/gpio_sig_map.h"

// Number of bits resulting out of X bytes of DCC payload data
// Each byte has one bit extra and at the end we have one EOF marker
#define DATA_LEN(X) ((X)*9+1)

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,2,0)
#error wrong IDF version
#endif

void setDCCBit1(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_1_HALFPERIOD;
  item->level1    = 0;
  item->duration1 = DCC_1_HALFPERIOD;
}

void setDCCBit0(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_0_HALFPERIOD;
  item->level1    = 0;
  item->duration1 = DCC_0_HALFPERIOD;
}

// special long zero to trigger scope
void setDCCBit0Long(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_0_HALFPERIOD + DCC_0_HALFPERIOD/10;
  item->level1    = 0;
  item->duration1 = DCC_0_HALFPERIOD + DCC_0_HALFPERIOD/10;
}

void setEOT(rmt_item32_t* item) {
  item->val = 0;
}

// This is an array that contains the this pointers
// to all uses channel objects. This is used to determine
// which of the channels was triggering the ISR as there
// is only ONE common ISR routine for all channels.
RMTChannel *channelHandle[8] = { 0 };

void IRAM_ATTR interrupt(rmt_channel_t channel, void *t) {
  RMTChannel *tt = channelHandle[channel];
  if (tt) tt->RMTinterrupt();
  if (channel == 0)
    DCCTimer::updateMinimumFreeMemoryISR(0);
}

RMTChannel::RMTChannel(pinpair pins, bool isMain) {
  byte ch;
  byte plen;
  if (isMain) {
    ch = 0;
    plen = PREAMBLE_BITS_MAIN;
  } else {
    ch = 2;
    plen = PREAMBLE_BITS_PROG;
  }
    
  // preamble
  preambleLen = plen+2; // plen 1 bits, one 0 bit and one EOF marker
  preamble = (rmt_item32_t*)malloc(preambleLen*sizeof(rmt_item32_t));
  for (byte n=0; n<plen; n++)
    setDCCBit1(preamble + n);      // preamble bits
#ifdef SCOPE
  setDCCBit0Long(preamble + plen); // start of packet 0 bit long version
#else
  setDCCBit0(preamble + plen);     // start of packet 0 bit normal version
#endif
  setEOT(preamble + plen + 1);     // EOT marker

  // idle
  idleLen = 28;
  idle = (rmt_item32_t*)malloc(idleLen*sizeof(rmt_item32_t));
  if (isMain) {
    for (byte n=0; n<8; n++)   // 0 to 7
      setDCCBit1(idle + n);
    for (byte n=8; n<18; n++)  // 8, 9 to 16, 17
      setDCCBit0(idle + n);
    for (byte n=18; n<26; n++) // 18 to 25
      setDCCBit1(idle + n);
  } else {
    for (byte n=0; n<26; n++)  // all zero
      setDCCBit0(idle + n);
  }
  setDCCBit1(idle + 26);     // end bit
  setEOT(idle + 27);         // EOT marker

  // data: max packet size today is 5 + checksum
  maxDataLen = DATA_LEN(MAX_PACKET_SIZE+1);  // plus checksum
  data = (rmt_item32_t*)malloc(maxDataLen*sizeof(rmt_item32_t));
  
  rmt_config_t config;
  // Configure the RMT channel for TX
  bzero(&config, sizeof(rmt_config_t));
  config.rmt_mode = RMT_MODE_TX;
  config.channel = channel = (rmt_channel_t)ch;
  config.clk_div = RMT_CLOCK_DIVIDER;
  config.gpio_num = (gpio_num_t)pins.pin;
  config.mem_block_num = 2; // With longest DCC packet 11 inc checksum (future expansion)
                            // number of bits needed is 22preamble + start +
                            // 11*9 + extrazero + EOT = 124
                            // 2 mem block of 64 RMT items should be enough

  ESP_ERROR_CHECK(rmt_config(&config));
  addPin(pins.invpin, true);
  /*
  // test: config another gpio pin
  gpio_num_t gpioNum = (gpio_num_t)(pin-1);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpioNum], PIN_FUNC_GPIO);
  gpio_set_direction(gpioNum, GPIO_MODE_OUTPUT);
  gpio_matrix_out(gpioNum, RMT_SIG_OUT0_IDX, 0, 0);
  */
  
  // NOTE: ESP_INTR_FLAG_IRAM is *NOT* included in this bitmask
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, ESP_INTR_FLAG_LOWMED|ESP_INTR_FLAG_SHARED));

  // DIAG(F("Register interrupt on core %d"), xPortGetCoreID());

  ESP_ERROR_CHECK(rmt_set_tx_loop_mode(channel, true));
  channelHandle[channel] = this; // used by interrupt
  rmt_register_tx_end_callback(interrupt, 0);
  rmt_set_tx_intr_en(channel, true);

  DIAG(F("Channel %d DCC signal for %s start"), config.channel, isMain ? "MAIN" : "PROG");

  // send one bit to kickstart the signal, remaining data will come from the
  // packet queue. We intentionally do not wait for the RMT TX complete here.
  //rmt_write_items(channel, preamble, preambleLen, false);
  RMTprefill();
  dataReady = false;
}

void RMTChannel::RMTprefill() {
  rmt_fill_tx_items(channel, preamble, preambleLen, 0);
  rmt_fill_tx_items(channel, idle, idleLen, preambleLen-1);
}

const byte transmitMask[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

int RMTChannel::RMTfillData(const byte buffer[], byte byteCount, byte repeatCount=0) {
  //int RMTChannel::RMTfillData(dccPacket packet) {
  // dataReady: Signals to then interrupt routine. It is set when
  // we have data in the channel buffer which can be copied out
  // to the HW. dataRepeat on the other hand signals back to
  // the caller of this function if the data has been sent enough
  // times (0 to 3 means 1 to 4 times in total).
  if (dataRepeat > 0) // we have still old work to do
    return dataRepeat;
  if (dataReady == true) // the packet is not copied out yet
    return 1000;
  if (DATA_LEN(byteCount) > maxDataLen) {  // this would overun our allocated memory for data
    DIAG(F("Can not convert DCC bytes # %d to DCC bits %d, buffer too small"), byteCount, maxDataLen);
    return -1;                          // something very broken, can not convert packet
  }

  // convert bytes to RMT stream of "bits"
  byte bitcounter = 0;
  for(byte n=0; n<byteCount; n++) {
    for(byte bit=0; bit<8; bit++) {
      if (buffer[n] & transmitMask[bit])
	setDCCBit1(data + bitcounter++);
      else
	setDCCBit0(data + bitcounter++);
    }
    setDCCBit0(data + bitcounter++); // zero at end of each byte
  }
  setDCCBit1(data + bitcounter-1);     // overwrite previous zero bit with one bit
  setEOT(data + bitcounter++);         // EOT marker
  dataLen = bitcounter;
  noInterrupts();                      // keep dataReady and dataRepeat consistnet to each other
  dataReady = true;
  dataRepeat = repeatCount+1;         // repeatCount of 0 means send once
  interrupts();
  return 0;
}

void IRAM_ATTR RMTChannel::RMTinterrupt() {
  //no rmt_tx_start(channel,true) as we run in loop mode
  //preamble is always loaded at beginning of buffer
  packetCounter++;
  if (!dataReady && dataRepeat == 0) { // we did run empty
    rmt_fill_tx_items(channel, idle, idleLen, preambleLen-1);
    return; // nothing to do about that
  }

  // take care of incoming data
  if (dataReady) {            // if we have new data, fill while preamble is running
    rmt_fill_tx_items(channel, data, dataLen, preambleLen-1);
    dataReady = false;
    if (dataRepeat == 0)       // all data should go out at least once
      DIAG(F("Channel %d DCC signal lost data"), channel);
  }
  if (dataRepeat > 0)         // if a repeat count was specified, work on that
    dataRepeat--;
}

bool RMTChannel::addPin(byte pin, bool inverted) {
  if (pin == UNUSED_PIN)
    return true;
  gpio_num_t gpioNum = (gpio_num_t)(pin);
  esp_err_t err;
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpioNum], PIN_FUNC_GPIO);
  err = gpio_set_direction(gpioNum, GPIO_MODE_OUTPUT);
  if (err != ESP_OK) return false;
  gpio_matrix_out(gpioNum, RMT_SIG_OUT0_IDX+channel, inverted, 0);
  if (err != ESP_OK) return false;
  return true;
}
bool RMTChannel::addPin(pinpair pins) {
  return addPin(pins.pin) && addPin(pins.invpin, true);
}
#endif //ESP32
