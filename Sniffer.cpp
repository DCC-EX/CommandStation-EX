/*
 *  © 2025 Harald Barth
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
#ifdef ARDUINO_ARCH_ESP32
#define DIAG_LED 33
#include "Sniffer.h"
#include "DIAG.h"
//extern Sniffer *DCCSniffer;

static void packeterror() {
  digitalWrite(DIAG_LED,HIGH);
}

static void clear_packeterror() {
  digitalWrite(DIAG_LED,LOW);
}

static bool halfbits2byte(uint16_t b, byte *dccbyte) {
/*
  if (b!=0 && b!=0xFFFF) {
    Serial.print("[ ");
    for(int n=0; n<16; n++) {
      Serial.print(b&(1<<n)?"1":"0");
    }
    Serial.println(" ]");
  }
*/
  for(byte n=0; n<8; n++) {
    switch (b & 0x03) {
    case 0x01:
    case 0x02:
      // broken bits
      packeterror();
      return false;
      break;
    case 0x00:
      bitClear(*dccbyte, n);
      break;
    case 0x03:
      bitSet(*dccbyte, n);
      break;
    }
    b = b>>2;
  }
  return true;
}

static void IRAM_ATTR blink_diag(int limit) {
  delay(500);
  for (int n=0 ; n<limit; n++) {
    digitalWrite(DIAG_LED,HIGH);
    delay(200);
    digitalWrite(DIAG_LED,LOW);
    delay(200);
  }
}

static bool IRAM_ATTR cap_ISR_cb(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata,void *user_data) {
  if (edata->cap_edge == MCPWM_BOTH_EDGE) {
    // should not happen at all
    // delays here might crash sketch
    blink_diag(2);
    return 0;
  }
  if (user_data) ((Sniffer *)user_data)->processInterrupt(edata->cap_value, edata->cap_edge == MCPWM_POS_EDGE);
//if (DCCSniffer)            DCCSniffer->processInterrupt(edata->cap_value, edata->cap_edge == MCPWM_POS_EDGE);
  
  return 0;
}

Sniffer::Sniffer(byte snifferpin) {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, snifferpin);
  // set capture edge, BIT(0) - negative edge, BIT(1) - positive edge
  // MCPWM_POS_EDGE|MCPWM_NEG_EDGE should be 3.
  //mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE|MCPWM_NEG_EDGE, 0);
  //mcpwm_isr_register(MCPWM_UNIT_0, sniffer_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
  //MCPWM0.int_ena.cap0_int_ena = 1;                      // Enable interrupt on CAP0 signal

  mcpwm_capture_config_t MCPWM_cap_config = { //Capture channel configuration
    .cap_edge = MCPWM_BOTH_EDGE,              // according to mcpwm.h
    .cap_prescale = 1,                        // 1 to 256 (see .h file)
    .capture_cb = cap_ISR_cb,                 // user defined ISR/callback
    .user_data = (void *)this                 // user defined argument to callback
  };
  pinMode(DIAG_LED ,OUTPUT);
  blink_diag(3); // so that we know we have DIAG_LED
  DIAG(F("Init sniffer on pin %d"), snifferpin);
  ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &MCPWM_cap_config));
}

#define SNIFFER_TIMEOUT 100L // 100 Milliseconds
bool Sniffer::inputActive(){
  unsigned long now = millis();
  return ((now - lastendofpacket) < SNIFFER_TIMEOUT);
}

#define DCC_TOO_SHORT 4000L // 4000 ticks are 50usec
#define DCC_ONE_LIMIT 6400L // 6400 ticks are 80usec

void IRAM_ATTR Sniffer::processInterrupt(int32_t capticks, bool posedge) {
  byte bit = 0;
  diffticks = capticks - lastticks;
  if (lastedge != posedge) {
    if (diffticks < DCC_TOO_SHORT) {
      return;
    }
    if (diffticks < DCC_ONE_LIMIT) {
      bit = 1;
    } else {
      bit = 0;
    }
    // update state variables for next round
    lastticks = capticks;
    lastedge = posedge;
    bitfield = bitfield << (uint64_t)1;
    bitfield = bitfield + (uint64_t)bit;

    // now the halfbit is in the bitfield. Analyze...
    
    if ((bitfield & 0xFFFFFF) == 0xFFFFFC){
      // This looks at the 24 last halfbits
      // and detects a preamble if
      // 22 are ONE and 2 are ZERO which is a
      // preabmle of 11 ONES and one ZERO
      if (inpacket) {
	// if we are already inpacket here we
	// got a preamble in the middle of a
	// packet
	packeterror();
      } else {
	clear_packeterror(); // everything fine again at end of preable after good packet
      }
      currentbyte = 0;
      dcclen = 0;
      inpacket = true;
      halfbitcounter = 18; // count 18 steps from 17 to 0 and then look at the byte
      return;
    }
    if (inpacket) {
      halfbitcounter--;
      if (halfbitcounter) {
	return; // wait until we have full byte
      } else {
	// have reached end of byte
	//if (currentbyte == 2) debugfield = bitfield;
	byte twohalfbits = bitfield & 0x03;
	switch (twohalfbits) {
	case 0x01:
	case 0x02:
	  // broken bits
	  inpacket = false;
	  packeterror();
	  return;
	  break;
	case 0x00:
	case 0x03:
	  // byte end
	  uint16_t b = (bitfield & 0x3FFFF)>>2; // take 18 halfbits and use 16 of them
	  if (!halfbits2byte(b, dccbytes + currentbyte)) {
	    // broken halfbits
	    inpacket = false;
	    packeterror();
	    return;
	  }
	  if (twohalfbits == 0x03) { // end of packet marker
	    inpacket = false;
	    dcclen = currentbyte+1;
	    debugfield = bitfield;
	    // put it into the out packet
	    if (fetchflag) {
	      // not good, should have been fetched
              // blink_diag(1);
	      packeterror(); // or better?
	    }
	    lastendofpacket = millis();
	    DCCPacket temppacket(dccbytes, dcclen);
	    if (!(temppacket == prevpacket)) {
	      // we have something new to offer to the fetch routine
	      outpacket.push_back(temppacket);
	      prevpacket = temppacket;
	      fetchflag = true;
	    }
	    return;
	  }
	  break;
	}
	halfbitcounter = 18;
	currentbyte++; // everything done for this end of byte
	if (currentbyte >= MAXDCCPACKETLEN) {
	  inpacket = false; // this is an error because we should have retured above
	  packeterror();    // when endof packet marker was active
	}
      }
    }
  } else { // lastedge == posedge
    // this should not happen, check later
  }
}

/*
static void IRAM_ATTR sniffer_isr_handler(void *) {
  DCCSniffer.processInterrupt();
}
*/
#endif // ESP32
