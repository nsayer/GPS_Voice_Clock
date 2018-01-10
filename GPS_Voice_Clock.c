/*

    GPS Voice Clock
    Copyright (C) 2018 Nicholas W. Sayer

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
    
  */

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <math.h>
#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include "GPS_Voice_Clock.h"
#include "pff.h"

// 32 MHz
#define F_CPU (32000000UL)

// CLK2X = 0. For 9600 baud @ 32 MHz: 
#define BSEL (12)
#define BSCALE (4)

// The serial buffer length
#define RX_BUF_LEN (96)
#define TX_BUF_LEN (96)

// These are return values from the DST detector routine.
// DST is not in effect all day
#define DST_NO 0
// DST is in effect all day
#define DST_YES 1
// DST begins at 0200
#define DST_BEGINS 2
// DST ends 0300 - that is, at 0200 pre-correction.
#define DST_ENDS 3

// The possible values for dst_mode
#define DST_OFF 0
#define DST_US 1
#define DST_MODE_MAX DST_US

// This is the ticks timer frequency - how fast ticks_cnt ticks
// Keep this synced with the configuration of Timer D5!
#define F_TICK (2000U)

// We want something like 50 ms.
#define DEBOUNCE_TICKS (20 * (F_TICK / 1000))

// The ticks on seconds 1-9 are 25 ms long
#define TICK_TICKS (25 * (F_TICK / 1000))

// The beep on second 0 is 500 ms long
#define BEEP_TICKS (500 * (F_TICK / 1000))

// Thanks to Gareth Evans at http://todbot.com/blog/2008/06/19/how-to-do-big-strings-in-arduino/
// Note that you must be careful not to use this macro more than once per "statement", lest you
// risk overwriting the buffer before it is used. So no using it inside methods that return
// strings that are then used in snprintf statements that themselves use this macro.
char p_buffer[96];
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

// The double audio buffers.
volatile unsigned char audio_buf[2][512];

volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char tx_buf[TX_BUF_LEN];
volatile unsigned int tx_buf_head, tx_buf_tail;
volatile unsigned char rx_str_len;
volatile unsigned char nmea_ready;

volatile unsigned long ticks_cnt;
volatile unsigned long second_start_tick;

volatile unsigned char gps_locked;
volatile unsigned char new_second;

unsigned char dst_mode;
unsigned char tz;
unsigned long debounce_time;
unsigned char button_down;
unsigned char second_in_block;
unsigned int utc_ref_year;
unsigned char utc_ref_mon;
unsigned char utc_ref_day;
unsigned char playing;

char intro_filename[16], hour_filename[16], minute_filename[16], second_filename[16];

FATFS fatfs;

// The possible values for tz
#define TZ_UTC 0
#define TZ_PAC 1
#define TZ_MTN 2
#define TZ_CEN 3
#define TZ_EAS 4
#define TZ_AK 5
#define TZ_HI 6

// The first letter of the intro filename is the zone.
static const char zone_letters[] PROGMEM = "UPMCEAHU";

static const char zone_hours[] PROGMEM = {0, -8, -7, -6, -5, -9, -10, 0};

static inline char tz_letter() {
	return pgm_read_byte(&(zone_letters[tz & 7]));
}
static inline char tz_hour() {
	return pgm_read_byte(&(zone_hours[tz & 7]));
}

static const unsigned char month_tweak[] PROGMEM = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

static inline unsigned char first_sunday(unsigned char m, unsigned int y) {
	// first, what's the day-of-week for the first day of whatever month?
	// From http://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week
	y -= m < 3;
	unsigned char month_tweak_val = pgm_read_byte(&(month_tweak[m - 1]));
	unsigned char dow = (y + y/4 - y/100 + y/400 + month_tweak_val + 1) % 7;

	// If the 1st is a Sunday, then the answer is 1. Otherwise, we count
	// up until we find a Sunday.
	return (dow == 0)?1:(8 - dow);
}

static inline unsigned char calculateDSTUS(const unsigned char d, const unsigned char m, const unsigned int y) {
	// DST is in effect between the 2nd Sunday in March and the first Sunday in November
	// The return values here are that DST is in effect, or it isn't, or it's beginning
	// for the year today or it's ending today.
	unsigned char change_day;
	switch(m) {
		case 1: // December through February
		case 2:
		case 12:
			return DST_NO;
		case 3: // March
			change_day = first_sunday(m, y) + 7; // second Sunday.
			if (d < change_day) return DST_NO;
			else if (d == change_day) return DST_BEGINS;
			else return DST_YES;
			break;
		case 4: // April through October
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
			return DST_YES;
		case 11: // November
			change_day = first_sunday(m, y);
			if (d < change_day) return DST_YES;
			else if (d == change_day) return DST_ENDS;
			else return DST_NO;
			break;
		default: // This is impossible, since m can only be between 1 and 12.
			return 255;
	}
}
static inline unsigned char calculateDST(const unsigned char d, const unsigned char m, const unsigned int y) {
	switch(dst_mode) {
		case DST_US:
			return calculateDSTUS(d, m, y);
		default: // off - should never happen
			return DST_NO;
	}
}

static inline void startLeapCheck();

static inline void handle_time(char h, unsigned char m, unsigned char s, unsigned char dst_flags) {
	// What we get is the current second. We have to increment it
	// to represent the *next* second.
	s++;
	second_in_block = s % 10; // which second is this within the 10 second block?
	// Now move up to the next block. We're going to figure out our announcing-time.
	s += 10;
	s -= second_in_block;

	// Note that this also handles leap-seconds. We wind up pinning to 0
	// twice. We can't do other than that because we'd need to know that
	// the second after 59 is 60 instead of 0, and we can't know that.
	if (s >= 60) { s = 0; m++; }
	if (m >= 60) { m = 0; h++; }
	if (h >= 24) { h = 0; }

	// Move to local standard time.
/*
	// To support half-hour zones, we may have to add 30 minutes
	if (0) m += 30; // we'd need to make a UI for this somehow
	while (m >= 60) h++, m -= 60;
*/
	h += tz_hour();
	while (h >= 24) h -= 24;
	while (h < 0) h += 24;

	unsigned char dst_offset = 0;
	if (dst_mode != DST_OFF) {
		// For Europe, decisions are at 0100. Everywhere else it's 0200.
		unsigned char decision_hour = 2;
		switch(dst_flags) {
			case DST_NO: dst_offset = 0; break; // do nothing
			case DST_YES: dst_offset = 1; break; // add one hour
			case DST_BEGINS:
				dst_offset = (h >= decision_hour)?1:0; // offset becomes 1 at 0200 (0100 EU)
				break;
			case DST_ENDS:
				// The *summer time* hour has to be the decision hour,
				// and we haven't yet made 'h' the summer time hour,
				// so compare it to one less than the decision hour.
				dst_offset = (h >= (decision_hour - 1))?0:1; // offset becomes 0 at 0200 (daylight) (0100 EU)
				break;
		}
		h += dst_offset;
		if (h >= 24) h -= 24;
	}

	// It's ok to write these whenever you like. They will only really ever change
	// during second zero of the block, which is reserved for the beep.
	if (tz == TZ_UTC) {
		// UTC has no DST.
		snprintf(intro_filename, sizeof(intro_filename), P("/ZONE/%c"), tz_letter());
	} else {
		snprintf(intro_filename, sizeof(intro_filename), P("/ZONE/%c%c"), tz_letter(), dst_offset?'D':'S');
	}
	snprintf(hour_filename, sizeof(hour_filename), P("/HOUR/%d"), h);
	snprintf(minute_filename, sizeof(minute_filename), P("/MINUTE/%d"), m);
	snprintf(second_filename, sizeof(second_filename), P("/SECOND/%d"), s);

	// Every hour, check to see if the leap second value in the receiver is out-of-date
	if (m == 30 && s == 0) startLeapCheck();
}

static inline void tx_char(const unsigned char c);
static inline void write_msg(const unsigned char *msg, const size_t length) {
	for(int i = 0; i < length; i++) {
		tx_char(msg[i]);
	}
}

const unsigned char PROGMEM leap_check_msg[] = { 0xa0, 0xa1, 0x00, 0x02, 0x64, 0x20, 0x44, 0x0d, 0x0a };
static inline void startLeapCheck(void) {
	// Ask for the time message. We expect a 0x64-0x8e in response
	unsigned char msg[sizeof(leap_check_msg)];
	memcpy_P(msg, leap_check_msg, sizeof(leap_check_msg));
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM leap_update_msg[] = { 0xa0, 0xa1, 0x00, 0x04, 0x64, 0x1f, 0x00, 0x01, 0x7a, 0x0d, 0x0a };
static inline void updateLeapDefault(const unsigned char leap_offset) {
	// This is a set leap-second default message. It will write the given
	// offset to flash.
	unsigned char msg[sizeof(leap_update_msg)];
	memcpy_P(msg, leap_update_msg, sizeof(leap_update_msg));
	msg[6] = leap_offset;
	msg[8] ^= leap_offset; // fix the checksum
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM get_utc_ref_msg[] = { 0xa0, 0xa1, 0x00, 0x02, 0x64, 0x16, 0x72, 0x0d, 0x0a };
static inline void startUTCReferenceFetch() {
	// This is a request for UTC reference date message. We expect a 0x64-0x8a in response
	unsigned char msg[sizeof(get_utc_ref_msg)];
	memcpy_P(msg, get_utc_ref_msg, sizeof(get_utc_ref_msg));
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM utc_ref_msg[] = { 0xa0, 0xa1, 0x00, 0x08, 0x64, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x71, 0x0d, 0x0a };
static inline void updateUTCReference(const unsigned int y, const unsigned char mon, const unsigned char d) {
	// This sets the UTC reference date, which controls the boundaries of the GPS week window
	unsigned char msg[sizeof(utc_ref_msg)];
	memcpy_P(msg, utc_ref_msg, sizeof(utc_ref_msg));
	msg[7] = (unsigned char)(y >> 8);
	msg[8] = (unsigned char)y;
	msg[9] = mon;
	msg[10] = d;
	for(int i = 7; i <= 10; i++) msg[12] ^= msg[i]; // fix checksum
	write_msg(msg, sizeof(msg));
}

static const char *skip_commas(const char *ptr, const int num) {
	for(int i = 0; i < num; i++) {
		ptr = strchr(ptr, ',');
		if (ptr == NULL) return NULL; // not enough commas
		ptr++; // skip over it
	}
	return ptr;
}

static const char hexes[] PROGMEM = "0123456789abcdef";

static unsigned char hexChar(unsigned char c) {
	if (c >= 'A' && c <= 'F') c += ('a' - 'A'); // make lower case
	const char* outP = strchr_P(hexes, c);
	if (outP == NULL) return 0;
	return (unsigned char)(outP - hexes);
}

static inline void handleGPS(const unsigned char *rx_sentence, const unsigned int str_len) {
	if (str_len >= 4 && rx_sentence[0] == 0xa0 && rx_sentence[1] == 0xa1) { // binary protocol message
		unsigned int payloadLength = (((unsigned int)rx_sentence[2]) << 8) | rx_sentence[3];
		if (str_len != payloadLength + 5) {
			return; // the A0, A1 bytes, length and checksum are added
		}
		unsigned int checksum = 0;
		for(int i = 0; i < payloadLength; i++) checksum ^= rx_sentence[i + 4];
		if (checksum != rx_sentence[payloadLength + 4]) {
			return; // checksum mismatch
		}
		if (rx_sentence[4] == 0x64 && rx_sentence[5] == 0x8a) {
			utc_ref_year = (rx_sentence[3 + 4] << 8) | rx_sentence[3 + 5];
			utc_ref_mon = rx_sentence[3 + 6];
			utc_ref_day = rx_sentence[3 + 7];
		} else if (rx_sentence[4] == 0x64 && rx_sentence[5] == 0x8e) {
			if (!(rx_sentence[15 + 3] & (1 << 2))) return; // GPS leap seconds invalid
			if (rx_sentence[13 + 3] == rx_sentence[14 + 3]) return; // Current and default agree
			updateLeapDefault(rx_sentence[14 + 3]);
		} else {
			return; // unknown binary protocol message
		}
	}

	if (str_len < 9) return; // No sentence is shorter than $GPGGA*xx
	// First, check the checksum of the sentence
	unsigned char checksum = 0;
	int i;
	for(i = 1; i < str_len; i++) {
		if (rx_sentence[i] == '*') break;
		checksum ^= rx_sentence[i];
	}
	if (i > str_len - 3) {
		return; // there has to be room for the "*" and checksum.
	}
	i++; // skip the *
	unsigned char sent_checksum = (hexChar(rx_sentence[i]) << 4) | hexChar(rx_sentence[i + 1]);
	if (sent_checksum != checksum) {
		return; // bad checksum.
	}
	  
	const char *ptr = (char *)rx_sentence;
	if (!strncmp_P(ptr, PSTR("$GPRMC"), 6)) {
		// $GPRMC,172313.000,A,xxxx.xxxx,N,xxxxx.xxxx,W,0.01,180.80,260516,,,D*74\x0d\x0a
		ptr = skip_commas(ptr, 1);
		if (ptr == NULL) return; // not enough commas
		char h = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		unsigned char min = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		unsigned char s = (ptr[4] - '0') * 10 + (ptr[5] - '0');
		ptr = skip_commas(ptr, 1);
		if (ptr == NULL) return; // not enough commas
		gps_locked = *ptr == 'A'; // A = AOK.
		ptr = skip_commas(ptr, 7);
		if (ptr == NULL) return; // not enough commas
		unsigned char d = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		unsigned char mon = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		unsigned int y = (ptr[4] - '0') * 10 + (ptr[5] - '0');

		// We must turn the two digit year into the actual A.D. year number.
		// As time goes forward, we can keep a record of how far time has gotten,
		// and assume that time will always go forwards. If we see a date ostensibly
		// in the past, then it "must" mean that we've actually wrapped and need to
		// add 100 years. We keep this "reference" date in sync with the GPS receiver,
		// as it uses the reference date to control the GPS week rollover window.
		y += 2000;
		while (y < utc_ref_year) y += 100; // If it's in the "past," assume time wrapped on us.

		if (utc_ref_year != 0 && y != utc_ref_year) {
			// Once a year, we should update the refence date in the receiver. If we're running on New Years,
			// then that's probably when it will happen, but anytime is really ok. We just don't want to do
			// it a lot for fear of burning the flash out in the GPS receiver.
			updateUTCReference(y, mon, d);
			utc_ref_year = y;
			utc_ref_mon = mon;
			utc_ref_day = d;
		}

		// The problem is that our D/M/Y is UTC, but DST decisions are made in the local
		// timezone. We can adjust the day against standard time midnight, and
		// that will be good enough. Don't worry that this can result in d being either 0
		// or past the last day of the month. Neither of those will match the "decision day"
		// for DST, which is the only day on which the day of the month is significant.
		if (h + tz_hour() < 0) d--;
		if (h + tz_hour() > 23) d++;
		unsigned char dst_flags = calculateDST(d, mon, y);
		handle_time(h, min, s, dst_flags);
	}
}

// ticks counter ISR
ISR(TCD5_OVF_vect) {
	TCD5.INTFLAGS = TC5_OVFIF_bm; // ack
	ticks_cnt++;
}

// PPS ISR
ISR(PORTC_INT_vect) {
	PORTC.INTFLAGS = _BV(0); // ack

	second_start_tick = ticks_cnt;
	new_second = 1;
	if (gps_locked) {
		PORTD.OUTSET = _BV(4); // turn on the tick
	}
}

// serial receive interrupt.
ISR(USARTC0_RXC_vect) {
	unsigned char rx_char = USARTC0.DATA;
 
	if (nmea_ready) return; // ignore serial until the buffer is handled 
	if (rx_str_len == 0 && !(rx_char == '$' || rx_char == 0xa0)) return; // wait for a "$" or A0 to start the line.
	rx_buf[rx_str_len] = rx_char;
	if (rx_char == 0x0d || rx_char == 0x0a) {
		rx_buf[rx_str_len] = 0; // null terminate
		nmea_ready = 1; // Mark it as ready
		return;
	}
	if (++rx_str_len == RX_BUF_LEN) {
		// The string is too long. Start over.
		rx_str_len = 0;
	}
}

ISR(USARTC0_DRE_vect) {
	if (tx_buf_head == tx_buf_tail) {
		// the transmit queue is empty.
		USARTC0.CTRLA &= ~USART_DREINTLVL_gm; // disable the TX interrupt.
		//USARTC0.CTRLA |= USART_DREINTLVL_OFF_gc; // redundant - off is a zero value
		return;
	}
	USARTC0.DATA = tx_buf[tx_buf_tail];
	if (++tx_buf_tail == TX_BUF_LEN) tx_buf_tail = 0; // point to the next char
}

// If the TX buffer fills up, then this method will block, which should be avoided.
static inline void tx_char(const unsigned char c) {
	int buf_in_use;
	do {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			buf_in_use = tx_buf_head - tx_buf_tail;
		}
		if (buf_in_use < 0) buf_in_use += TX_BUF_LEN;
		wdt_reset(); // we might be waiting a while.
	} while (buf_in_use >= TX_BUF_LEN - 2) ; // wait for room in the transmit buffer

	tx_buf[tx_buf_head] = c;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// this needs to be atomic, because an intermediate state is tx_buf_head
		// pointing *beyond* the end of the buffer.
		if (++tx_buf_head == TX_BUF_LEN) tx_buf_head = 0; // point to the next free spot in the tx buffer
	}
	//USARTC0.CTRLA &= ~USART_DREINTLVL_gm; // this is redundant - it was already 0
	USARTC0.CTRLA |= USART_DREINTLVL_LO_gc; // enable the TX interrupt. If it was disabled, then it will trigger one now.
}

unsigned long ticks() {
	unsigned long out;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		out = ticks_cnt;
	}
	return out;
}

static unsigned char check_button() {
	unsigned long now = ticks();
	if (debounce_time != 0 && now - debounce_time < DEBOUNCE_TICKS) {
		// We don't pay any attention to the buttons during debounce time.
		return 0;
	} else {
		debounce_time = 0; // debounce is over
	}
	unsigned char status = PORT_SW & BTN_0_bm;
	status ^= BTN_0_bm; // invert the button - 0 means down.
	if (!((button_down == 0) ^ (status == 0))) return 0; // either no button is down, or a button is still down

	// Something *changed*, which means we must now start a debounce interval.
	debounce_time = now;
	if (!debounce_time) debounce_time++; // it's not allowed to be zero

	if (!button_down && status) {
		button_down = 1; // a button has been pushed
		return 1;
	}
	if (button_down && !status) {
		button_down = 0; // a button has been released
		return 0;
	}
	__builtin_unreachable(); // we'll never get here.
}

void play_file(char *filename) {
	unsigned int cnt;
	// select the file
	if (pf_open(filename)) {
		LED_PORT.OUTSET = SD_ERR_bm;
		return;
	}

	// first, make sure the transfer complete flags are clear.
	EDMA.INTFLAGS |= EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm;

	if (pf_read((void*)(audio_buf[0]), sizeof(audio_buf[0]), &cnt)) {
		LED_PORT.OUTSET = SD_ERR_bm;
		return;
	}
	EDMA.CH0.TRFCNT = cnt;
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm; // start
        if (cnt != sizeof(audio_buf[0])) {
		// last read - just return.
		return;
	}

	// Set up the next block immediately
	if (pf_read((void*)(audio_buf[1]), sizeof(audio_buf[1]), &cnt)) {
		LED_PORT.OUTSET = SD_ERR_bm;
		return;
	}
	if (cnt == 0) return; // no actual data - so the first block was the end.
	EDMA.CH2.TRFCNT = cnt;
	EDMA.CH2.CTRLA |= EDMA_CH_REPEAT_bm; // take over from the first channel
	if (cnt == sizeof(audio_buf[1])) {
		// If this isn't the last block, then tell the top layer to
		// continue filling buffers as they become empty.
		playing = 1;
	}
}

// main() never returns.
void __ATTR_NORETURN__ main(void) {

	// We have a 16 MHz crystal. Use the PLL to double that to 32 MHz.

	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	while(!(OSC.STATUS & OSC_XOSCRDY_bm)) ; // wait for it.

	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | (2 << OSC_PLLFAC_gp); // PLL from XOSC, mult by 2
	OSC.CTRL |= OSC_PLLEN_bm;
	while(!(OSC.STATUS & OSC_PLLRDY_bm)) ; // wait for it.

	_PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_PLL_gc); // switch to it
	OSC.CTRL &= ~(OSC_RC2MEN_bm); // we're done with the 2 MHz osc.

	//wdt_enable(WDTO_1S); // This is broken on XMegas.
	// This replacement code doesn't disable interrupts (but they're not on now anyway)
	_PROTECTED_WRITE(WDT.CTRL, WDT_PER_256CLK_gc | WDT_ENABLE_bm | WDT_CEN_bm);
	while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take
	// We don't want a windowed watchdog.
	_PROTECTED_WRITE(WDT.WINCTRL, WDT_WCEN_bm);
	while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take

	// Leave on only the parts of the chip we use.
	PR.PRGEN = PR_XCL_bm | PR_RTC_bm;
	PR.PRPA = PR_ADC_bm | PR_AC_bm;
	PR.PRPC = PR_TWI_bm | PR_HIRES_bm;
	PR.PRPD = PR_USART0_bm;

	PORTA.DIRCLR = 0xff; // All of port A is inputs
	PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; //... except that pin 2 is DAC output
	PORTA.PIN0CTRL = PORT_OPC_PULLUP_gc; // Button gets pull-up
	PORTA.PIN4CTRL = PORT_OPC_PULLUP_gc; // all of the option switches get pull-ups
	PORTA.PIN5CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN7CTRL = PORT_OPC_PULLUP_gc;

	PORTC.OUTSET = _BV(3) | _BV(4); // !SD_CS and TXD default to high
	PORTC.DIRSET = _BV(3) | _BV(4) | _BV(5) | _BV(7); // TXD and most of SPI is output
	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc; // pull card DO pin up

	// Trigger an interrupt on the rising edge of PPS.
	PORTC.PIN0CTRL = PORT_ISC_RISING_gc;
	PORTC.INTCTRL = PORT_INTLVL_MED_gc;
	PORTC.INTMASK = _BV(0);

	PORTD.OUTSET = CRDPWR_bm | AUPWR_bm; // start with SD power and audio turned off
	PORTD.DIRSET = _BV(0) | _BV(1) | _BV(6) | _BV(7); // power switches and LEDS are out

	rx_str_len = 0;
	tx_buf_head = tx_buf_tail = 0;
	nmea_ready = 0;

	// 9600 baud async serial, 8N1, low priority interrupt on receive
	USARTC0.CTRLA = USART_DRIE_bm | USART_RXCINTLVL_LO_gc;
	USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;
	USARTC0.CTRLD = 0;
	USARTC0.BAUDCTRLA = BSEL & 0xff;
	USARTC0.BAUDCTRLB = (BSEL >> 8) | (BSCALE << USART_BSCALE_gp);

	// TCC4 is an 8 kHz clock for triggering DMA to the DAC for audio
	// playback.
	TCC4.CTRLA = TC45_CLKSEL_DIV8_gc; // 2 MHz timer clocking.
	TCC4.CTRLB = 0;
	TCC4.CTRLC = 0;
	TCC4.CTRLD = 0;
	TCC4.CTRLE = 0;
	TCC4.INTCTRLA = 0;
	TCC4.INTCTRLB = 0;
	TCC4.PER = 499; // 8 kHz

	// TCC5 is unused

	// TCD5 generates 1 kHz output on OC5A. This is toggled on
	// and off by making OC5A (PD4) an output or not.
	TCD5.CTRLA = TC45_CLKSEL_DIV64_gc; // 2 MHz clocking
	TCD5.CTRLB = TC45_WGMODE_FRQ_gc; // frequency generation mode
	TCD5.CTRLC = 0;
	TCD5.CTRLD = 0;
	TCD5.CTRLE = TC45_CCAMODE_COMP_gc;
        TCD5.INTCTRLA = TC45_OVFINTLVL_HI_gc;
	TCD5.INTCTRLB = 0;
	TCD5.CCA = 249; // toggle at 2 kHz -> output 1 kHz

	// Event 0 is used to trigger DAC conversion during audio output.
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC4_OVF_gc;
	EVSYS.CH0CTRL = 0;

        DACA.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
        DACA.CTRLB = DAC_CHSEL_SINGLE_gc | DAC_CH0TRIG_bm; // Trigger a conversion on event 0 - from the 8 kHz timer
        DACA.CTRLC = DAC_REFSEL_AVCC_gc | DAC_LEFTADJ_bm; // lop off the low 4 bits of each sample
        DACA.EVCTRL = DAC_EVSEL_0_gc; // trigger event 0

        // Load factory calibration into the DAC
        NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
        DACA.CH0GAINCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA0GAINCAL));
        DACA.CH0OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA0OFFCAL));
        DACA.CH1GAINCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA1GAINCAL));
        DACA.CH1OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACA1OFFCAL));
        NVM.CMD = NVM_CMD_NO_OPERATION_gc;

        EDMA.CTRL = EDMA_RESET_bm;
        while(EDMA.CTRL & EDMA_RESET_bm); // wait for it

        // DMA is two double-buffered, standard channels.
        EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD02_gc | EDMA_DBUFMODE_BUF0123_gc | EDMA_PRIMODE_RR0123_gc;

        EDMA.CH0.CTRLA = EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm; // single-shot, two byte burst
        EDMA.CH0.CTRLB = 0; // no interrupts
        EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;
        EDMA.CH0.DESTADDRCTRL = EDMA_CH_RELOAD_BURST_gc | EDMA_CH_DIR_INC_gc;
        EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_DACA_CH0_gc;
        EDMA.CH0.DESTADDR = (unsigned int)&(DACA.CH0DATA);
        EDMA.CH0.ADDR = (unsigned int)&(audio_buf[0]);

        // Channel 2 is configured exactly the same way as channel 0, but with the 2nd memory buffer.
        EDMA.CH2.CTRLA = EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm; // single-shot, two byte burst
        EDMA.CH2.CTRLB = 0;
        EDMA.CH2.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;
        EDMA.CH2.DESTADDRCTRL = EDMA_CH_RELOAD_BURST_gc| EDMA_CH_DIR_INC_gc;
        EDMA.CH2.TRIGSRC = EDMA_CH_TRIGSRC_DACA_CH0_gc;
        EDMA.CH2.DESTADDR = (unsigned int)&(DACA.CH0DATA);
        EDMA.CH2.ADDR = (unsigned int)&(audio_buf[1]);

	ticks_cnt = 0;
	utc_ref_year = 0;
	gps_locked = 0;
	debounce_time = 0;
	button_down = 0;
	second_in_block = 0;
	second_start_tick = 0;
	new_second = 0;
	playing = 0;
	*intro_filename = 0;
	*hour_filename = 0;
	*minute_filename = 0;
	*second_filename = 0;

        PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
        sei();

	// Switches are inverted. Flip them and shift them.
	unsigned char cfg = (PORT_SW ^ DIPSW_gm) >> DIPSW_gp;
	dst_mode = (cfg & 0x8)?DST_US:DST_OFF;
	tz = cfg & 0x7;
	if (tz == TZ_UTC) dst_mode = DST_OFF; // no DST for UTC.

	startUTCReferenceFetch();

        if (pf_mount(&fatfs)) {
		LED_PORT.OUTSET = SD_ERR_bm;
		while(1) wdt_reset(); // Die hard
	}

	while(1) {
		// The big, main loop. Our tasks:
		// 1. Pet the watchdog
		wdt_reset();

		// 2. Handle serial sentence completions
		if (nmea_ready) {
			// We're doing this here to get the heavy processing
			// out of an ISR. It *is* a lot of work, after all.
			// But this may take a long time, so we need to let
			// the serial ISR keep working.
			unsigned char temp_buf[RX_BUF_LEN];
			unsigned int temp_len = rx_str_len;
			memcpy(temp_buf, (const char *)rx_buf, temp_len);
			rx_str_len = 0; // clear the buffer
			nmea_ready = 0;
			handleGPS(temp_buf, temp_len);
			continue;
		}

		// 3. Fill completed audio buffers if background audio playback is in progress
		if (EDMA.INTFLAGS & (EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm)) {
			// A DMA transfer completed, so we must at least ACK it.
			// which channel?
			unsigned char chan = (EDMA.INTFLAGS & EDMA_CH0TRNFIF_bm)?0:1;
                	EDMA.INTFLAGS |= chan?EDMA_CH2TRNFIF_bm:EDMA_CH0TRNFIF_bm; // ack

			if (!playing) continue; // This just means the last block finished.

			unsigned int cnt;
			if (pf_read((void*)(audio_buf[chan]), sizeof(audio_buf[chan]), &cnt)) {
				// abort
				playing = 0;
				LED_PORT.OUTSET = SD_ERR_bm;
				continue;
			}
			if (cnt == 0) {
				// There was no data in the final read, so don't bother with it.
				playing = 0;
				continue;
			}
			(chan?&(EDMA.CH2):&(EDMA.CH0))->TRFCNT = cnt;
			(chan?&(EDMA.CH2):&(EDMA.CH0))->CTRLA |= EDMA_CH_REPEAT_bm; // continue with this buffer
			if (cnt != sizeof(audio_buf[chan])) {
				// This is the last audio chunk, so we are done. Stop paying attention.
				// DMA will stop after this buffer plays because the other one won't have had
				// REPEAT turned on.
				playing = 0;
			}
			continue;
		}

		// 4. Track GPS status on the GPS lock LED
		if (gps_locked) {
			LED_PORT.OUTCLR = GPS_ERR_bm;
		} else {
			LED_PORT.OUTSET = GPS_ERR_bm;
			continue;
		}

		// 5. End the tick or beep that was started by the PPS ISR
		unsigned long ticks_in_second = ticks() - second_start_tick;
		unsigned long ticklen = second_in_block == 0?BEEP_TICKS:TICK_TICKS;
		if (ticks_in_second > ticklen) {
			PORTD.DIRCLR = _BV(4); // turn off the ticking/beeping
		}

		// 6. If this is a new second (set by PPS ISR), then kick off scheduled playback events
		if (new_second) {
			new_second = 0;
			switch(second_in_block) {
				case 1: // "At the tone..." intro for timezone
					play_file(intro_filename);
					break;
				case 5: // "10 hours"
					play_file(hour_filename);
					break;
				case 6: // "57 minutes"
					play_file(minute_filename);
					break;
				case 7: // "Exactly."
					play_file(second_filename);
					break;
			}
			continue;
		}

		// 7. Check the button for user interactions
		unsigned char button = check_button();
		if (button) {
			PORTD.OUTTGL = AUPWR_bm; // toggle the audio on and off
		}
	}
	__builtin_unreachable();
}
