/*

    XMega DMA DAC audio test program
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

#define F_CPU (32000000UL)

#include <stddef.h>  
#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <math.h>
#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include "pff.h"

#define FILENAME "/AUDIO.RAW"

// double buffer for audio I/O.
static volatile unsigned char audio_buf[2][512];

volatile unsigned long millis_cnt;

ISR(TCD5_OVF_vect) {
	TCD5.INTFLAGS = TC5_OVFIF_bm; // ack
	millis_cnt++;
}

void __ATTR_NORETURN__ main(void) {

        // We have a 16 MHz crystal. Use the PLL to double that to 32 MHz.

        // Run the CPU at 32 MHz using the RC osc.
        OSC.CTRL |= OSC_RC32MEN_bm;
        while(!(OSC.STATUS & OSC_RC32MRDY_bm)) ; // wait for it.

        _PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_RC32M_gc); // switch to it
        OSC.CTRL &= ~(OSC_RC2MEN_bm); // we're done with the 2 MHz osc.

        // Turn on the 32 kHz reference oscillator
        OSC.XOSCCTRL = OSC_X32KLPM_bm | OSC_XOSCSEL_32KHz_gc;
        OSC.CTRL |= OSC_XOSCEN_bm;
        while(!(OSC.STATUS & OSC_XOSCRDY_bm)) ; // wait for it.

        // Set up the DFLL to discipline the 32 MHz RC oscillator from the crystal
        OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
        DFLLRC32M.COMP1 = (unsigned char)(F_CPU / 1024);
        DFLLRC32M.COMP2 = (unsigned char)((F_CPU / 1024) >> 8);
        DFLLRC32M.CTRL = DFLL_ENABLE_bm;

        // Leave on only the parts of the chip we use.
        PR.PRGEN = PR_XCL_bm | PR_RTC_bm;
        PR.PRPA = PR_ADC_bm | PR_AC_bm;
        PR.PRPC = PR_TWI_bm | PR_HIRES_bm;
        PR.PRPD = PR_USART0_bm;

	PORTA.DIRCLR = 0xff; // everything is an input.
	PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; //... except that pin 2 is DAC output

	PORTC.OUTSET = _BV(4); // de-assert CD_CS - high
	PORTC.DIRSET = _BV(4) | _BV(5) | _BV(7);
	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc; // pull card output pin up

	// All of port D is output
	PORTD.DIRSET = 0xff;
	PORTD.OUTCLR = 0xff; // start all low.

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
	EDMA.CH0.TRFCNT = sizeof(audio_buf[0]);
	EDMA.CH0.ADDR = (unsigned int)&(audio_buf[0]);

	// Channel 2 is configured exactly the same way as channel 0, but with the 2nd memory buffer.
	EDMA.CH2.CTRLA = EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm; // single-shot, two byte burst
	EDMA.CH2.CTRLB = 0;
	EDMA.CH2.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH2.DESTADDRCTRL = EDMA_CH_RELOAD_BURST_gc| EDMA_CH_DIR_INC_gc;
	EDMA.CH2.TRIGSRC = EDMA_CH_TRIGSRC_DACA_CH0_gc;
	EDMA.CH2.DESTADDR = (unsigned int)&(DACA.CH0DATA);
	EDMA.CH2.TRFCNT = sizeof(audio_buf[1]);
	EDMA.CH2.ADDR = (unsigned int)&(audio_buf[1]);

	// Event 0 is the overflow from TCC4, which will free-run at 8 kHz.
	// The DAC will use that as its sample clock.
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC4_OVF_gc;
        EVSYS.CH0CTRL = 0;

        // TCC4 is an 8 kHz clock for triggering DMA to the DAC for audio
        // playback.
        TCC4.CTRLA = TC45_CLKSEL_DIV8_gc; // 4 MHz timer clocking.
        TCC4.CTRLB = 0;
        TCC4.CTRLC = 0;
        TCC4.CTRLD = 0;
        TCC4.CTRLE = 0;
        TCC4.INTCTRLA = 0;
        TCC4.INTCTRLB = 0;
        TCC4.PER = 499; // 8 kHz

	// TCC5 is unused

        // TCD5 generates 1 kHz output on OC5A. This is toggled on
        // and off in software. As a side effect, it keeps a milli timer for the SD subsystem
        TCD5.CTRLA = TC45_CLKSEL_DIV64_gc; // 500 kHz clocking
        TCD5.CTRLB = TC45_WGMODE_FRQ_gc; // frequency generation mode
        TCD5.CTRLC = 0;
        TCD5.CTRLD = 0;
        TCD5.CTRLE = TC45_CCAMODE_COMP_gc;
        TCD5.INTCTRLA = TC45_OVFINTLVL_HI_gc;
        TCD5.INTCTRLB = 0;
        TCD5.CCA = 249; // toggle at 2 kHz -> output 1 kHz

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
        sei();

	FATFS fatfs;

	FRESULT status;
	if ((status = pf_mount(&fatfs))) goto fail;
	if ((status = pf_open(FILENAME))) goto fail;

	unsigned char done = 0, first = 1, chan = 1;
	do {
		chan = !chan;
		unsigned int cnt;
		if ((status = pf_read((void*)(audio_buf[chan]), sizeof(audio_buf[chan]), &cnt))) goto fail;
		(chan?&(EDMA.CH2):&(EDMA.CH0))->TRFCNT = cnt;
		done = (cnt != sizeof(audio_buf[chan])); // last read?
		if (!done) {
			(chan?&(EDMA.CH2):&(EDMA.CH0))->CTRLA |= EDMA_CH_REPEAT_bm; // repeat
		}
		if (first) {
			first = 0;
			(chan?&(EDMA.CH2):&(EDMA.CH0))->CTRLA |= EDMA_CH_ENABLE_bm; // start
			if (!done)
				continue; // don't wait for the first transfer
		}
		while(!(EDMA.INTFLAGS & (EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm))) ; // wait for complete
		EDMA.INTFLAGS |= EDMA_CH0TRNFIF_bm | EDMA_CH2TRNFIF_bm; // ack
	} while(!done);

	PORTD.OUT = _BV(0); // success
	while(1);
fail:
	PORTD.OUT = (status << 5) | _BV(1);
	while(1);
}
