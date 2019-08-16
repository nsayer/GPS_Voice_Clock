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

// This file is the implementation of the Petit FAT FS Disk I/O
// skeleton for SD card I/O over SPI.

#include <avr/io.h>
#include <avr/wdt.h>
#include "GPS_Voice_Clock.h"
#include "diskio.h"

static inline __attribute__ ((always_inline)) unsigned char SPI_byte(unsigned char data) {
	SPIC.DATA = data;
	while(!(SPIC.STATUS & SPI_IF_bm)); // wait for complete
	return SPIC.DATA;
}

static unsigned char waitForStart(unsigned int timeout) {
        unsigned char byte;
        for(unsigned long now = ticks(); ticks() - now < timeout; ) {
		wdt_reset();
                if ((byte = SPI_byte(0xff)) != 0xff) break;
        }
        return (byte != 0xfe);
}

static unsigned char waitForIdle(unsigned int timeout) {
        for(unsigned long now = ticks(); ticks() - now < timeout; ) {
		wdt_reset();
                if (SPI_byte(0xff) == 0xff) return 0; //idle
        }
        return 1; // timeout
}

#define BLOCKSIZE (512)

#define INIT_TIMEOUT (2000 * (F_TICK / 1000))
#define CMD_TIMEOUT (300 * (F_TICK / 1000))
#define RW_TIMEOUT (500 * (F_TICK / 1000))
#define PWR_TIMEOUT (100 * (F_TICK / 1000))

// ready state is all bits 0
#define R1_READY_STATE 0
#define R1_IDLE_STATE _BV(0)
#define R1_ERASE_RESET _BV(1)
#define R1_ILLEGAL_CMD _BV(2)
#define R1_CMD_CRC_ERR _BV(3)
#define R1_ERASE_SEQ_ERR _BV(4)
#define R1_ADDR_ERR _BV(5)
#define R1_PARAM_ERR _BV(6)
// bit 7 is always 0.
 
static unsigned char sendCommand_R1(unsigned char cmd, unsigned long arg) {
        unsigned char response;
        if (waitForIdle(CMD_TIMEOUT)) {
                response = 0xff;
                return response;
        }

        SPI_byte(cmd | 0x40);
        SPI_byte((uint8_t)(arg >> 24));
        SPI_byte((uint8_t)(arg >> 16));
        SPI_byte((uint8_t)(arg >> 8));
        SPI_byte((uint8_t)(arg));
        switch(cmd) {
                case 0: SPI_byte(0x95); // checksum for command 0
                        break;
                case 8: SPI_byte(0x87); // checksum for command 8 with 0x1aa payload
                        break;
                default: SPI_byte(0xff); // bogus checksum
                        break;
        }
        response = 0xff;
        for(unsigned long now = ticks(); ticks() - now < CMD_TIMEOUT; ) {
                response = SPI_byte(0xff);
                if (!(response & 0x80)) break;
        }
        return response;
}

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (void)
{
	DSTATUS stat = 0;

	PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc; // pull card DO pin up

	// enable SPI, mode 0, master, 250 kHz
	SPIC.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV128_gc;

	DEASSERT_CS;
	SDPWR_ON;
	for(unsigned long now = ticks(); ticks() - now < PWR_TIMEOUT; );
	wdt_reset();

	for(int i = 0; i < 10; i++) // send 80 clocks
		SPI_byte(0xff);

	ASSERT_CS;

	unsigned char init_success = 0;
	for(unsigned long now = ticks(); ticks() - now < INIT_TIMEOUT; ) {
		wdt_reset();
		if (sendCommand_R1(0, 0) == R1_IDLE_STATE) {
			init_success = 1;
			break;
		}
	}
	if (!init_success) { stat = STA_NOINIT; goto fail; }

	if (sendCommand_R1(8, 0x1aaUL) & R1_ILLEGAL_CMD) { stat = STA_NOINIT; goto fail; }
	unsigned long r3resp = 0;
	for(int i = 0; i < 4; i++) {
		r3resp <<= 8;
		r3resp |= SPI_byte(0xff);
	}
	if ((r3resp & 0xff) != 0xaa) { stat = STA_NOINIT; goto fail; }

	init_success = 0;
	for(unsigned long now = ticks(); ticks() - now < INIT_TIMEOUT; ) {
		wdt_reset();
		sendCommand_R1(55, 0);
		if (sendCommand_R1(41, 0x40000000UL) == R1_READY_STATE) {
			init_success = 1;
			break;
		}
	}
	if (!init_success) { stat = STA_NOINIT; goto fail; }
	SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc; // Full speed!
	// fall through

fail:
	DEASSERT_CS;

	return stat;
}



/*-----------------------------------------------------------------------*/
/* Read Partial Sector                                                   */
/*-----------------------------------------------------------------------*/

DRESULT disk_readp (
	BYTE* buff,		/* Pointer to the destination object */
	DWORD sector,	/* Sector number (LBA) */
	UINT offset,	/* Offset in the sector */
	UINT count		/* Byte count (bit15:destination) */
)
{
	DRESULT res = RES_OK;

	ASSERT_CS;
	if (waitForIdle(RW_TIMEOUT)) goto fail;

	if (sendCommand_R1(17, sector) != R1_READY_STATE) goto fail;

	if (waitForStart(RW_TIMEOUT)) goto fail;

	for(int i = 0; i < offset; i++) {
		SPI_byte(0xff); // skip initial
	}
	for(int i = 0; i < count; i++) {
		((unsigned char*)buff)[i] = SPI_byte(0xff);
	}
	for(int i = 0; i < (BLOCKSIZE - (offset + count)); i++) {
		SPI_byte(0xff); // skip trailer
	}
	SPI_byte(0xff); // CRC
	SPI_byte(0xff); // CRC
	goto done;
fail:
	res = RES_ERROR;
done:
	DEASSERT_CS;
	return res;
}



/*-----------------------------------------------------------------------*/
/* Write Partial Sector                                                  */
/*-----------------------------------------------------------------------*/

DRESULT disk_writep (
	const BYTE* buff,	/* Pointer to the data to be written, NULL:Initiate/Finalize write operation */
	DWORD sc		/* Sector number (LBA) or Number of bytes to send */
)
{
	// read-only.
	return RES_ERROR;
}

