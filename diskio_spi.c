/*

    Evil Simon
    Copyright (C) 2019 Nicholas W. Sayer

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
// skeleton for SPI flash chip I/O.

#include <avr/io.h>
#include <avr/wdt.h>
#include "diskio.h"
#include "Evil_Simon.h"

static inline __attribute__ ((always_inline)) unsigned char SPI_byte(unsigned char data) {
	SPIC.DATA = data;
	while(!(SPIC.STATUS & SPI_IF_bm)); // wait for complete
	return SPIC.DATA;
}

#define BLOCKSIZE (512UL)

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (void)
{

	DEASSERT_CS;
	SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc; // Full speed!

	return 0;
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

	unsigned long start_addr = sector * BLOCKSIZE + offset;

	ASSERT_CS;
	SPI_byte(0x3); // READ command
	SPI_byte(((unsigned char)(start_addr >> 16))); // send 24 bits of address, high byte first
	SPI_byte(((unsigned char)(start_addr >> 8)));
	SPI_byte(((unsigned char)(start_addr >> 0)));
	for(int i = 0; i < count; i++) {
		((unsigned char*)buff)[i] = SPI_byte(0xff);
	}
	DEASSERT_CS;
	return RES_OK;
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

