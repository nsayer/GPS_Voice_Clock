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

extern unsigned long ticks();

// how many times does ticks() increment per second?
// Keep this synced with the configuration of Timer D5!
#define F_TICK (2000U)

// outputs
// Port C - !CS
#define CRDCS_bm _BV(4)
// Pord D - !PWR
#define AUPWR_bm _BV(1)

// The diagnostic LEDs
#define LED_PORT PORTD
#define FIX_bm _BV(5)
#define GPS_ERR_bm _BV(6)
#define SD_ERR_bm _BV(7)

// inputs
// The button
#define PORT_SW PORTA.IN
#define BTN_0_bm _BV(0)
// The DIP switches are bits 4-7 of port A
#define DIPSW_gm (0xf0)
#define DIPSW_gp (4)

#define ASSERT_CS (PORTC.OUTCLR = CRDCS_bm)
#define DEASSERT_CS (PORTC.OUTSET = CRDCS_bm)
