# GPS_Voice_Clock

This is the source code for https://hackaday.io/project/28949-gps-talking-clock

Note that this codebase includes Petit FAT FS. Petit FAT FS can be found at http://elm-chan.org/fsw/ff/00index_p.html

The code I've written, exclusive of Petit FAT FS (but *inclusive* of the portion of diskio.c that is the actual code
that reads from SD), is licensed under GPL V2. That does not apply to Petit, of course, which remains covered by what
is essentially an MIT license.

## Usage

Build the code with the included Makefile, which assumes you have an avr-gcc with XMega support in your path. Upload
it to the controller on the board with avrdude and a PDI capable programmer.

DIP switch 4 selects whether to use daylight savings time or not. Switches 3 to 0 select the time zone as follows:

0. UTC (in this case, the DST switch is ignored and DST is always disabled)
1. Pacific time
2. Mountain time
3. Central time
4. Eastern time
5. Alaska time
6. Hawaii time
7. unused

The button toggles the audio on and off. The two lights on the board indicate a GPS error or an SD card error, respectively.

The SD card should be FAT16 or FAT32 formatted. All of the directory names should be uppercase.

The SD card has the following main directories defined: "ZONE", "HOUR", "MINUTE" and "SECOND".

The files in the "ZONE" directory are named with two letter names (except for UTC, which is just the letter "U"). The first letter
is the time zone in question - P, M, C, E, A or H. The second letter is either S for standard time or D for daylight savings time.
Each file is excpected to be 4 seconds long at most. Each is intended to have something like "At the tone, Pacific Standard Time will be"

The files in the other three directories have names that are just numbers (no leading digit for single digit values). The numbers are
0-23 for HOUR, 0-59 for MINUTE and 0, 10, 20, 30, 40, 50 for second. Each should last no longer than 1 second and should just be the
number and units. As a special case, the 0 seconds file should simply say "Exactly."

The audio files are little-endian 16 bit unsigned 8 kHz mono raw audio samples. You can create them from some other format with sox. The
command line arguments for the output should be -t raw -c 1 -e unsigned -b 16 -r 8000.

The SD_audio.zip file in this repository can be unpacked into the top directory of a suitably formatted card.

The card cannot be hot-swapped. Remove the card only when the board is off. The current firmware cannot recover from SD mounting errors
without being power-cycled.
