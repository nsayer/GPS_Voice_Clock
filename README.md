# GPS_Voice_Clock

This is the source code for https://hackaday.io/project/28949-gps-talking-clock

Note that this codebase includes Petit FAT FS. Petit FAT FS can be found at http://elm-chan.org/fsw/ff/00index_p.html

The code I've written, exclusive of Petit FAT FS (but *inclusive* of the portion of diskio.c that is the actual code
that reads from flash), is licensed under GPL V2. That does not apply to Petit, of course, which remains covered by what
is essentially an MIT license.

## Usage

Build the code with the included Makefile, which assumes you have an avr-gcc with XMega support in your path. Upload
it to the controller on the board with avrdude and a PDI capable programmer.

Note that there are two options for the audio storage: either SD card or SPI flash. You should select the option in
the Makefile that matches your hardware.

If your hardware uses SPI flash storage, then you should either erase the XMega processor or hold it in RESET while
updating the SPI flash chip. If you don't, then the processor may attempt to talk to the flash chip resulting in
data corruption.

DIP switch 3 selects whether to perform chimes or not. Switches 2 to 0 select the time zone. The available
time zone selections are governed by a configuration file (called CONFIG.TXT) in the top level of the filesystem. The config
file has 8 lines. Each line is a comma separated list of 3 fields. The first field is a single character representing the
first letter of the name of the zone file pair in the ZONE directory. The second field is the offset from UTC for standard
time in the zone. The third field is a string for the DST configuration for the zone. The available strings are "OFF", "US",
"EU", "AU" and "NZ". If a zone is configured for no DST then the file in the ZONE directory is just a single letter long - the
letter in the first field. Otherwise, the filename is two characters long, with the second being either "S" or "D" depending
on whether standard time or daylight savings time is in effect.

The button toggles the audio on and off. The two lights on the board indicate a GPS error or an SD card error, respectively. SD
card errors can either be failures to mount the card, config file format errors (which are fatal), or missing files (which,
generally, are not).

The flash storage should be formatted as a FAT12, FAT16 or FAT32 filesystem in an MBR partition. All of the directory and file names should be uppercase.

The filesystem has the following main directories defined: "ZONE", "HOUR", "MINUTE" and "SECOND".

The files in the "ZONE" directory are named with one or two letter names as specified in the configuration file. Each file is
excpected to be 4 seconds long at most. Each is intended to say something like "At the tone, Pacific Standard Time will be"

The files in the other three directories have names that are just numbers (no leading digit for single digit values). The numbers are
0-23 for HOUR, 0-59 for MINUTE and 0, 10, 20, 30, 40, 50 for second. Each should last no longer than 1 second and should just be the
number and units. As a special case, the 0 seconds file should simply say "Exactly."

The top level directory can also have any number of files called CHIMEn (or CHIMEnn) that will play at the indicated minute
past the hour (for every such file present). If CHIME0 is present, there may also be a file called STROKE that (if present)
is played once for every (AM/PM converted) hour immediately following the end of CHIME0. The filesystem image included in the
source tree has the files to perform Westminster Quarters.

The chimes can be independently turned on and off with dip switch 3. If the time messaging is turned on when it's time
to chime, the chime will be skipped.

The audio files are µ-law encoded 8 bit 8 kHz mono raw audio samples. You can create them from some other format with sox. The
command line arguments for the output should be -t raw -c 1 -e mu-law -b 8 -r 8000.

The SD_audio.zip file in this repository can be unpacked into the top directory of a suitably formatted image. If chimes are desired, then one of the chime zip files can be unzipped to the same place.

If present, the SD card cannot be hot-swapped. Remove the card only when the board is off. The current firmware cannot recover from SD mounting errors
without being power-cycled.
