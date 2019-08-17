
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = atmelice_pdi

OUT=GPS_Voice_Clock

DISK_TYPE=sd
#DISK_TYPE=spi

CHIP = atxmega32e5

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude
DUDEOPTS = -B 0.1
OPTS = -Os -g -std=c11 -Wall -Wno-main -fno-tree-switch-conversion

CFLAGS = -mmcu=$(CHIP) $(OPTS)

all:	$(OUT).hex $(OUT).hex

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

GPS_Voice_Clock.elf: GPS_Voice_Clock.o pff.o diskio_$(DISK_TYPE).o

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f *.hex *.elf *.o

flash:	$(OUT).hex
	$(AVRDUDE) $(DUDEOPTS) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:$(OUT).hex

