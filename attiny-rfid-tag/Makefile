OBJECTS    = main.o
DEVICE     = attiny85
CLOCK      = 125000

PROGRAMMER = -c avrisp -P /dev/ttyACM0 -v -v -v -F -b 19200
#PROGRAMMER = -c usbtiny -v -F -b 19200
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -Iusbdrv -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
 
main.hex: Makefile main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

main.elf: Makefile $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

simulate: main.hex
	echo "avr-gdb main.elf -ex 'target remote :1234' -ex 'layout asm'"
	simavr -g -m attiny85 -f 8000000 -ff main.hex

size: main.elf
	avr-size --format=avr --mcu=$(DEVICE) main.elf

install: main.hex
	$(AVRDUDE) -U flash:w:main.hex:i
 
fuse:
	# fuse bit calculator: http://www.engbedded.com/fusecalc/
	# $(AVRDUDE) -U lfuse:w:0x62:m # original clock
	$(AVRDUDE) -U lfuse:w:0xc0:m # external clock for rfid emulation
 
.c.o:
	$(COMPILE) -c $< -o $@

clean:
	rm -f main.hex main.elf $(OBJECTS)
