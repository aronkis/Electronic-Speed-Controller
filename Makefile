CC=avr-gcc
OBJCOPY=avr-objcopy
FLASHER=avrdude

CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega328p
INCLUDES=source/*.c

PORT=/dev/ttyUSB1
PARTNO=atmega328p
BAUD=57600
PROGRAMMER=arduino

all: build

build: compile
	${OBJCOPY} main.elf -O ihex main.hex

compile:
	${CC} main.c ${INCLUDES} -o main.elf ${CFLAGS}

ison:
	${FLASHER} -c ${PROGRAMMER} -p ${PARTNO} -P ${PORT} -b ${BAUD}

flash: build 
	${FLASHER} -c ${PROGRAMMER} -p ${PARTNO} -P ${PORT} -b ${BAUD} -U flash:w:main.hex:a
	@make clean

clean:
	@rm -rf *.hex *.elf