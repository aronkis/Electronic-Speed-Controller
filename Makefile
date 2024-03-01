CC=avr-gcc
OBJCOPY=avr-objcopy
FLASHER=avrdude

CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega328p
INCLUDES=src/*.c
BUILD_PATH=build
BUILD_NAME=main

PORT=/dev/ttyUSB0
PARTNO=atmega328p
BAUD=57600
PROGRAMMER=arduino

all: build

build: compile
	${OBJCOPY} ${BUILD_PATH}/${BUILD_NAME}.elf -O ihex ${BUILD_PATH}/${BUILD_NAME}.hex

compile:
	${CC} ${BUILD_NAME}.c ${INCLUDES} -o ${BUILD_PATH}/${BUILD_NAME}.elf ${CFLAGS}

ison:
	${FLASHER} -c ${PROGRAMMER} -p ${PARTNO} -P ${PORT} -b ${BAUD}

flash: build 
	${FLASHER} -c ${PROGRAMMER} -p ${PARTNO} -P ${PORT} -b ${BAUD} -U flash:w:${BUILD_PATH}/${BUILD_NAME}.hex:a
	@make clean

clean:
	@rm ${BUILD_PATH}/*.hex ${BUILD_PATH}/*.elf