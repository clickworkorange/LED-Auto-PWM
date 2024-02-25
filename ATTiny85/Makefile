MCU     = attiny85
F_CPU   = 8000000
CC      = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS  = -Wall -Wno-multichar -Wno-overflow -Wno-unknown-pragmas -Os -mmcu=${MCU} -DF_CPU=${F_CPU} # -Iinclude 
TARGET  = led_auto_pwm
INCLUDE = 
SRCS    = led_auto_pwm.c ${INCLUDE}

all: bin hex

bin:
	${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}

hex:
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex

elf:${TARGET}.o
	${CC} ${CFLAGS} ${INCLUDE} -o ${TARGET}.elf ${TARGET}.o

debug: CFLAGS += -g -O0
debug: all elf

flash:
	avrdude -p ${MCU} -P usb -c dragon_isp -U flash:w:${TARGET}.hex:i -F

clean:
	rm -f *.bin *.hex *.elf