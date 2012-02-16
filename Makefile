MCU = attiny2313
F_CPU = 1000000
TARGET = nixie-clock
SRC = nixie-clock.c USI_TWI_Master.c

include avr-tmpl.mk

