#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "USI_TWI_Master.h"

#define INPUT_DELAY 250

#define BCDMASK (1<<PD0 | 1<<PD1 | 1<<PD2 | 1<<PD3)
#define MULTIMASK (1<<PB0 | 1<<PB1 | 1<<PB2 | 1<<PB3)

#define PCF8583_WRITE_ADDRESS ( 0xA0 & ~(0x01) )
#define PCF8583_READ_ADDRESS  ( PCF8583_WRITE_ADDRESS | 0x01 )
/* used to fetch clock data from I2C
 * PCF8583 memory layout:
 * [ 1/10 seconds | 1/100 seconds ] 0x1
 * [   10 seconds |     1 seconds ] 0x2
 * [   10 minutes |     1 minutes ] 0x3
 * [   10 hours   |     1 hours   ] 0x4
 */
static uint8_t buffer_i2c[6] = {0};

static struct {
	int8_t h;
	int8_t m;
	int8_t s;
} clock = {
	13,
	37,
	0
};

static volatile int8_t rotary_input = 0;

static enum t_mode {
	M_CLOCK = 0,
	M_SETHOUR,
	M_SETMINUTE,
	M_MAX
} mode;

static void get_clock(void) {
	buffer_i2c[0] = PCF8583_WRITE_ADDRESS;
	buffer_i2c[1] = 0x02; // start of time data
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 2);
	buffer_i2c[0] = PCF8583_READ_ADDRESS;
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 4);
	clock.s = (buffer_i2c[1] & 0x0F) + (buffer_i2c[1] >> 4)*10;
	clock.m = ((buffer_i2c[2] & 0x0F) + (buffer_i2c[2] >> 4)*10);
	clock.h = ((buffer_i2c[3] & 0x0F) + (buffer_i2c[3] >> 4)*10);
}

static void set_clock(void) {
        buffer_i2c[0] = PCF8583_WRITE_ADDRESS;
	buffer_i2c[1] = 0x01; // start of time data
	buffer_i2c[2] = 0; /* set 1/100 seconds to 0 */
	buffer_i2c[3] = ( ((clock.s/10)<<4) | (clock.s%10) );
	buffer_i2c[4] = ( ((clock.m/10)<<4) | (clock.m%10) );
	buffer_i2c[5] = ( ((clock.h/10)<<4) | (clock.h%10) );
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 6);
}

int main(void) {
	/* initialize BCD pins and decimal dot */
	DDRD = ( BCDMASK | 1<<PD5 );
	/* initialize multiplexing pins */
	DDRB = MULTIMASK;

	/* initialize input pins */
	PORTA = (1<<PA0 | 1<<PA1); // pullups

	/* rotary encoder */
	PORTD |= (1<<PD6 | 1<<PD4); // pullups

	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS01);
	OCR0A = 0xFA;

	TIMSK = (1<<OCIE0A);

	USI_TWI_Master_Initialise();

	sei();

	while (1) {
		if (~PINA & 1<<PA0) {
			mode = ((M_MAX+mode-1) % M_MAX);
			_delay_ms(INPUT_DELAY);
		}
		if (~PINA & 1<<PA1) {
			mode = ((M_MAX+mode+1) % M_MAX);
			_delay_ms(INPUT_DELAY);
		}
		if (rotary_input) {
			switch (mode) {
				case M_SETHOUR:
					clock.h = (clock.h+rotary_input)%24;
					if (clock.h < 0) clock.h += 24;
					break;
				case M_SETMINUTE:
					clock.m = (clock.m+rotary_input)%60;
					if (clock.m < 0) clock.m += 60;
					clock.s = 0; /* reset seconds */
					break;
				default:
					/* nothing to do */
					break;
			}
			rotary_input = 0;
			set_clock();
		}
		get_clock();
	}
}

static void set_bcd(uint8_t i) {
	PORTD &= ~BCDMASK;
	PORTD |= (i & BCDMASK);
}

static void display_tube(uint8_t n) {
	PORTB |= MULTIMASK;
	uint8_t val = 10;
	PORTD &= ~(1<<PD5);
	switch (n) {
		case 3:
			if (mode == M_CLOCK || mode == M_SETHOUR) {
				val = (clock.h / 10);
			}
			break;
		case 2:
			if (mode == M_CLOCK || mode == M_SETHOUR) {
				val = clock.h % 10;
			}
			break;
		case 1:
			if (clock.s%2) {
				PORTD |= (1<<PD5);
			}
			if (mode == M_CLOCK || mode == M_SETMINUTE) {
				val = clock.m / 10;
			}
			break;
		case 0:
			if (mode == M_CLOCK || mode == M_SETMINUTE) {
				val = clock.m % 10;
			}
			break;
		default:
			val = 10; /* disabled */
	}
	set_bcd( val );
	PORTB &= ~(1<<n);
}

static int8_t get_rotary_steps(uint8_t a, uint8_t oldA, uint8_t b, uint8_t oldB) {
	if (oldA && !a && !b && !oldB) return 1;
	if (!oldA && a && !b && !oldB) return -1;
	return 0;
}

static void check_rotary(void) {
	static uint8_t oldA = 0;
	static uint8_t oldB = 0;
	uint8_t a = !!(~PIND & 1<<PD4);
	uint8_t b = !!(~PIND & 1<<PD6);
	rotary_input += get_rotary_steps(a, oldA, b, oldB);
	oldA = a;
	oldB = b;
}

ISR(TIMER0_COMPA_vect) {
	static uint8_t active_tube = 0;
	display_tube(active_tube);
	active_tube++;
	active_tube %= 4;
	check_rotary();
}
