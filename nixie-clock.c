#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

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
 * [          day/year            ] 0x5
 * [            month             ] 0x6
 */
static uint8_t buffer_i2c[9] = {0};

#define YEAR_ZERO 2000

static struct {
	int16_t year;
	int8_t month;
	int8_t day;
	int8_t h;
	int8_t m;
	int8_t s;
} clock = {
	2012,
	1,
	1,
	13,
	37,
	0
};

static volatile int8_t rotary_input = 0;

static enum t_mode {
	M_CLOCK = 0,
	M_DATE,
	M_YEAR,
	M_SETHOUR,
	M_SETMINUTE,
	M_SETDAY,
	M_SETMONTH,
	M_SETYEAR,
	M_MAX
} mode;

static void get_clock(void) {
	memset(buffer_i2c, 0, sizeof(buffer_i2c));
	buffer_i2c[0] = PCF8583_WRITE_ADDRESS;
	buffer_i2c[1] = 0x02; // start of time data
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 2);
	buffer_i2c[0] = PCF8583_READ_ADDRESS;
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 6);
	clock.s = (buffer_i2c[1] & 0x0F) + (buffer_i2c[1] >> 4)*10;
	clock.m = ((buffer_i2c[2] & 0x0F) + (buffer_i2c[2] >> 4)*10);
	clock.h = ((buffer_i2c[3] & 0x0F) + (buffer_i2c[3] >> 4)*10);

	uint8_t year = (buffer_i2c[4]>>6); // only four years, we save a base value somewhere else
	clock.day = ((buffer_i2c[4] & 0x0F) + ((buffer_i2c[4] & 0x3F) >> 4)*10);
	clock.month = ((buffer_i2c[5] & 0x0F) + ((buffer_i2c[5] & 0x1F) >> 4)*10);

	int16_t year_base = YEAR_ZERO;
	/* get the base year */
	buffer_i2c[0] = PCF8583_WRITE_ADDRESS;
	buffer_i2c[1] = 0x10; // where the custom data goes
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 2);
	buffer_i2c[0] = PCF8583_READ_ADDRESS;
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 3);
	year_base += (int8_t)buffer_i2c[1];

	clock.year = year + year_base;
}

static void set_clock(void) {
	memset(buffer_i2c, 0, sizeof(buffer_i2c));
        buffer_i2c[0] = PCF8583_WRITE_ADDRESS;
	buffer_i2c[1] = 0x01; // start of time data
	buffer_i2c[2] = 0; /* set 1/100 seconds to 0 */
	buffer_i2c[3] = ( ((clock.s/10)<<4) | (clock.s%10) );
	buffer_i2c[4] = ( ((clock.m/10)<<4) | (clock.m%10) );
	buffer_i2c[5] = ( ((clock.h/10)<<4) | (clock.h%10) );

	buffer_i2c[6] = ( ((clock.day/10)<<4) | (clock.day%10) ) | ((clock.year%4)<<6);
	buffer_i2c[7] = ( ((clock.month/10)<<4) | (clock.month%10) );
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 8);

	/* save the base year */
	int8_t year_base = clock.year - (clock.year%4) - YEAR_ZERO;
	buffer_i2c[1] = 0x10;
	buffer_i2c[2] = year_base;
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 4);
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
		get_clock();
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
					set_clock();
					break;
				case M_SETMINUTE:
					clock.m = (clock.m+rotary_input)%60;
					if (clock.m < 0) clock.m += 60;
					clock.s = 0; /* reset seconds */
					set_clock();
					break;
				case M_SETMONTH:
					clock.month = ((clock.month-1+rotary_input)%12)+1;
					if (clock.month <= 0) clock.month += 12;
					set_clock();
					break;
				case M_SETDAY:
					clock.day = ((clock.day-1+rotary_input)%31)+1;
					if (clock.day <= 0) clock.day += 31;
					set_clock();
					break;
				case M_SETYEAR:
					clock.year = clock.year+rotary_input;
					set_clock();
					break;
				default:
					/* nothing to do */
					break;
			}
			rotary_input = 0;
		}
	}
}

static void set_bcd(uint8_t i) {
	PORTD &= ~BCDMASK;
	PORTD |= (i & BCDMASK);
}

static void display_tube(uint8_t n) {
	PORTB |= MULTIMASK;
	uint8_t val = 10;
	uint8_t blink = (clock.s%2 == 0);
	PORTD &= ~(1<<PD5);
	switch (n) {
		case 3:
			if (mode == M_CLOCK || mode == M_SETMINUTE || (mode == M_SETHOUR && blink)) {
				val = (clock.h / 10);
			} else if (mode == M_DATE || mode == M_SETMONTH || (mode == M_SETDAY && blink)) {
				val = (clock.day / 10);
			} else if (mode == M_YEAR || (mode == M_SETYEAR && blink)) {
				val = (clock.year / 1000)%10;
			}
			break;
		case 2:
			if (mode == M_CLOCK || mode == M_SETMINUTE || (mode == M_SETHOUR && blink)) {
				val = clock.h % 10;
			} else if (mode == M_DATE || mode == M_SETMONTH || (mode == M_SETDAY && blink)) {
				val = (clock.day % 10);
			} else if (mode == M_YEAR || (mode == M_SETYEAR && blink)) {
				val = (clock.year / 100)%10;
			}
			break;
		case 1:
			if (
				((mode == M_CLOCK || mode == M_SETHOUR || mode == M_SETMINUTE) && clock.s%2) ||
				mode == M_DATE || mode == M_SETDAY || mode == M_SETMONTH
			) {
				PORTD |= (1<<PD5);
			}
			if (mode == M_CLOCK || mode == M_SETHOUR || (mode == M_SETMINUTE && blink)) {
				val = clock.m / 10;
			} else if (mode == M_DATE || mode == M_SETDAY || (mode == M_SETMONTH && blink)) {
				val = clock.month / 10;
			} else if (mode == M_YEAR || (mode == M_SETYEAR && blink)) {
				val = (clock.year / 10)%10;
			}
			break;
		case 0:
			if (mode == M_CLOCK || mode == M_SETHOUR || (mode == M_SETMINUTE && blink)) {
				val = clock.m % 10;
			} else if (mode == M_DATE || mode == M_SETDAY || (mode == M_SETMONTH && blink)) {
				val = clock.month % 10;
			} else if (mode == M_YEAR || (mode == M_SETYEAR && blink)) {
				val = clock.year % 10;
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
