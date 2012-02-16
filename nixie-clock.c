#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define INPUT_DELAY 250

#define BCDMASK (1<<PD0 | 1<<PD1 | 1<<PD2 | 1<<PD3)
#define MULTIMASK (1<<PB0 | 1<<PB1 | 1<<PB2 | 1<<PB3)

static struct {
	uint8_t h;
	uint8_t m;
	uint8_t s;
} clock = {
	13,
	37,
	0
};

int main(void) {
	/* initialize BCD pins */
	DDRD = BCDMASK;
	/* initialize multiplexing pins */
	DDRB = MULTIMASK;
	/* initialize input pins */
	PORTA = (1<<PA0 | 1<<PA1); // pullups

	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS01);
	OCR0A = 0xFA;

	TIMSK = (1<<OCIE0A);
	sei();

	while (1) {
		if (~PINA & 1<<PA0) {
			clock.h = (clock.h + 1)%24;
			_delay_ms(INPUT_DELAY);
		}
		if (~PINA & 1<<PA1) {
			clock.m = (clock.m + 1)%60;
			_delay_ms(INPUT_DELAY);
		}
	}
}

static void set_bcd(uint8_t i) {
	PORTD &= ~BCDMASK;
	switch(i) {
		case 0:
			break;
		case 1:
			PORTD |= 1<<PD0;
			break;
		case 2:
			PORTD |= 1<<PD1;
			break;
		case 3:
			PORTD |= (1<<PD0 | 1<<PD1);
			break;
		case 4:
			PORTD |= (1<<PD2);
			break;
		case 5:
			PORTD |= (1<<PD0 | 1<<PD2);
			break;
		case 6:
			PORTD |= (0<<PD0 | 1<<PD1 | 1<<PD2 | 0<<PD3);
			break;
		case 7:
			PORTD |= (1<<PD0 | 1<<PD1 | 1<<PD2 | 0<<PD3);
			break;
		case 8:
			PORTD |= (0<<PD0 | 0<<PD1 | 0<<PD2 | 1<<PD3);
			break;
		case 9:
			PORTD |= (1<<PD0 | 0<<PD1 | 0<<PD2 | 1<<PD3);
			break;
		default:
			PORTD |= (1<<PD0 | 1<<PD1 | 1<<PD2 | 1<<PD3);
	}
}

static void display_tube(uint8_t n) {
	PORTB |= MULTIMASK;
	uint8_t val = 10;
	switch (n) {
		case 3:
			val = (clock.h / 10);
			break;
		case 2:
			val = clock.h % 10;
			break;
		case 1:
			val = clock.m / 10;
			break;
		case 0:
			val = clock.m % 10;
	}
	set_bcd( val );
	PORTB &= ~(1<<n);
}

ISR(TIMER0_COMPA_vect) {
	static uint8_t active_tube = 0;
	display_tube(active_tube);
	active_tube++;
	active_tube %= 4;
}
