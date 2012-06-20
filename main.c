#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MAX_SPEED (250)
#define STOP_SPEED (125)

void doCycle(unsigned char n) {
	PORTB |= (1 << PB3);
	_delay_us(1000);
	int i;
	for (i = 0; i < n; ++i) {
		_delay_us(4);
	}
	if (n != STOP_SPEED) {
		PORTB &= ~(1 << PB3);
	}
	for (i = 0; i < MAX_SPEED - n; ++i) {
		_delay_us(4);
	}
	_delay_ms(18);
}

unsigned char speed = 0;
unsigned char high = 0;

ISR(TIMER1_OVF_vect)
{
	++high;
}

ISR(INT0_vect)
{
	if (MCUCR != 0x03) {
		// turn the debug light off
	//	PORTB &= ~(1 << PB4);
		MCUCR=0x03;
		unsigned char low = TCNT1;
		unsigned short v = ((high << 8) | low);
		if (v > MAX_SPEED + MAX_SPEED) {
			speed = MAX_SPEED;
		} else if (v < MAX_SPEED) {
			speed = 0;
		} else {
			speed = v - MAX_SPEED;
		}
		if (speed > 115 && speed < 135) {
	//		PORTB |= (1 << PB4);
			speed = STOP_SPEED;
		}
	} else {
		MCUCR=0x02;
		high = 0;
    		TCNT1 = 0;
	}
}

unsigned char rot_high = 0;

ISR(TIMER0_OVF_vect)
{
	++rot_high;
	if (rot_high > 10) {
		PORTB &= ~(1 << PB4);
	}
}
unsigned char rotation_count = 0;

ISR(ANA_COMP_vect)
{
	unsigned short t = (rot_high << 8) + TCNT0;
	PORTB &= ~(1 << PB4);
	if (t < 500) {
		PORTB |= (1 << PB4);
	}
	rot_high = 0;
	TCNT0 = 0;
}

/**
 * For the tiny85, PB2 (pin 7) is the RX input, PB3 (pin 2) is the servo output, and PB1 (pin 6, AIN1) is reserved for the sensor input
 * PB4 (pin 3) is the diagnostic LED output
 */
int main(void)
{
// set the clock prescaler to 1 to get us an 8MHz clock
    CLKPR = 0x80;
    CLKPR = 0x00;
    DDRB = (1 << DDB4) | (1 << DDB3);
    PORTB = 0x00;
    ACSR = (1 << ACBG) | (1 << ACIE);
    MCUCR = 0x03; // look for rising edge on INT0
    GIMSK = (1 << INT0);
    TIMSK = (1 << TOIE1) | (1 << TOIE0); // enable counter 1 and 0  overflow interrupts
    PCMSK = 0x00; // will be 0x01 when we have sensor input?
    TCCR1 = 0x06; // CK/32
    TCCR0B = 0x03; // CK/64
    sei();
    /*for (;;) {
	_delay_ms(500);
	PORTB ^= 0x28;
    }*/
    for(;;){
	unsigned char i;
	for (i = 0; i < 255; ++i) {
		doCycle(speed);
	}
    }
    return 0;   /* never reached */
}

