#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void doCycle(unsigned char n) {
	PORTB |= 0x01;
	_delay_us(1000);
	int i;
	for (i = 0; i < n; ++i) {
		_delay_us(7);
	}
	PORTB &= 0xfe;
	for (i = 0; i < 150-n; ++i) {
		_delay_us(7);
	}
	_delay_ms(18);
}

unsigned char interrupt_state = 0;
unsigned char interrupt_count = 0;
unsigned char speed = 0;

ISR(INT0_vect)
{

	interrupt_state ^= 0xff;
	if (MCUCR != 0x03) {
		MCUCR=0x03;
		unsigned char low = TCNT1L;
		unsigned char high = TCNT1H;
		unsigned short v = ((TCNT1H << 8) | TCNT1L);
		if (v > 150 + 150) {
			speed = 150;
		} else if (v < 150) {
			speed = 0;
		} else {
			speed = v - 150;
		}
		if (speed > 75) {
			PORTB |= 0x02;
		} else {
			PORTB &= 0xfd;
		}
	} else {
		MCUCR=0x02;
		TCNT1H = 0;
    		TCNT1L = 0;
	}
}

int main(void)
{
    DDRB = 0x03;
    PORTB = 0x02;
    DDRD = 0x00;
    MCUCR = 0x03;
    GIMSK = 0x40;
    TCCR1B = 0x03; // CK/64
    sei();
    for(;;){
	unsigned char i;
	for (i = 0; i < 255; ++i) {
		doCycle(speed);
	}
    }
    return 0;   /* never reached */
}

