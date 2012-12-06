#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "avrutils.h"
#include "motor.h"

#ifndef NO_OF_MOTORS
	#error "'NO_OF_MOTORS' is not defined."
#endif

uint8_t motor_i = 0;
ESC* all_escs;

void setCompare(uint16_t time) {
	uint16_t start_time;
	readCNT1(start_time);
	uint16_t end_time = start_time + time;
	OCR1AH = end_time >> 8;
	OCR1AL = end_time;
    TIMSK1 |= (1 << OCIE1A);
}


uint8_t indexes[NO_OF_MOTORS];

extern void sort();

void serviceEscs(ESC escs[]) {
	all_escs = escs;
	sort();
	// assume MAX_MOTORS and count are both 2
	/*if (escs[0].drive > escs[1].drive) {
		indexes[0] = 1; indexes[1] = 0;
	} else if (escs[1].drive >= escs[0].drive) {
		indexes[1] = 1; indexes[0] = 0;
	}*/
	motor_i = 0;
	for (uint8_t i = 0; i < NO_OF_MOTORS; ++i) {
			setBit(escs[i].port, escs[i].pin);
	}
	setCompare((1000/8) + (escs[indexes[0]].drive >> 1));
	_delay_ms(20);
}



ISR(TIM1_COMPA_vect) 
{
	ESC* esc = &all_escs[indexes[motor_i++]];
	clearBit(esc->port, esc->pin);
	while (motor_i < NO_OF_MOTORS) {
		ESC* esc2 = &all_escs[indexes[motor_i]];
		uint8_t diff = esc2->drive - esc->drive;
		if (diff == 0) {
			// turn this one off too and check the next
			clearBit(esc2->port, esc2->pin);
			esc = esc2;
			++motor_i;
		} else {
			setCompare(diff >> 1);
			break;
		}
	}
	if (motor_i == NO_OF_MOTORS) {
		TIMSK1 &= ~(1 << OCIE1A);
	}
}

