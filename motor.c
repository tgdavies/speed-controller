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

static void setCompare(uint16_t time) {
	uint16_t start_time;
	readCNT1(start_time);
	uint16_t end_time = start_time + time;
	OCR1AH = end_time >> 8;
	OCR1AL = end_time;
    TIMSK1 |= (1 << OCIE1A);
}


uint8_t indexes[NO_OF_MOTORS];

extern void sort();

void setupEscs(ESC escs[]) {
	all_escs = escs;
	for (uint8_t i = 0; i < NO_OF_MOTORS; ++i) {
		setDDRBit(all_escs[i].port, all_escs[i].pin);
	}
}


static void two_second_constant(int drive_value) {
	for (uint8_t i = 0; i < NO_OF_MOTORS; ++i) {
		all_escs[i].drive = drive_value;
	}
	for (int i = 0; i < 100; ++i) {
        serviceEscs(all_escs);
    }
}

void calibrateEscs() {
	two_second_constant(MAX_SPEED);
    two_second_constant(MAX_SPEED);
    two_second_constant(0);
    two_second_constant(0);
    two_second_constant(MAX_SPEED/2);
    two_second_constant(MAX_SPEED/2);
    two_second_constant(MAX_SPEED/2);
    two_second_constant(MAX_SPEED/2);
}

void serviceEscs() {
	sort();
	// assume MAX_MOTORS and count are both 2
	/*if (escs[0].drive > escs[1].drive) {
		indexes[0] = 1; indexes[1] = 0;
	} else if (escs[1].drive >= escs[0].drive) {
		indexes[1] = 1; indexes[0] = 0;
	}*/
	motor_i = 0;
	for (uint8_t i = 0; i < NO_OF_MOTORS; ++i) {
			setBit(all_escs[i].port, all_escs[i].pin);
	}
	setCompare((1000/8) + (all_escs[indexes[0]].drive >> 1));
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

