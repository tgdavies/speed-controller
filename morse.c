#include <stdint.h>
#include "morse.h"

uint8_t bit_no = 0;
uint8_t count = 0;

/**
	Call this method each unit time.
	It returns ON, OFF, UNCHANGED, FINISHED
 */
uint8_t morse(uint8_t current_char) {
	uint8_t length = current_char >> 5;
	if (bit_no < length) {
		// send a bit
		uint8_t bit = current_char & (1 << (4 - bit_no));
		if (count == 0) {
                        ++count;
			return ON;
		} else if ((bit && (count == 3)) || ((!bit) && (count == 1))) {
                        count = 0;
                        ++bit_no;
			return OFF;
		} /*else if ((bit && (count == 4)) || ((!bit) && (count == 2))) {
			count = 0;
			return UNCHANGED;
		}*/
	} else {
		if (count == 3) {
			bit_no = 0;
			count = 0;
			return FINISHED;
		}
	}
	++count;
	return UNCHANGED;
}