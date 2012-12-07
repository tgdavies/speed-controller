#include <stdint.h>
#include "avrutils.h"

uint16_t timer_diff(uint16_t start, uint16_t end) {
	if (start > end) {
		return 0xffff - start + end;
	} else {
		return end - start;
	}
}