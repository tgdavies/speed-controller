#include <stdio.h>
#include "morse.h"

int main(int argc, char** argv) {
	uint8_t status = 0;
	uint8_t on = 0;
	while (status != FINISHED) {
		status = morse(0x68);
		if (status == ON) {
			on = 1;
		} else if (status ==  OFF) {
			on = 0;
		}
		if (status != FINISHED) {
			if (on) {
				printf("*");
			} else {
				printf(" ");
			}
		}
	}
}
