#include <stdio.h>
#include <stdint.h>
#include "motor.h"

uint8_t indexes[NO_OF_MOTORS];
ESC* all_escs;
extern void sort();

int main(int argc, char** argv) {
	ESC escs[NO_OF_MOTORS] = {
		{B, 1, 4},
		{B, 1, 1},
		{B, 1, 3},
		{B, 1, 2},
	};
	all_escs = escs;
	printf("testing sort...\n");
	sort();
	for (int i = 0; i < NO_OF_MOTORS; ++i) {
		int index = indexes[i];
		printf("%d: drive = %d\n", i, escs[index].drive);
		if (i+1 !=  escs[index].drive) {
			printf("FAIL!\n");
		}
	}
}
