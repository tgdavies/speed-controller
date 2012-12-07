#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "avrutils.h"
#include "brushlesssensor.h"

#ifndef NO_OF_SENSORS
	#error "'NO_OF_SENSORS' is not defined."
#endif


static SENSOR all_sensors[NO_OF_SENSORS];

void setupSensors(SENSOR sensors[]) {
	all_sensors = sensors;
	for (uint8_t i = 0; i < NO_OF_SENSORS; ++i) {
		setPCINT(all_sensors[i].port, all_sensors[i].pin); // this assumes that PCINTx = PORTx (and PCINT[8+x] = PORTx)
		if (all_sensors[i].port == A) {
		GIMSK |= all_sensors[i].port == A ? (1 << PCIE0) : (1 << PCIE1);
	}
}

void processSensors() {
	for (uint8_t i = 0; i < NO_OF_SENSORS; ++i) {
		SENSOR* sensor = &all_sensors[i];
		if (sensor->rev_timer_overflow_count > 1) {
			sensor->actual_revs_per_second = 0;
			sensor->sensor_start_time = 0;
			sensor->sensor_end_time = 0;
		} else if (sensor->sensor_start_time != 0 && sensor->sensor_end_time != 0) {
			uint16_t actual_time_per_rev = timer_diff(sensor->sensor_start_time, sensor->sensor_end_time);
			if (actual_time_per_rev > 200) {
				//green(1);
				actual_time_per_rev >>= 2;
				//actual_revs_per_second = (ROTATIONS_PER_CALC * UINT16_C(125000/4)) / actual_time_per_rev;
				actual_revs_history[average_index++] = (ROTATIONS_PER_CALC * UINT16_C(125000/4)) / actual_time_per_rev;
				average_index &= 0x3;
				actual_revs_per_second = (actual_revs_history[0] + actual_revs_history[1] + actual_revs_history[2] + actual_revs_history[3]) >> 2;
				//green(0);
				sensor_start_time = sensor_end_time = 0;
			}
		}
	}
}

#define ROTATIONS_PER_CALC	UINT16_C(1)
ISR(PCINT0_vect)
{
	static uint8_t oldValue = 0;
	uint8_t newValue = PINA;
	if (((oldValue ^ newValue) & (1 << SENSOR_P)) && (newValue & (1 << SENSOR_P))) {
		static uint8_t rotation_count = 0;
		static uint16_t start_time = 0;
		static uint16_t end_time = 0;
		end_time = TCNT1L;
		end_time += TCNT1H << 8;
		if (start_time != 0) {
			uint16_t diff = timer_diff(start_time, end_time);
			if (diff > 50) {
				//red((counter++) & 0x01);
				++rotation_count;
				if (rotation_count == 6) { // we've done 1 revolutions, see how long it took in 1/125000ths of a second
					rotation_count = 0;
					sensor_end_time = end_time;
					sensor_start_time = rev_time_start;
					rev_time_start = sensor_end_time;
					rev_timer_overflow_count = 0;
				}   
			}
		}
		start_time = end_time;
	}
	oldValue = newValue;
}
#define AVERAGE_COUNT	(4)
uint16_t actual_revs_history[AVERAGE_COUNT];
uint8_t average_index = 0;

void process_sensor_result() {
	static uint8_t counter = 0;
	if (rev_timer_overflow_count > 1) {
		actual_revs_per_second = 0;
		sensor_start_time = 0;
		sensor_end_time = 0;
	} else if (sensor_start_time != 0 && sensor_end_time != 0) {
		uint16_t actual_time_per_rev = timer_diff(sensor_start_time, sensor_end_time);
		if (actual_time_per_rev > 200) {
		    //green(1);
			actual_time_per_rev >>= 2;
			//actual_revs_per_second = (ROTATIONS_PER_CALC * UINT16_C(125000/4)) / actual_time_per_rev;
			actual_revs_history[average_index++] = (ROTATIONS_PER_CALC * UINT16_C(125000/4)) / actual_time_per_rev;
			average_index &= 0x3;
			actual_revs_per_second = (actual_revs_history[0] + actual_revs_history[1] + actual_revs_history[2] + actual_revs_history[3]) >> 2;
			//green(0);
			sensor_start_time = sensor_end_time = 0;
		}
	}
}
