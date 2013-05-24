#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "avrutils.h"
#include "brushlesssensor.h"

//#ifndef NO_OF_SENSORS
//	#error "'NO_OF_SENSORS' is not defined."
//#endif
#define ROTATIONS_PER_CALC	UINT16_C(1)


extern SENSOR all_sensors[NO_OF_SENSORS];
static uint8_t i, j, k;

void setupSensors() {
    //for (i = 0; i < NO_OF_SENSORS; ++i) {
    //	enablePCINT(all_sensors[i].port, all_sensors[i].pin); // this assumes that PCINTx = PORTx (and PCINT[8+x] = PORTx)
    //	green(1);
    //	GIMSK |= all_sensors[i].port == A ? (1 << PCIE0) : (1 << PCIE1);
    //}
    GIMSK |= (1 << PCIE0);
    PCMSK0 |= (1 << PA2) | (1 << PA3);
}

void processSensors() {
    for (i = 0; i < NO_OF_SENSORS; ++i) {
        SENSOR* sensor = &all_sensors[i];
        if (sensor->rev_timer_overflow_count > 1) {
            sensor->actual_revs_per_second = 0;
            sensor->sensor_start_time = 0;
            sensor->sensor_end_time = 0;
            sensor->rev_timer_overflow_count = 0;
        } else if (sensor->sensor_start_time != 0 && sensor->sensor_end_time != 0) {
            uint16_t actual_time_per_rev = timer_diff(sensor->sensor_start_time, sensor->sensor_end_time);
            if (actual_time_per_rev > 200) {
                actual_time_per_rev >>= 2;
#ifndef AVERAGE_COUNT
                sensor->actual_revs_per_second = (ROTATIONS_PER_CALC * UINT16_C(125000 / 4)) / actual_time_per_rev;
#else
                sensor->actual_revs_history[sensor->average_index++] = (ROTATIONS_PER_CALC * UINT16_C(125000 / 4)) / actual_time_per_rev;
                sensor->average_index &= 0x3;
                sensor->actual_revs_per_second = (sensor->actual_revs_history[0] + sensor->actual_revs_history[1] + sensor->actual_revs_history[2] + sensor->actual_revs_history[3]) >> 2;
#endif
                sensor->sensor_start_time = sensor->sensor_end_time = 0;
                sensor->no_result_count = 0;
            }

        } else {
            if (sensor->no_result_count++ > 20) {
                sensor->actual_revs_per_second = 0;
                sensor->no_result_count = 0;
            }
        }
    }
}

ISR(TIM1_OVF_vect) {
    for (j = 0; j < NO_OF_SENSORS; ++j) {
        ++(all_sensors[j].rev_timer_overflow_count);
    }
}

static SENSOR* sensor;
static uint16_t diff;
static uint8_t oldValue = 0;
static uint8_t newValue;

ISR(PCINT0_vect) {
    newValue = PINA;
    for (k = 0; k < NO_OF_SENSORS; ++k) {
        sensor = &all_sensors[k];
        if (sensor->port == A && ((oldValue ^ newValue) & (1 << sensor->pin)) && (newValue & (1 << sensor->pin))) {

            sensor->end_time = TCNT1L;
            sensor->end_time += TCNT1H << 8;
            if (sensor->start_time != 0) {

                diff = timer_diff(sensor->start_time, sensor->end_time);

                ++sensor->rotation_count;
                if (sensor->rotation_count == 6) { // we've done 1 revolutions, see how long it took in 1/125000ths of a second
                    sensor->rotation_count = 0;
                    sensor->sensor_end_time = sensor->end_time;
                    sensor->sensor_start_time = sensor->rev_time_start;
                    sensor->rev_time_start = sensor->sensor_end_time;
                    sensor->rev_timer_overflow_count = 0;
                }
            }
            sensor->start_time = sensor->end_time;
        }
    }
    oldValue = newValue;
}