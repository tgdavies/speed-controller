#include <avr/io.h>
#include <stdint.h>
#include "avrutils.h"
#include "motor.h"
#include "brushlesssensor.h"
#include "pid.h"

extern ESC all_escs[NO_OF_MOTORS];
extern SENSOR all_sensors[NO_OF_MOTORS];
extern PID all_pids[NO_OF_MOTORS];
extern uint16_t desired_revs_per_second;
extern uint8_t desired_direction;

static uint8_t i;
static PID* pid;
static uint8_t previous_desired_direction;
static int16_t diff;
static int16_t total;

void processPids() {
    ++i;

    if (i > 10) { // do this every 200ms
        for (i = 0; i < NO_OF_MOTORS; ++i) { //NB we are using i for two things
            pid = &all_pids[i];
            if (desired_direction != previous_desired_direction) {
                pid->integral = 0;
            }

            diff = desired_revs_per_second - all_sensors[i].actual_revs_per_second;
            if (diff < 0) diff = -diff;

            if (diff > 5) { // don't react to noise
                diff = (diff >> 3) + 1;
                if (desired_revs_per_second < all_sensors[i].actual_revs_per_second) {
                    pid->integral -= diff;
                } else if (desired_revs_per_second > all_sensors[i].actual_revs_per_second) {
                    pid->integral += diff;
                }
            }

            total = desired_revs_per_second * 10;
            total = (desired_revs_per_second / 3) + pid->integral;

            // restrict value to the allowed range
            if (total < 0) {
                total = 0;
            } else if (total > MAX_SPEED / 2) {
                total = MAX_SPEED / 2;
            }


            // convert to an ESC output value, depending on current direction
            if (desired_direction == AHEAD) {
                all_escs[i].drive = total + MAX_SPEED / 2;
            } else if (desired_direction == ASTERN) {
                all_escs[i].drive = MAX_SPEED / 2 - total;
            } else {
                all_escs[i].drive = STOP_SPEED;
            }
        }
        previous_desired_direction = desired_direction;
        i = 0;
    }
}