//#define AVERAGE_COUNT	(4) doesn't seem to improve performance :-(

typedef struct _sensor {
	uint8_t port;
	uint8_t pin;
	uint16_t rev_time_start;
	uint8_t rev_timer_overflow_count;
	uint8_t rotation_count;
	uint16_t start_time;
	uint16_t end_time;
	uint16_t sensor_start_time;
	uint16_t sensor_end_time;
        uint8_t no_result_count;
#ifdef AVERAGE_COUNT
	uint16_t actual_revs_history[AVERAGE_COUNT];
	uint8_t average_index;
#endif
	uint16_t actual_revs_per_second;
} SENSOR;


void setupSensors();
void processSensors();

