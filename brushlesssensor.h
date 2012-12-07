typedef struct _sensor {
	uint8_t port;
	uint8_t pin;
	uint16_t revTimeStart;
	uint8_t revTimerOverflowCount;
	uint8_t rotation_count;
	uint16_t start_time;
	uint16_t end_time;
	uint16_t actual_revs_history[AVERAGE_COUNT];
	uint8_t average_index;
} SENSOR;


void setupSensors(SENSOR sensors[]);
void processSensors();

