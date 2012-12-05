#define MAX_MOTORS	(4)
#define AVERAGE_COUNT (4)

#define MAX_SPEED INT16_C(240)
#define STOP_SPEED INT16_C(120)

#define A	(1)
#define B	(2)

typedef struct _motor {
	uint8_t sensePort;
	uint8_t sensePortPin;
	uint16_t revTimeStart;
	uint8_t revTimerOverflowCount;
	uint8_t rotation_count;
	uint16_t start_time;
	uint16_t end_time;
	uint16_t actual_revs_history[AVERAGE_COUNT];
	uint8_t average_index;
} MOTOR;

typedef struct _esc {
	uint8_t port;
	uint8_t pin;
	uint8_t drive;
} ESC;

#define setBit(port, bit) if (port == A) { PORTA |= (1 << bit); } else { PORTB |= (1 << bit); } 
#define clearBit(port, bit) if (port == A) { PORTA &= ~(1 << bit); } else { PORTB &= ~(1 << bit); } 
#define testBit(var, port, bit) if (port == A) { var = PORTA & (1 << bit); } else { var = PORTB & (1 << bit); } 

void serviceEscs(ESC escs[], uint8_t count);

