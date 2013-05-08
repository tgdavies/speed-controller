
#define AHEAD 1
#define ASTERN 2

typedef struct _pid {
	int8_t integral;
} PID;

void processPids();