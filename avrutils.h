// defines for each port
#define A	(1)
#define B	(2)

#define readCNT1(X)         X = TCNT1L; \
        					X += TCNT1H << 8;
        					
#define setBit(port, bit) if (port == A) { PORTA |= (1 << bit); } else { PORTB |= (1 << bit); } 
#define clearBit(port, bit) if (port == A) { PORTA &= ~(1 << bit); } else { PORTB &= ~(1 << bit); } 
#define testBit(var, port, bit) if (port == A) { var = PINA & (1 << bit); } else { var = PINB & (1 << bit); } 

#define setDDRBit(port, bit) if (port == A) { DDRA |= (1 << bit); } else { DDRB |= (1 << bit); } 
#define clearDDRBit(port, bit) if (port == A) { DDRA &= ~(1 << bit); } else { DDRB &= ~(1 << bit); }

#define enablePCINT(port, bit) if (port == A) { PCMSK0 |= (1 << bit); } else { PCMSK1 |= (1 << bit); } 

#define LED1_DD (DDA1)
#define LED1_P (PA1)
#define LED2_DD (DDA0)
#define LED2_P (PA0)

#define red(on) if (on) { \
		PORTA &= ~(1 << LED1_P); \
	} else { \
		PORTA |= (1 << LED1_P); \
	}

#define green(on) if (on) { \
		PORTA &= ~(1 << LED2_P); \
	} else { \
		PORTA |= (1 << LED2_P); \
	}
	
#define timer_diff(start, end) (start > end) ? (0xffff - start + end) : (end - start)