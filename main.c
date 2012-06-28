#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "pid.h"

#define UART_SOFT_DDR DDRB 
#define UART_SOFT_PORT PORTB 
#define UART_SOFT_PIN PB4
#define UART_SOFT_BAUD (4800)

#define UART_SOFT_DELAY_US (int)(1000000.0/((float) UART_SOFT_BAUD)+0.5) 


#define softUartInit()   UART_SOFT_DDR |= (1<<UART_SOFT_PIN); \
                  UART_SOFT_PORT |= (1<<UART_SOFT_PIN)
                  
#define DEBUG_CHARS_SIZE 9
                  
char debugChars[DEBUG_CHARS_SIZE];
uint8_t sendDebug = 0;

void softUartSendChar(char txData) { 
     //txData = ~txData; 
     UART_SOFT_PORT &= ~(1<<UART_SOFT_PIN); 
     _delay_us(UART_SOFT_DELAY_US); 
   for (char i=0; i<8; i++) 
     { 
       if( txData & 1 ) 
            UART_SOFT_PORT |= (1<<UART_SOFT_PIN); 
       else 
            UART_SOFT_PORT &= ~(1<<UART_SOFT_PIN); 
       txData >>= 1; 
      _delay_us(UART_SOFT_DELAY_US); 
    } 
   UART_SOFT_PORT |= (1<<UART_SOFT_PIN); 
   _delay_us(UART_SOFT_DELAY_US); 
}

char hexDigit(int n) {
	if (n < 0xA) {
		return '0' + n;
	} else {
		return 'A' - 0xA + n;
	}
}

void clearDebugChars() {
	for (int i = 0; i < DEBUG_CHARS_SIZE; ++i) {
		debugChars[i] = 0;
	}
}

uint8_t debugIndex(uint8_t sizeNeeded) {
	uint8_t i = 0;
	while (i < DEBUG_CHARS_SIZE && debugChars[i] != 0)
		++i;
	if (DEBUG_CHARS_SIZE - i >= sizeNeeded) {
		sendDebug = 1;
		return i;
	} else {
		return 0xff;
	}
}

void log8(uint8_t n) {
	uint8_t i = debugIndex(2);
	if (i != 0xff) {
		debugChars[i++] = hexDigit(n >> 4);
		debugChars[i] = hexDigit(n & 0x0F);
	}
}

void log16(uint16_t n) {
	uint8_t i = debugIndex(4);
	if (i != 0xff) {
		log8(n >> 8);
		log8(n & 0xFF);
	}
}

logSep() {
	uint8_t i = debugIndex(1);
	if (i != 0xff) {
		debugChars[i] = ',';
	}
}

logCRNL() {
	uint8_t i = debugIndex(2);
	if (i != 0xff) {
		debugChars[i++] = '\r';
		debugChars[i] = '\n';
	}
}


void softUartSendString (char *txData) 
{ 
   while(*txData != '\0') softUartSendChar(*txData++); 
} 

void debugOn() {
    //PORTB |= (1 << PB4);
}

void debugOff() {
    //PORTB &= ~(1 << PB4);
}

void debugToggle() {
    //PORTB ^= (1 << PB4);
}

/*! \brief P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
#define K_P     0.20
#define K_I     0.10
#define K_D     0.00

//! Parameters for regulator
struct PID_DATA pidData;

#define MAX_SPEED INT16_C(250)
#define STOP_SPEED INT16_C(125)
#define DEAD_BAND (5)

#define MAX_RPS INT16_C(100)
// controller input value after confining to 0..MAX_SPEED
int16_t speed = 0;

// time to drive the motor in multiples of 4us
uint16_t drive = STOP_SPEED;

uint16_t actual_revs_per_second = 0;
uint16_t desired_revs_per_second = 0;

#define AHEAD 1
#define ASTERN 2
uint8_t desired_direction = 0;

void doCycle() {
	/*if (drive != 0) {
		debugOn();
	} else {
		debugOff();
	}*/
    PORTB |= (1 << PB3);
    _delay_us(1020);
    uint8_t i;
    for (i = 0; i < drive; ++i) {
        _delay_us(4);
    }
    if (speed != STOP_SPEED) { // this is the processed controller input, not the PID value
        PORTB &= ~(1 << PB3);
    }
    for (i = 0; i < MAX_SPEED - drive; ++i) {
        _delay_us(4);
    }
    if (sendDebug) {
		for (i = 0; i < DEBUG_CHARS_SIZE; ++i) {
			softUartSendChar(debugChars[i]);
		}
		sendDebug = 0;
		clearDebugChars();
	} else {
	    _delay_ms(18);
	}
}


uint8_t high = 0;

ISR(TIMER1_OVF_vect)
{
    ++high;
}
uint16_t error = 0;

ISR(INT0_vect)
{
    if (MCUCR != 0x03) {
        //debugOff();

        MCUCR=0x03;
        uint8_t low = TCNT1;
        uint16_t v = ((high << 8) | low);
        if (v > MAX_SPEED + MAX_SPEED) {
            speed = MAX_SPEED;
        } else if (v < MAX_SPEED) {
            speed = 0;
        } else {
            speed = v - MAX_SPEED;
        }

        if (speed > STOP_SPEED-DEAD_BAND && speed < STOP_SPEED+DEAD_BAND) {
            speed = drive = STOP_SPEED;
            desired_revs_per_second = 0;
            desired_direction = 0;
            //debugOn();
        } else if (speed >= STOP_SPEED+DEAD_BAND) {
            desired_revs_per_second = ((speed - 130) * MAX_RPS) / (MAX_SPEED - 130);
            desired_direction = AHEAD;
        } else if (speed <= STOP_SPEED-DEAD_BAND) {
            desired_revs_per_second = ((120-speed) * MAX_RPS) / 120;
            desired_direction = ASTERN;
        }

        clearDebugChars();
        //log8(speed);
        //logSep();
        log8(desired_revs_per_second & 0xff);
        logSep();
        log8(actual_revs_per_second & 0xff);
        //log8(v & 0xff);
        //logSep();
        //log8(error & 0xff);
        logCRNL();
		if (desired_revs_per_second == 0) {
		}
    } else {
        MCUCR=0x02;
        high = 0;
        TCNT1 = 0;
    }
}

uint8_t rot_high = 0;

// this counts timer 0 overflows to time revolutions
ISR(TIMER0_OVF_vect)
{
    ++rot_high; 
}


#define ROTATIONS_PER_CALC	UINT16_C(5)
ISR(ANA_COMP_vect)
{
	static uint8_t rotation_count = 0;
    ++rotation_count;
	 if (rotation_count == ROTATIONS_PER_CALC * 2) { // we've done 5 revolutions, see how long it took in 1/7812.5ths of a second
        rotation_count = 0;
        if (speed == STOP_SPEED) {
			actual_revs_per_second = 0;
		} else {
			uint16_t actual_time_per_5_revs = (rot_high << 8) + TCNT0;
			actual_revs_per_second = (ROTATIONS_PER_CALC * UINT16_C(7812)) / actual_time_per_5_revs;
		}
		rot_high = 0;
		TCNT0 = 0;
    }   
}



/**
 * For the tiny85, PB2 (pin 7) is the RX input, PB3 (pin 2) is the servo output, and PB1 (pin 6, AIN1) is reserved for the sensor input
 * PB4 (pin 3) is the diagnostic LED output
 */
int main(void)
{
// set the clock prescaler to 1 to get us an 8MHz clock
    CLKPR = 0x80;
    CLKPR = 0x00;
    DDRB = (1 << DDB4) | (1 << DDB3); // outputs for servo and diagnostic LED
    PORTB = 0x00;
    ACSR = (1 << ACBG) | (1 << ACIE); // analog comparator interrupt on toggle
    MCUCR = 0x03; // look for rising edge on INT0
    GIMSK = (1 << INT0);
    TIMSK = (1 << TOIE1) | (1 << TOIE0); // enable counter 1 and 0  overflow interrupts
    TCCR1 = 0x06; // CK/32, i.e. 250KHz, or 976Hz overflow interrupt.
    TCCR0B = 0x05; // CK/1024 i.e. 7812.5 Hz or 30Hz overflow interrupt.
    
    pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);

    sei();
	uint8_t pid_cycle_counter = 0;
	uint8_t previous_desired_direction = AHEAD;
    for(;;){
            ++pid_cycle_counter;
			if (desired_direction == 0) {
				pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);
			}
			if (pid_cycle_counter > 10 && speed != STOP_SPEED) { // do this every 200ms
				debugOff();
				pid_cycle_counter = 0;
				if (desired_direction != previous_desired_direction) {
				    pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);
				    previous_desired_direction = desired_direction;
				}
				drive = pid_Controller(desired_revs_per_second, actual_revs_per_second, &pidData);
				
				if (drive > MAX_SPEED/2) {
					drive = MAX_SPEED/2;
				} else if (drive <= 0) {		
					drive = 0;
				}
				if (desired_direction == AHEAD) {
					drive = drive + MAX_SPEED/2;
				} else {
					drive = MAX_SPEED/2 - drive;
				}
				error = desired_revs_per_second - actual_revs_per_second;
				if (error < 0) {
					error = -error;
				}
				
			}
            doCycle();
    }
    return 0;   /* never reached */
}

