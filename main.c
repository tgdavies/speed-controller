#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "avrutils.h"
#include "brushlesssensor.h"
#include "motor.h"
#include "pid.h"

#define RX_DD	(DDB2)
#define RX_P	(PB2) // Must be INT0 pin
#define MOTOR_DD (DDB0)
#define MOTOR2_DD (DDA7)
#define MOTOR_P (PB0)
#define MOTOR2_P (PA7)
#define SENSOR_DD (DDA2)
#define SENSOR2_DD (DDA3)
#define SENSOR_P (PA2)
#define SENSOR2_P (PA3) // does that work?
#define LED1_DD (DDA1)
#define LED1_P (PA1)
#define LED2_DD (DDA0)
#define LED2_P (PA0)

#define ISC0_RISE (0x03)
#define ISC0_FALL (0x02)

#define UART_SOFT_DDR DDRB 
#define UART_SOFT_PORT PORTB 
#define UART_SOFT_PIN PB1
#define UART_SOFT_BAUD (4800)

#define UART_SOFT_DELAY_US (int)(1000000.0/((float) UART_SOFT_BAUD)+0.5) 


#define softUartInit()   UART_SOFT_DDR |= (1<<UART_SOFT_PIN); \
                  UART_SOFT_PORT |= (1<<UART_SOFT_PIN)

#define DEBUG_CHARS_SIZE 9

#define readCNT1(X)         X = TCNT1L; \
        					X += TCNT1H << 8;

/*                  
char debugChars[DEBUG_CHARS_SIZE];
uint8_t sendDebug = 0;

void softUartSendChar(char txData) { 
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

void logSep() {
        uint8_t i = debugIndex(1);
        if (i != 0xff) {
                debugChars[i] = ',';
        }
}

void logCRNL() {
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
}*/


#define MAX_RPS INT16_C(50)

#define RX_COUNTS_PER_MS	125
#define RX_COUNT_STOP		62
#define RX_COUNT_DEAD_BAND	5

// controller input value after confining to 0..MAX_SPEED
volatile int16_t speed = 0;

// time to drive the motor in multiples of 4us
//volatile uint8_t drive = STOP_SPEED;

// really revs per second / 10
//volatile uint16_t actual_revs_per_second = 0;
volatile uint16_t desired_revs_per_second = 0;

#define AHEAD 1
#define ASTERN 2
volatile uint8_t desired_direction = 0;

#define BIT_DELAY_US(d,n) if (d & n) { _delay_us(n); }

void delay_us(uint16_t delay) {
    BIT_DELAY_US(delay, 0x200);
    BIT_DELAY_US(delay, 0x100);
    BIT_DELAY_US(delay, 0x80);
    BIT_DELAY_US(delay, 0x40);
    BIT_DELAY_US(delay, 0x20);
    BIT_DELAY_US(delay, 0x10);
    BIT_DELAY_US(delay, 0x08);
    BIT_DELAY_US(delay, 0x04);
    BIT_DELAY_US(delay, 0x02);
    BIT_DELAY_US(delay, 0x01);
}



//uint16_t debug_error = 0;
volatile uint16_t rx_signal_start_time = 0;
volatile uint16_t rx_signal_end_time = 0;

ISR(INT0_vect) {
    if (MCUCR != ISC0_RISE) {

        MCUCR = ISC0_RISE;
        rx_signal_end_time = TCNT1L;
        rx_signal_end_time += TCNT1H << 8;


        /*clearDebugChars();
        //log8(speed);
        //logSep();
        log8(desired_revs_per_second & 0xff);
        logSep();
        log16(actual_revs_per_second);
        //log8(v & 0xff);
        //logSep();
        //log8(error & 0xff);
        logCRNL();
                if (desired_revs_per_second == 0) {
                }*/
    } else {
        MCUCR = ISC0_FALL;
        rx_signal_start_time = TCNT1L;
        rx_signal_start_time += TCNT1H << 8;
        rx_signal_end_time = 0;
    }
}

void process_rx_result() {
    if (rx_signal_start_time != 0 && rx_signal_end_time != 0) {
        uint16_t v = timer_diff(rx_signal_start_time, rx_signal_end_time);
        if (v > RX_COUNTS_PER_MS * 3) { // invalid
            speed = RX_COUNT_STOP;
        } else if (v > RX_COUNTS_PER_MS * 2) {
            speed = RX_COUNTS_PER_MS;
        } else if (v < RX_COUNTS_PER_MS) {
            speed = 0;
        } else {
            speed = v - RX_COUNTS_PER_MS;
        }

        if (speed > RX_COUNT_STOP - RX_COUNT_DEAD_BAND && speed < RX_COUNT_STOP + RX_COUNT_DEAD_BAND) {
            speed = RX_COUNT_STOP;
            desired_revs_per_second = 0;
            desired_direction = 0;
            //debugOn();

        } else if (speed >= RX_COUNT_STOP + RX_COUNT_DEAD_BAND) {
            desired_revs_per_second = ((speed - (RX_COUNT_STOP + RX_COUNT_DEAD_BAND)) * MAX_RPS) / (RX_COUNTS_PER_MS - (RX_COUNT_STOP + RX_COUNT_DEAD_BAND));
            desired_direction = AHEAD;
        } else if (speed <= RX_COUNT_STOP - RX_COUNT_DEAD_BAND) {
            desired_revs_per_second = (((RX_COUNT_STOP - RX_COUNT_DEAD_BAND) - speed) * MAX_RPS) / (RX_COUNT_STOP - RX_COUNT_DEAD_BAND);
            desired_direction = ASTERN;
        }
        /*if (desired_direction && desired_revs_per_second < 10) {
                desired_revs_per_second = 10;
        }*/
        rx_signal_start_time = 0;
        rx_signal_end_time = 0;
    }
}



ESC all_escs[2];

SENSOR all_sensors[2];

PID all_pids[2];

uint8_t count = 0;

extern uint8_t rxBuf[];
extern uint8_t txBuf[];


/*	uint8_t pid_cycle_counter = 0;
        uint8_t previous_desired_direction = AHEAD;
    uint16_t previous_desired_revs_per_second = 0;
    int16_t integral = 0; */

/**
 * For the tiny84, PB2 (pin 5) is the RX input, PB0 (pin 2) is the servo output, and PA2 (pin 11, AIN1) is the sensor input
 * PB4 (pin 3) is the diagnostic LED output
 */
int main(void) {
    // set the clock prescaler to 1 to get us an 8MHz clock -- the CKDIV8 fuse is programmed by default, so initial prescaler is /8
    CLKPR = 0x80;
    CLKPR = 0x00;

    all_sensors[0].port = A;
    all_sensors[0].pin = PA2;
    all_sensors[1].port = A;
    all_sensors[1].pin = PA3;
    all_escs[0].port = B;
    all_escs[0].pin = PB0;
    all_escs[1].port = A;
    all_escs[1].pin = PA7;
    //DDRB = (1 << MOTOR_DD); // output for motor 1
    DDRA |= (1 << LED1_DD) | (1 << LED2_DD); // outputs for diagnostic LEDs and motor 2
    PORTA = 0x00;
    PORTB = 0x00;
    //PCMSK0 = (1 << PCINT2); // set pin change interrupts on PCINT2
    // set the analog comparator to use the internal reference and enable interrupt on rising edge
    //ACSR = (1 << ACBG) | (1 << ACIE) | (1 << ACIS1) | (1 << ACIS0);
    MCUCR = ISC0_RISE; // any change ISC0_RISE; // look for rising edge on INT0
    // enable interrupt INT0 for RX signal
    GIMSK |= (1 << INT0);
    TIMSK1 |= (1 << TOIE1) /*| (1 << OCIE1A) | (1 << OCIE1B)*/; // enable counter 1 overflow interrupt
    TCCR1B = 0x03; // CK/64, i.e. 125KHz (0.008ms), or 488Hz MSB or 1.9Hz overflow.
    // initialise the ESC with a central setting
    red(1);
    green(0);
    setupEscs();
    usiTwiSlaveInit(0x10);
    sei();
    calibrateEscs();
    green(1);
    setupSensors();
    red(1);

    desired_direction = 0;
    desired_revs_per_second = 0;
    rxBuf[1] = 0;
    for (;;) {
        
        process_rx_result();
        if (rxBuf[0]) {
            desired_direction = (rxBuf[0] & 0x80) ? AHEAD : ASTERN;
            desired_revs_per_second = rxBuf[0] & 0x7f;
        }
        green(rxBuf[1]);
        processSensors();
        processPids();

        serviceEscs();
        txBuf[0] = desired_direction;
        txBuf[1] = desired_revs_per_second & 0xff;
        txBuf[2] = all_sensors[0].actual_revs_per_second & 0xff;
        txBuf[3] = all_escs[0].drive;
        txBuf[4] = all_sensors[1].actual_revs_per_second & 0xff;
        txBuf[5] = all_escs[1].drive;

        ++count;
        
    }
    return 0; /* never reached */
}

