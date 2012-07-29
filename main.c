#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define UART_SOFT_DDR DDRB 
#define UART_SOFT_PORT PORTB 
#define UART_SOFT_PIN PB1
#define UART_SOFT_BAUD (4800)

#define UART_SOFT_DELAY_US (int)(1000000.0/((float) UART_SOFT_BAUD)+0.5) 


#define softUartInit()   UART_SOFT_DDR |= (1<<UART_SOFT_PIN); \
                  UART_SOFT_PORT |= (1<<UART_SOFT_PIN)
                  
#define DEBUG_CHARS_SIZE 9
                  
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
}

#define MAX_SPEED INT16_C(250)
#define STOP_SPEED INT16_C(125)
#define DEAD_BAND (5)

#define MAX_RPS INT16_C(200)
// controller input value after confining to 0..MAX_SPEED
int16_t speed = 0;

// time to drive the motor in multiples of 4us
uint16_t drive = STOP_SPEED;

uint16_t actual_revs_per_second = 0;
uint16_t desired_revs_per_second = 0;

#define AHEAD 1
#define ASTERN 2
uint8_t desired_direction = 0;

// init vars
  int processValue = 0; // holds the actual speed (RPS)
  int setPoint; // holds the desired speed (RPS)


  // the tuning constants -----------------------------------

  int PFactor = 3; // the Proportional tuning constant
  int IFactor = 5; // the Integral tuning constant
  int DFactor = 0; // the Derivative tuning constant
 //----------------------------------------------------------

  int error = 0;
  int Pterm = 0;
  int Iterm = 0;
  int Dterm = 0;
  int lastProcessValue = 0;
  int lastSetPoint = 0;
  int sumError = 0;
  int DprocessValue = 0;
  int firstExecution = 0;
  int initialError = 0;

  int output;

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
uint16_t debug_error = 0;

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
        log16(actual_revs_per_second);
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


  
void pidInit() {
	error = 0;
	Pterm = 0;
	Iterm = 0;
	Dterm = 0;
	lastProcessValue = 0;
	lastSetPoint = 0;
	sumError = 0;
	DprocessValue = 0;
	firstExecution = 0;
	initialError = 0;
}

void pidController() {
    error = setPoint - processValue;
    sumError = sumError + error;

    // >>> calculate Iterm
    // --------------------
    Iterm = IFactor * sumError; // <---- scaled

    // first time through
    if (firstExecution < 1){

      if (firstExecution == 0){
        //sumError = Manual_value / Ifactor
        firstExecution = 1;
        initialError = error;
      }

      // wait for error to over correct via Iterm
      // before going on to normal operation
      if ( (initialError > 0 && error < 0) || (initialError < 0 && error > 0) ) {
        firstExecution = 2;
        lastProcessValue = processValue;
      }

      lastSetPoint = setPoint;

    // normal operation
    } else if (processValue >= 0) {

      // >>> calculate Dterm
      // -------------------
      DprocessValue = lastProcessValue - processValue;
      lastProcessValue = processValue;

      Dterm = DFactor * DprocessValue; // <--- scaled


      if (setPoint == lastSetPoint) {
        // setpoint has not changed

        // >>> calculate Pterm
        // -------------------
        Pterm = PFactor * error; // <--- scaled

      } else {
        // setPoint has changed

        // reset PD terms
        Pterm = 0;
        Dterm = 0;

        if (setPoint > lastSetPoint && processValue > setPoint){
          lastSetPoint = setPoint;
          lastProcessValue = processValue;
        }
        if (setPoint < lastSetPoint && processValue < setPoint){
          lastSetPoint = setPoint;
          lastProcessValue = processValue;
        }
      } // close (setpoint == lastsetpoint) else

    }// close else if (precessValue >= 0)


    output = ((Pterm + Iterm + Dterm) / 100); // <--- scaled back down
}

/**
 * For the tiny85, PB2 (pin 7) is the RX input, PB3 (pin 2) is the servo output, and PB1 (pin 6, AIN1) is reserved for the sensor input
 * PB4 (pin 3) is the diagnostic LED output
 */

#define RX_DD	(DDB2)
#define RX_P	(PB2) // Must be INT0 pin
#define MOTOR_DD (DDB0)
#define MOTOR_P (PB0)
#define SENSOR_DD (DDA2)
#define SENSOR_P (PA2) // Must be AIN1
#define LED1_DD (DDA1)
#define LED1_P (PA1)
#define LED2_DD (DDA0)
#define LED2_P (PA0)

#define ISC0_RISE (0x03)
#define ISC0_FALL (0x02)

int main(void)
{
// set the clock prescaler to 1 to get us an 8MHz clock -- the CKDIV8 fuse is programmed by default, so initial prescaler is /8
    CLKPR = (1 << CLKPCE);
    CLKPR = 0x00;
    DDRB = (1 << MOTOR_DD); // outputs for servo
    DDRA = 3; //(1 << LED1_DD) | (1 << LED2_DD); // outputs for diagnostic LEDs
    PORTA = 0x00;
    PORTB = 0x00;
    for (;;) {
    }
    // set the analog comparator to use the internal reference and enable interrupt on toggle
    ACSR = (1 << ACBG) | (1 << ACIE);
    MCUCR = ISC0_RISE; // look for rising edge on INT0
    // enable interrupt INT0
    GIMSK = (1 << INT0);
   // TIMSK = (1 << TOIE1) | (1 << TOIE0); // enable counter 1 and 0  overflow interrupts
   // TCCR1 = 0x06; // CK/32, i.e. 250KHz, or 976Hz overflow interrupt.
    TCCR0B = 0x05; // CK/1024 i.e. 7812.5 Hz or 30Hz overflow interrupt.
    
    pidInit();

    sei();
	uint8_t pid_cycle_counter = 0;
	uint8_t previous_desired_direction = AHEAD;
    for(;;){
            ++pid_cycle_counter;
			if (desired_direction == 0) {
			    pidInit();
			    output = 0;
			}
			if (pid_cycle_counter > 2 && speed != STOP_SPEED) { // do this every 200ms
				debugOff();
				pid_cycle_counter = 0;
				if (desired_direction != previous_desired_direction) {
				    pidInit();
				    previous_desired_direction = desired_direction;
				    output = 0;
				}
				setPoint = desired_revs_per_second;
				processValue = actual_revs_per_second;
				//pidController();
				if (desired_revs_per_second < actual_revs_per_second) {
					--output;
				} else if (desired_revs_per_second > actual_revs_per_second) {
					++output;
				}
				
				if (output > MAX_SPEED/2) {
					output = MAX_SPEED/2;
				} else if (output <= 0) {		
					output = 0;
				}
				if (desired_direction == AHEAD) {
					drive = output + MAX_SPEED/2;
				} else {
					drive = MAX_SPEED/2 - output;
				}
				debug_error = desired_revs_per_second - actual_revs_per_second;
				if (debug_error < 0) {
					debug_error = -debug_error;
				}
				
			}
            doCycle();
    }
    return 0;   /* never reached */
}

