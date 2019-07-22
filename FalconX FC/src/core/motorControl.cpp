// FalconX FC
// Motor Control library
// Dhiraj Wishal	Dec 4th 2018

#include "motorControl.h"
#include "utils/defines.h"
#include "variables.h"
#include "utils/interrupt.h"
#include "utils/io.h"

/*
	motor 0 = TL(pin 3) - CW
	motor 1 = TR(pin 5) - CCW
	motor 2 = BL(pin 6) - CCW
	motor 3 = BR(pin 9) - CW
*/

/* ---------- ########## \\\\\\\\\\ INTERRUPT HANDLING CODE ////////// ########## ---------- */

#define usToTicks(_us)		(((16000000 / 1000000)* _us) / 8)
#define ticksToUs(_ticks)	(( (unsigned)_ticks * 8)/ (1600000 / 1000000))
#define MOTOR_MIN()			MIN_VAL
#define MOTOR_MAX()			MAX_VAL
#define TRIM_DURATION		2

int MotorCount = 0;
int Channel;
int valTicks, ticks;
int motorTicks[4] = {};

static inline void interruptsHandler()
{
	if (Channel < 0)	//Refresh check
		TCNT1 = 0;
	else {
		writeLOW(Channel);
	}

	Channel++;
	if (Channel < MotorCount) {
		OCR1A = TCNT1 + motorTicks[Channel];
		writeHIGH(Channel);	//Set pin HIGH
	}
	else {

		if (((unsigned)TCNT1) + 4 < usToTicks(REFRESH_INTERVAL)) // allow a few ticks to ensure the next OCR1A not missed
			OCR1A = (unsigned int)usToTicks(REFRESH_INTERVAL);
		else
			OCR1A = TCNT1 + 4;
		Channel = -1; // this will get incremented at the end of the refresh period to start again at the first channel
	}
}

SIGNAL(TIMER1_COMPA_vect)
{
	interruptsHandler();
}

inline static void initTimer()
{
	TCCR1A = 0;
	TCCR1B = (1 << CS11);
	TCNT1 = 0;
	TIFR1 |= (1 << OCF1A);
	TIMSK1 |= (1 << OCIE1A);
}

/* ---------- ########## \\\\\\\\\\ END ////////// ########## ---------- */

inline void motorConfig(int x) {
	DDRD |= (1 << 3);	//PWM pin 3
	DDRD |= (1 << 5);	//PWM pin 5
	DDRD |= (1 << 6);	//PWM pin 6
	DDRB |= (1 << 1);	//PWM pin 9
	MotorCount = x;
	initTimer();
}

void motorWrite(int motor, int value) {
	if (value < MOTOR_MIN())         // ensure pulse width is valid
		value = MOTOR_MIN();
	else if (value > MOTOR_MAX())
		value = MOTOR_MAX();

	value -= TRIM_DURATION;
	value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

	int oldSREG = SREG;
	cli();
	motorTicks[motor] = value;
	SREG = oldSREG;
}

void writeHIGH(int x) {
	switch (x) {
	case 0:
		PORTD = 0x08;
		break;
	case 1:
		PORTD = 0x20;
		break;
	case 2:
		PORTD = 0x40;
		break;
	case 3:
		PORTB = 0x02;
		break;
	default:
		PORTD = 0x00;
		PORTD = 0x00;
		PORTD = 0x00;
		PORTB = 0x00;
		break;
	}
}

void writeLOW(int y) {
	switch (y) {
	case 0:
		PORTD = 0x00;	//pwm pin 3 LOW
		break;
	case 1:
		PORTD = 0x00;	//pwm pin 5 LOW
		break;
	case 2:
		PORTD = 0x00;	//pwm pin 6 LOW
		break;
	case 3:
		PORTB = 0x00;	//pwm pin 9 LOW
		break;
	default:
		break;
	}
}

inline void writeAllMotors(int var) {
	motorWrite(0, var);
	motorWrite(1, var);
	motorWrite(2, var);
	motorWrite(3, var);
}

inline int valToMotor(int value) {
	return ((value - 1000) * (180 - 0)) / (2000 - 1000) + 0;
}
