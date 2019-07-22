#ifndef _MOTORCONTROL_h
#define _MOTORCONTROL_h

#define MIN_PULSE_WIDTH			544
#define MAX_PULSE_WIDTH			2400
#define DEFAULT_PULSE_WIDTH		MID_VAL
#define REFRESH_INTERVAL		20000

void motorConfig(int);			//motorConfig(var); var = number of motors connected
void motorWrite(int, int);		//motorWrite(num, var); num = motor number, var = pulse width
int valToMotor(int);			//valToMotor(var); from 0-180 values to 1000-2000
void writeAllMotors(int);		//writeAllMotors(var); writes a value to all the motors

void writeHIGH(int);
void writeLOW(int);

#endif
