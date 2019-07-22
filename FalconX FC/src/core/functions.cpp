// FalconX FC
// Functions library
// Dhiraj Wishal	Dec 4th 2018

#include "functions.h"
#include "utils/defines.h"
#include "motorControl.h"
#include "variables.h"
#include "Arduino.h"

void mixer(int throttle, int pitch, int roll, int yaw) {
	/* ---------- PITCH CONTROL ---------- */
	if (pitch > MID_VAL) {
		motorWrite(0, throttle);
		motorWrite(1, throttle);
		motorWrite(2, throttle + diff(pitch));
		motorWrite(3, throttle + diff(pitch));
	}
	if (pitch < MID_VAL) {
		motorWrite(0, throttle + diff(pitch));
		motorWrite(1, throttle + diff(pitch));
		motorWrite(2, throttle);
		motorWrite(3, throttle);
	}

	/* ---------- ROLL CONTROL ---------- */
	if (roll > MID_VAL) {
		motorWrite(0, throttle + diff(roll));
		motorWrite(1, throttle);
		motorWrite(2, throttle + diff(roll));
		motorWrite(3, throttle);
	}
	if (roll < MID_VAL) {
		motorWrite(0, throttle);
		motorWrite(1, throttle + diff(roll));
		motorWrite(2, throttle);
		motorWrite(3, throttle + diff(roll));
	}

	/* ---------- YAW CONTROL ---------- */
	if (yaw > MID_VAL) {
		motorWrite(0, throttle - diff(yaw));
		motorWrite(1, throttle + diff(yaw));
		motorWrite(2, throttle + diff(yaw));
		motorWrite(3, throttle - diff(yaw));
	}
	if (yaw < MID_VAL) {
		motorWrite(0, throttle + diff(yaw));
		motorWrite(1, throttle - diff(yaw));
		motorWrite(2, throttle - diff(yaw));
		motorWrite(3, throttle + diff(yaw));
	}

	/* ---------- DEFAULT CONTROLS ---------- */
	if (pitch == MID_VAL && roll == MID_VAL && yaw == MID_VAL) {
		writeAllMotors(throttle);
	}
}

inline int combine(int var1, int var2) {
	if (var2 > MIN_VAL) var2 -= MIN_VAL;
	return var1 + var2;
}

inline int diff(int var)
{
	if (var > MID_VAL) return var - MID_VAL;
	if (var < MID_VAL) return MID_VAL - var;
	if (var == MID_VAL) return MID_VAL;
}

#if defined(debug)
long now, old;
inline void initCycTime() {
	now = millis();
}
inline void getCycTime(double val) {
	val = now - old;
	old = now;
}
#endif


inline double mapper(int var, int fromMin, int fromMax, int toMin, int toMax) {
	return ((var - fromMin) * (toMax - toMin) / (fromMax - fromMin)) + toMin;
}