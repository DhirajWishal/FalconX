/*
 Name:		FalconX_FC.ino
 Author:	Dhiraj Wishal
 Project:	FalconX Flight Controller 2.0
 Created:	12/4/2018 11:38:49 AM	(DEVELOPMENTS ONGOING)
*/

#include "src/FalconX.h"

/* ---------- ########## \\\\\\\\\\ GLOBAL VARIABLES AND CONSTANTS ////////// ########## ---------- */

int16_t THROTTLE = MIN_THROTTLE, PITCH = MID_VAL, ROLL = MID_VAL, YAW = MID_VAL, AUX1 = MID_VAL, AUX2 = MID_VAL;
//int16_t pTHROTTLE, pPITCH, pROLL, pYAW, pAUX1, pAUX2;
//int16_t FTHROTTLE, FPITCH, FROLL, FYAW;
double ACCX, ACCY, ACCZ, GYRX, GYRY, GYRZ, TEMP;
double ACCDEFX, ACCDEFY, ACCDEFZ;
int16_t GYR_MID_X = 0, GYR_MID_Y = 0, GYR_MID_Z = 0;

bool state = true;

/* ---------- ########## \\\\\\\\\\ OBJECTS/ FUNCTIONS/ LOCAL DEFINITIONS ////////// ########## ---------- */

const int16_t SetPoint = 1500;
//double kp = 1, ki = 2, kd = 1;

/* ---------- ########## \\\\\\\\\\ CLASS FUNCTION DEFINITIONS ////////// ########## ---------- */

MPU6050 sensor(&ACCX, &ACCY, &ACCZ, &GYRX, &GYRY, &GYRZ, &TEMP);
IMFS stabilize(&ACCX, &ACCY, &ACCZ, &GYRX, &GYRY, &GYRZ, SetPoint);

// the setup function runs once when you press reset or power the board
void setup() {
#if defined(debug)
	Serial.begin(115200);
	Serial.flush();
	Serial.println("# DEBUG INITIATED, RUNNING SETUP...");
#endif
#if defined(calibOnCycle)
	stabilize.updateIMFS(ACCDEFX, ACCDEFY, ACCDEFZ, GYR_MID_X, GYR_MID_Y, GYR_MID_Z);
#endif

	sensor.initMPU6050();
	sensor.callibrateMPU6050(&ACCDEFX, &ACCDEFY, &ACCDEFZ);

	motorConfig(4);

	stabilize.setLimitsIMFS(MIN_VAL, MAX_VAL, ACC_MIN, ACC_MAX, GYR_MIN, GYR_MAX);
	stabilize.updateIMFS(ACCDEFX, ACCDEFY, ACCDEFZ, GYR_MID_X, GYR_MID_Y, GYR_MID_Z);

	initWireInputs();

	pinMode(13, OUTPUT);

#if defined(debug)
	Serial.println("# JUMPING TO LOOP...");
#endif
}

// the loop function runs over and over again until power down or reset
void loop() {
#if defined(debug)
	//Serial.println("# RUNNING ON LOOP...");
#endif
	getWireInputs(THROTTLE, PITCH, ROLL, YAW, AUX1, AUX2);

	sensor.getMPU6050();
	//Serial.println("GOT MPU6050 VALUES");

	//stabilize.updateIMFS(ACCDEFX, ACCDEFY, ACCDEFZ, GYR_MID_X, GYR_MID_Y, GYR_MID_Z);
	//stabilize.computeIMFS(THROTTLE, PITCH, ROLL, YAW, true);
	stabilize.automaticControlIMFS(THROTTLE, true);
	stabilize.outputIMFS(&THROTTLE, &PITCH, &ROLL, &YAW, &AUX1, &AUX2);
	//Serial.println("GOT STABILIZING VALUES");

	//debugger();

	mixer(THROTTLE, PITCH, ROLL, YAW);

	//strobe();
}

inline void debugger() {
	Serial.print(THROTTLE); Serial.print("\t");
	Serial.print(PITCH); Serial.print("\t");
	Serial.print(ROLL); Serial.print("\t");
	Serial.print(YAW); Serial.print("\t");
	Serial.println();
}

inline void strobe() {
	digitalWrite(13, state);
	state != state;
}

inline void FailsafeMode() {
	stabilize.automaticControlIMFS(THROTTLE, true);
}