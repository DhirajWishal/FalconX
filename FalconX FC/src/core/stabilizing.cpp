/*
 Name:		stabilizing.cpp
 Created:	12/22/2018 2:31:38 PM
 Author:	Dhiraj Wishal

 This code is designed for UAVs for smooth flight stabilizing originally designed for drones.
 This code/ library handles the Throttle, Pitch, Roll, Yaw as the main parameters but also can
 be used to handle AUX1 and AUX2. All the stabilizing functions are included inside the class and
 can be called upon runtime thus increasing efficiency, reliability and accuracy of the code.
 This library is made as a stand-alone code which does not use any other external library but only uses the
 arduino.h library to call timing functions.

 This code/ library is still on Development/ Debugging stage.
 AI functions are yet to be included and defined.
*/

#include "stabilizing.h"
#include "utils/defines.h"

#include <ctime>

/* ---------- ########## \\\\\\\\\\ BEGIN DEFINITIONS ////////// ########## ---------- */

// Intelligent Multiaxial Flight Stabilizer
IMFS::IMFS(double* ax, double* ay, double* az, double* gx, double* gy, double* gz, double setpoint) {
	myACCX = ax;
	myACCY = ay;
	myACCZ = az;
	myGYRX = gx;
	myGYRY = gy;
	myGYRZ = gz;
	mySetPoint = setpoint;
}

IMFS::~IMFS() {
}

// manual mode. computeIMFS(THROTTLE, PITCH, ROLL, YAW, (bool)COMMAND)
void IMFS::computeIMFS(double Throttle, double Pitch, double Roll, double Yaw, bool command) {
	timer = clock();
	currentTime = (float)timer / CLOCKS_PER_SEC;

	*pTHROTTLE = Throttle;
	*pPITCH = Pitch;
	*pROLL = Roll;
	*pYAW = Yaw;

	if (command != true) {
		if (*myACCX != defAccX || *myACCY != defAccY || *myACCZ != defAccZ ||
			*myGYRX != midGyrX || *myGYRY != midGyrY || *myGYRZ != midGyrZ) {
			command = true;
		}
	}

	if (command == true) {
		error[GYR][X] = midGyrX - *myGYRX;
		error[GYR][Y] = midGyrY - *myGYRY;
		error[GYR][Z] = midGyrZ - *myGYRZ;

		error[ACC][X] = defAccX - *myACCX;
		error[ACC][Y] = defAccY - *myACCY;
		error[ACC][Z] = defAccZ - *myACCZ;

		correction[GYR][X] += error[GYR][X] * getDerivatives(oldError[GYR][X], error[GYR][X], oldTime, currentTime);
		correction[GYR][Y] += error[GYR][Y] * getDerivatives(oldError[GYR][Y], error[GYR][Y], oldTime, currentTime);
		correction[GYR][Z] += error[GYR][Z] * getDerivatives(oldError[GYR][Z], error[GYR][Z], oldTime, currentTime);

		intelligence();

		limitPitchRotation_l = prolimit(limitACCX_l, minACC, maxACC, MIN, MAX);
		limitPitchRotation_h = prolimit(limitACCX_h, minACC, maxACC, MIN, MAX);
		limitRollRotation_l = prolimit(limitACCY_l, minACC, maxACC, MIN, MAX);
		limitRollRotation_h = prolimit(limitACCY_h, minACC, maxACC, MIN, MAX);
		limitYawRotation_l = prolimit(limitACCZ_l, minACC, maxACC, MIN, MAX);
		limitYawRotation_h = prolimit(limitACCZ_h, minACC, maxACC, MIN, MAX);

		if (error[ACC][Z] != defAccZ) * pPITCH += prolimit(error[ACC][Z], minACC, maxACC, MIN, MAX);
		if (error[ACC][X] != defAccX) * pROLL += prolimit(error[ACC][X], minACC, maxACC, MIN, MAX);

		if (error[GYR][X] > midGyrX) * pPITCH += PitchDiv / correction[GYR][X];
		if (error[GYR][X] < midGyrX) * pPITCH -= PitchDiv / correction[GYR][X];
		if (error[GYR][Y] > midGyrY) * pROLL += RollDiv / correction[GYR][Y];
		if (error[GYR][Y] < midGyrY) * pROLL -= RollDiv / correction[GYR][Y];
		if (error[GYR][Z] > midGyrZ) * pYAW += YawDiv / correction[GYR][Z];
		if (error[GYR][Y] < midGyrZ) * pYAW -= YawDiv / correction[GYR][Z];

		outputThrottle = *pTHROTTLE;
		outputPitch = *pPITCH;
		outputRoll = *pROLL;
		outputYaw = *pYAW;
	}
	else
	{
		//Do something here 
	}
	oldError[ACC][X] = error[ACC][X];
	oldError[ACC][Y] = error[ACC][Y];
	oldError[ACC][Z] = error[ACC][Z];

	oldError[GYR][X] = error[GYR][X];
	oldError[GYR][Y] = error[GYR][Y];
	oldError[GYR][Z] = error[GYR][Z];

	oldTime = currentTime;
}

// automaticControlIMFS(THROTTLE, (bool)ACTIVATE)
void IMFS::automaticControlIMFS(int throt, bool activate) {
	*pTHROTTLE = throt;
	if (activate == true) {
		//Gyro X
		if (*myGYRX == midGyrX) outputThrottle = *pTHROTTLE;
		if (*myGYRX > midGyrX) {
			double error = *myGYRX - midGyrX;
			outputPitch = prolimit(error, minGYR, maxGYR, MIN, MAX);
		}
		if (*myGYRX < midGyrX) {
			double error = midGyrX - *myGYRX;
			outputPitch = prolimit(error, minGYR, maxGYR, MIN, MAX);
		}

		//Gyro Y
		if (*myGYRY == midGyrY) outputThrottle = *pTHROTTLE;
		if (*myGYRY > midGyrY) {
			double error = *myGYRY - midGyrY;
			outputYaw = prolimit(error, minGYR, maxGYR, MIN, MAX);
		}
		if (*myGYRY < midGyrY) {
			double error = midGyrY - *myGYRY;
			outputYaw = prolimit(error, minGYR, maxGYR, MIN, MAX);
		}

		//Gyro Z
		if (*myGYRZ == midGyrZ) outputThrottle = *pTHROTTLE;
		if (*myGYRZ > midGyrZ) {
			double error = *myGYRZ - midGyrZ;
			outputRoll = prolimit(error, minGYR, maxGYR, MIN, MAX);
		}
		if (*myGYRZ < midGyrZ) {
			double error = midGyrZ - *myGYRZ;
			outputRoll = prolimit(error, minGYR, maxGYR, MIN, MAX);
		}

		//Accelero rotations
		double errorX, errorY;
		errorX = defAccX - *myACCX;
		errorY = defAccY - *myACCY;

		if (errorX > midGyrX) outputPitch = prolimit(errorX, minACC, maxACC, MIN, MAX);
		if (errorX < midGyrX) outputPitch = prolimit(errorX, minACC, maxACC, MIN, MAX);
		if (errorY > midGyrY) outputRoll = prolimit(errorY, minACC, maxACC, MIN, MAX);
		if (errorY < midGyrY) outputRoll = prolimit(errorY, minACC, maxACC, MIN, MAX);
	}
	else {
		outputThrottle = *pTHROTTLE;
	}
	outputThrottle = *pTHROTTLE;
}

inline void IMFS::updateIMFS(double adefx, double adefy, double adefz, double gmidx, double gmidy, double gmidz) {
	defAccX = adefx;
	defAccY = adefy;
	defAccZ = adefz;
	midGyrX = gmidx;
	midGyrY = gmidy;
	midGyrZ = gmidz;
}

inline void IMFS::setLimitsIMFS(double min, double max, double amin, double amax, double gmin, double gmax) {
	MIN = min;
	MAX = max;
	MID = (MIN + MAX) / 2;
	minACC = amin;
	maxACC = amax;
	minGYR = gmin;
	maxGYR = gmax;
}

inline double IMFS::getDerivatives(double x1, double x2, double y1, double y2) {
	return (y1 - y2) / (x1 - x2);
}

inline float IMFS::prolimit(double var, double fromMin, double fromMax, double toMin, double toMax) {
	return ((var - fromMin) * (toMax - toMin) / (fromMax - fromMin)) + toMin;
}

inline float IMFS::proportionate(double fromMin, double fromMax, double toMin, double toMax) {
	return (toMax - toMin) / (fromMax - fromMin) + toMin;
}

void IMFS::outputIMFS(int16_t * Throttle, int16_t * Pitch, int16_t * Roll, int16_t * Yaw, int16_t * aux1, int16_t * aux2) {
	*Throttle = outputThrottle;
	*Pitch = outputPitch;
	*Roll = outputRoll;
	*Yaw = outputYaw;
	*aux1 = outputAUX1;
	*aux2 = outputAUX2;
}

// Internal intelligence functions
inline void IMFS::initIntelligentIMFS(int x) {
	intelliState = x;
}

void IMFS::intelligence() {
	if (intelliState == true) {
		errorHistory[0][x] = error[ACC][X] + error[GYR][X];
		errorHistory[1][x] = error[ACC][Y] + error[GYR][Y];
		errorHistory[2][x] = error[ACC][Z] + error[GYR][Z];

		correctionHistory[0][x] = correction[ACC][X] + correction[GYR][X];
		correctionHistory[1][x] = correction[ACC][Y] + correction[GYR][Y];
		correctionHistory[2][x] = correction[ACC][Z] + correction[GYR][Z];

		timeScale[x] = (currentTime + oldTime) / 2;

		/* PitchDiv (All X), RollDiv (All Y), YawDiv (All Z) */

		x++;
	}
}

/* ---------- ########## \\\\\\\\\\ END OF DEFINITIONS ////////// ########## ---------- */
