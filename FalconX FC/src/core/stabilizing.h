/*
 Name:		stabilizing.h
 Created:	12/22/2018 2:31:38 PM
 Author:	Dhiraj Wishal

 Intelligent Multi-axial Flight Stabilizer (IMFS)
 This Flight stabilizer is only internded to work as a primary flight stabilizer for arial vehicles (UAVs)
 Does not control one axis at a time, uses all 3 axises to function which includes both gyro and accelero.

 Still in development stage.
*/

#include "defines.h"

#include <cstdint>
#include <vector>
#include <memory>

/* ---------- ########## \\\\\\\\\\ BEGIN CLASS AND FUNCTIONS ////////// ########## ---------- */

const enum sensor {
	ACC,
	GYR
};

const enum axis {
	X,
	Y,
	Z
};

/* Intelligent Multi-axial Flight Stabilizer */
class IMFS {
public:
	//Main class function
	IMFS(double*, double*, double*, double*, double*, double*, double);		//*IMFS(&accX, &accY, &accZ, &gyrX, &gyrY, &gyrZ, setPoint); (Miltiaxial stabilizer)
	~IMFS();

	//Other class functions
	void computeIMFS(double, double, double, double, bool);					//*computeTMFS(throttle, pitch, roll, yaw, command); Outputs trim value
	void automaticControlIMFS(int, bool);									//automaticControlIMFS(throttle, activate); Automatically stabilize the drone when no pitch, roll or yaw given (AGGRESSIVE) (THIS FUNCTON REQUIRES mixer() FUNCTION TO WORK)
	void updateIMFS(double, double, double, double, double, double);		//updateIMFS(defAccX, defAccY, defAccZ, midGyrX, midGyrY, midGyrZ); updates unaddressed variables for mid values
	void setLimitsIMFS(double, double, double, double, double, double);		//setLimitsIMFS(min, max, Amin, Amax, Gmin, Gmax);
	void outputIMFS(int16_t*, int16_t*, int16_t*, int16_t*, int16_t*, int16_t*);	//*outputIMFS(&Throttle, &Pitch, &Roll, &Yaw, &Aux1, &Aux2);

	void initIntelligentIMFS(int);											//initIntelligenceIMFS(state); DEACTIVATE/ ACTIVATE

private:
	//Private Functions
	double getDerivatives(double, double, double, double);					//d = getDerivatives(x1, x2, y1, y2);
	float prolimit(double, double, double, double, double);					//p = prolimit(var, fromMin, fromMax, toMin, toMax);
	float proportionate(double, double, double, double);					//pr = proportionate(fromMin, fromMax, toMin, toMax);
	void intelligence();													//intelligence function

	//private global in-class variables
	double* myACCX, * myACCY, * myACCZ;
	double* myGYRX, * myGYRY, * myGYRZ;
	uint16_t mySetPoint = MID_VAL;
	uint16_t* pTHROTTLE, * pPITCH, * pROLL, * pYAW;								//Computational and Final variables
	uint16_t MIN = MAX_VAL, MAX = MAX_VAL, MID = MID_VAL;

	//computational variables (error detection and correction)
	// variable[sensor][axis](0 = X, 1 = Y, 2 = Z)
	std::vector<std::vector<int8_t>> error = { { NULL, NULL, NULL}, { NULL, NULL, NULL} };
	std::vector<std::vector<int8_t>> oldError = { { NULL, NULL, NULL}, { NULL, NULL, NULL} };
	std::vector<std::vector<int16_t>> correction = { { NULL, NULL, NULL}, { NULL, NULL, NULL} };

	int16_t limitPitchRotation_l, limitRollRotation_l, limitYawRotation_l;
	int16_t limitPitchRotation_h, limitRollRotation_h, limitYawRotation_h;
	float PitchDiv = 1000, RollDiv = 1000, YawDiv = 1000;
	clock_t timer;
	float oldTime, currentTime;

	//sensor mids and other values
	int8_t defAccX = 0, defAccY = 0, defAccZ = 0;							//defAccX = default Acc values (when placed on a horizontal flat place)
	int8_t midGyrX = 0, midGyrY = 0, midGyrZ = 0;
	int8_t minACC = -90, maxACC = 90;
	int16_t minGYR = -500, maxGYR = 500;
	const int8_t limitACCX_l = -30, limitACCY_l = -30, limitACCZ_l = -30;	//lower limits for turning angles
	const int8_t limitACCX_h = 30, limitACCY_h = 30, limitACCZ_h = 30;		//higher limits for turnign angles

	//intelligent functions
	enum state { DEACTIVATE, ACTIVATE };									// ACTIVATE or DEACTIVATE intelligent functions
	int8_t intelliState = false;
	int16_t errorHistory[3][historyLimit];									//History of errors
	int16_t correctionHistory[3][historyLimit];								//History of corrections
	std::vector<int16_t> currentDirv;
	int16_t timeScale[historyLimit];
	int8_t x = NULL;


	//Outputs
	int16_t outputThrottle, outputPitch, outputRoll, outputYaw, outputAUX1, outputAUX2;
};
