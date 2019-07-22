// FalconX FC
// Input(s) library
// Dhiraj Wishal	Dec 4th 2018

#include "inputs.h"
#include "utils/defines.h"
#include "variables.h"
#include "Wire/src/Wire.h"

/*
#include <CPPM.h>

inline void initCPPM() {
	CPPM.begin();
}

inline void getCPPM(int throttle, int pitch, int roll, int yaw, int aux1, int aux2) {
	if (CPPM.synchronized()) {
		roll = CPPM.read_us(CPPM_AILE);
		pitch = CPPM.read_us(CPPM_ELEV);
		throttle = CPPM.read_us(CPPM_THRO);
		yaw = CPPM.read_us(CPPM_RUDD);
		aux1 = CPPM.read_us(CPPM_GEAR);
		aux2 = CPPM.read_us(CPPM_AUX1);
	}
}
*/

inline void initWireInputs() {

}

inline void getWireInputs(double throt, double pitch, double roll, double yaw, double aux1, double aux2) {

}