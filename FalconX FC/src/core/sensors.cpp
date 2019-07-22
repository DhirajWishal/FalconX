// FalconX FC
// Sensor manipulation
// Dhiraj Wishal	Dec 4th 2018

/*
		MPU6050 filteration process:
			Accelero:
				AX * 5.5 / 1024.0
				AY * 5.5 / 1024.0
				AZ * 5.5 / 1024.0

			Gyro:
				GX * 250.0 / 16384.0
				GY * 250.0 / 16384.0
				GZ * 250.0 / 16384.0

		MPU6050 Limits:
			Accelero: -90 <= a <= 90
			Gyro: -500 <= g <= 500
*/

#include "sensors.h"
#include "utils/defines.h"
#include "variables.h"

#ifdef __cplusplus

// ----------
#else
#include <Wire.h>

// ----------
#endif

MPU6050::MPU6050(double *accx, double *accy, double *accz, double *gyrx, double *gyry, double *gyrz, double *temp) {
	ACCX = accx;
	ACCY = accy;
	ACCZ = accz;
	GYRX = gyrx;
	GYRY = gyry;
	GYRZ = gyrz;
	TEMP = temp;
}

inline void MPU6050::initMPU6050() {
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
}

void MPU6050::getMPU6050() {
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_ADDR, 14, true);

	*ACCX = (((Wire.read() << 8 | Wire.read()) / FILTERFACTOR) * ACCFACTOR) / ACCDIV;
	*ACCY = (((Wire.read() << 8 | Wire.read()) / FILTERFACTOR) * ACCFACTOR) / ACCDIV;
	*ACCZ = (((Wire.read() << 8 | Wire.read()) / FILTERFACTOR) * ACCFACTOR) / ACCDIV;
	*TEMP = (Wire.read() << 8 | Wire.read());
	*GYRX = (((Wire.read() << 8 | Wire.read()) / FILTERFACTOR) * GYROFACTOR) / GYRODIV;
	*GYRY = (((Wire.read() << 8 | Wire.read()) / FILTERFACTOR) * GYROFACTOR) / GYRODIV;
	*GYRZ = (((Wire.read() << 8 | Wire.read()) / FILTERFACTOR) * GYROFACTOR) / GYRODIV;
}

inline void MPU6050::callibrateMPU6050(double *cx, double *cy, double *cz) {
	getMPU6050();

	cx = ACCX;
	cy = ACCY;
	cz = ACCZ;
	cz = ACCZ;
}