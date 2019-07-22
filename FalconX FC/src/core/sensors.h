#ifndef _SENSORS_h
#define _SENSORS_h

class MPU6050 {
public:
	MPU6050(double*, double*, double*, double*, double*, double*, double*);
	void initMPU6050();
	void getMPU6050();
	void callibrateMPU6050(double*, double*, double*);

private:
	double *ACCX, *ACCY, *ACCZ;
	double *GYRX, *GYRY, *GYRZ;
	double *TEMP;

	double cAccX, cAccY, cAccZ;
};

#endif
