#ifndef _FUNCTIONS_h
#define _FUNCTIONS_h

void mixer(int, int, int, int);	//mixer(fTHROTTLE, fPITCH, fROLL, fYAW);
int combine(int, int);			//combine(val1, val2) eg:THROTTLE, pTHROTTLE
int diff(int);					//diff(var);

void initCycTime();
void getCycTime(double);

inline double mapper(int, int, int, int, int);		//mapped = mapper(var, fromMin, fromMax, toMin, toMax);

#endif
