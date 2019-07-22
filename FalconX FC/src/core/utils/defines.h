#ifndef _DEFINES_h
#define _DEFINES_h


/* ---------- RUNTIME DEFINITIONS ---------- */

#define MIN_VAL				1000
#define MID_VAL				1500
#define MAX_VAL				2000

#define MIN_COMMAND			1000
#define MIN_THROTTLE		1000

#define MPU_ADDR			0x68
#define FILTERFACTOR		1
#define ACCFACTOR			5.5
#define ACCDIV				1024.0
#define GYROFACTOR			250.0
#define GYRODIV				16384.0
#define ACC_MIN				-90
#define ACC_MID				0
#define ACC_MAX				90
#define GYR_MIN				-500
#define GYRMID				0
#define GYR_MAX				500

#define historyLimit		255

/* ---------- OTHER DEFINITIONS ---------- */

#define debug
#define calibOnCycle

#endif