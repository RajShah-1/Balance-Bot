#ifndef _MPULib_h_
#define _MPULib_h_

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


class MPULib
{
public:
	MPULib(double fCut=5.0);

	void init(int xGyOffset=0, int yGyOffset=0, int zGyOffset=0, 
		int xAccOffset=0, int yAccOffset=0, int zAccOffset=0);
	void propagateArrays(int ax, int ay, int az, int gx, int gy, int gz);
	double getPitch(void);
	double getRoll(void);
	void iterate(void);
	void printRollPitch(void);

private:
	MPU6050 accGyro;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	double fCut = 5.0;
	int accData[3], gyData[3], gyDataPrev[3];
	double accFData[3], gyFData[3];
	double pitch, roll;

	void lowPassFilter(void);
	void highPassFilter(void);
	void compFilterPitch(void);
	void compFilterRoll(void);
};

#endif
