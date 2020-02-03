#ifndef _MPULib_h_
#define _MPULib_h_

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


class MPULib
{
public:
	MPULib(double fCut=5.0, double alphaVal = 0.03);

	void init(int xGyOffset=0, int yGyOffset=0, int zGyOffset=0, 
		int xAccOffset=0, int yAccOffset=0, int zAccOffset=0);
	void attachMPUInterrupt(void);
	void propagateArrays(int ax, int ay, int az, int gx, int gy, int gz);
	double getPitch(void);
	double getRoll(void);
	double getThetaDot(void);
	void iterate(void);
	void printRollPitch(void);
	void MPULibISR(void);

private:
	MPU6050 accGyro;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	double fCut;
	double dT, alpha;
	unsigned long timeStamp;
	double accData[3], gyData[3], gyDataPrev[3];
	double accFData[3], gyFData[3];
	double pitch, roll, prevPitch, pitchRate;
	const byte interruptPin;

	void findPitchRate(void);
	void lowPassFilter(void);
	void highPassFilter(void);
	void compFilterPitch(void);
	void compFilterRoll(void);
};

#endif
