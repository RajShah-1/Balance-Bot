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


private:
	MPU6050 accGyro;
	double accFx, gyFx;
	double fCut, dT, alpha, TauF, alphaF;
	unsigned long timeStamp;
	double pitch, roll, prevPitch, pitchRate;
	const byte interruptPin;

	void findPitchRate(void);
	void lowPassFilter(void);
	void highPassFilter(void);
	void compFilterPitch(void);
	void compFilterRoll(void);
};
extern MPULib sensor;

void MPULibISR(void);

#endif
