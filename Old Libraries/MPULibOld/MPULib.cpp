#include "MPULib.h"


MPULib::MPULib(double fCutVal = 5.0, double alphaVal = 0.03) : fCut(fCutVal), alpha(alphaVal), pitch(0.0), roll(0.0){
	for(int i = 0; i < 3; ++i)
    	accData[i] = gyData[i] = gyDataPrev[i] = accFData[i] = gyFData[i] = 0;
}

void MPULib::init(int xGyOffset = 0, int yGyOffset = 0, int zGyOffset = 0,
	int xAccOffset = 0, int yAccOffset = 0, int zAccOffset = 0){
	
	Wire.begin();

	Serial.println("Initializing I2C devices...");
	accGyro.initialize();

	
  	// verify connection
  	Serial.println("Testing device connections...");
  	Serial.println(accGyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	Serial.println("Updating internal sensor offsets...");
  
	accGyro.setXGyroOffset(xGyOffset);
	accGyro.setYGyroOffset(yGyOffset);
	accGyro.setZGyroOffset(zGyOffset);

	accGyro.setXAccelOffset(xAccOffset);
	accGyro.setYAccelOffset(yAccOffset);
	accGyro.setZAccelOffset(zAccOffset);

	Serial.println("MPULib OFFSETS CHECK::");
	Serial.print(accGyro.getXAccelOffset());Serial.print("\t");
	Serial.print(accGyro.getYAccelOffset());Serial.print("\t");
	Serial.print(accGyro.getZAccelOffset());Serial.print("\t");
	Serial.print(accGyro.getXGyroOffset());Serial.print("\t");
	Serial.print(accGyro.getYGyroOffset());Serial.print("\t");
	Serial.print(accGyro.getZGyroOffset());Serial.print("\t");
	Serial.print("\n");
	Serial.print("fCut: ");
	Serial.println(fCut);
	Serial.println("CHECK OVER");
	
	
	timeStamp = micros();
}

double MPULib::getPitch(void) { return pitch; }

double MPULib::getRoll(void) { return roll; }

double MPULib::getThetaDot(void) { return pitchRate; }

void MPULib::iterate(void){
	unsigned long currTime = micros();
	dT = (double)(currTime-timeStamp)/1e6;
	timeStamp = currTime;
	Serial.print("dT: ");
	Serial.println(dT);

	accGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	propagateArrays(ax, ay, az, gx, gy, gz);
  	lowPassFilter();
  	highPassFilter();
  	compFilterPitch();
  	compFilterRoll();
	// findPitchRate();
}

void MPULib::propagateArrays(int ax, int ay, int az, int gx, int gy, int gz){
  accData[0] = ax;
  accData[1] = ay;
  accData[2] = az;
  for(int i = 0; i < 3; ++i)
    gyDataPrev[i] = gyData[i];
  gyData[0] = (double)gx/131.0;
  gyData[1] = (double)gy/131.0;
  gyData[2] = (double)gz/131.0;
}

void MPULib::lowPassFilter(void){
	double Tau = 1/(2*PI*fCut);
  	double alphaF = Tau/(Tau+dT);
  	for(int i = 0; i < 3; ++i){
	    accFData[i] = (1-alphaF)*accData[i]+alphaF*accFData[i];
  	}
}

void MPULib::highPassFilter(void){
  	double Tau = 1/(2*PI*fCut);
  	double alphaF = Tau/(Tau+dT);
  	for(int i = 0; i < 3; ++i){
	    gyFData[i] = (1-alphaF)*gyFData[i]+(1-alphaF)*(gyData[i]-gyDataPrev[i]);
  	}
}

void MPULib::compFilterPitch(void){
  	double accP = atan2(accFData[0], sqrt(accFData[1]*accFData[1] + accFData[2]*accFData[2]))*180/PI;
  	double gyP = gyFData[0]*dT;
	Serial.print("pitch ACC: ");
	Serial.print(accP); Serial.print("\tGY: ");
	Serial.println(pitch+gyP);
  	
	pitch = (1-alpha)*(pitch+gyP)+alpha*accP;
	
	Serial.print("pitchFinal\t");
	Serial.println(pitch);
}

void MPULib::compFilterRoll(void){
	double accR = atan2(accFData[1], sqrt(accFData[0]*accFData[0] + accFData[2]*accFData[2]))*180/PI;
  	double gyR = gyFData[1]*0.01;
	
	Serial.print("Roll ACC: ");
	Serial.print(accR); Serial.print("\tGY: ");
	Serial.println(roll+gyR);
  	
	roll = (1-alpha)*(roll+gyR)+alpha*accR;	

	Serial.print("rollFinal\t");
	Serial.println(roll);
}

void MPULib::findPitchRate(void){
	// pitchRate = (pitch-prevPitch)/(millis()-timeStamp);
	// timeStamp = millis();
	prevPitch = pitch;
}

void MPULib::printRollPitch(void){
	Serial.print("Pitch: ");
  	Serial.print(pitch);
  	Serial.print("\tRoll: ");
  	Serial.println(roll);
  	Serial.print("\tPitchRate: ");
  	Serial.println(pitchRate);
}