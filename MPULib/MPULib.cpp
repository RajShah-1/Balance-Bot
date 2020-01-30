#include "MPULib.h"


MPULib::MPULib(double fCutVal = 5.0) : fCut(fCutVal), pitch(0.0), roll(0.0){
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
}

double MPULib::getPitch(void) { return pitch; }

double MPULib::getRoll(void) { return roll; }

void MPULib::iterate(void){
	accGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	propagateArrays(ax, ay, az, gx, gy, gz);
  	lowPassFilter();
  	highPassFilter();
  	compFilterPitch();
  	compFilterRoll();
}

void MPULib::propagateArrays(int ax, int ay, int az, int gx, int gy, int gz){
  accData[0] = ax;
  accData[1] = ay;
  accData[2] = az;
  for(int i = 0; i < 3; ++i)
    gyDataPrev[i] = gyData[i];
  gyData[0] = gx;
  gyData[1] = gy;
  gyData[2] = gz;
}

void MPULib::lowPassFilter(void){
	double dT = 0.01;
  	double Tau = 1/(2*PI*fCut);
  	double alpha = Tau/(Tau+dT);
  	for(int i = 0; i < 3; ++i){
	    accFData[i] = (1-alpha)*accData[i]+alpha*accFData[i];
  	}
}

void MPULib::highPassFilter(void){
	double dT = 0.01;
  	double Tau = 1/(2*PI*fCut);
  	double alpha = Tau/(Tau+dT);
  	for(int i = 0; i < 3; ++i){
	    gyFData[i] = (1-alpha)*gyFData[i]+(1-alpha)*(gyData[i]-gyDataPrev[i]);
  	}
}
void MPULib::compFilterPitch(void){
	double alpha = 0.03;
  	double accP = atan2(accFData[0], sqrt(accFData[1]*accFData[1] + accFData[2]*accFData[2]))*180/PI;
  	double gyP = gyFData[0]*0.01;
  	pitch = (1-alpha)*(pitch+gyP)+alpha*accP;
}
void MPULib::compFilterRoll(void){
	double alpha = 0.03;
  	double accR = atan2(accFData[1], sqrt(accFData[0]*accFData[0] + accFData[2]*accFData[2]))*180/PI;
  	double gyR = gyFData[1]*0.01;
  	roll = (1-alpha)*(roll+gyR)+alpha*accR;	
}
void MPULib::printRollPitch(void){
	Serial.print("Pitch: ");
  	Serial.print(pitch);
  	Serial.print("\tRoll: ");
  	Serial.println(roll);
}
