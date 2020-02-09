#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t gxPrev;
unsigned long timestamp;
double accFx, accFy, accFz, gyFx;
double dT;
MPU6050 accGyro;

void MPULibISR(void){
    dT = (double)(micros()-timeStamp)/1e6;
	timeStamp = micros();
    Serial.print("dT: ");Serial.println(dT);
	accGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accFx = (1-alphaF)*ax+alphaF*accFx;
    accFy = (1-alphaF)*ay+alphaF*accFy;
    accFz = (1-alphaF)*az+alphaF*accFz;
    gyFx = (1-alphaF)*gyFx+(1-alphaF)*(gx-gxPrev);   
    accP = atan2(accFx, sqrt(accFy*accFy + accFz*accFz));
	pitchPrev = pitch;
    pitch = (1-alpha)*(pitch+gyFx*dT)+alpha*accP;
    pitchRate = (pitch-pitchPrev);
}