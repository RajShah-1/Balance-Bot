#include "I2Cdev.h"
#include "MPU6050.h"

#include "Wire.h"
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long timeStamp;
double gyX,accP,gyP, accX, accY,accZ, dT;


void setup() {
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    timeStamp=micros();
}

void loop() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyX = (double)gx/131.0;
    accX = ax; accY = ay; accZ = az;
    accP = atan(accY/abs(accZ))*180/PI;
    dT = micros()-timeStamp;
    timeStamp = micros();
    gyP += gx*dT/1e6;
    Serial.print("GY: ");
    Serial.println(gyP);
    Serial.print("Acc: ");
    Serial.println(accP);
    delay(1); 
}
