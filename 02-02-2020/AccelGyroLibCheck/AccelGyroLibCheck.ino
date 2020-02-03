//#include "I2Cdev.h"
//#include "MPU6050.h"
//#include "Wire.h"
#include "MPULib.h"

const byte intPin = 19;

MPULib sensor( 10.0, 0.15);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  sensor.init(13, 47, -916, -3768, -1314, 1419);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.iterate();
  sensor.printRollPitch();
  Serial.println(millis()/1000.0);
}
