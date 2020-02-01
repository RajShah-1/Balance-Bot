#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPULib.h"


MPULib sensor(10);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  sensor.init(13, 47, -916, -3768, -1314, 1419);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.iterate();
  sensor.printRollPitch();
  delay(1);
}
