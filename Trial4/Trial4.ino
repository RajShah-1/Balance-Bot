#include <MPULib.h>

MPU6050 mpu;
MPULib sensor(mpu, (byte)19);

void setup() {
  // put your setup code here, to run once:
  sensor.initMPU();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.iterate();
  sensor.printStates();
}
