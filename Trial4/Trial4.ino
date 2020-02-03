#include <MPULib.h>

volatile bool isMPUReady = false;

MPU6050 myMPU;
const byte sensorInterrupt = 19;
MPULib sensor(myMPU, sensorInterrupt);

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  sensor.initMPU();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.iterate();
  sensor.printStates();
}
