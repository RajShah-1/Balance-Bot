#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Motor.h>
#include "TimerInterrupts.h"


#define MPUInterruptPIN 19

volatile int16_t ax, ay, az;
volatile int16_t gx, gy, gz;

volatile unsigned long timeStampTimer3, currTimeTimer3, dTTimer3;
volatile unsigned long timeStampTimer4, currTimeTimer4, dTTimer4;

const double fCut = 5.0;
const double alpha   = 0.05;
const double dT = 2e-3;

unsigned long timeStampMPU, currTime, timeStampLoop;
long delayValLoop;

volatile double accFx, accFy, accFz, gyFx, gxVal, gxPrev;
volatile double accP, gyP, Tau;
double alphaHPF, alphaLPF;
volatile double pitch, pitchPrev, pitchRate; 

MPU6050 mpu;

volatile bool isMPUReady;

void setup() {
  Serial.begin(115200);
  initMPUSensor();
  delay(1000);
  initTimer3();
  initTimer4();
  sei();
}

void loop() {
//  cli();
  // put your main code here, to run repeatedly:
  Serial.println(dTTimer3);
//  sei();
  delay(1000);
}

ISR(TIMER3_COMPA_vect){
  currTimeTimer3 = micros();
  dTTimer3 = currTimeTimer3-timeStampTimer3;
  timeStampTimer3 = currTimeTimer3;
}
ISR(TIMER4_COMPA_vect){
  currTimeTimer3 = micros();
  dTTimer3 = currTimeTimer3-timeStampTimer3;
  timeStampTimer3 = currTimeTimer3;
}

//ISR(TIMER4_COMPA_vect){
//  if(isMPUReady){
//    isMPUReady = false; 
//    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    gxVal = (double)gx/131.0;
//    accFx = alphaLPF*accFx+(1-alphaLPF)*ax;
//    accFy = alphaLPF*accFy+(1-alphaLPF)*ay;
//    accFz = alphaLPF*accFz+(1-alphaLPF)*az;
//    // HPF
//    gyFx = (1-alphaHPF)*(gxVal-gxPrev)+(1-alphaHPF)*gyFx;
//  
//    // Raw gy and acc pitch
//    accP = -atan2(accFy, abs(accFz))*180/PI;
//    gyP = dT*gyFx;
//  
//    // Complimentary filter
//    pitchPrev = pitch;
//    pitch = (1-alpha)*(gyP+pitch)+alpha*accP;
//    pitchRate = (pitch-pitchPrev)/dT;
//  }
//}

void MPUSensorISR(void){
  isMPUReady = true;
}

void initMPUSensor(){
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  pinMode(MPUInterruptPIN, INPUT);

  Serial.println(F("Initializing DMP..."));
  int devStatus = mpu.dmpInitialize();

  if(devStatus != 0){
    Serial.println("+++++++++ERROR INITIALIZING DMP++++++++");
  }

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println("Updating internal sensor offsets...");

  mpu.setXGyroOffset(13);
  mpu.setYGyroOffset(47);
  mpu.setZGyroOffset(-916);

  mpu.setXAccelOffset(-3768);
  mpu.setYAccelOffset(-1314);
  mpu.setZAccelOffset(1419);

//  mpu.setXGyroOffset(-893);
//  mpu.setYGyroOffset(36);
//  mpu.setZGyroOffset(16);
//
//  mpu.setXAccelOffset(-3843);
//  mpu.setYAccelOffset(-1289);
//  mpu.setZAccelOffset(4915);

  Serial.print(mpu.getXAccelOffset());Serial.print("\t");
  Serial.print(mpu.getYAccelOffset());Serial.print("\t");
  Serial.print(mpu.getZAccelOffset());Serial.print("\t");
  Serial.print(mpu.getXGyroOffset());Serial.print("\t");
  Serial.print(mpu.getYGyroOffset());Serial.print("\t");
  Serial.print(mpu.getZGyroOffset());Serial.print("\t");Serial.print("\n");

  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(MPUInterruptPIN));
  Serial.println(F(")..."));
  attachInterrupt(digitalPinToInterrupt(MPUInterruptPIN), MPUSensorISR, RISING);
  
  Tau = 1/(2*PI*fCut);
  accP = gyP = pitch = accFx = accFy = accFz = gxPrev = gyFx = 0;
  alphaHPF = alphaLPF = Tau/(Tau+dT);
}
