#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Motor.h>

#define MPUInterruptPIN 19

const byte lM1 = 22;  //M1 - OUT1 - IN1 - 22
const byte lM2 = 23;  //M2 - OUT2 - IN2 - 23
const byte lME = 11;  //ENA - 12
const byte lMA = 18;  //A - 18
const byte lMB = 17;  //B - 17

const byte rM1 = 30;  //M2 - OUT4 - IN4 - 30
const byte rM2 = 31;  //M1 - OUT3 - IN3 - 31
const byte rME = 12;  //ENB - 13
const byte rMA = 3;   //A - 2
const byte rMB = 4;   //B - 3

int16_t ax, ay, az;
int16_t gx, gy, gz;

double K[] = {4, 4, 0, 0};

const double fCut = 5.0;
const double alpha   = 0.05;

unsigned long timeStampMPU, currTime;

double accFx, accFy, accFz, gyFx, gxVal, gxPrev;
double accP, gyP, dT, Tau;
double alphaHPF, alphaLPF;
double pitch, pitchPrev, pitchRate; 

Motor leftMotor(lM1, lM2, lME, lMA, lMB);
Motor rightMotor(rM1, rM2, rME, rMA, rMB);

MPU6050 mpu;

volatile bool isMPUReady;

void setup(){
  Serial.begin(115200);
  attachMotorInterrupts();
  initMPUSensor();
}

void loop(){
  if(isMPUReady){
    readMPUData();
    isMPUReady = false; 
  }
  //lqr();
  Serial.print("Left Motor Speed : ");
  Serial.println(leftMotor.encoder.getPhiDot());
  Serial.print("Motor Motor Speed : ");
  Serial.println(rightMotor.encoder.getPhiDot());
  forward(6);
  Serial.println("\n\n");
}

void MPUSensorISR(void){
  isMPUReady = true;
}

void readMPUData(){
  currTime = micros();
  dT = (double)(currTime - timeStampMPU)/1e6;
  timeStampMPU = currTime;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gxVal = (double)gx/131.0;

  alphaHPF = alphaLPF = Tau/(Tau+dT);
  
  // LPF
  accFx = alphaLPF*accFx+(1-alphaLPF)*ax;
  accFy = alphaLPF*accFy+(1-alphaLPF)*ay;
  accFz = alphaLPF*accFz+(1-alphaLPF)*az;
  // HPF
  gyFx = (1-alphaHPF)*(gxVal-gxPrev)+(1-alphaHPF)*gyFx;

  // Raw gy and acc pitch
  accP = -atan2(accFy, abs(accFz))*180/PI;
  gyP = dT*gyFx;

  // Complimentary filter
  pitchPrev = pitch;
  pitch = (1-alpha)*(gyP+pitch)+alpha*accP;
  pitchRate = (pitch-pitchPrev)/dT;
}

double lqr() {
  double torque, voltage;
  torque = -0.25*(K[0]*x() + K[1]*xDot() + K[2]*pitch*PI/180 + K[3]*pitchRate*PI/180) ;
  voltage = (RESISTANCE*torque/(MOTOR_CONSTANT*GEAR_RATIO)); // + (xDot()/0.03)*GEAR_RATIO*MOTOR_CONSTANT);
  forward(voltage);
  // Range: 0.245-1.69
  return torque;
}

double x() {
  return (leftMotor.encoder.getX() + rightMotor.encoder.getX()) / 2;
}

double xDot() {
  return (leftMotor.encoder.getXDot() + rightMotor.encoder.getXDot()) / 2;
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
  
  mpu.setXGyroOffset(-893);
  mpu.setYGyroOffset(36);
  mpu.setZGyroOffset(16);

  mpu.setXAccelOffset(-3843);
  mpu.setYAccelOffset(-1289);
  mpu.setZAccelOffset(4915);

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
}