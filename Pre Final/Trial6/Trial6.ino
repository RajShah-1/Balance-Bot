#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Motor.h>

#define MPUInterruptPIN 19 // use pin 2 on Arduino Uno & most boards

const byte lM1 = 22;  //M1 - OUT1 - IN1 - 22
const byte lM2 = 23;  //M2 - OUT2 - IN2 - 23
const byte lME = A6;  //ENA - A0
const byte lMA = 18;  //A - 18
const byte lMB = 17;  //B - 17

const byte rM1 = 30;  //M2 - OUT4 - IN4 - 30
const byte rM2 = 31;  //M1 - OUT3 - IN3 - 31
const byte rME = A7;  //ENB - A1
const byte rMA = 3;   //A - 2
const byte rMB = 4;   //B - 3

const double F_CUT = 5.0;
const double alpha = 0.05;
//double K[] = {-1, -1.5816, -5.4143, -1.2476};
double K[] = {  -10.0000,   -25.0662 , -147.7354 ,  -35.2567};
MPU6050 mpu;

Motor leftMotor(lM1, lM2, lME, lMA, lMB);
Motor rightMotor(rM1, rM2, rME, rMA, rMB);

int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long timeStamp;
double accFx, accFy, accFz, gyFx, accP, gyP, gxVal, gxPrev;
double dT, alphaLPF, alphaHPF, Tau, pitch, pitchRate, pitchPrev;

volatile bool isMPUReady = false;

void readMPUData(MPU6050 mpu){
    dT = (double)(micros()-timeStamp)/1e6;
    timeStamp = micros();
    gxPrev = gxVal;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gxVal = (double)gx/131.0;
    alphaHPF = alphaLPF = Tau/(Tau+dT);
    gyFx = (1-alphaHPF)*(gxVal-gxPrev)+(1-alphaHPF)*gyFx;
    accFx = alphaLPF*accFx+(1-alphaLPF)*ax;
    accFy = alphaLPF*accFy+(1-alphaLPF)*ay;
    accFz = alphaLPF*accFz+(1-alphaLPF)*az;
    accP = -atan(accFy/abs(accFz))*180/PI;

    gyP = dT*gyFx;

    pitchPrev = pitch;
    pitch = (1-alpha)*(gyP+pitch)+alpha*accP;
//    pitch = -pitch;
    pitchRate = (pitch-pitchPrev)/dT;
}

void MPULibISR(void){
  isMPUReady = true;
}

void setup(){
  initAll();
}

void initMPUSensor(){
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(MPUInterruptPIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  Serial.println(F("Initializing DMP..."));
  int devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-901);
  mpu.setYGyroOffset(44);
  mpu.setZGyroOffset(19);
  
  mpu.setXAccelOffset(-3991);
  mpu.setYAccelOffset(-1309);
  mpu.setZAccelOffset(4911);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0){
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(MPUInterruptPIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(MPUInterruptPIN), MPULibISR, RISING);
    int mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  Tau = 1/(2*PI*F_CUT);
  accP = gyP = pitch = accFx = accFy = accFz = gxPrev = gyFx = 0;
}

void loop(){
  if(isMPUReady){
    readMPUData(mpu);
    isMPUReady = false;
  }
//  Serial.print("x = ");
//  Serial.println(x());
//  Serial.print("xDot = ");
//  Serial.println(xDot());
  Serial.print("accPitch = ");
  Serial.print(accP);
  Serial.print("\tgyPitch = ");
  Serial.println(pitch+gyP);
  Serial.print("Pitch = ");
  Serial.println(pitch);
  
//  Serial.print("pitchRate = ");
//  Serial.println(pitchRate);
//  Serial.println("");
//  double tVal = lqr();
//  Serial.print(1e4*tVal);
//  Serial.print(" ");
//  Serial.print("Torque = ");
//  Serial.println(x()*1e4);
  delay(10);  
}


void initAll() {
  Serial.begin(115200);
  attachMotorInterrupts();
  initMPUSensor();
}

double x() {
  return (leftMotor.encoder.getX() + rightMotor.encoder.getX()) / 2;
}

double xDot() {
  return (leftMotor.encoder.getXDot() + rightMotor.encoder.getXDot()) / 2;
}

double lqr() {
  double torque;
  torque = -(K[0]*x() + K[1]*xDot() + K[2]*pitch*PI/180 + K[3]*pitchRate*PI/180) ;
  leftMotor.generate(torque);
  // 0.245-1.69
  rightMotor.generate(torque);
  return torque;
}
