#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Motor.h>

#define MPUInterruptPIN 19

const byte lM1 = 22;  //M1 - OUT1 - IN1 - 22
const byte lM2 = 23;  //M2 - OUT2 - IN2 - 23
<<<<<<< HEAD
const byte lME = 11;  //ENA - 12
=======
const byte lME = 11;  //ENA - 11
>>>>>>> 32faaf62c06bf465fca6fce00f744c4506d297bb
const byte lMA = 18;  //A - 18
const byte lMB = 17;  //B - 17

const byte rM1 = 30;  //M2 - OUT4 - IN4 - 30
const byte rM2 = 31;  //M1 - OUT3 - IN3 - 31
<<<<<<< HEAD
const byte rME = 12;  //ENB - 13
=======
const byte rME = 12;  //ENB - 12
>>>>>>> 32faaf62c06bf465fca6fce00f744c4506d297bb
const byte rMA = 3;   //A - 2
const byte rMB = 4;   //B - 3

int16_t ax, ay, az;
int16_t gx, gy, gz;

<<<<<<< HEAD
double K[] = {4, 4, 0, 0};
=======
// ORIGINAL double K[] = {-0.0183,   -0.2152,   -2.6887,   -1.3270};
// MYTRIAL-1 
double K[] = {-0.0804,   -0.3282,   -2.0895,   -0.6782};
// PDF TRIAL-1 double K[] = {-11.1366  -16.2444  -52.5420  -12.6611};
>>>>>>> 32faaf62c06bf465fca6fce00f744c4506d297bb

const double fCut = 5.0;
const double alpha   = 0.05;

unsigned long timeStampMPU, currTime, timeStampLoop, delayValLoop;

double accFx, accFy, accFz, gyFx, gxVal, gxPrev;
double accP, gyP, dT, Tau;
double alphaHPF, alphaLPF;
double pitch, pitchPrev, pitchRate; 

<<<<<<< HEAD
Motor leftMotor(lM1, lM2, lME, lMA, lMB);
Motor rightMotor(rM1, rM2, rME, rMA, rMB);
=======
Motor leftMotor(lM1, lM2, lME, lMA, lMB, 20);
Motor rightMotor(rM1, rM2, rME, rMA, rMB, 20);
>>>>>>> 32faaf62c06bf465fca6fce00f744c4506d297bb

MPU6050 mpu;

volatile bool isMPUReady;

void setup(){
  Serial.begin(115200);
//  attachTimerInterruptmpuMPU();
  attachMotorInterrupts();
  initMPUSensor();
}

void loop(){
  timeStampLoop = micros();

  
  rightMotor.forward(36);
  if(isMPUReady){
    readMPUData();
    isMPUReady = false; 
  }
<<<<<<< HEAD
  //lqr();
  Serial.print("Left Motor Speed : ");
  Serial.println(leftMotor.encoder.getPhiDot());
  Serial.print("Motor Motor Speed : ");
  Serial.println(rightMotor.encoder.getPhiDot());
  forward(6);
  Serial.println("\n\n");
=======
  Serial.print("x = ");
  Serial.println(x());
  Serial.print("xDot = ");
  Serial.println(xDot());
  Serial.print("Pitch = ");
  Serial.print(accP);Serial.print("\t");
  Serial.println(pitch);
  Serial.print("pitchRate = ");
  Serial.println(pitchRate);
  double tVal = lqr();
  Serial.println(tVal);
  
  delayValLoop = 2000-timeStampLoop;
  if(delayValLoop > 0){
    delayMicroseconds(delayValLoop);
  }else{
    Serial.println("Nooooo......");
  }
>>>>>>> 32faaf62c06bf465fca6fce00f744c4506d297bb
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
  torque = -(K[0]*x() + K[1]*xDot() + K[2]*pitch*PI/180 + K[3]*pitchRate*PI/180) ;
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
}

void attachTimerInterruptmpuMPU(){
  cli(); //stop interrupts

  // TIMER-4

  //set timer4 interrupt at 5KHz with preScaler = 8
  TCCR4A = 0; // set entire TCCR1A register to 0
  TCCR4B = 0; // same for TCCR1B
  TCNT4 = 0;  //initialize counter value to 0
  
  // set compare match register for 1hz increments
  
  OCR4A = 15624 / 1; // = (16*10^6) / (preScaler*fInterrupt) - 1 (must be <65536) 0.KHz
  //  OCR4A = 15624/1; 1Hz
  // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
     TCCR4B |= (1 << CS12) | (1 << CS10);
  // Use preScaler = 1
//  TCCR4B |= (1 << CS10);
    
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);

  sei(); //allow interrupts
  Serial.println("Timer Interrrupts attached.");
}
