#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define MPUInterruptPIN 19

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

const double fCut = 5.0;
const double alpha = 0.05;

unsigned long timeStampMPU, currTime;

double accFx, accFy, accFz, gyFx, gxVal, gxPrev;
double accP, gyP, dT, Tau;
double alphaHPF, alphaLPF;
double pitch, pitchPrev, pitchRate; 

volatile bool isMPUReady;

void setup(){
  Serial.begin(115200);
  initMPUSensor();
}

void loop(){
  if(isMPUReady){
    readMPUData();
    isMPUReady = false; 
  }
  Serial.print("accP: ");
  Serial.print(accP);
  Serial.print("\tgyP: ");
  Serial.print(pitch+gyP);
  Serial.print("\tPITCH: ");
  Serial.println(pitch);
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
