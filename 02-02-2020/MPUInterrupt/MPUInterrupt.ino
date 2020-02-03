#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


void initSensor(int xGyOffset = 0, int yGyOffset = 0, int zGyOffset = 0,
  int xAccOffset = 0, int yAccOffset = 0, int zAccOffset = 0);


MPU6050 mpu;

const byte sensorInterrupt = 19;
const double F_CUT = 5.0;
const double alpha = 0.03;

volatile int16_t ax, ay, az;
volatile int16_t gx, gy, gz;
volatile int16_t gxPrev;
volatile unsigned long timeStamp;
volatile double accFx, accFy, accFz, gyFx, accP;
volatile double dT, alphaF, Tau, pitch, pitchRate, pitchPrev;

int i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gxPrev = 0;
  timeStamp = micros();
  accFx = accFy = accFz = gyFx = accP = 0;
  // dT, alphaF, 
  pitchPrev = 0;
  Tau = 1/(2*PI*F_CUT);
  initSensor();
  attachInterruptAccGy();
}

void loop() {
  Serial.println(pitch);
  delay(100);
}

void MPULibISR(void){
    dT = (double)(micros()-timeStamp)/1e6;
    timeStamp = micros();
//    Serial.print("dT: ");Serial.println(dT);
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    alphaF = Tau/(Tau+dT);
    accFx = (1-alphaF)*ax+alphaF*accFx;
    accFy = (1-alphaF)*ay+alphaF*accFy;
    accFz = (1-alphaF)*az+alphaF*accFz;
    gyFx = (1-alphaF)*gyFx+(1-alphaF)*(gx-gxPrev);   
    accP = atan2(accFx, sqrt(accFy*accFy + accFz*accFz));
    pitchPrev = pitch;
    pitch = (1-alpha)*(pitch+gyFx*dT)+alpha*accP;
    pitchRate = (pitch-pitchPrev)/dT;
}

void attachInterruptAccGy(){
  attachInterrupt(
    digitalPinToInterrupt(sensorInterrupt), 
    MPULibISR,
    RISING
  );
}


void initSensor(int xGyOffset = 0, int yGyOffset = 0, int zGyOffset = 0,
  int xAccOffset = 0, int yAccOffset = 0, int zAccOffset = 0){
  
  Wire.begin();
//  Wire.setClock(400000);
  
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  pinMode(sensorInterrupt, INPUT);

  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  
  // load and configure the DMP
  
  Serial.println("Initializing DMP...");
  int devStatus = mpu.dmpInitialize();
  Serial.println("Updating internal sensor offsets...");
  
  mpu.setXGyroOffset(xGyOffset);
  mpu.setYGyroOffset(yGyOffset);
  mpu.setZGyroOffset(zGyOffset);

  mpu.setXAccelOffset(xAccOffset);
  mpu.setYAccelOffset(yAccOffset);
  mpu.setZAccelOffset(zAccOffset);

  Serial.println("MPULib OFFSETS CHECK::");
  Serial.print(mpu.getXAccelOffset());Serial.print("\t");
  Serial.print(mpu.getYAccelOffset());Serial.print("\t");
  Serial.print(mpu.getZAccelOffset());Serial.print("\t");
  Serial.print(mpu.getXGyroOffset());Serial.print("\t");
  Serial.print(mpu.getYGyroOffset());Serial.print("\t");
  Serial.print(mpu.getZGyroOffset());Serial.print("\t");
  Serial.print("\n");
  Serial.println("CHECK OVER");

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(sensorInterrupt));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(sensorInterrupt), MPULibISR, RISING);
      int mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      
      // get expected DMP packet size for later comparison
//      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  
  timeStamp = micros();
}

//void initSensor(int xGyOffset = 0, int yGyOffset = 0, int zGyOffset = 0,
//  int xAccOffset = 0, int yAccOffset = 0, int zAccOffset = 0){
//  
//  Wire.begin();
//
//  Serial.println("Initializing I2C devices...");
//  mpu.initialize();
//
//  
//    // verify connection
//    Serial.println("Testing device connections...");
//    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//
//  Serial.println("Updating internal sensor offsets...");
//  
//  mpu.setXGyroOffset(xGyOffset);
//  mpu.setYGyroOffset(yGyOffset);
//  mpu.setZGyroOffset(zGyOffset);
//
//  mpu.setXAccelOffset(xAccOffset);
//  mpu.setYAccelOffset(yAccOffset);
//  mpu.setZAccelOffset(zAccOffset);
//
//  Serial.println("MPULib OFFSETS CHECK::");
//  Serial.print(mpu.getXAccelOffset());Serial.print("\t");
//  Serial.print(mpu.getYAccelOffset());Serial.print("\t");
//  Serial.print(mpu.getZAccelOffset());Serial.print("\t");
//  Serial.print(mpu.getXGyroOffset());Serial.print("\t");
//  Serial.print(mpu.getYGyroOffset());Serial.print("\t");
//  Serial.print(mpu.getZGyroOffset());Serial.print("\t");
//  Serial.print("\n");
//  Serial.println("CHECK OVER");
//  
//  
//  timeStamp = micros();
//}
