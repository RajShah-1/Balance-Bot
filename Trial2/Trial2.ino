#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN 19 // use pin 2 on Arduino Uno & most boards

const double F_CUT = 5.0;
const double alpha = 0.02;

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

//    Serial.print("dT: ");
//    Serial.print(dT);
//    Serial.print(" gyFx: ");
//    Serial.print(gyFx);
//    Serial.print(" gxPrev: ");
//    Serial.print(gxPrev);
//    Serial.print(" gxVal: ");
//    Serial.println(gxVal);
    
    accFx = alphaLPF*accFx+(1-alphaLPF)*ax;
    accFy = alphaLPF*accFy+(1-alphaLPF)*ay;
    accFz = alphaLPF*accFz+(1-alphaLPF)*az;
    accP = atan(accFy/abs(accFz))*180/PI;
    gyP = -dT*gyFx;
//    Serial.print("accP: ");
//    Serial.println(accP);
//    Serial.print("gyII: ");
//    Serial.println(gyP+pitch);
//    Serial.print(" gyP: ");
//    Serial.println(gyP);
    pitchPrev = pitch;
    pitch = (1-alpha)*(gyP+pitch)+alpha*accP;
//    Serial.print(pitch);Serial.print(" = ");
//    Serial.print(1-alpha);Serial.print("*(");
//    Serial.print(gyP);Serial.print(" + ");
//    Serial.print(pitch);Serial.print(") + ");
//    Serial.print(alpha);Serial.print(" * ");
//    Serial.println(accP);
//    Serial.println();
    pitchRate = (pitch-pitchPrev)/dT;
    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print(" PitchRate: ");
    Serial.println(pitchRate);
}

void MPULibISR(void){
  isMPUReady = true;
}

void setup(){
  Serial.begin(115200);
  initMPUSensor();
}


void initMPUSensor(){
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
//  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available()); // wait for data
//  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
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
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), MPULibISR, RISING);
    int mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
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
}
