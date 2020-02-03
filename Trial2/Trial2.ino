#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN 19 // use pin 2 on Arduino Uno & most boards

const double F_CUT = 5.0;
const double alpha = 2;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t gxPrev;
unsigned long timeStamp;
double accFx, accFy, accFz, gyFx, accP;
double dT, alphaLPF, alphaHPF, Tau, pitch, pitchRate, pitchPrev;

volatile bool isMPUReady = false;

void readMPUData(MPU6050 mpu){
    dT = (double)(micros()-timeStamp)/1e6;
    timeStamp = micros();
    gxPrev = gx;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx = gx/131;
    alphaHPF = alphaLPF = Tau/(Tau+dT);
    accFx = (1-alphaLPF)*ax+alphaLPF*accFx;
    accFy = (1-alphaLPF)*ay+alphaLPF*accFy;
    accFz = (1-alphaLPF)*az+alphaLPF*accFz;
    gyFx = (1-alphaHPF)*gyFx+(1-alphaHPF)*(gx-gxPrev);   
    accP = atan2(accFx, sqrt(accFy*accFy + accFz*accFz))*180/PI;
    pitchPrev = pitch;
    Serial.println("DATA::___________");
    Serial.print("gyroPitch: "); Serial.print(pitch+gyFx*dT);
    Serial.print("accPitch: "); Serial.print(accP);
    pitch = (1-alpha)*(pitch+gyFx*dT)+alpha*accP;
    pitchRate = (pitch-pitchPrev)/dT;
}

void MPULibISR(void){
  isMPUReady = true;
}
void setup(){
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  int devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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
}

void loop(){
  if(isMPUReady){
    readMPUData(mpu);
    isMPUReady = false;
  }
}
