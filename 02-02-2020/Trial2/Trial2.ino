#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN 19 // use pin 2 on Arduino Uno & most boards

void initSensor(int xGyOffset = 0, int yGyOffset = 0, int zGyOffset = 0,
  int xAccOffset = 0, int yAccOffset = 0, int zAccOffset = 0);


const byte sensorInterrupt = 19;
const double F_CUT = 5.0;
const double alpha = 0.03;

volatile int16_t ax, ay, az;
volatile int16_t gx, gy, gz;
volatile int16_t gxPrev;
volatile unsigned long timeStamp;
volatile double accFx, accFy, accFz, gyFx, accP;
volatile double dT, alphaF, Tau, pitch, pitchRate, pitchPrev;

void MPULibISR(void){
    dT = (double)(micros()-timeStamp)/1e6;
    timeStamp = micros();
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

void setup()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);
  while (!Serial)
    ; // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ; // empty buffer
  while (!Serial.available())
    ; // wait for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  int devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
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
    bool dmpReady = true;

    // get expected DMP packet size for later comparison
    int packetSize = mpu.dmpGetFIFOPacketSize();
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

  // configure LED for output
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(){
  Serial.println(pitch);
}
