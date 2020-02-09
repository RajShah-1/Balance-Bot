#include "I2Cdev.h"
#include "i2c_lib.h"
#include "TimerInterrupts.h"

#include "MPU6050.h"
#include <Motor.h>

#define ACCEL_YOUT_H 0x3D  
#define ACCEL_ZOUT_H 0x3F 
#define GYRO_XOUT_H 0x43

#define MPUInterruptPIN 19

const int MPUAddress = 0x68 << 1; // Device address in which is also included the 8th bit for selecting the mode, read in this case.

int16_t ay, az, gx;
uint8_t tmpBytesArr2[2];

const double fCut = 5.0;
const double alpha   = 0.05;

volatile double accFx, accFy, accFz, gyFx, gxVal, gxPrev;
volatile double accP, gyP, dT, Tau;
volatile double alphaHPF, alphaLPF;
volatile double pitch, pitchPrev, pitchRate; 

MPU6050 mpu;

volatile unsigned long currTime, timeStampMPU;
volatile unsigned long timeStampTimer4, currTimeTimer4, dTTimer4;

volatile bool isMPUReady;

void setup() {
  Serial.begin(115200);
  initMPUSensor();
  initTimer4();
  sei();
}

void loop() {
  Serial.println(pitch);
//  delay(1000);
}

ISR(TIMER4_COMPA_vect){
  currTime = micros();
  dT = (double)(currTime - timeStampMPU)/1e6;
  timeStampMPU = currTime;
  
  check_status(i2c_read_multi_byte(MPUAddress, ACCEL_YOUT_H, 2, tmpBytesArr2));
  ay = ((int16_t)tmpBytesArr2[0] << 8) | tmpBytesArr2[1];
  check_status(i2c_read_multi_byte(MPUAddress, ACCEL_ZOUT_H, 2, tmpBytesArr2));
  az = ((int16_t)tmpBytesArr2[0] << 8) | tmpBytesArr2[1];
  check_status(i2c_read_multi_byte(MPUAddress, GYRO_XOUT_H, 2, tmpBytesArr2));
  gx = ((int16_t)tmpBytesArr2[0] << 8) | tmpBytesArr2[1];

  gxVal = (double)gx/131.0;
  alphaHPF = alphaLPF = Tau/(Tau+dT);
  
  accFy = alphaLPF*accFy+(1-alphaLPF)*ay;
  accFz = alphaLPF*accFz+(1-alphaLPF)*az;
  gyFx = (1-alphaHPF)*(gxVal-gxPrev)+(1-alphaHPF)*gyFx;

  accP = -atan2(accFy, abs(accFz))*180/PI;
  gyP = dT*gyFx;

  pitchPrev = pitch;
  pitch = (1-alpha)*(gyP+pitch)+alpha*accP;
  pitchRate = (pitch-pitchPrev)/dT;
}

void check_status(STAT status){
  if(status != OK){
    Serial.print("ERROR: ");
    Serial.println(status);
  }
}

void MPUSensorISR(void){
  isMPUReady = true;
}

void initMPUSensor(void){
  
  Wire.begin();
//  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  
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
  
  i2c_init();
  Serial.println("I2C Reset");
  Serial.println(TWSR, HEX);
  Tau = 1/(2*PI*fCut);
  accP = gyP = pitch = accFx = accFy = accFz = gxPrev = gyFx = 0;
}
