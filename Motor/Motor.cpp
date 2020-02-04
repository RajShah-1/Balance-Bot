/*
  Encoders.h - A Library for Encoders
  Created by Shivam Malviya, Jan 30, 2020.
*/
#include "Arduino.h"
#include "Motor.h"

Motor::Motor(byte M1, byte M2, byte EN, byte A, byte B) {
    this->M1 = M1;
    this->M2 = M2;
    this->EN = EN;
    this->encoder.A = A;
    this->encoder.B = B;

    init();
}

void Motor::init() {
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(EN, OUTPUT);
    
    stop(); 
}

void Motor::stop() {
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(EN, 0);
}

void Motor::forward(int speed) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(EN, speed);
}

void Motor::reverse(int speed) {
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(EN, speed);
}

void Motor::generate(double torque) {
    int voltage;
    voltage = RESISTANCE * torque / (MOTOR_CONSTANT * GEAR_RATIO) + encoder.getPhiDot() * MOTOR_CONSTANT;
    voltage = voltage * 1023 / 12;
    if (voltage > 0) {
        forward(voltage);
    }
    else {
        reverse(-voltage);
    }
}

void leftEncoderInterruptA() { 
    leftMotor.encoder.lastStateA = !leftMotor.encoder.lastStateA;
    Serial.print("Hello");
    if (leftMotor.encoder.lastStateA != digitalRead(leftMotor.encoder.B)) {
        leftMotor.encoder.rotation++;
    } 
    else {
        leftMotor.encoder.rotation--;  
    }
}

void rightEncoderInterruptA() { 
    rightMotor.encoder.lastStateA = !rightMotor.encoder.lastStateA;
    if (rightMotor.encoder.lastStateA != digitalRead(rightMotor.encoder.B)) {
        rightMotor.encoder.rotation--;
    } 
    else {
        rightMotor.encoder.rotation++;  
    } 
}

void attachMotorInterrupts() {
  attachInterrupt(
    digitalPinToInterrupt(leftMotor.encoder.A), 
    leftEncoderInterruptA,
    CHANGE
    );

  attachInterrupt(
    digitalPinToInterrupt(rightMotor.encoder.A), 
    rightEncoderInterruptA,
    CHANGE
    );
}