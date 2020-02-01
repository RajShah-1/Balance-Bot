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

void Motor::generate(float torque) {
    int volatge;
    voltage = RESISTANCE * torque / (MOTOR_CONSTANT * GEAR_RATIO) + encoder.getPhiDot() * MOTOR_CONSTANT;
    voltage = volatge * 1023 / 12;
    if (voltage > 0) {
        forward(voltage);
    }
    else {
        reverse(-volatge);
    }
}

void rightEncoderInterruptA() { 
    if (rightMotor.encoder.lastStateA != rightMotor.encoder.lastStateB) {
        rightMotor.encoder.rotation++;
    } 
    else {
        rightMotor.encoder.rotation--;  
    }
    rightMotor.encoder.lastStateA = !rightMotor.encoder.lastStateA; 
}

void rightEncoderInterruptB() { 
    if (rightMotor.encoder.lastStateB != rightMotor.encoder.lastStateA) {
        rightMotor.encoder.rotation--;
    } 
    else {
        rightMotor.encoder.rotation++;  
    }
    rightMotor.encoder.lastStateB = !rightMotor.encoder.lastStateB;
}

void leftEncoderInterruptA() { 
    if (leftMotor.encoder.lastStateA != leftMotor.encoder.lastStateB) {
        leftMotor.encoder.rotation++;
    } 
    else {
        leftMotor.encoder.rotation--;  
    }
    leftMotor.encoder.lastStateA = !leftMotor.encoder.lastStateA;
}

void leftEncoderInterruptB() { 
    if (leftMotor.encoder.lastStateB != leftMotor.encoder.lastStateA) {
        leftMotor.encoder.rotation--;
    } 
    else {
        leftMotor.encoder.rotation++;  
    }
    leftMotor.encoder.lastStateB = !leftMotor.encoder.lastStateB;
}
