/*
	Encoders.h - A Library for Encoders
	Created by Shivam Malviya, Jan 30, 2020.
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "Encoders.h"

#define RESISTANCE 7
#define MOTOR_CONSTANT 0.036956
#define GEAR_RATIO 30

class Motor {

    private:
        byte M1;
        byte M2;
        byte EN;
        
    public:
        Encoders encoder;
    
    public:
        Motor(byte M1, byte M2, byte EN, byte A, byte B);
        void init();
        void stop();
        void forward(int speed);
        void reverse(int speed);
        void generate(double torque);
};

void rightEncoderInterruptA();
void leftEncoderInterruptA();
void attachMotorInterrupts();

extern Motor rightMotor;
extern Motor leftMotor;

#endif