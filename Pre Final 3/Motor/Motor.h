/*
	Encoders.h - A Library for Encoders
	Created by Shivam Malviya, Jan 30, 2020.
*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "Encoders.h"
#include "digitalWriteFast.h"

#define RESISTANCE 7
#define MOTOR_CONSTANT 0.036956
#define GEAR_RATIO 30
#define OFFSET_FACTOR 5
#define LEFT_ENCODER_A 18
#define LEFT_ENCODER_B 17
#define RIGHT_ENCODER_A 3
#define RIGHT_ENCODER_B 4

class Motor {

    private:
        byte M1;
        byte M2;
        byte EN;
        
    public:
        Encoders encoder;
        unsigned int MotorOffset;
    
    public:
        Motor(byte M1, byte M2, byte EN, byte A, byte B, unsigned int motorOffset = 0);
        void init();
        void stop();
        void forward(int speed);
        void reverse(int speed);
        void generate(int voltage);
};

void rightEncoderInterruptA();
void leftEncoderInterruptA();
void attachMotorInterrupts();
void forward(double voltage);

extern Motor rightMotor;
extern Motor leftMotor;

#endif