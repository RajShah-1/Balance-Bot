/*
	Encoders.h - A Library for Encoders
	Created by Shivam Malviya, Jan 30, 2020.
*/
#ifndef Encoders_h
#define Encoders_h

#include "Arduino.h"

#define RADIUS 0.03
#define RESOLUTION 1080

class Encoders {

    public:
        byte A;
        byte B;
        byte lastStateA;
        byte lastStateB;
        int rotation;

    public:
        Encoders(byte A=0, byte B=0);
        void init();
        float getPhi();
        float getPhiDot();
        float getX();
        float getXDot();
};

#endif