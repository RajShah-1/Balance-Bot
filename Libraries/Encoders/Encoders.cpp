/*
	Encoders.h - A Library for Encoders
	Created by Shivam Malviya, Jan 30, 2020.
*/
#include "Arduino.h"
#include "Encoders.h"

Encoders::Encoders(byte A=0, byte B=0) {
	this->A = A;
	this->B = B;

	init();
}

void Encoders::init() {
	pinMode(A, OUTPUT);
	pinMode(B, OUTPUT);

	rotation = 0;
	lastStateA = digitalRead(A);
	lastStateB = digitalRead(B);    
}

float Encoders::findPhi() {
	return (2 * M_PI * rotation)/RESOLUTION;
}

float Encoders::findPhiDot() {
	int samplingTime = 30; 
	int rotation1, rotation2;
	rotation1 = rotation;
	delay(samplingTime);
	rotation2 = rotation;
	return (2000 * M_PI * (rotation2 - rotation1)) / (RESOLUTION * samplingTime);
}
float Encoders::findX() {
	return findPhi() * RADIUS;
}
float Encoders::findXDot() {
	return findPhiDot() * RADIUS;
}
