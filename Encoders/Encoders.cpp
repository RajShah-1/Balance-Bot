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
	pinMode(A, INPUT);
	pinMode(B, INPUT);

	rotation = 0;
	timeStamp = ((double) micros()) / 1e6;
	lastStateA = digitalRead(A);
}

double Encoders::getPhi() {
	return (2 * M_PI * rotation)/RESOLUTION;
}

double Encoders::getPhiDot() { 
	int rotationPrev, rotationCurr;
	unsigned long currentTime;
	rotationCurr = rotation;
	currentTime = ((double) micros()) / 1e6;
	return (2000 * M_PI * (rotationCurr - rotationPrev)) / (RESOLUTION * (currentTime - timeStamp));
	timeStamp = currentTime;
	rotationPrev = rotationCurr;
}

double Encoders::getX() {
	return getPhi() * RADIUS;
}

double Encoders::getXDot() {
	return getPhiDot() * RADIUS;
}