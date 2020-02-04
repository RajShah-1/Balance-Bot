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
	timeStampEncoder = ((double) micros()) / 1e6;
	lastStateA = digitalRead(A);
}

double Encoders::getPhi() {
	return (2.0 * M_PI * rotation)/(double) RESOLUTION;
}

double Encoders::getPhiDot() {
	unsigned long dT = pulseIn(A, HIGH, 50000);
	Serial.println(((double) RESOLUTION * dT));
	Serial.println((2 * M_PI) / ((double) RESOLUTION * dT));
	return (2 * M_PI) / ((double) RESOLUTION * dT);
	
}

double Encoders::getX() {
	return getPhi() * RADIUS;
}

double Encoders::getXDot() {
	return getPhiDot() * RADIUS;
}