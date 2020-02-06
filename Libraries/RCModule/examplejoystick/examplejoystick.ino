#include "RCModule.h"

RCModule rcmodule(9, 10, 11, 12, 13);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  rcmodule.iterate();
  // put your main code here, to run repeatedly:

}
