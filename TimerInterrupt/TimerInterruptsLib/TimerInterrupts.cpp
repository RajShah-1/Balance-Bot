#include "TimerInterrupts.h"

void initTimer4(void){
  // Reset Timer4 control reg 3  
  TCCR4A = 0;

  // Set to CTC mode
  TCCR4B |= (1 << WGM42);
  TCCR4B &= ~(1 << WGM41);
  TCCR4B &= ~(1 << WGM40);
  
  // Set to prescaler of 1
  TCCR4B |= (1 << CS41);
  TCCR4B &= ~(1 << CS40);
  TCCR4B &= ~(1 << CS42);

  // Enable Timer4 compare value
  TCNT4 = t4_initVal;
  OCR4A = t4_compareVal;

  // Enable Timer Compare Match Interrupt
  TIMSK4 = (1 << OCIE4A);
}

void initTimer3(void){
  // Reset Timer3 control reg 3  
  TCCR3A = 0;

  // Set to CTC mode
  TCCR3B |= (1 << WGM32);
  TCCR3B &= ~(1 << WGM31);
  TCCR3B &= ~(1 << WGM30);
  
  // Set to prescaler of 1
  TCCR3B |= (1 << CS30);
  TCCR3B &= ~(1 << CS31);
  TCCR3B &= ~(1 << CS32);

  // Enable Timer3 compare value
  TCNT3 = t3_initVal;
  OCR3A = t3_compareVal;

  // Enable Timer Compare Match Interrupt
  TIMSK3 = (1 << OCIE3A);
}