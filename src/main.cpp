/*
   Inverted Pendulum on a Cart
   Author: Jahmar James
   Date: 10th October 2023
   Description: Controls a cart to balance an inverted pendulum
*/

#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}