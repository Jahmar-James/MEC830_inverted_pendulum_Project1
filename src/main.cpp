/*
   Inverted Pendulum on a Cart
   Author: Jahmar James
   Date: 10th October 2023
   Description: Controls a cart to balance an inverted pendulum
*/

#include <Arduino.h>
#include "Sensors/Ultrasonic_Sensor/UltrasonicSensor.h"

// Pins
const int TRIGGER_PIN = 2;
const int ECHO_PIN = 3;
const int MAX_DISTANCE = 200;
UltrasonicSensor base_distance_sensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


void setup() {
  

}


void loop() {
  // put your main code here, to run repeatedly:
}
