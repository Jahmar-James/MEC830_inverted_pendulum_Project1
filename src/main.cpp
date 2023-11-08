/*
   Inverted Pendulum on a Cart
   Author: Jahmar James
   Date: 10th October 2023
   Description: Controls a cart to balance an inverted pendulum
*/
#include <Arduino.h>
#include "Sensors/Ultrasonic_Sensor/UltrasonicSensor.h"
#include "Sensors/Accelerometer/Accelerometer.h"
#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>
#include <NewPing.h>
#include <PID_v1.h>

// Ultrasonic Sensor
const int TRIGGER_PIN = 10;
const int ECHO_PIN = 11;
const int MAX_DISTANCE = 60;
UltrasonicSensor base_distance_sensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
// NewPing base_distance_sensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Continuous rotation servo
const int SERVO_PIN = 9;
Servo servoMotor; 
const int SERVO_STOP_SIGNAL = 1500;  // 1.5 ms pulse for neutral position

// Best Performance: both pins have interrupt capability
// Arduino Uno	2, 3, LED PIN do not use 13
const int ENCODER_OUT_A = 2;
const int ENCODER_OUT_B = 3;
Encoder Pendulum_Encoder(ENCODER_OUT_A, ENCODER_OUT_B);

// I2C Pins - for IMU
const int SDA_PIN = A4;
const int SCL_PIN = A5;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float Temperature;
AccelerometerSensor base_IMU;

float elapsed_time, current_time, previous_time;

// pudh buttons
const int STOP_BUTTON = 12;
const int START_BUTTON = 13;

double Setpoint, Input, Output;
double Kp = 6, Ki = 0.5, Kd = 0.05;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//functions declarations
void serial_print(float distance, float acceleration, int angle, double pid_output);
void handle_start_button_press();
void handle_stop_button_press();
void control_loop();
void check_boundaries();
void stop_servo();

bool isSystemActive = false;

void setup() {
  Serial.begin(9600);
  
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  
  servoMotor.attach(SERVO_PIN);
  Pendulum_Encoder.write(0);
  Temperature = base_IMU.mpu.getTemp();

  // PULL up buttons
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);

  Serial.println("Calibrating...");  // Handled by the library and hidden in Object Facade
  delay(500);
  previous_time = millis();  //start timer
}


void loop() {
  // Check for start and stop button presses
  handle_start_button_press();
  handle_stop_button_press();

  // Check if the system should be active
  if (isSystemActive) {
    current_time = millis();
    elapsed_time = current_time - previous_time;
    previous_time = current_time;
    Serial.print("Time Elapsed: ");
    Serial.println(elapsed_time);
    control_loop();
  }else {
    stop_servo();
  }

  check_boundaries(); 

}

void control_loop() {;
  AccX = base_IMU.read();
  float distance = base_distance_sensor.read();
  int angle = Pendulum_Encoder.read();
  Input = angle;
  myPID.Compute();

  serial_print(distance, AccX, angle, Output);

  if (Output > 0) {
    servoMotor.writeMicroseconds(SERVO_STOP_SIGNAL + Output);
  } else {
    servoMotor.writeMicroseconds(SERVO_STOP_SIGNAL + Output);
  }
  delay(10);
}

void serial_print(float distance, float acceleration, int angle, double pid_output){
  Serial.print("Distance: ");
  Serial.println(distance);
  Serial.print("Acceleration: ");
  Serial.println(acceleration);
  Serial.print("Encoder count: ");
  Serial.println(angle);
  Serial.print("PID Output: ");
  Serial.println(pid_output);
}

void handle_start_button_press() {
  static unsigned long last_debounce_time = 0; // Last time the output pin was toggled
  const unsigned long debounce_delay = 50;    // the debounce time; increase if the output flickers

  if (digitalRead(START_BUTTON) == LOW && (millis() - last_debounce_time) > debounce_delay) {
    isSystemActive = true;
    last_debounce_time = millis();
  }
}

void handle_stop_button_press() {
  static unsigned long last_debounce_time = 0; 
  const unsigned long debounce_delay = 50;    

 if (digitalRead(STOP_BUTTON) == LOW && (millis() - last_debounce_time) > debounce_delay) {
    isSystemActive = false;
    last_debounce_time = millis();
  }
}

void stop_servo() {
  servoMotor.writeMicroseconds(SERVO_STOP_SIGNAL);
}


/*
Sonar.ping_median(iterations [, max_cm_distance]) - Do multiple pings (default=5),
 discard out of range pings and return median in microseconds. 
 [max_cm_distance] allows you to optionally set a new max distance.
*/

 void check_boundaries() {
  float distance = base_distance_sensor.sonar.ping_median(5);
  if (distance < 2|| distance > 48) {
    stop_servo();
    isSystemActive = false;
    delay(1000);
  }

  // Always print out the system status for debugging
  Serial.print("System Active: ");
  Serial.print(isSystemActive ? "YES" : "NO");
  Serial.print(" Distance: ");
  Serial.println(distance);
}
