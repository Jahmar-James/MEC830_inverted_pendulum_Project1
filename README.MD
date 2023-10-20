# Inverted Pendulum Balancing Project

## Overview
This project aims to balance an inverted pendulum on a moving cart. The system is controlled by an Arduino Uno and uses various sensors and actuators.

## Directory Structure
Explained in this document. Refer to respective component folders for specific README files, if available.

## Components
- **HAL**: Hardware Abstraction Layer for sensors.
- **CartPendulum**: Core logic for balancing the pendulum.
- **Sensors**: Individual classes for Rotary Encoder, Accelerometer, and Ultrasonic sensor.
- **MotorControl**: Logic to control the motor for the cart's movement.
- **SerialComm**: Module for serial data communication with a Python-based GUI.
- **DataLogging**: Module for logging important system states for debugging.

## Third-Party Libraries
- NewPing: For ultrasonic sensor.
- Rotary Encoder Library: For reading rotary encoder.
- Servo Motor Library: For controlling the motor.
- Kalman Filter Library: Optional, for sensor fusion.
- PID Library: Optional, for PID control algorithm.