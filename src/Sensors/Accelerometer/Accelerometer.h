#ifndef ACCELEROMETER_SENSOR_H
#define ACCELEROMETER_SENSOR_H

#include "HAL/SensorHAL.h"
#include "Wire.h"
#include "SimpleKalmanFilter.h"
#include <MPU6050_tockn.h>

class AccelerometerSensor : public SensorHAL {
public:
    AccelerometerSensor(); // Constructor
    float read() override;         // Reads acceleration (let's assume X-axis for simplicity)
    void calibrate() override;     // Calibrate the accelerometer
    float filter() override;       // Filter the raw accelerometer data
    bool run_unit_tests() override;
    void simulate() override;

    // Constants for Kalman Filter
    static constexpr float PROCESS_NOISE = 2.0;
    static constexpr float MEASUREMENT_NOISE = 2.0;
    static constexpr float ESTIMATION_ERROR = 0.01;

    // Constants for MPU6050 - complementary filter coefficients
    static constexpr float COMPLEMENTARY_FILTER_COEFFICIENT = 0.98;
    static constexpr float COMPLEMENTARY_FILTER_COEFFICIENT_INV = 0.02; // MUST BE 1 
    MPU6050 mpu;

private: 
  SimpleKalmanFilter kalmanFilter;
  float rawAcceleration;
  float filteredAcceleration;
  
  void _readRawAcceleration();  // Read raw acceleration data
  
};

#endif // ACCELEROMETER_SENSOR_H
