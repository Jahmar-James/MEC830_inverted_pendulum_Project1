#include "Accelerometer.h"

// Constructor
AccelerometerSensor::AccelerometerSensor() 
  : mpu(Wire,COMPLEMENTARY_FILTER_COEFFICIENT,COMPLEMENTARY_FILTER_COEFFICIENT_INV ), kalmanFilter(PROCESS_NOISE, MEASUREMENT_NOISE, ESTIMATION_ERROR)
{
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);  
}

float AccelerometerSensor::read() {
  _readRawAcceleration();
  AccelerometerSensor::filter();
  return filteredAcceleration;
}

void AccelerometerSensor::calibrate() {
   mpu.calcGyroOffsets(true); 
}

float AccelerometerSensor::filter() {
  filteredAcceleration = kalmanFilter.updateEstimate(rawAcceleration);
  return filteredAcceleration;
}

bool AccelerometerSensor::run_unit_tests() {
  // Implement unit tests for the accelerometer if needed
  return true;
}

void AccelerometerSensor::simulate() {
  // Implement simulation logic if needed
}

void AccelerometerSensor::_readRawAcceleration() {
  mpu.update();
  rawAcceleration = mpu.getAccX();
}
