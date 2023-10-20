#ifndef SENSOR_HAL_H
#define SENSOR_HAL_H

class SensorHAL {
public:
  virtual float read() = 0;      // High level Read the sensor
  virtual void calibrate() = 0; // Calibration routine
  virtual float filter() = 0;   // Software Signal conditioning 
  virtual bool run_unit_tests() = 0; // Unit testing (optional)
  virtual void simulate() = 0;   // HIL simulation (optional)
};

#endif // SENSOR_HAL_H
