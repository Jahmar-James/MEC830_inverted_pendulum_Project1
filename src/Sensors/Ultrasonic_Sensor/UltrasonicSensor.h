#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <NewPing.h>
#include "HAL/SensorHAL.h"

const int KNOWN_DISTANCE = 50.0;  // cm

class UltrasonicSensor : public SensorHAL {
public:
  UltrasonicSensor(int triggerPin, int echoPin, int maxDistance); // Constructor
  float read() override;
  void calibrate() override;
  float filter() override;
  bool run_unit_tests() override;
  void simulate() override;

private:
  NewPing sonar;
  int triggerPin;
  int echoPin;
  int maxDistance;
  float _filter(int sample_count = 10);
  float read_with_temperature_adjustment(float temperature);
  float calibrate_distance_for_temperature(float raw_distance, float temperature);
  void _calibrate( const float known_distance, const int calibration_readings = 10);
  float calibration_offset = 0.0;
  bool calibrated = false; // TODO: Force calibration before reading is distance is known
};

#endif // ULTRASONIC_SENSOR_H
