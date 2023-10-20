#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <NewPing.h>
#include "SensorHAL.h"

class UltrasonicSensor : public SensorHAL {
public:
  UltrasonicSensor(int triggerPin, int echoPin, int maxDistance);
  void initialize() override;
  float read() override;
  void calibrate() override;
  float read_with_temperature_adjustment(float temperature);

private:
  NewPing sonar;
  float filter(int sample_count = 10);
  float calibrate_distance_For_temperature(float raw_distance, float temperature);
};

#endif // ULTRASONIC_SENSOR_H
