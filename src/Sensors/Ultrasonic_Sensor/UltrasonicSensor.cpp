#include <NewPing.h>
#include "UltrasonicSensor.h"

// Method Implementations

// Constructor for the UltrasonicSensor class using NewPing library
UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin, int maxDistance)
  : sonar(triggerPin, echoPin, maxDistance), triggerPin(triggerPin), echoPin(echoPin), maxDistance(maxDistance)
{
  // Additional hardware-specific initialization if needed
}

float UltrasonicSensor::read() {
  return filter() + calibration_offset;
}

float UltrasonicSensor::read_with_temperature_adjustment(float temperature) {
  float filtered_distance = filter();
  return ( calibrate_distance_for_temperature(filtered_distance, temperature) + calibration_offset);
}

float UltrasonicSensor::filter() {
  int sample_count = 10;
  return _filter(sample_count);
}

float UltrasonicSensor::_filter(int sample_count) {
  float sum = 0;
  for (int i = 0; i < sample_count; ++i) {
    float sample = sonar.ping_cm();
    sum += sample;
  }
  return sum / sample_count;
}

void UltrasonicSensor::calibrate() {
    int calibration_readings = 10;
  _calibrate(KNOWN_DISTANCE, calibration_readings);
}

void UltrasonicSensor::_calibrate( const float known_distance, const int calibration_readings) {
    float sum = 0;
    for (int i = 0; i < calibration_readings; ++i) {
        float sample = sonar.ping_cm();
        sum += sample;
    }
    float average_reading = sum / calibration_readings;

    // Calculate offset
    calibration_offset = known_distance - average_reading;

    // Set calibrated flag
    calibrated = true;
}

float UltrasonicSensor::calibrate_distance_for_temperature(float raw_distance, float temperature) {
  float speed_of_sound_at_20C = 343.0;
  float speed_of_sound = 331.4 + 0.6 * temperature;
  float calibration_factor = speed_of_sound / speed_of_sound_at_20C;
  return raw_distance * calibration_factor;
}

bool UltrasonicSensor::run_unit_tests() {
  return false;
}

void UltrasonicSensor::simulate() {
  // Simulation logic here
}