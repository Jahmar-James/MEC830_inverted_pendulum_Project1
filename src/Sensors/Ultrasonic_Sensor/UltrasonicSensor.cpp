#include <NewPing.h>
#include "SensorHAL.h"

class UltrasonicSensor : public SensorHAL {
public:
    UltrasonicSensor(int triggerPin, int echoPin, int maxDistance) : sonar(triggerPin, echoPin, maxDistance) {}
 
    void initialize() override {}
    float read() override { return (filter() + calibration_offset) ;}
    float read_with_temperature_adjustment(float temperature) {
    float filtered_distance = filter();
        return ( calibrate_distance_for_temperature(filtered_distance, temperature) + calibration_offset);}
    void calibrate() override {}

private:
    NewPing sonar;
    float filter(int sample_count = 10);
    float calibrate_distance_for_temperature(float raw_distance, float temperature = 20.0); // 20.0 is room temperature
    float calibration_offset = 0.0;
    bool calibrated = false; // TODO: Force calibration before reading is distance is known
};


// Method Implementations
float UltrasonicSensor::filter(int sample_count) {
  float sum = 0;
  for (int i = 0; i < sample_count; ++i) {
    float sample = sonar.ping_cm();
    sum += sample;
  }
  return sum / sample_count;
}

float UltrasonicSensor::calibrate_distance_for_temperature(float raw_distance, float temperature) {
  float speed_of_sound_at_20C = 343.0;
  float speed_of_sound = 331.4 + 0.6 * temperature;
  float calibration_factor = speed_of_sound / speed_of_sound_at_20C;
  return raw_distance * calibration_factor;
}

void UltrasonicSensor::calibrate( const float known_distance, const int calibration_readings) {
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

