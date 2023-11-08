#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <NewPing.h>
#include "HAL/SensorHAL.h"
#include <EEPROM.h>

// function prototypes
int write_float_to_EEPROM(int start_address, float value);
float read_float_from_EEPROM(int start_address);

class UltrasonicSensor : public SensorHAL {
public:
  UltrasonicSensor(int triggerPin, int echoPin, int maxDistance); // Constructor
  float read() override;
  void calibrate() override;
  float filter() override;
  bool run_unit_tests() override;
  void simulate() override;

  // Constants
  static constexpr float DEFAULT_KNOWN_DISTANCE = 0.0; // 0 cm
  static constexpr int DEFAULT_NUM_READINGS = 10;
  static constexpr float DEFAULT_ALPHA = 0.1;
  static constexpr float DEFAULT_CALIBRATION_OFFSET = 0.0;
  static constexpr float MEAN_ADDRESS = 0;
  static constexpr float STD_DEV_ADDRESS = MEAN_ADDRESS + sizeof(float);
  static constexpr float LOWER_BOUND_ADDRESS = STD_DEV_ADDRESS + sizeof(float);
  static constexpr float UPPER_BOUND_ADDRESS = LOWER_BOUND_ADDRESS + sizeof(float);
  static constexpr float CALIBRATION_OFFSET_ADDRESS = UPPER_BOUND_ADDRESS + sizeof(float);

  NewPing sonar;

private:
  int triggerPin;
  int echoPin;
  int maxDistance;
  
  // Calibration
  float read_with_temperature_adjustment(float temperature);
  float calibrate_distance_for_temperature(float raw_distance, float temperature);
  void _calibrate_sensor(const float known_distance = DEFAULT_KNOWN_DISTANCE, const int num_reading = DEFAULT_NUM_READINGS);
  float calibration_offset = DEFAULT_CALIBRATION_OFFSET;
  float stored_calibration_offset = read_float_from_EEPROM(CALIBRATION_OFFSET_ADDRESS);
  bool calibrated = false; // TODO: Force calibration before reading is distance is known

  // Bayesian update
  float _filter(const int new_readings_count = DEFAULT_NUM_READINGS, const float alpha = DEFAULT_ALPHA);
  float prior_mean = read_float_from_EEPROM(MEAN_ADDRESS);
  float prior_std_dev = read_float_from_EEPROM(STD_DEV_ADDRESS);
  float lower_bound = read_float_from_EEPROM(LOWER_BOUND_ADDRESS);
  float upper_bound = read_float_from_EEPROM(UPPER_BOUND_ADDRESS);
  
};

#endif // ULTRASONIC_SENSOR_H
