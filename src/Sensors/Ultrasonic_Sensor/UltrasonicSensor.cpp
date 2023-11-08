#include <NewPing.h>
#include "UltrasonicSensor.h"

// Helper Functions

int write_float_to_EEPROM(int start_address, float value) {
  byte* p = (byte*)(void*)&value;
  for (size_t i = 0; i < sizeof(value); i++)
    EEPROM.write(start_address + i, *p++);

  return start_address + sizeof(value);
}

float read_float_from_EEPROM(int start_address) {
  float value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (size_t i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(start_address + i);

  return value;
}

void bayesian_update(float prior_mean, float prior_std_dev, float new_readings[], int new_readings_count, float alpha, float &updated_mean, float &updated_std_dev) {
  /*
 * Function to perform a simplified Bayesian update on sensor readings.
 * Assumptions: 
 * 1. The sensor readings are normally (Gaussian) distributed.
 * 2. The mean and standard deviation are sufficient statistics to represent this distribution.
 */

  float new_mean = (1 - alpha) * prior_mean;
  float sum_squared_diff = (1 - alpha) * prior_std_dev * prior_std_dev;

  // Update mean and variance based on new readings
  for (int i = 0; i < new_readings_count; ++i) {
    new_mean += alpha * new_readings[i] / new_readings_count;
    sum_squared_diff += alpha * pow(new_readings[i] - new_mean, 2) / new_readings_count;
  }

  float new_std_dev = sqrt(sum_squared_diff);

  // Assign updated values to output parameters
  updated_mean = new_mean;
  updated_std_dev = new_std_dev;
}

// Object Method Implementations

// Constructor for the UltrasonicSensor class using NewPing library
UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin, int maxDistance)
  : sonar(triggerPin, echoPin, maxDistance), triggerPin(triggerPin), echoPin(echoPin), maxDistance(maxDistance)
{
  // Additional hardware-specific initialization if needed
}

float UltrasonicSensor::read() {
  return sonar.ping_cm();
  // return filter() + calibration_offset;
}

float UltrasonicSensor::read_with_temperature_adjustment(float temperature) {
  float filtered_distance = filter();
  return ( calibrate_distance_for_temperature(filtered_distance, temperature) + calibration_offset);
}

float UltrasonicSensor::filter() {
   return _filter(); 
}

float UltrasonicSensor::_filter(const int new_readings_count,const float alpha) {
  
  float new_readings[new_readings_count];
  float sum = 0;
  float sum_squared_diff = 0;
  for (int i = 0; i < new_readings_count; ++i) {
    delay(50);
    new_readings[i] = sonar.ping_cm();
    sum += new_readings[i];
  }
  float updated_mean = sum / new_readings_count;
  for (int i = 0; i < new_readings_count; ++i) { sum_squared_diff += pow(new_readings[i]- updated_mean, 2);}
  float  updated_std_dev = sqrt(sum_squared_diff/ (float)(new_readings_count - 1));
  bayesian_update(prior_mean, prior_std_dev, new_readings, new_readings_count, alpha, updated_mean, updated_std_dev);
  
  // Update prior and bounds
  prior_mean = updated_mean;
  prior_std_dev = updated_std_dev;

  float margin_of_error = 1.960 * updated_mean / sqrt((float)new_readings_count);
  lower_bound = updated_mean - margin_of_error;
  upper_bound = updated_mean + margin_of_error;

  if ( updated_mean < lower_bound || updated_mean > upper_bound) {
    Serial.println("Out of bounds");
    return _filter(new_readings_count, alpha);
  }else{
  return updated_mean;
  }
}

void UltrasonicSensor::calibrate() {
  _calibrate_sensor();
}

void UltrasonicSensor::_calibrate_sensor(const float known_distance, const int num_reading){
  //Defualt num of reading 50 
  //IF the know value is not provide it will not create a calibrlibration offset
  float readings[num_reading];

  float sum = 0;
  for (int i = 0; i < num_reading; ++i) {
    readings[i] = sonar.ping_cm();
    sum += readings[i];}
  
  float mean = sum / num_reading;

  float sum_sqaured_diff = 0;
  for (int i = 0; i < num_reading; ++i) {
    sum_sqaured_diff += pow(readings[i]- mean, 2); 
}
  float std_s = sqrt(sum_sqaured_diff/ (float)(num_reading - 1));
  float calibration_offset = known_distance - mean;

  const float z_value = 1.960; //95% confidence interval
  const float margin_of_error = z_value * std_s / sqrt((float)num_reading);
  const float lower_bound = mean - margin_of_error;
  const float upper_bound = mean + margin_of_error;

  // write to eeprom 
  const float starting_address = 0;
  const float std_address = write_float_to_EEPROM(starting_address, mean);
  const float lower_bound_address = write_float_to_EEPROM(std_address, std_s);
  const float upper_bound_address = write_float_to_EEPROM(lower_bound_address, lower_bound);
  const float calibration_offset_address = write_float_to_EEPROM(upper_bound_address, upper_bound);
  if (known_distance == 0){
    //do nothing
  } else{
    float calibration_offset = known_distance - mean;
    // write to eeprom
    write_float_to_EEPROM(calibration_offset_address, calibration_offset);
    calibrated = true;
 }
  // Output to console
  Serial.print("Mean: ");
  Serial.println(mean);
  Serial.print("Mean Address: ");
  Serial.println(starting_address);

  Serial.print("Standard Deviation: ");
  Serial.println(std_s);
  Serial.print("Standard Deviation Address: ");
  Serial.println(std_address);

  Serial.print("Lower Bound: ");
  Serial.println(lower_bound);
  Serial.print("Lower Bound Address: ");
  Serial.println(lower_bound_address);

  Serial.print("Upper Bound: ");
  Serial.println(upper_bound);
  Serial.print("Upper Bound Address: ");
  Serial.println(upper_bound_address);

  Serial.print("Calibration Offset: ");
  Serial.println(calibration_offset);
  Serial.print("Calibration Offset Address: ");
  Serial.println(calibration_offset_address);

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
  static float simulated_distance = 50.0;  // Starting distance in cm
  static float obstacle_velocity = 0.1;    // Obstacle moving towards the sensor, in cm per cycle
  static float belief = 50.0;  // Initial belief

  // Constants
  const int new_readings_count = 5;  // Number of simulated readings to generate
  const float alpha = 0.5;  // Weighting factor for Bayesian update

  // Initialize arrays and variables
  float new_readings[new_readings_count];
  float updated_mean = 0;
  float updated_std_dev = 0;

  // Generate simulated data
  for (int i = 0; i < new_readings_count; ++i) {
    float noise = random(-5, 5);  // +/- 5 cm noise
    new_readings[i] = simulated_distance + noise + (obstacle_velocity * i);  // Simulate sensor reading
  }

  // Update simulated distance and belief
  simulated_distance += obstacle_velocity;
  if (simulated_distance > 100 || simulated_distance < 10) {
    obstacle_velocity = -obstacle_velocity;  // Reverse direction
  }

  // Perform Bayesian update
  bayesian_update(belief, belief, new_readings, new_readings_count, alpha, updated_mean, updated_std_dev);
  belief = updated_mean;  // Update the belief with the new mean

  // Output to console
  Serial.print("Simulated Distance: ");
  Serial.println(simulated_distance);
  Serial.print("Filtered Distance (belief): ");
  Serial.println(belief);
}
