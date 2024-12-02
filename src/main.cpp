#include <Arduino.h>

// PWM pins for distance sensors
#define PW_PIN1 25
#define PW_PIN2 26

// Motor pins
#define MOTOR_1 33
#define MOTOR_2 32

// Constants
#define RELIABILITY_THRESHOLD 10 // Distance sensors' reliability threshold (10 cm)
#define SAFE_DISTANCE 150        // Safe distance threshold (150 cm)
#define BUFFER_SIZE 20           // Buffer size for filtering

const uint16_t MAX_DISTANCE_US = 18500; // Maximum pulse width in microseconds
const uint16_t MAX_DISTANCE_CM = 640;   // Maximum valid distance in cm

// Buffers for sensor readings
uint16_t sensor1Readings[BUFFER_SIZE] = {0};
uint16_t sensor2Readings[BUFFER_SIZE] = {0};

// Indices for circular buffers
uint8_t buffer1_index = 0;
uint8_t buffer2_index = 0;

// Variables for stable distances
uint16_t stableDistance1 = 0;
uint16_t stableDistance2 = 0;

// Motor control variable
uint8_t motorMode;

// User-defined functions
uint16_t measureDistance(uint8_t PW_PIN);
void write_to_buffer(uint16_t *buffer, uint16_t newVal, uint8_t *index);
uint16_t weighted_average_filter(uint16_t *buffer);
uint16_t filter_with_threshold(uint16_t current, uint16_t previous);
uint8_t select_motorMode(uint16_t sensor1_value, uint16_t sensor2_value);

void setup() {
  Serial.begin(115200);

  // Configure pins for sensors
  pinMode(PW_PIN1, INPUT);
  pinMode(PW_PIN2, INPUT);

  // Configure PWM for motors
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_1, 0);
  ledcAttachPin(MOTOR_2, 1);

  // Test motors
  ledcWrite(0, 255);
  ledcWrite(1, 255);
  delay(2000);
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  Serial.println("Distance measurement started...");
}

void loop() {
  static bool measureSensor1 = true; // Toggle between sensors

  uint16_t rawDistance = measureSensor1 ? measureDistance(PW_PIN1) : measureDistance(PW_PIN2);

  if (measureSensor1) {
    // Process sensor 1
    write_to_buffer(sensor1Readings, rawDistance, &buffer1_index);
    uint16_t filteredDistance = weighted_average_filter(sensor1Readings);
    stableDistance1 = filter_with_threshold(filteredDistance, stableDistance1);
    Serial.print("Sensor 1 Distance: ");
    Serial.println(stableDistance1);
  } else {
    // Process sensor 2
    write_to_buffer(sensor2Readings, rawDistance, &buffer2_index);
    uint16_t filteredDistance = weighted_average_filter(sensor2Readings);
    stableDistance2 = filter_with_threshold(filteredDistance, stableDistance2);
    Serial.print("Sensor 2 Distance: ");
    Serial.println(stableDistance2);
  }

  // Toggle between sensors
  measureSensor1 = !measureSensor1;

  static uint8_t motor1Intensity = 0;
  static uint8_t motor2Intensity = 0;
  static uint8_t motor1IntensityBoth = 0;
  static uint8_t motor2IntensityBoth = 0;

  // Determine motor mode
  motorMode = select_motorMode(stableDistance1, stableDistance2);

  // Control motors based on motor mode
  switch (motorMode) {
    case 1: // No vibration
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      break;
    case 2: // Vibrate motor 1
      ledcWrite(1, 0);
      motor1Intensity = map(stableDistance1, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
      ledcWrite(0, motor1Intensity);
      break;
    case 3: // Vibrate motor 2
      ledcWrite(0, 0);
      motor2Intensity = map(stableDistance2, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
      ledcWrite(1, motor2Intensity);
      break;
    case 4: // Vibrate both motors
      motor1IntensityBoth = map(stableDistance1, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
      motor2IntensityBoth = map(stableDistance2, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
      ledcWrite(0, motor1IntensityBoth);
      ledcWrite(1, motor2IntensityBoth);
      break;
    default: // No vibration
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      break;
  }

  delay(50); // Delay to avoid sensor interference
}

// Measure distance from ultrasonic sensor
uint16_t measureDistance(uint8_t PW_PIN) {
  uint16_t pulseWidth = pulseIn(PW_PIN, HIGH, MAX_DISTANCE_US);

  if (pulseWidth == 0 || pulseWidth > MAX_DISTANCE_US) {
    return 0; // Invalid reading
  }
  uint16_t distance = pulseWidth / 58; // Convert to cm
  if (distance > MAX_DISTANCE_CM) {
    return 0; // Ignore distances beyond the max range
  }
  return distance;
}

// Write new value to the circular buffer
void write_to_buffer(uint16_t *buffer, uint16_t newVal, uint8_t *index) {
  buffer[*index] = newVal;
  *index = (*index + 1) % BUFFER_SIZE; // Circular buffer logic
}

// Weighted average filter
uint16_t weighted_average_filter(uint16_t *buffer) {
  uint32_t sum = 0;
  uint16_t weight = 1;
  for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
    sum += buffer[i] * weight;
    weight++;
  }
  return sum / (BUFFER_SIZE * (BUFFER_SIZE + 1) / 2);
}

// Discard outliers based on a threshold
uint16_t filter_with_threshold(uint16_t current, uint16_t previous) {
  const uint16_t THRESHOLD = 5; // Allowable fluctuation in cm
  if (abs(current - previous) > THRESHOLD) {
    return previous; // Ignore large jumps
  }
  return current;
}

// Determine motor mode based on sensor distances
uint8_t select_motorMode(uint16_t sensor1_value, uint16_t sensor2_value) {
  if (sensor1_value > SAFE_DISTANCE && sensor2_value > SAFE_DISTANCE) {
    return 1; // No vibration
  } else if (sensor1_value <= SAFE_DISTANCE && sensor1_value > RELIABILITY_THRESHOLD && sensor2_value > SAFE_DISTANCE) {
    return 2; // Vibrate motor 1
  } else if (sensor2_value <= SAFE_DISTANCE && sensor2_value > RELIABILITY_THRESHOLD && sensor1_value > SAFE_DISTANCE) {
    return 3; // Vibrate motor 2
  } else if (sensor1_value <= SAFE_DISTANCE && sensor2_value <= SAFE_DISTANCE) {
    return 4; // Vibrate both motors
  }
  return 1; // Default: No vibration
}
