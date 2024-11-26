#include <Arduino.h>

// PWM pins for distance sensors
#define PW_PIN1 25
#define PW_PIN2 26

// Constants
#define BUFFER_SIZE 10
const uint16_t MAX_DISTANCE_US = 18500; // Maximum pulse width in microseconds
const uint16_t MAX_DISTANCE_CM = 640;   // Maximum valid distance in cm

// Global variables
uint16_t sensor1Readings[BUFFER_SIZE] = {0};
uint16_t sensor2Readings[BUFFER_SIZE] = {0};
uint8_t buffer1_index = 0;
uint8_t buffer2_index = 0;

uint16_t previousReading1 = 0;
uint16_t previousReading2 = 0;

uint16_t measureDistance(uint8_t PW_PIN);
void write_to_buffer(uint16_t *buffer, uint16_t newVal, uint8_t *index);
uint16_t moving_average_filter(uint16_t *buffer);

void setup() {
  Serial.begin(115200);

  pinMode(PW_PIN1, INPUT);
  pinMode(PW_PIN2, INPUT);

  Serial.println("Distance measurement started...");
}

void loop() {
  //bruker en sensor om angen for å unngå interferens
  uint16_t rawDistance1 = measureDistance(PW_PIN1);
  delay(50); // delay med god margin til MAX_DISTANCE
  uint16_t rawDistance2 = measureDistance(PW_PIN2);

  //oppdater buffer
  write_to_buffer(sensor1Readings, rawDistance1, &buffer1_index);
  write_to_buffer(sensor2Readings, rawDistance2, &buffer2_index);

  uint16_t filteredDistance1 = moving_average_filter(sensor1Readings);
  uint16_t filteredDistance2 = moving_average_filter(sensor2Readings);

  /*if (filteredDistance1 > 0) {
    Serial.print("Sensor 1 Distance: ");
    Serial.print(filteredDistance1);
    Serial.println(" cm");
  }*/

  if (filteredDistance2 > 0) {
    Serial.print("Sensor 2 Distance: ");
    Serial.print(filteredDistance2);
    Serial.println(" cm");
  }

  delay(10); // Delay for stabilitet
}

uint16_t measureDistance(uint8_t PW_PIN) {
  uint16_t pulseWidth = pulseIn(PW_PIN, HIGH, MAX_DISTANCE_US);

  if (pulseWidth > 0) {
    uint16_t distance = pulseWidth / 58; // puls til cm verdi
    if (distance <= MAX_DISTANCE_CM) {
      return distance;
    }
  }
  return 0; // 0 for ugyldige målinger som kommer fra f.eks støy
}

void write_to_buffer(uint16_t *buffer, uint16_t newVal, uint8_t *index) {
  if (newVal > 0 && newVal < MAX_DISTANCE_CM) { // Sjekk at ny verdi er gyldig
    buffer[*index] = newVal;
    *index = (*index + 1) % BUFFER_SIZE; // inkrementer i den sirkulære
  }
}

uint16_t moving_average_filter(uint16_t *buffer) {
  uint32_t accumulated_sum = 0;
  for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
    accumulated_sum += buffer[i];
  }
  return accumulated_sum / BUFFER_SIZE;
}