#include <Arduino.h>

// PWM pins for distance sensors
#define PW_PIN1 25
#define PW_PIN2 26

#define MOTOR_1 33
#define MOTOR_2 32

#define RELIABILITY_THRESHOLD 10 //avstandssensors pålitelighet er funnet ekseperimentelt til å være ca 10cm
#define SAFE_DISTANCE 150 //objekter 150cm fra hodet regnes som sikkert

#define BUFFER_SIZE 15

const uint16_t MAX_DISTANCE_US = 18500; // Maximum pulse width in microseconds
const uint16_t MAX_DISTANCE_CM = 640;   // Maximum valid distance in cm

//For å lagre avlesinger av sensorene i hver sin array
uint16_t sensor1Readings[BUFFER_SIZE] = {0};
uint16_t sensor2Readings[BUFFER_SIZE] = {0};

//starte bufferen på 0
uint8_t buffer1_index = 0;
uint8_t buffer2_index = 0;

// variabler for logikk
uint16_t previousReading1 = 0;
uint16_t previousReading2 = 0;

uint8_t motorMode;

// brukerdefinerte funksjoner 
uint16_t measureDistance(uint8_t PW_PIN);
void write_to_buffer(uint16_t *buffer, uint16_t newVal, uint8_t *index);
uint16_t moving_average_filter(uint16_t *buffer);
uint8_t select_motorMode(uint16_t sensor1_value, uint16_t sensor2_value);

void setup() {
  Serial.begin(115200);

  //Sensorenes PW pinner vil lese den ultrasoniske pulslengden
  pinMode(PW_PIN1, INPUT);
  pinMode(PW_PIN2, INPUT);

  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);

  ledcAttachPin(MOTOR_1, 0);
  ledcAttachPin(MOTOR_2, 1);

  //test for å sjekke at motorene fungerer/er koblet skikkelig
  ledcWrite(0, 255);
  ledcWrite(1, 255);
  delay(2000);
  ledcWrite(0, 0);
  ledcWrite(1, 255);

  Serial.println("Distance measurement started...");
}

void loop() {
  uint16_t filteredDistance1 = 0;
  uint16_t filteredDistance2 = 0;

  static unsigned long lastSensorReadTime = 0;
  const uint8_t sensorInterval = 50;

  static bool measureSensor1 = true;

  /*
  * regner ut verdi for sensor hver for seg, for å unngå interferens, Ved å toggle en bool 
  * bruker ikke blokkerende tidskontroll for kontinuerlig vibrasjon.
  */

  if (millis() - lastSensorReadTime >= sensorInterval) {

    if (measureSensor1) {
      uint16_t rawDistance1 = measureDistance(PW_PIN1);
      write_to_buffer(sensor1Readings, rawDistance1, &buffer1_index);
      filteredDistance1 = moving_average_filter(sensor1Readings);
      Serial.print("Sensor 1 Distance: ");
      Serial.println(filteredDistance1);
    } else {
      uint16_t rawDistance2= measureDistance(PW_PIN2);
      write_to_buffer(sensor2Readings, rawDistance2, &buffer2_index);
      filteredDistance2 = moving_average_filter(sensor2Readings);
      Serial.print("Sensor 2 Distance: ");
      Serial.println(filteredDistance2);
    }

    measureSensor1 = !measureSensor1; // Toggle sensor
  } 

  motorMode = select_motorMode(filteredDistance1, filteredDistance2);

  static uint8_t motor1_Intensity = 0;
  static uint8_t motor2_Intensity = 0;

  //kan endre de forskjellige motortilstandene i hver case
  switch (motorMode)
  {
  case 1: //ingen vibrasjon
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    break;
  case 2: //vibrasjon på en motor
    ledcWrite(1, 0); 
    motor1_Intensity = map(filteredDistance1, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
    ledcWrite(0, motor1_Intensity);
    break;
  case 3: // vibrasjon på en motor
    ledcWrite(0, 0);
    motor2_Intensity = map(filteredDistance2, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
    ledcWrite(1, motor2_Intensity);
    break;
  case 4: //virasjon på begge motorer
    motor1_Intensity = map(filteredDistance1, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
    motor2_Intensity = map(filteredDistance2, RELIABILITY_THRESHOLD, SAFE_DISTANCE, 255, 0);
    ledcWrite(0, motor1_Intensity);
    ledcWrite(1, motor2_Intensity);
    break;
  default: // ingen vib
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    break;
  }
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

uint8_t select_motorMode(uint16_t sensor1_value, uint16_t sensor2_value) {
  if (sensor1_value > SAFE_DISTANCE && sensor2_value > SAFE_DISTANCE) {
    return 1; // No vibration
  } else if (sensor1_value <= SAFE_DISTANCE && sensor1_value > RELIABILITY_THRESHOLD && sensor2_value > SAFE_DISTANCE) {
    return 2; // Motor 1 vibration
  } else if (sensor2_value <= SAFE_DISTANCE && sensor2_value > RELIABILITY_THRESHOLD && sensor1_value > SAFE_DISTANCE) {
    return 3; // Motor 2 vibration
  } else if (sensor1_value <= SAFE_DISTANCE && sensor2_value <= SAFE_DISTANCE) {
    return 4; // Both motors vibrate
  }
  return 1; // Default: no vibration
}
