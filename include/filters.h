uint16_t moving_average_filter(uint16_t *buffer) {
  uint32_t accumulated_sum = 0;
  for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
    accumulated_sum += buffer[i];
  }
  return accumulated_sum / BUFFER_SIZE;
}

uint16_t exponential_moving_average(uint16_t newValue, float &ema) {
  // Apply EMA formula: EMA = α * newValue + (1 - α) * ema
  ema = ALPHA * newValue + (1 - ALPHA) * ema;
  return (uint16_t)ema; // Return filtered value as an integer
}

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