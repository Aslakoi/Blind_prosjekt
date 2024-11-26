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