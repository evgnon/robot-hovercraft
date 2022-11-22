

// #define BOARD_LED PB3


// method to avoid outliers since this could lead to problems when operating the hovercraft
float stableRead(uint8_t ir_pin) {
  // take 3 measurements
  float m_1 = analogRead(ir_pin);
  float m_2 = analogRead(ir_pin);
  float m_3 = analogRead(ir_pin);

  // take the min of min
  float measure = min(min(m_1, m_2), m_3);
  return measure;
}

float getDistance(float reading) {
  float distance = 9462/(reading - 16.92);

	if (distance > 150) return 151;
	else if (distance < 20) return 19;
	else return distance;
}