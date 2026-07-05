#define BATTERY_ADC_PIN 3

#define BATTERY_GAIN 8.332

void battery_adcInit() {
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
}

unsigned int GetBatteryVoltage() {
  uint32_t adc_reading = 0;
  const uint8_t sampleCount = 5;

  for (int i = 0; i < sampleCount; i++) {
    adc_reading += analogReadMilliVolts(BATTERY_ADC_PIN);
  }
  adc_reading /= sampleCount;

  uint16_t mv = adc_reading;

  return (unsigned int)((float)mv * BATTERY_GAIN);
}

float battery_percent(uint16_t mv) {
  struct Point {
    uint16_t v;
    float p;
  };

  static const Point table[] = {
      {9000, 0}, {9600, 5}, {10000, 10}, {10400, 20}, {10800, 30}, {11000, 40}, {11200, 50}, {11400, 60}, {11600, 70}, {11800, 80}, {12000, 90}, {12400, 95}, {12600, 100}};

  constexpr int n = sizeof(table) / sizeof(table[0]);

  if (mv <= table[0].v) return table[0].p;
  if (mv >= table[n - 1].v) return table[n - 1].p;

  for (int i = 0; i < n - 1; ++i) {
    uint16_t v0 = table[i].v;
    uint16_t v1 = table[i + 1].v;
    float p0 = table[i].p;
    float p1 = table[i + 1].p;

    if (mv <= v1) {
      float t = float(mv - v0) / float(v1 - v0);
      return p0 + t * (p1 - p0);
    }
  }

  return 0.0f;
}

float battery_percent_filter(float new_percent) {
  static float filtered = -1.0f;
  const float alpha = 0.05f;
  if (filtered < 0.0f) {
    filtered = new_percent;
  } else {
    filtered = filtered + alpha * (new_percent - filtered);
  }

  return filtered;
}

void displayBatteryVoltage() {
  char buf[32];

  uint32_t mv = GetBatteryVoltage();
  float volts = mv / 1000.0f;

  float percent = battery_percent_filter(battery_percent(mv));

  const int x = 88;
  const int x2 = x - 6;
  const int y1 = 12;
  const int y2 = 26;
  oled_clear_area(x, 0, 40, 32);

  snprintf(buf, sizeof(buf), "%.2fV", volts);
  oled_print(x2, y1, buf);

  snprintf(buf, sizeof(buf), "%.1f%%", percent);
  oled_print(x, y2, buf);

  static bool inverted = false;
  if (percent < 7) {
    inverted = !inverted;
  } else {
    inverted = false;
  }
  oled_mode(inverted);

  oled_update();
}
