#define SG90_PIN 15
#define SG90_FREQ 50
#define SG90_RES_BITS 13
#define SG90_CHANNEL 2
#define SG90_MAX_DUTY ((1 << SG90_RES_BITS) - 1)

void sg90_pwm_init() {
  bool ok = ledcAttachChannel(
      SG90_PIN,
      SG90_FREQ,
      SG90_RES_BITS,
      SG90_CHANNEL);
  if (!ok) {
    publish_text("An error occurred: Failed to attach ledc servo channel!");
  }
}

const int32_t SERVO_CENTER = 500;
const int32_t SERVO_MIN = 0;
const int32_t SERVO_MAX = 1000;
int32_t last_servo_pos = 0;
int32_t Ctrl_sg90(int32_t input) {
  input = input + SERVO_CENTER;

  if (input < SERVO_MIN) input = SERVO_MIN;
  if (input > SERVO_MAX) input = SERVO_MAX;

  last_servo_pos = input - SERVO_CENTER;

  const int min_us = 1160;
  const int max_us = 1840;

  int pulse_us = map(input, SERVO_MIN, SERVO_MAX, min_us, max_us);

  uint32_t duty = (uint32_t)SG90_MAX_DUTY * pulse_us / 20000;

  if (!ledcWriteChannel(SG90_CHANNEL, duty)) {
    publish_text("An error occurred: Failed to write ledc servo channel!");
  };

  return pulse_us;
}
