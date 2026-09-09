#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define MOTOR_A_IN1_PIN 45
#define MOTOR_A_IN2_PIN 48
#define MOTOR_A_PWM_PIN 47

#define MOTOR_A_ENCODE1_PIN 14
#define MOTOR_A_ENCODE2_PIN 21

#define DIR_STOP 0
#define DIR_UP 1
#define DIR_DOWN 2
#define MOTOR_SPEED_MAX 400

#define LEDC_MAX_DUTY 8191
#define LEDC_MOTOR_A_CHANNEL 0
#define LEDC_MOTOR_TIMER 0

volatile int32_t encoder_ticks = 0;
volatile uint8_t encoder_last_state = 0;
volatile uint32_t encoder_invalid_transitions = 0;

static portMUX_TYPE encoder_ticks_mux = portMUX_INITIALIZER_UNLOCKED;

uint16_t motor_a_target_speed = 0;
int32_t motor_a_check_speed = 0;
uint8_t motor_a_dir = DIR_STOP;

struct MotorCommand {
  int32_t value;
  uint32_t received_ms;
};

static QueueHandle_t motor_command_queue = nullptr;

void IRAM_ATTR encoder_ISR() {
  uint8_t A = digitalRead(MOTOR_A_ENCODE1_PIN);
  uint8_t B = digitalRead(MOTOR_A_ENCODE2_PIN);
  uint8_t state = (A << 1) | B;

  portENTER_CRITICAL_ISR(&encoder_ticks_mux);

  uint8_t transition = (encoder_last_state << 2) | state;

  switch (transition) {
    // match the 202602 encoder convention, forward should increase ticks
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoder_ticks--;
      break;

    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      encoder_ticks++;
      break;

    default:
      if (encoder_last_state != state) {
        encoder_invalid_transitions++;
      }
      break;
  }

  encoder_last_state = state;

  portEXIT_CRITICAL_ISR(&encoder_ticks_mux);
}
int32_t read_encoder_ticks() {
  portENTER_CRITICAL(&encoder_ticks_mux);
  int32_t ticks = encoder_ticks;
  portEXIT_CRITICAL(&encoder_ticks_mux);
  return ticks;
}

uint32_t read_encoder_invalid_transitions() {
  portENTER_CRITICAL(&encoder_ticks_mux);
  uint32_t invalid = encoder_invalid_transitions;
  portEXIT_CRITICAL(&encoder_ticks_mux);
  return invalid;
}

void motorA_pwm_init() {
  bool ok = ledcAttachChannel(
      MOTOR_A_PWM_PIN, 2000, 13, LEDC_MOTOR_A_CHANNEL);

  if (!ok) {
    error_loop("Failed to attach motor PWM channel");
  }
}

void motorA_io_init() {
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_A_PWM_PIN, OUTPUT);
}

void motorA_encoder_init() {
  pinMode(MOTOR_A_ENCODE1_PIN, INPUT_PULLUP);
  pinMode(MOTOR_A_ENCODE2_PIN, INPUT_PULLUP);

  uint8_t A = digitalRead(MOTOR_A_ENCODE1_PIN);
  uint8_t B = digitalRead(MOTOR_A_ENCODE2_PIN);

  portENTER_CRITICAL(&encoder_ticks_mux);
  encoder_last_state = (A << 1) | B;
  encoder_ticks = 0;
  encoder_invalid_transitions = 0;
  portEXIT_CRITICAL(&encoder_ticks_mux);

  attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCODE1_PIN), encoder_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCODE2_PIN), encoder_ISR, CHANGE);
}

float GetSpeedInfo() {
  float distance = (float)read_encoder_ticks() /** (1000 / ENCODER_UPDATE_SPEED)*/ * 3.1416f * 65.0f / ((40.0f / 20.0f) * (11.0f * 4.0f) * 9.6f);
  return distance;
}

void Motor_A_SetLevel(uint8_t dir, uint16_t duty) {
  if (duty > LEDC_MAX_DUTY) duty = LEDC_MAX_DUTY;

  if (dir == DIR_UP) {
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  } else if (dir == DIR_DOWN) {
    digitalWrite(MOTOR_A_IN1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
  } else {
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
    duty = 0;
  }

  ledcWriteChannel(LEDC_MOTOR_A_CHANNEL, duty);
}

void Motor_A_Control(uint8_t dir, uint16_t speed) {
  if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;

  motor_a_dir = dir;
  motor_a_target_speed = speed;

  uint16_t duty = map(speed, 0, MOTOR_SPEED_MAX, 0, LEDC_MAX_DUTY);

  Motor_A_SetLevel(dir, duty);
}

void motor_submit_command(int32_t value) {
  MotorCommand command = {constrain(value, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX), millis()};
  if (motor_command_queue != nullptr) {
    xQueueOverwrite(motor_command_queue, &command);
  }
}

void motor_command_task(void *) {
  MotorCommand command = {0, 0};
  int32_t applied = 0;
  for (;;) {
    xQueueReceive(motor_command_queue, &command, pdMS_TO_TICKS(10));
    int32_t value = command.value;
    if ((uint32_t)(millis() - command.received_ms) >= MOTOR_COMMAND_TIMEOUT_MS) {
      command.value = 0;
      value = 0;
    }
    if (value != applied) {
      uint8_t dir = value > 0 ? DIR_DOWN : value < 0 ? DIR_UP : DIR_STOP;
      Motor_A_Control(dir, (uint16_t)(value < 0 ? -value : value));
      applied = value;
    }
  }
}

void motor_init() {
  motorA_io_init();
  motorA_pwm_init();
  motorA_encoder_init();
  Motor_A_Control(DIR_STOP, 0);
  motor_command_queue = xQueueCreate(1, sizeof(MotorCommand));
  if (motor_command_queue == nullptr ||
      xTaskCreatePinnedToCore(motor_command_task, "motor_command", 2048, nullptr,
                              3, nullptr, 0) != pdPASS) {
    error_loop("Failed to start motor command watchdog");
  }
}

void encoder_check_speed() {
  motor_a_check_speed = (int32_t)GetSpeedInfo();
}
