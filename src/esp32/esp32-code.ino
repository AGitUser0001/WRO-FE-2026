#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32S3)
#pragma message "Building for ESP32-S3: ensure micro_ros_arduino esp32s3 binary exists"
#endif

#include <ETH.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <micro_ros_arduino.h>

#define WDT_TIMEOUT_SEC 5
#define MOTOR_COMMAND_TIMEOUT_MS 400
#ifndef WRO_ROS_DOMAIN_ID
#define WRO_ROS_DOMAIN_ID 255
#endif

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

#include <sstream>
#include <string>

rcl_publisher_t publisher;
rcl_subscription_t subscriber_oled;
rcl_subscription_t subscriber_servo;
rclc_executor_t executor;
rclc_support_t support;
rcl_init_options_t support_init_options;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t subscriber_motor;
rcl_timer_t timer_motor;
std_msgs__msg__Int32 recv_msg_motor;
static uint32_t last_motor_command_ms = 0;
static bool motor_command_active = false;
static bool motor_timeout_stopped = true;

rcl_publisher_t encoder_publisher;
std_msgs__msg__Int32 send_msg_encoder;
#define ENCODER_UPDATE_SPEED 50
std_msgs__msg__Int32 recv_msg_servo;

std_msgs__msg__String recv_msg_oled;

std_msgs__msg__String send_msg;

rcl_subscription_t subscriber_audio;
std_msgs__msg__Int32MultiArray recv_msg_audio;

rcl_timer_t timer_adc;

#define STRING_BUFFER_SIZE 100

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} agent_state;

#define EXECUTE_EVERY_N_MS(MS, X)              \
  do {                                         \
    static volatile int64_t _t = -1;           \
    if (_t == -1) {                            \
      _t = uxr_millis();                       \
    }                                          \
    if ((int64_t)(uxr_millis() - _t) > (MS)) { \
      X;                                       \
      _t = uxr_millis();                       \
    }                                          \
  } while (0)

static char *buf_send_msg = nullptr;
static char *buf_recv_oled = nullptr;
static int32_t *buf_recv_audio = nullptr;

#define RCINIT(fn)                                           \
  {                                                          \
    rcl_ret_t temp_rc = fn;                                  \
    if ((temp_rc != RCL_RET_OK)) {                           \
      char buf[80];                                          \
      snprintf(buf, sizeof(buf), "RCINIT ERR %s:%d code=%d", \
               __FUNCTION__, __LINE__, temp_rc);             \
      log_e("%s", buf);                                      \
      rcl_reset_error();                                     \
      return false;                                          \
    }                                                        \
  }

#define RCCHECK(fn)                                                                        \
  {                                                                                        \
    rcl_ret_t temp_rc = fn;                                                                \
    if ((temp_rc != RCL_RET_OK)) {                                                         \
      char buf[80];                                                                        \
      snprintf(buf, sizeof(buf), "ERR at %s:%d code=%d", __FUNCTION__, __LINE__, temp_rc); \
      error_loop(buf);                                                                     \
    }                                                                                      \
  }
#define RCSOFTCHECK(fn)                                                                    \
  {                                                                                        \
    rcl_ret_t temp_rc = fn;                                                                \
    if ((temp_rc != RCL_RET_OK)) {                                                         \
      char buf[80];                                                                        \
      snprintf(buf, sizeof(buf), "ERR at %s:%d code=%d", __FUNCTION__, __LINE__, temp_rc); \
      log_e("%s", buf);                                                                    \
    }                                                                                      \
  }

void publish_text(const char *text, bool check = true) {
  size_t len = strlen(text);
  if (len >= send_msg.data.capacity)
    len = send_msg.data.capacity - 1;

  memcpy(send_msg.data.data, text, len);
  send_msg.data.data[len] = '\0';
  send_msg.data.size = len;

  if (check) {
    RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
  } else {
    rcl_publish(&publisher, &send_msg, NULL);
  }
}

void error_loop(const char *msg = "An error occurred") {
  while (1) {
    log_e("%s", msg);
    delay(100);
  }
}

void oled_subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg =
      (const std_msgs__msg__String *)msgin;

  std::string input(msg->data.data, msg->data.size);

  std::stringstream ss(input);
  std::string command;

  while (std::getline(ss, command)) {

    while (!command.empty() && command.front() == ' ')
      command.erase(0, 1);
    while (!command.empty() && command.back() == ' ')
      command.pop_back();

    if (command.empty())
      continue;

    if (command == "clear") {
      oled_clear();
      continue;
    }

    if (command == "update") {
      oled_update();
      continue;
    }

    if (command.rfind("print ", 0) == 0) {
      std::stringstream cs(command);

      std::string cmd;
      int x, y;
      std::string text;

      if (!(cs >> cmd >> x >> y)) {
        publish_text("ERR: print needs 2 numbers");
        continue;
      }

      std::getline(cs, text);

      if (!text.empty() && text[0] == ' ')
        text.erase(0, 1);

      oled_print(x, y, text.c_str());
      continue;
    }

    if (command.rfind("clear ", 0) == 0) {
      std::stringstream cs(command);

      std::string cmd;
      int x, y, w, h;

      if (!(cs >> cmd >> x >> y >> w >> h)) {
        publish_text("ERR: clear needs 4 numbers");
        continue;
      }

      oled_clear_area(x, y, w, h);
      continue;
    }

    if (command.rfind("mode ", 0) == 0) {
      std::stringstream cs(command);

      std::string cmd;
      bool inverted;

      if (!(cs >> cmd >> inverted)) {
        publish_text("ERR: mode needs 1 number that is 0 or 1");
        continue;
      }

      oled_mode(inverted);
      continue;
    }

    char buf[100];
    snprintf(buf, sizeof(buf), "OLED: Unknown cmd: %s", command.c_str());
    publish_text(buf);
  }

  publish_text("OLED: Commands processed");
}

void servo_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  int32_t value = msg->data;
  int32_t set_value = Ctrl_sg90(value);

  char buf[100];
  snprintf(buf, sizeof(buf), "SERVO: Received servo_cmd: %d, set_value=%d\r\n", value, set_value);
  publish_text(buf);
}

extern uint16_t motor_a_target_speed;
extern int32_t motor_a_check_speed;
extern uint8_t motor_a_dir;

#ifndef DIR_STOP
#define DIR_STOP 0
#endif
#ifndef DIR_UP
#define DIR_UP 1
#endif
#ifndef DIR_DOWN
#define DIR_DOWN 2
#endif

void motor_safety_stop() {
  Motor_A_Control(DIR_STOP, 0);
  last_motor_command_ms = 0;
  motor_command_active = false;
  motor_timeout_stopped = true;
}

void motor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    encoder_check_speed();

    uint32_t now_ms = millis();
    bool command_stale =
        motor_command_active &&
        ((uint32_t)(now_ms - last_motor_command_ms) > MOTOR_COMMAND_TIMEOUT_MS);

    if (command_stale && !motor_timeout_stopped) {
      motor_safety_stop();
      publish_text("MOTOR SAFETY: command timeout, forced stop");
    }

    publish_encoder(motor_a_check_speed);
  }
}

void publish_encoder(int32_t data) {
  send_msg_encoder.data = data;
  RCSOFTCHECK(rcl_publish(&encoder_publisher, &send_msg_encoder, NULL));
}

void motor_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  int32_t value = msg->data;

  uint8_t dir;
  uint16_t speed;

  if (value == 0) {
    dir = DIR_STOP;
    speed = 0;
  } else {
    if (value > 0) {
      dir = DIR_DOWN;
    } else {
      dir = DIR_UP;
    }

    int32_t abs_val = (value > 0) ? value : -value;
    speed = (uint16_t)abs_val;
  }

  last_motor_command_ms = millis();
  motor_command_active = (value != 0);
  motor_timeout_stopped = (value == 0);
  Motor_A_Control(dir, speed);

  char buf[100];
  snprintf(buf, sizeof(buf), "MOTOR: Received motor_cmd: %ld, dir=%d, speed=%u\r\n", (long)value, (int)dir, (unsigned int)speed);
  publish_text(buf);
}

void audio_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  if (msg != NULL && msg->data.size >= 2) {
    float freq = (float)msg->data.data[0];
    int duration_ms = (int)msg->data.data[1];

    if (freq < 20.0f) freq = 20.0f;
    if (freq > 1000.0f) freq = 1000.0f;

    if (duration_ms < 0) duration_ms = 0;
    if (duration_ms > 3000) duration_ms = 3000;

    startTone(freq, duration_ms);

    char log_buf[100];
    snprintf(log_buf, sizeof(log_buf), "TONE: %.0fHz, %dms\r\n", freq, duration_ms);
    publish_text(log_buf);
  } else if (msg != NULL) {
    publish_text("AUDIO ERROR: Received array too small!\r\n");
  } else {
    publish_text("AUDIO ERROR: Received NULL msg!\r\n");
  }
}

void adc_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    displayBatteryVoltage();
    publish_text("ADC: Updated voltage and battery display");
  }
}

extern void odom_udp_init(const char *jetson_ip);
extern void odom_udp_task_start();

bool create_entities() {
  motor_safety_stop();

  memset(&publisher, 0, sizeof(publisher));
  memset(&encoder_publisher, 0, sizeof(encoder_publisher));
  memset(&subscriber_oled, 0, sizeof(subscriber_oled));
  memset(&subscriber_servo, 0, sizeof(subscriber_servo));
  memset(&subscriber_motor, 0, sizeof(subscriber_motor));
  memset(&subscriber_audio, 0, sizeof(subscriber_audio));
  memset(&timer_motor, 0, sizeof(timer_motor));
  memset(&timer_adc, 0, sizeof(timer_adc));
  memset(&executor, 0, sizeof(executor));
  memset(&node, 0, sizeof(node));
  memset(&support, 0, sizeof(support));
  support_init_options = rcl_get_zero_initialized_init_options();
  allocator = rcl_get_default_allocator();

  RCINIT(rcl_init_options_init(&support_init_options, allocator));
  RCINIT(rcl_init_options_set_domain_id(&support_init_options, WRO_ROS_DOMAIN_ID));
  RCINIT(rclc_support_init_with_options(
      &support,
      0,
      NULL,
      &support_init_options,
      &allocator));

  RCINIT(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  RCINIT(rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/microROS/status"));

  std_msgs__msg__String__init(&send_msg);
  send_msg.data.data = buf_send_msg;
  send_msg.data.capacity = STRING_BUFFER_SIZE;
  send_msg.data.size = 0;

  RCINIT(rclc_publisher_init_best_effort(
      &encoder_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/microROS/encoder_data"));
  std_msgs__msg__Int32__init(&send_msg_encoder);

  RCINIT(rclc_subscription_init_default(
      &subscriber_oled, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/microROS/oled_display"));
  std_msgs__msg__String__init(&recv_msg_oled);
  recv_msg_oled.data.data = buf_recv_oled;
  recv_msg_oled.data.capacity = 500;
  recv_msg_oled.data.size = 0;

  RCINIT(rclc_subscription_init_best_effort(
      &subscriber_servo, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/microROS/servo_control"));
  std_msgs__msg__Int32__init(&recv_msg_servo);

  RCINIT(rclc_subscription_init_best_effort(
      &subscriber_motor, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/microROS/motor_control"));
  std_msgs__msg__Int32__init(&recv_msg_motor);

  RCINIT(rclc_subscription_init_default(
      &subscriber_audio, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/microROS/audio_play"));
  std_msgs__msg__Int32MultiArray__init(&recv_msg_audio);
  recv_msg_audio.data.data = buf_recv_audio;
  recv_msg_audio.data.capacity = 2;
  recv_msg_audio.data.size = 0;

  RCINIT(rclc_timer_init_default(
      &timer_motor, &support,
      RCL_MS_TO_NS(ENCODER_UPDATE_SPEED),
      motor_timer_callback));

  RCINIT(rclc_timer_init_default(
      &timer_adc, &support,
      RCL_MS_TO_NS(5000),
      adc_timer_callback));

  RCINIT(rclc_executor_init(&executor, &support.context, 6, &allocator));

  RCINIT(rclc_executor_add_subscription(&executor, &subscriber_oled, &recv_msg_oled, &oled_subscription_callback, ON_NEW_DATA));
  RCINIT(rclc_executor_add_subscription(&executor, &subscriber_servo, &recv_msg_servo, &servo_subscription_callback, ON_NEW_DATA));
  RCINIT(rclc_executor_add_subscription(&executor, &subscriber_motor, &recv_msg_motor, &motor_subscription_callback, ON_NEW_DATA));
  RCINIT(rclc_executor_add_subscription(&executor, &subscriber_audio, &recv_msg_audio, &audio_subscription_callback, ON_NEW_DATA));
  RCINIT(rclc_executor_add_timer(&executor, &timer_motor));
  RCINIT(rclc_executor_add_timer(&executor, &timer_adc));

  if (RMW_RET_OK != rmw_uros_sync_session(1000)) {
    log_w("Clock sync failed – timestamps will be based on local ESP32 clock");
  } else {
    log_i("Clock synced with agent.");
  }

  static bool odom_udp_started = false;
  if (!odom_udp_started) {
    odom_udp_init("192.168.10.1");
    odom_udp_task_start();
    odom_udp_started = true;
  }

  publish_text("Setup completed.");
  log_i("micro-ROS entities created. Odom reset to origin.");
  return true;
}

void destroy_entities() {
  motor_safety_stop();
  Ctrl_sg90(0);
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);

  rcl_publisher_fini(&publisher, &node);
  rcl_publisher_fini(&encoder_publisher, &node);

  rcl_subscription_fini(&subscriber_oled, &node);
  rcl_subscription_fini(&subscriber_servo, &node);
  rcl_subscription_fini(&subscriber_motor, &node);
  rcl_subscription_fini(&subscriber_audio, &node);

  rcl_timer_fini(&timer_motor);
  rcl_timer_fini(&timer_adc);

  rcl_node_fini(&node);
  rclc_support_fini(&support);
  rcl_init_options_fini(&support_init_options);

  log_i("micro-ROS entities destroyed.");
}

void setup() {
  Wire.begin(13, 12);
  oled_init();
  sg90_pwm_init();
  motor_init();
  battery_adcInit();
  audio_init();

  ethernet_init("192.168.10.1", "192.168.10.2");
  set_microros_ethernet_udp_transports("192.168.10.1", 8888);

  buf_send_msg = (char *)malloc(STRING_BUFFER_SIZE);
  buf_recv_oled = (char *)malloc(500);
  buf_recv_audio = (int32_t *)malloc(2 * sizeof(int32_t));

  agent_state = WAITING_AGENT;

  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT_SEC * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);
  log_i("Hardware init done. WDT=%ds. Waiting for micro-ROS agent...", WDT_TIMEOUT_SEC);
}

void loop() {
  esp_task_wdt_reset();
  switch (agent_state) {

  case WAITING_AGENT:
    motor_safety_stop();
    EXECUTE_EVERY_N_MS(500, if (!ETH.linkUp()) { log_w("Ethernet link down, waiting..."); } else {
          agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_AVAILABLE : WAITING_AGENT;
          if (agent_state == AGENT_AVAILABLE) log_i("Agent found!"); });
    break;

  case AGENT_AVAILABLE:
    agent_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
    if (agent_state == WAITING_AGENT) {
      destroy_entities();
    }
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(2000,
                       agent_state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1))
                                         ? AGENT_CONNECTED
                                         : AGENT_DISCONNECTED;
                       if (agent_state == AGENT_DISCONNECTED) log_w("Agent lost! Reconnecting..."););
    if (agent_state == AGENT_CONNECTED) {

      rcl_ret_t spin_rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      if (spin_rc != RCL_RET_OK && spin_rc != RCL_RET_TIMEOUT) {
        log_w("executor spin error %d, forcing disconnect", (int)spin_rc);
        agent_state = AGENT_DISCONNECTED;
      }
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    set_microros_ethernet_udp_transports("192.168.10.1", 8888);
    delay(500);
    log_i("Transport re-registered. Entering WAITING_AGENT...");
    agent_state = WAITING_AGENT;
    break;

  default:
    agent_state = WAITING_AGENT;
    break;
  }
}
