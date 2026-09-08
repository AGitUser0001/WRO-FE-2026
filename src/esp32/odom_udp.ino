#include <ETH.h>

static NetworkUDP odom_udp;
#define ODOM_UDP_PORT 9999
static IPAddress odom_target_ip;

extern int32_t last_servo_pos;
extern int32_t read_encoder_ticks();

void odom_udp_init(const char *jetson_ip) {
  odom_target_ip.fromString(jetson_ip);
  odom_udp.begin(ODOM_UDP_PORT + 1);
  log_i("Odom UDP sender initialized, target %s:%d", jetson_ip, ODOM_UDP_PORT);
}

void odomUdpTask(void *param) {
  uint8_t buf[12];

  while (true) {
    uint32_t now = millis();

    int32_t ticks = read_encoder_ticks();
    int32_t servo = last_servo_pos;

    memcpy(&buf[0], &ticks, 4);
    memcpy(&buf[4], &servo, 4);
    memcpy(&buf[8], &now,   4);

    odom_udp.beginPacket(odom_target_ip, ODOM_UDP_PORT);
    odom_udp.write(buf, 12);
    odom_udp.endPacket();

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void odom_udp_task_start() {
  xTaskCreatePinnedToCore(
      odomUdpTask,
      "odom_udp", 4096, NULL, 2, NULL, 0);
  log_i("Odom UDP task started on core 0 at 20Hz");
}
