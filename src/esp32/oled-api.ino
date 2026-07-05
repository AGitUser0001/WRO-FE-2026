#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

SemaphoreHandle_t i2cMutex;
void oled_init() {
  i2cMutex = xSemaphoreCreateMutex();

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tf);
  oled.sendBuffer();
  xSemaphoreGive(i2cMutex);
}

void oled_clear() {
  oled.clearBuffer();
}

void oled_print(uint8_t x, uint8_t y, const char *text) {
  oled.setFont(u8g2_font_6x12_tf);
  oled.drawStr(x, y, text);
}

void oled_mode(bool invert) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  oled.sendF("c", invert ? 0xA7 : 0xA6);
  xSemaphoreGive(i2cMutex);
}

void oled_update() {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  oled.sendBuffer();
  xSemaphoreGive(i2cMutex);
}

void oled_clear_area(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  oled.setDrawColor(0);
  oled.drawBox(x, y, w, h);
  oled.setDrawColor(1);
}
