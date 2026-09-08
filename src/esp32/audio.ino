#include <driver/i2s.h>

// https://github.com/pschatzmann/arduino-audio-driver
#include "AudioBoard.h"

/* ---------- I2C ---------- */
#define I2C_SDA 13
#define I2C_SCL 12
#define ES8311_ADDR 0x18

/* ---------- I2S ---------- */
#define I2S_MCK 8
#define I2S_BCK 11
#define I2S_WS 9
#define I2S_DO 46
#define I2S_DI 10

/* ---------- AMP ---------- */

AudioBoard board(AudioDriverES8311, NoPins);

void setup_i2s() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_MSB,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 128,
      .use_apll = true};

  i2s_pin_config_t pin_config = {
      .mck_io_num = 8,
      .bck_io_num = 11,
      .ws_io_num = 9,
      .data_out_num = 46,
      .data_in_num = 10};

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void audio_init() {
  log_i("Starting audio initialization");

  CodecConfig cfg;

  cfg.input_device = ADC_INPUT_LINE1;
  cfg.output_device = DAC_OUTPUT_ALL;
  cfg.i2s.mode = MODE_SLAVE;
  cfg.i2s.fmt = I2S_NORMAL;
  cfg.i2s.rate = RATE_44K;
  cfg.i2s.bits = BIT_LENGTH_16BITS;
  cfg.i2s.channels = CHANNELS2;
  cfg.i2s.signal_type = SIGNAL_DIGITAL;

  log_i("Starting codec");

  bool ok = board.begin(cfg);

  if (ok) {
    log_i("Codec initialized successfully");
  } else {
    log_e("Codec initialization failed");
  }

  //!!!DO NOT RANDOMLY INCREASE THIS!!! - max. 50
  board.setVolume(45);

  delay(100);
  log_i("Calling setup_i2s");
  setup_i2s();
  log_i("Called setup_i2s");

  log_i("Starting audio task");
  xTaskCreatePinnedToCore(
      audioTask,
      "audio",
      4096,
      NULL,
      1,
      NULL,
      1);
  log_i("Started audio task");
}

volatile bool tone_active = false;
volatile float tone_freq = 440.0f;
volatile int samples_remaining = 0;
volatile float tone_phase = 0.0f;

void startTone(float freq, int duration_ms) {
  tone_freq = freq;
  samples_remaining = (44100 * duration_ms) / 1000;
  tone_phase = 0;
  tone_active = true;
}

//!!!DO NOT RANDOMLY INCREASE THIS!!! - max. 3000
#define AMPLITUDE 3000
void audioTask(void *param) {
  const int SAMPLE_RATE = 44100;
  const int BUFFER_SAMPLES = 128;

  int16_t buffer[BUFFER_SAMPLES];
  while (true) {

    if (tone_active) {

      float step = 2.0f * PI * tone_freq / SAMPLE_RATE;

      int n = BUFFER_SAMPLES;
      if (samples_remaining < n) n = samples_remaining;

      for (int i = 0; i < n; i++) {
        float sample_f = sinf(tone_phase) * (AMPLITUDE * 1.0f);

        if (samples_remaining < 50) {
          float fade = (float)samples_remaining / 50.0f;
          sample_f *= fade;
        }

        int16_t sample = (int16_t)sample_f;
        buffer[i] = sample;

        tone_phase += step;
        if (tone_phase >= 2.0f * PI) tone_phase -= 2.0f * PI;

        samples_remaining--;
      }

      size_t written;
      i2s_write(I2S_NUM_0, buffer, n * sizeof(int16_t), &written, portMAX_DELAY);

      if (samples_remaining <= 0) {
        tone_active = false;
      }

    } else {

      memset(buffer, 0, sizeof(buffer));

      size_t written;
      i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &written, portMAX_DELAY);
    }
  }
}

/*
void loop() {
  startTone(660, 150);
  delay(180);
  startTone(880, 150);
  delay(5000);
}
*/
