/*
 * ES8388 beep via espressif/esp_codec_dev — ESP32-Audio-Kit (A1S V2.2 A541)
 *
 * Pins (internal to module):
 *   I2C  SCL=GPIO32  SDA=GPIO33   (ES8388 ctrl)
 *   I2S  MCLK=GPIO0  BCLK=GPIO27  WS=GPIO25  DOUT=GPIO26
 *   PA   GPIO21 HIGH = NS4150 amp enable
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8388_codec.h"

static const char *TAG = "beep";

/* ---------- pin assignments ---------- */
#define I2C_SCL         GPIO_NUM_32
#define I2C_SDA         GPIO_NUM_33
#define I2S_MCK         GPIO_NUM_0
#define I2S_BCK         GPIO_NUM_27
#define I2S_WS          GPIO_NUM_25
#define I2S_DOUT        GPIO_NUM_26
#define PA_ENABLE_PIN   GPIO_NUM_21

/* ---------- audio ---------- */
#define SAMPLE_RATE     44100
#define TONE_HZ         1000
#define AMPLITUDE       28000
#define BEEP_MS         3000
#define PAUSE_MS        1000
#define BUF_FRAMES      256

static i2s_chan_handle_t      s_i2s_tx;
static esp_codec_dev_handle_t s_play_dev;
static int16_t s_buf[BUF_FRAMES * 2];   /* interleaved L, R */
static float   s_phase = 0.0f;

/* ------------------------------------------------------------------ */
/*  Board init                                                          */
/* ------------------------------------------------------------------ */

static void board_init(void)
{
    /* --- I2C bus (ESP-IDF v5 new master API) --- */
    static i2c_master_bus_handle_t s_i2c_bus;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_NUM_0,
        .sda_io_num          = I2C_SDA,
        .scl_io_num          = I2C_SCL,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    /* --- I2S master, Philips 16-bit stereo, MCLK=256×Fs --- */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCK,
            .bclk = I2S_BCK,
            .ws   = I2S_WS,
            .dout = I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx));
    ESP_LOGI(TAG, "I2S started: MCLK=GPIO%d %d Hz 16-bit stereo", I2S_MCK, SAMPLE_RATE);

    vTaskDelay(pdMS_TO_TICKS(100));   /* let MCLK stabilise before codec init */

    /* --- esp_codec_dev: I2S data interface (wraps our tx handle) --- */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .tx_handle = s_i2s_tx,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    assert(data_if);

    /* --- esp_codec_dev: I2C control interface
     *     Address 0x20 = ES8388 7-bit addr 0x10 shifted for this driver  --- */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .addr       = ES8388_CODEC_DEFAULT_ADDR,   /* 0x20 */
        .bus_handle = s_i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(ctrl_if);

    /* --- esp_codec_dev: GPIO interface (used for PA enable pin) --- */
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    assert(gpio_if);

    /* --- ES8388 codec interface --- */
    es8388_codec_cfg_t es8388_cfg = {
        .ctrl_if     = ctrl_if,
        .gpio_if     = gpio_if,
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,   /* codec is I2S slave, ESP32 is master */
        .pa_pin      = PA_ENABLE_PIN,
        .pa_reverted = false,   /* HIGH = amp on */
    };
    const audio_codec_if_t *codec_if = es8388_codec_new(&es8388_cfg);
    assert(codec_if);

    /* --- Codec device (output only) --- */
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = codec_if,
        .data_if  = data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
    };
    s_play_dev = esp_codec_dev_new(&dev_cfg);
    assert(s_play_dev);

    /* --- Open: configure sample rate, word length, channels --- */
    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = SAMPLE_RATE,
        .channel         = 2,
        .bits_per_sample = 16,
    };
    int ret = esp_codec_dev_open(s_play_dev, &fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "esp_codec_dev_open failed: %d", ret);
    }

    /* --- Output volume: 60 out of 100 --- */
    esp_codec_dev_set_out_vol(s_play_dev, 60);

    ESP_LOGI(TAG, "ES8388 ready via esp_codec_dev");
}

/* ------------------------------------------------------------------ */
/*  Beep                                                                */
/* ------------------------------------------------------------------ */

static void play_beep(int duration_ms)
{
    const float phase_inc = 2.0f * (float)M_PI * TONE_HZ / SAMPLE_RATE;
    const int total_frames = SAMPLE_RATE * duration_ms / 1000;
    int sent = 0;

    while (sent < total_frames) {
        int frames = BUF_FRAMES;
        if (sent + frames > total_frames) frames = total_frames - sent;

        for (int i = 0; i < frames; i++) {
            int16_t s = (int16_t)(sinf(s_phase) * AMPLITUDE);
            s_buf[i * 2]     = s;
            s_buf[i * 2 + 1] = s;
            s_phase += phase_inc;
            if (s_phase >= 2.0f * (float)M_PI) s_phase -= 2.0f * (float)M_PI;
        }

        esp_codec_dev_write(s_play_dev, s_buf, frames * 4 /* bytes */);
        sent += frames;
    }
    ESP_LOGI(TAG, "Beep done: %d frames", sent);
}

/* ------------------------------------------------------------------ */
/*  app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-Audio-Kit beep (esp_codec_dev)");
    board_init();

    while (1) {
        ESP_LOGI(TAG, "Beep %d Hz for %d ms", TONE_HZ, BEEP_MS);
        play_beep(BEEP_MS);
        vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
    }
}
