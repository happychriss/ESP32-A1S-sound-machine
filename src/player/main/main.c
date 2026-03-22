/*
 * ESP32-Audio-Kit MP3 player with WS2812B LED visualizer stub
 *
 * Step 1: SD card → MP3 decode (chmorgan/esp-audio-player) → ES8388 → speaker
 * Step 2 (TODO): PCM tap → FFT → led_strip (WS2812B, 30 LEDs, GPIO22)
 *
 * Pins (internal to module):
 *   I2C   SCL=GPIO32  SDA=GPIO33          (ES8388 ctrl)
 *   I2S   MCLK=GPIO0  BCLK=GPIO27  WS=GPIO25  DOUT=GPIO26
 *   PA    GPIO21 HIGH = NS4150 amp enable
 *   SD    CLK=GPIO14  CMD=GPIO15  D0=GPIO2  D3=GPIO13  (SDMMC slot 1, 1-bit)
 *   LED   GPIO22 = WS2812B data (30 LEDs) — stub, enabled in step 2
 */

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8388_codec.h"

#include "audio_player.h"
#include "driver/i2s_types.h"

/* WS2812B stub — uncomment when step 2 is implemented */
/* #include "led_strip.h" */

static const char *TAG = "player";

/* ---------- pin assignments ---------- */
#define I2C_SCL         GPIO_NUM_32
#define I2C_SDA         GPIO_NUM_33
#define I2S_MCK         GPIO_NUM_0
#define I2S_BCK         GPIO_NUM_27
#define I2S_WS          GPIO_NUM_25
#define I2S_DOUT        GPIO_NUM_26
#define PA_ENABLE_PIN   GPIO_NUM_21

/* SD — SDMMC slot 1, 1-bit mode */
#define SD_CLK          GPIO_NUM_14
#define SD_CMD          GPIO_NUM_15
#define SD_D0           GPIO_NUM_2

/* LED strip — WS2812B */
#define LED_GPIO        GPIO_NUM_22
#define LED_COUNT       30

/* SD mount point */
#define MOUNT_POINT     "/sdcard"

/* ---------- globals ---------- */
static i2s_chan_handle_t        s_i2s_tx;
static esp_codec_dev_handle_t   s_play_dev;
static SemaphoreHandle_t        s_track_done;

/* playlist */
#define MAX_TRACKS      64
static char  s_tracks[MAX_TRACKS][300];
static int   s_track_count = 0;
static int   s_track_idx   = 0;

/* ------------------------------------------------------------------ */
/*  PCM write callback — called by esp-audio-player with decoded PCM   */
/*  This is also the tap point for the LED FFT visualizer (step 2)     */
/* ------------------------------------------------------------------ */

static esp_err_t i2s_write_cb(void *buf, size_t len, size_t *written, uint32_t timeout_ms)
{
    /* TODO step 2: copy buf to a ring buffer for FFT analysis */

    esp_err_t ret = esp_codec_dev_write(s_play_dev, buf, (int)len);
    *written = (ret == ESP_OK) ? len : 0;
    return ret;
}

/* ------------------------------------------------------------------ */
/*  Clock-set callback — called when decoder needs a different rate    */
/* ------------------------------------------------------------------ */

static esp_err_t i2s_clk_set_cb(uint32_t rate, uint32_t bits, i2s_slot_mode_t ch)
{
    int channels = (ch == I2S_SLOT_MODE_STEREO) ? 2 : 1;
    ESP_LOGI(TAG, "clk_set: %lu Hz, %lu bit, %d ch", rate, bits, channels);

    esp_codec_dev_close(s_play_dev);

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = (int)rate,
        .channel         = channels,
        .bits_per_sample = (int)bits,
    };
    int ret = esp_codec_dev_open(s_play_dev, &fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "esp_codec_dev_open failed: %d", ret);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Player event callback                                               */
/* ------------------------------------------------------------------ */

static void player_event_cb(audio_player_cb_ctx_t *ctx)
{
    if (ctx->audio_event == AUDIO_PLAYER_CALLBACK_EVENT_IDLE) {
        xSemaphoreGive(s_track_done);
    }
}

/* ------------------------------------------------------------------ */
/*  SD card init                                                        */
/* ------------------------------------------------------------------ */

static void sd_init(void)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = SD_CLK;
    slot.cmd   = SD_CMD;
    slot.d0    = SD_D0;
    slot.width = 1;   /* 1-bit mode */

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 4096,
    };

    sdmmc_card_t *card;
    ESP_ERROR_CHECK(esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &mount, &card));
    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(TAG, "SD mounted at %s", MOUNT_POINT);
}

/* ------------------------------------------------------------------ */
/*  Scan SD for .mp3 files                                              */
/* ------------------------------------------------------------------ */

static void scan_mp3(void)
{
    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "Cannot open %s", MOUNT_POINT);
        return;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && s_track_count < MAX_TRACKS) {
        size_t len = strlen(ent->d_name);
        if (len < 4) continue;
        /* case-insensitive .mp3 check */
        const char *ext = ent->d_name + len - 4;
        if (ext[0] == '.' &&
            (ext[1] == 'm' || ext[1] == 'M') &&
            (ext[2] == 'p' || ext[2] == 'P') &&
            (ext[3] == '3')) {
            snprintf(s_tracks[s_track_count], sizeof(s_tracks[0]) - 1,
                     "%s/%s", MOUNT_POINT, ent->d_name);
            ESP_LOGI(TAG, "  [%d] %s", s_track_count, s_tracks[s_track_count]);
            s_track_count++;
        }
    }
    closedir(dir);
    ESP_LOGI(TAG, "Found %d MP3 files", s_track_count);
}

/* ------------------------------------------------------------------ */
/*  Audio codec + I2S init                                              */
/* ------------------------------------------------------------------ */

static void audio_init(void)
{
    /* --- I2C bus --- */
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

    /* --- I2S master, 44.1 kHz 16-bit stereo, MCLK=256×Fs --- */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 44100,
            .clk_src        = I2S_CLK_SRC_DEFAULT,   /* NOT APLL — avoids broadband noise */
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
    ESP_LOGI(TAG, "I2S started: MCLK=GPIO%d 44100 Hz 16-bit stereo", I2S_MCK);

    vTaskDelay(pdMS_TO_TICKS(100));   /* let MCLK stabilise before codec init */

    /* --- esp_codec_dev interfaces --- */
    audio_codec_i2s_cfg_t i2s_cfg = { .tx_handle = s_i2s_tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    assert(data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .addr       = ES8388_CODEC_DEFAULT_ADDR,   /* 0x20 = 7-bit 0x10 shifted */
        .bus_handle = s_i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(ctrl_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    assert(gpio_if);

    es8388_codec_cfg_t es8388_cfg = {
        .ctrl_if     = ctrl_if,
        .gpio_if     = gpio_if,
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .pa_pin      = PA_ENABLE_PIN,
        .pa_reverted = false,
    };
    const audio_codec_if_t *codec_if = es8388_codec_new(&es8388_cfg);
    assert(codec_if);

    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = codec_if,
        .data_if  = data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
    };
    s_play_dev = esp_codec_dev_new(&dev_cfg);
    assert(s_play_dev);

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = 44100,
        .channel         = 2,
        .bits_per_sample = 16,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(s_play_dev, &fs));
    esp_codec_dev_set_out_vol(s_play_dev, 70);

    ESP_LOGI(TAG, "ES8388 ready");
}

/* ------------------------------------------------------------------ */
/*  app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-Audio-Kit MP3 player");

    /* SD must be mounted before I2S init to avoid GPIO25/26 conflict */
    sd_init();
    scan_mp3();

    if (s_track_count == 0) {
        ESP_LOGE(TAG, "No MP3 files found on SD card — halting");
        return;
    }

    audio_init();

    s_track_done = xSemaphoreCreateBinary();
    assert(s_track_done);

    /* --- configure esp-audio-player --- */
    audio_player_config_t ap_cfg = {
        .mute_fn        = NULL,
        .write_fn       = i2s_write_cb,
        .clk_set_fn     = i2s_clk_set_cb,
        .priority       = 5,
        .coreID         = 1,           /* audio decode on core 1 */
        .force_stereo   = true,        /* upmix mono → stereo for ES8388 */
    };
    ESP_ERROR_CHECK(audio_player_new(ap_cfg));
    audio_player_callback_register(player_event_cb, NULL);

    ESP_LOGI(TAG, "Starting playlist (%d tracks)", s_track_count);

    while (1) {
        const char *path = s_tracks[s_track_idx];
        ESP_LOGI(TAG, "Playing [%d/%d]: %s", s_track_idx + 1, s_track_count, path);

        FILE *fp = fopen(path, "rb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot open %s", path);
        } else {
            /* fp will be fclose()d by the library when playback ends */
            audio_player_play(fp);
            /* wait for IDLE event (track finished or error) */
            xSemaphoreTake(s_track_done, portMAX_DELAY);
        }

        s_track_idx = (s_track_idx + 1) % s_track_count;
    }
}
