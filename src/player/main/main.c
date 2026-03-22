/*
 * ESP32-Audio-Kit MP3 player + WS2812B LED spectrum analyzer
 *
 * SD card → MP3 decode (chmorgan/esp-audio-player) → ES8388 → speaker
 * LED show (GPIO22, 30 LEDs):
 *   - FFT spectrum: 30 bands, red (bass) → violet (treble)
 *   - Fast attack / slow decay smoothing → natural peak-hold feel
 *   - Beat detection from bass bins → white flash overlay
 *   - Auto-gain adapts to track loudness
 *   - Fades to black when idle
 *
 * Keys (active LOW, internal pull-up):
 *   KEY3  GPIO19 = Previous track
 *   KEY4  GPIO23 = Next track
 *   (KEY1/GPIO36 skipped — input-only, no pull-up)
 *   (KEY2/GPIO13 skipped — SD D3 conflict)
 *
 * Pins (internal to module):
 *   I2C   SCL=GPIO32  SDA=GPIO33          (ES8388 ctrl)
 *   I2S   MCLK=GPIO0  BCLK=GPIO27  WS=GPIO25  DOUT=GPIO26
 *   PA    GPIO21 HIGH = NS4150 amp enable
 *   SD    CLK=GPIO14  CMD=GPIO15  D0=GPIO2   (SDMMC slot 1, 1-bit)
 *   LED   GPIO22 = WS2812B data (30 LEDs, external 5V power)
 */

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
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
#include "led_strip.h"
#include "esp_dsp.h"
#include "driver/gpio.h"

static const char *TAG = "player";

/* ---------- pin assignments ---------- */
#define I2C_SCL         GPIO_NUM_32
#define I2C_SDA         GPIO_NUM_33
#define I2S_MCK         GPIO_NUM_0
#define I2S_BCK         GPIO_NUM_27
#define I2S_WS          GPIO_NUM_25
#define I2S_DOUT        GPIO_NUM_26
#define PA_ENABLE_PIN   GPIO_NUM_21
#define SD_CLK          GPIO_NUM_14
#define SD_CMD          GPIO_NUM_15
#define SD_D0           GPIO_NUM_2
#define LED_GPIO        GPIO_NUM_22
#define LED_COUNT       30
#define MOUNT_POINT     "/sdcard"

/* Keys — active LOW, internal pull-up, 40 ms debounce, 400 ms lockout */
#define KEY_PREV_GPIO   GPIO_NUM_19   /* KEY3 — previous track */
#define KEY_NEXT_GPIO   GPIO_NUM_23   /* KEY4 — next track     */
#define BTN_DEBOUNCE_MS  40
#define BTN_LOCKOUT_MS  400

/* ---------- FFT / LED show parameters ---------- */
#define FFT_SIZE            512
/* stream buffer holds 8 FFT-blocks worth of mono float samples */
#define STREAM_BUF_BYTES    (FFT_SIZE * 8 * sizeof(float))

/* frequency range mapped to the 30 LEDs */
#define FREQ_LO_HZ          40.0f
#define FREQ_HI_HZ          8000.0f

/* smoothing: fast attack so peaks register instantly, slow decay for peak-hold feel */
#define ATTACK_ALPHA        0.65f
#define DECAY_ALPHA         0.12f

/* beat detection */
#define BEAT_BASS_LEDS      5       /* use first N LEDs (bass) */
#define BEAT_RATIO          1.8f    /* spike must exceed rolling avg × this */
#define BEAT_MIN_GAP_MS     200
#define BEAT_FLASH_FRAMES   8

/* ---------- globals ---------- */
static i2s_chan_handle_t        s_i2s_tx;
static esp_codec_dev_handle_t   s_play_dev;
static SemaphoreHandle_t        s_track_done;
static led_strip_handle_t       s_led_strip;
static StreamBufferHandle_t     s_pcm_stream;
static volatile uint32_t        s_sample_rate = 44100;

/* playlist */
#define MAX_TRACKS 64
static char s_tracks[MAX_TRACKS][300];
static int  s_track_count = 0;
static int  s_track_idx   = 0;

/* button → main-loop track override (-1 = normal advance) */
static volatile int s_next_track_override = -1;

/* ------------------------------------------------------------------ */
/*  LED show helpers                                                    */
/* ------------------------------------------------------------------ */

/* h: 0–360, s/v: 0–1  →  r/g/b: 0–255 */
static void hsv_to_rgb(float h, float s, float v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    float c  = v * s;
    float x  = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m  = v - c;
    float r1, g1, b1;
    int   sec = (int)(h / 60.0f) % 6;
    switch (sec) {
        case 0: r1=c; g1=x; b1=0; break;
        case 1: r1=x; g1=c; b1=0; break;
        case 2: r1=0; g1=c; b1=x; break;
        case 3: r1=0; g1=x; b1=c; break;
        case 4: r1=x; g1=0; b1=c; break;
        default: r1=c; g1=0; b1=x; break;
    }
    *r = (uint8_t)((r1 + m) * 255.0f);
    *g = (uint8_t)((g1 + m) * 255.0f);
    *b = (uint8_t)((b1 + m) * 255.0f);
}

/* Build logarithmic frequency→bin map for the current sample rate */
static void init_led_bins(uint32_t sample_rate,
                          int bin_start[], int bin_end[])
{
    float hz_per_bin = (float)sample_rate / FFT_SIZE;
    for (int i = 0; i < LED_COUNT; i++) {
        float f_lo = FREQ_LO_HZ * powf(FREQ_HI_HZ / FREQ_LO_HZ,
                                        (float)i       / LED_COUNT);
        float f_hi = FREQ_LO_HZ * powf(FREQ_HI_HZ / FREQ_LO_HZ,
                                        (float)(i + 1) / LED_COUNT);
        int b_lo = (int)(f_lo / hz_per_bin);
        int b_hi = (int)(f_hi / hz_per_bin) + 1;
        if (b_lo < 1)                 b_lo = 1;             /* skip DC */
        if (b_hi > FFT_SIZE / 2 - 1) b_hi = FFT_SIZE / 2 - 1;
        if (b_hi <= b_lo)             b_hi = b_lo + 1;
        bin_start[i] = b_lo;
        bin_end[i]   = b_hi;
    }
}

/* ------------------------------------------------------------------ */
/*  LED show task — runs on core 0                                      */
/* ------------------------------------------------------------------ */

static void led_task(void *arg)
{
    /* Work buffers in BSS (not on task stack) */
    static float fft_buf[FFT_SIZE * 2];   /* complex interleaved */
    static float pcm_buf[FFT_SIZE];
    static float hann[FFT_SIZE];
    static int   bin_start[LED_COUNT];
    static int   bin_end[LED_COUNT];
    static float display[LED_COUNT];      /* smoothed per-band brightness */

    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    dsps_wind_hann_f32(hann, FFT_SIZE);

    uint32_t cur_rate = s_sample_rate;
    init_led_bins(cur_rate, bin_start, bin_end);

    float    running_max = 1.0f;
    float    bass_avg    = 0.0f;
    int      beat_flash  = 0;
    TickType_t last_beat = 0;
    memset(display, 0, sizeof(display));

    while (1) {
        /* Adapt bin map if sample rate changed mid-playlist */
        uint32_t new_rate = s_sample_rate;
        if (new_rate != cur_rate) {
            cur_rate = new_rate;
            init_led_bins(cur_rate, bin_start, bin_end);
        }

        /* Wait up to 50 ms for a full FFT block of PCM */
        size_t got = xStreamBufferReceive(s_pcm_stream, pcm_buf,
                                          FFT_SIZE * sizeof(float),
                                          pdMS_TO_TICKS(50));

        if (got < FFT_SIZE * sizeof(float)) {
            /* No audio — graceful fade to black */
            for (int i = 0; i < LED_COUNT; i++) {
                display[i] *= 0.85f;
                float    hue = 270.0f * i / (LED_COUNT - 1);
                uint8_t  r, g, b;
                hsv_to_rgb(hue, 1.0f, display[i], &r, &g, &b);
                led_strip_set_pixel(s_led_strip, i, r, g, b);
            }
            led_strip_refresh(s_led_strip);
            continue;
        }

        /* --- FFT --- */
        for (int i = 0; i < FFT_SIZE; i++) {
            fft_buf[i * 2]     = pcm_buf[i] * hann[i];  /* real, windowed */
            fft_buf[i * 2 + 1] = 0.0f;                   /* imag = 0 */
        }
        dsps_fft2r_fc32(fft_buf, FFT_SIZE);
        dsps_bit_rev2r_fc32(fft_buf, FFT_SIZE);

        /* --- Magnitude per LED band (average over bins in band) --- */
        float new_mag[LED_COUNT];
        for (int led = 0; led < LED_COUNT; led++) {
            float sum = 0.0f;
            int   cnt = bin_end[led] - bin_start[led];
            for (int k = bin_start[led]; k < bin_end[led]; k++) {
                float re = fft_buf[k * 2];
                float im = fft_buf[k * 2 + 1];
                sum += sqrtf(re * re + im * im);
            }
            new_mag[led] = cnt > 0 ? sum / cnt : 0.0f;
        }

        /* --- Auto-gain: track running peak, decay slowly --- */
        for (int i = 0; i < LED_COUNT; i++) {
            if (new_mag[i] > running_max) running_max = new_mag[i];
        }
        running_max *= 0.998f;
        if (running_max < 1.0f) running_max = 1.0f;

        /* --- Normalise + perceptual log compression → [0, 1] --- */
        float norm[LED_COUNT];
        for (int i = 0; i < LED_COUNT; i++) {
            float n  = new_mag[i] / running_max;
            norm[i]  = log10f(1.0f + 9.0f * n);   /* 0→0, 1→1 */
        }

        /* --- Asymmetric smoothing + bass energy for beat detection --- */
        float bass_energy = 0.0f;
        for (int i = 0; i < LED_COUNT; i++) {
            float alpha = (norm[i] > display[i]) ? ATTACK_ALPHA : DECAY_ALPHA;
            display[i]  = alpha * norm[i] + (1.0f - alpha) * display[i];
            if (i < BEAT_BASS_LEDS) bass_energy += display[i];
        }
        bass_energy /= BEAT_BASS_LEDS;

        /* --- Beat detection --- */
        bass_avg = 0.05f * bass_energy + 0.95f * bass_avg;
        TickType_t now = xTaskGetTickCount();
        if (bass_energy > bass_avg * BEAT_RATIO &&
            (now - last_beat) > pdMS_TO_TICKS(BEAT_MIN_GAP_MS)) {
            beat_flash = BEAT_FLASH_FRAMES;
            last_beat  = now;
        }

        /* beat_add: 1.0 at flash peak, ramps down to 0 over BEAT_FLASH_FRAMES */
        float beat_add = (beat_flash > 0)
                         ? (float)beat_flash / BEAT_FLASH_FRAMES
                         : 0.0f;
        if (beat_flash > 0) beat_flash--;

        /* --- Render to strip ---
         *  Hue: 0° red (LED 0, bass) → 270° violet (LED 29, treble)
         *  Beat flash: blend each pixel toward white proportional to beat_add
         */
        for (int i = 0; i < LED_COUNT; i++) {
            float v = display[i];
            if (v < 0.0f) v = 0.0f;
            if (v > 1.0f) v = 1.0f;

            float   hue = 270.0f * i / (LED_COUNT - 1);
            uint8_t r, g, b;
            hsv_to_rgb(hue, 1.0f, v, &r, &g, &b);

            if (beat_add > 0.0f) {
                /* blend toward white — stronger for brighter bands */
                float w = beat_add * 0.75f;
                r = (uint8_t)(r + (uint8_t)((255 - r) * w));
                g = (uint8_t)(g + (uint8_t)((255 - g) * w));
                b = (uint8_t)(b + (uint8_t)((255 - b) * w));
            }

            led_strip_set_pixel(s_led_strip, i, r, g, b);
        }
        led_strip_refresh(s_led_strip);
    }
}

/* ------------------------------------------------------------------ */
/*  Button task — polls KEY_PREV and KEY_NEXT, active LOW              */
/*  Debounce: 2 consecutive LOW reads (20 ms poll → 40 ms stable).    */
/*  After action: 400 ms lockout to prevent repeat.                   */
/* ------------------------------------------------------------------ */

static void button_task(void *arg)
{
    /* Configure both keys: input, pull-up, no interrupt */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << KEY_PREV_GPIO) | (1ULL << KEY_NEXT_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    int prev_stable = 0;   /* consecutive LOW count for PREV key */
    int next_stable = 0;   /* consecutive LOW count for NEXT key */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));

        /* --- PREV key --- */
        if (gpio_get_level(KEY_PREV_GPIO) == 0) {
            prev_stable++;
        } else {
            prev_stable = 0;
        }
        if (prev_stable == 2) {   /* just became stable-LOW */
            prev_stable = 99;     /* saturate so we don't re-trigger */
            int target = (s_track_idx - 1 + s_track_count) % s_track_count;
            s_next_track_override = target;
            ESP_LOGI(TAG, "KEY PREV → track %d", target + 1);
            audio_player_stop();
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            prev_stable = 0;
        }

        /* --- NEXT key --- */
        if (gpio_get_level(KEY_NEXT_GPIO) == 0) {
            next_stable++;
        } else {
            next_stable = 0;
        }
        if (next_stable == 2) {
            next_stable = 99;
            int target = (s_track_idx + 1) % s_track_count;
            s_next_track_override = target;
            ESP_LOGI(TAG, "KEY NEXT → track %d", target + 1);
            audio_player_stop();
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            next_stable = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  PCM write callback — audio decode task (core 1) taps here          */
/* ------------------------------------------------------------------ */

static esp_err_t i2s_write_cb(void *buf, size_t len,
                               size_t *written, uint32_t timeout_ms)
{
    /* Downmix stereo int16 → mono float, push to LED stream buffer.
     * Process in 64-frame chunks to stay off the stack. */
    if (s_pcm_stream) {
        const int16_t *src    = (const int16_t *)buf;
        int            frames = (int)(len / 4);   /* 2 ch × 2 bytes */
        float          chunk[64];
        for (int f = 0; f < frames; ) {
            int n = frames - f;
            if (n > 64) n = 64;
            for (int j = 0; j < n; j++) {
                int idx  = (f + j) * 2;
                chunk[j] = (src[idx] + src[idx + 1]) * (1.0f / 65536.0f);
            }
            xStreamBufferSend(s_pcm_stream, chunk, n * sizeof(float), 0);
            f += n;
        }
    }

    esp_err_t ret = esp_codec_dev_write(s_play_dev, buf, (int)len);
    *written = (ret == ESP_OK) ? len : 0;
    return ret;
}

/* ------------------------------------------------------------------ */
/*  Clock-set callback — decoder calls this on sample-rate change       */
/* ------------------------------------------------------------------ */

static esp_err_t i2s_clk_set_cb(uint32_t rate, uint32_t bits,
                                  i2s_slot_mode_t ch)
{
    int channels = (ch == I2S_SLOT_MODE_STEREO) ? 2 : 1;
    ESP_LOGI(TAG, "clk_set: %lu Hz, %lu bit, %d ch", rate, bits, channels);

    s_sample_rate = rate;   /* LED task re-maps bins next frame */

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
    /* LED show starts/stops automatically via the stream buffer.
     * We only need the IDLE event to advance the playlist. */
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
    host.slot         = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = SD_CLK;
    slot.cmd   = SD_CMD;
    slot.d0    = SD_D0;
    slot.width = 1;

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 4096,
    };

    sdmmc_card_t *card;
    ESP_ERROR_CHECK(esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot,
                                             &mount, &card));
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
        const char *ext = ent->d_name + len - 4;
        if (ext[0] == '.' &&
            (ext[1] == 'm' || ext[1] == 'M') &&
            (ext[2] == 'p' || ext[2] == 'P') &&
            (ext[3] == '3')) {
            snprintf(s_tracks[s_track_count], sizeof(s_tracks[0]) - 1,
                     "%s/%s", MOUNT_POINT, ent->d_name);
            ESP_LOGI(TAG, "  [%d] %s", s_track_count,
                     s_tracks[s_track_count]);
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

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0,
                                                            I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 44100,
            .clk_src        = I2S_CLK_SRC_DEFAULT,   /* NOT APLL */
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
            .invert_flags = { .mclk_inv = false,
                              .bclk_inv = false,
                              .ws_inv   = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx));
    ESP_LOGI(TAG, "I2S started: 44100 Hz 16-bit stereo");

    vTaskDelay(pdMS_TO_TICKS(100));   /* MCLK stable before codec init */

    audio_codec_i2s_cfg_t i2s_cfg = { .tx_handle = s_i2s_tx };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    assert(data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .addr       = ES8388_CODEC_DEFAULT_ADDR,
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
    ESP_LOGI(TAG, "ESP32-Audio-Kit MP3 player + LED spectrum analyzer");

    /* SD must mount before I2S init (GPIO25/26 conflict) */
    sd_init();
    scan_mp3();

    if (s_track_count == 0) {
        ESP_LOGE(TAG, "No MP3 files on SD card — halting");
        return;
    }

    /* WS2812B LED strip */
    led_strip_config_t led_cfg = {
        .strip_gpio_num         = LED_GPIO,
        .max_leds               = LED_COUNT,
        .led_model              = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = { .resolution_hz = 10000000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_cfg, &rmt_cfg, &s_led_strip));
    led_strip_clear(s_led_strip);
    ESP_LOGI(TAG, "LED strip ready: %d LEDs on GPIO%d", LED_COUNT, LED_GPIO);

    /* PCM stream buffer — LED task wakes when one full FFT block arrives */
    s_pcm_stream = xStreamBufferCreate(STREAM_BUF_BYTES,
                                       FFT_SIZE * sizeof(float));
    assert(s_pcm_stream);

    audio_init();

    /* LED show task on core 0, priority below audio decode (5) */
    xTaskCreatePinnedToCore(led_task, "led_fft", 12288, NULL, 3, NULL, 0);

    s_track_done = xSemaphoreCreateBinary();
    assert(s_track_done);

    audio_player_config_t ap_cfg = {
        .mute_fn      = NULL,
        .write_fn     = i2s_write_cb,
        .clk_set_fn   = i2s_clk_set_cb,
        .priority     = 5,
        .coreID       = 1,        /* decode on core 1 */
        .force_stereo = true,     /* upmix mono → stereo for ES8388 */
    };
    ESP_ERROR_CHECK(audio_player_new(ap_cfg));
    audio_player_callback_register(player_event_cb, NULL);

    ESP_LOGI(TAG, "Starting playlist (%d tracks)", s_track_count);

    /* Button task on core 0, lowest priority */
    xTaskCreatePinnedToCore(button_task, "buttons", 2048, NULL, 2, NULL, 0);

    while (1) {
        const char *path = s_tracks[s_track_idx];
        ESP_LOGI(TAG, "Playing [%d/%d]: %s",
                 s_track_idx + 1, s_track_count, path);

        FILE *fp = fopen(path, "rb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot open %s", path);
        } else {
            /* Library owns fclose() when playback ends */
            audio_player_play(fp);
            xSemaphoreTake(s_track_done, portMAX_DELAY);
        }

        /* Check for button-driven track override, else advance normally */
        int override = s_next_track_override;
        if (override >= 0) {
            s_next_track_override = -1;
            s_track_idx = override;
        } else {
            s_track_idx = (s_track_idx + 1) % s_track_count;
        }
    }
}
