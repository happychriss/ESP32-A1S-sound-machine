/*
 * ESP32-Audio-Kit MP3 player + WS2812B LED mood show
 *
 * SD card → MP3 decode (chmorgan/esp-audio-player) → ES8388 → speaker
 * LED show (GPIO22, 30 LEDs) — mood-driven, not spectrum:
 *   - Beat phase drives brightness: full flash at beat, exponential decay to dark
 *   - Spectral centroid drives colour: bass-heavy → warm (red/orange),
 *     treble-bright → cool (blue/violet) — changes slowly with song character
 *   - RMS energy sets ambient floor: quiet passages dim, loud passages glow
 *   - On each beat: hues reshuffled in a spread around current centroid colour
 *   - Beat white-flash (saturation drops to near-zero at beat moment)
 *
 * Keys (active LOW):
 *   KEY1  GPIO36 = Pause / Resume (input-only pin, external pull-up on board)
 *   KEY3  GPIO19 = Previous track  (internal pull-up)
 *   KEY4  GPIO23 = Next track      (internal pull-up)
 *   KEY5  GPIO18 = Volume down     (internal pull-up)
 *   KEY6  GPIO5  = Volume up       (internal pull-up)
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
#include "esp_random.h"
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

/* Keys — active LOW, 40 ms debounce, 400 ms lockout
 * KEY1/GPIO36: input-only, NO internal pull-up (relies on board external pull-up)
 * KEY3–KEY6:   normal GPIOs with internal pull-up                               */
#define KEY_PAUSE_GPIO  GPIO_NUM_36   /* KEY1 — pause / resume (input-only pin)  */
#define KEY_PREV_GPIO   GPIO_NUM_19   /* KEY3 — previous track                   */
#define KEY_NEXT_GPIO   GPIO_NUM_23   /* KEY4 — next track                       */
#define KEY_VOLD_GPIO   GPIO_NUM_18   /* KEY5 — volume down                      */
#define KEY_VOLU_GPIO   GPIO_NUM_5    /* KEY6 — volume up                        */
#define BTN_DEBOUNCE_MS  40
#define BTN_LOCKOUT_MS  400
#define VOL_STEP         8            /* volume change per press (0–100 scale)   */
#define VOL_INIT        70

/* ---------- FFT / LED mood show parameters ---------- */
#define FFT_SIZE            512
/* stream buffer holds 8 FFT-blocks worth of mono float samples */
#define STREAM_BUF_BYTES    (FFT_SIZE * 8 * sizeof(float))

/* spectral centroid → mood hue mapping */
#define CENTROID_LO_HZ      300.0f  /* below this = full bass → hue 0 (red)  */
#define CENTROID_HI_HZ     5000.0f  /* above this = full treble → hue HUE_RANGE */
#define HUE_RANGE           240.0f  /* red (0°) → blue/violet (240°)          */
#define HUE_SPREAD          160.0f  /* hue arc spread around centroid per beat */

/* beat-phase brightness */
#define BEAT_DECAY_K          5.0f  /* higher = darker between beats (more blinky) */
#define AMBIENT_FACTOR        0.20f /* max ambient brightness from RMS energy   */
#define FLASH_SAT_MIN         0.08f /* saturation at beat moment (near-white)   */

/* beat detection */
#define BEAT_RATIO            1.5f  /* bass energy spike: rolling avg × this    */
#define BEAT_FLUX_RATIO       1.8f  /* spectral flux spike: rolling avg × this  */
#define BEAT_MIN_GAP_MS       220   /* minimum ms between beats (~273 BPM max)  */
#define BEAT_PERIOD_INIT_MS   550   /* starting estimate ~109 BPM               */

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
static volatile int  s_next_track_override = -1;
static int           s_volume    = VOL_INIT;  /* current volume 0–100    */
static volatile bool s_is_paused = false;     /* pause state flag         */
static SemaphoreHandle_t s_resume_sem;        /* given by KEY1 to resume  */
/* set by player_event_cb on PLAYING — LED task uses it to reset gain + hues */
static volatile bool s_track_started = false;

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

/* Spread LED_COUNT hues evenly across HUE_SPREAD degrees centred on hue_base,
 * then Fisher-Yates shuffle — every LED gets a distinct hue in the mood palette. */
static void shuffle_hues_mood(float *hues, float hue_base)
{
    float start = hue_base - HUE_SPREAD / 2.0f;
    for (int i = 0; i < LED_COUNT; i++) {
        hues[i] = fmodf(start + (HUE_SPREAD / (LED_COUNT - 1)) * i + 360.0f, 360.0f);
    }
    for (int i = LED_COUNT - 1; i > 0; i--) {
        int j = (int)(esp_random() % (i + 1));
        float tmp = hues[i]; hues[i] = hues[j]; hues[j] = tmp;
    }
}

/* ------------------------------------------------------------------ */
/*  LED show task — runs on core 0                                      */
/* ------------------------------------------------------------------ */

static void led_task(void *arg)
{
    /* Work buffers in BSS */
    static float fft_buf[FFT_SIZE * 2];      /* complex interleaved          */
    static float pcm_buf[FFT_SIZE];
    static float hann[FFT_SIZE];
    static float prev_mag[FFT_SIZE / 2];     /* for spectral flux            */
    static float led_hue[LED_COUNT];         /* hue per LED, mood-shuffled   */

    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    dsps_wind_hann_f32(hann, FFT_SIZE);

    /* Start with a neutral mid-spectrum mood (green-ish) */
    float centroid_norm  = 0.5f;
    shuffle_hues_mood(led_hue, centroid_norm * HUE_RANGE);

    float      rms_display    = 0.0f;
    float      rms_max        = 0.001f;
    float      bass_avg       = 0.0f;
    float      flux_avg       = 0.0f;
    uint32_t   beat_period_ms = BEAT_PERIOD_INIT_MS;
    TickType_t last_beat_tick = 0;
    memset(prev_mag, 0, sizeof(prev_mag));

    while (1) {

        /* New track → reset all state */
        if (s_track_started) {
            s_track_started  = false;
            centroid_norm    = 0.5f;
            rms_display      = 0.0f;
            rms_max          = 0.001f;
            bass_avg         = 0.0f;
            flux_avg         = 0.0f;
            beat_period_ms   = BEAT_PERIOD_INIT_MS;
            last_beat_tick   = xTaskGetTickCount();
            memset(prev_mag, 0, sizeof(prev_mag));
            shuffle_hues_mood(led_hue, centroid_norm * HUE_RANGE);
        }

        /* Wait up to 50 ms for a full FFT block */
        size_t got = xStreamBufferReceive(s_pcm_stream, pcm_buf,
                                          FFT_SIZE * sizeof(float),
                                          pdMS_TO_TICKS(50));

        TickType_t now = xTaskGetTickCount();

        if (got < FFT_SIZE * sizeof(float)) {
            /* Idle — fade toward black following beat phase */
            uint32_t ms = (uint32_t)(pdTICKS_TO_MS(now - last_beat_tick));
            float phase = (float)ms / (float)beat_period_ms;
            if (phase > 1.5f) phase = 1.5f;
            float v = rms_display * AMBIENT_FACTOR * expf(-2.0f * phase);
            for (int i = 0; i < LED_COUNT; i++) {
                uint8_t r, g, b;
                hsv_to_rgb(led_hue[i], 1.0f, v, &r, &g, &b);
                led_strip_set_pixel(s_led_strip, i, r, g, b);
            }
            led_strip_refresh(s_led_strip);
            rms_display *= 0.85f;
            continue;
        }

        /* --- RMS energy (no FFT needed) --- */
        float rms_sq = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) rms_sq += pcm_buf[i] * pcm_buf[i];
        float rms = sqrtf(rms_sq / FFT_SIZE);
        if (rms > rms_max)  rms_max = rms;
        rms_max    *= 0.9995f;
        if (rms_max < 0.001f) rms_max = 0.001f;
        float rms_norm = rms / rms_max;
        rms_display = 0.35f * rms_norm + 0.65f * rms_display;

        /* --- FFT --- */
        for (int i = 0; i < FFT_SIZE; i++) {
            fft_buf[i * 2]     = pcm_buf[i] * hann[i];
            fft_buf[i * 2 + 1] = 0.0f;
        }
        dsps_fft2r_fc32(fft_buf, FFT_SIZE);
        dsps_bit_rev2r_fc32(fft_buf, FFT_SIZE);

        /* --- Single pass over bins: centroid + bass energy + spectral flux --- */
        float cen_num    = 0.0f, cen_den = 0.0f;
        float bass_energy = 0.0f;
        float flux        = 0.0f;
        float hz_per_bin  = (float)s_sample_rate / FFT_SIZE;
        int   bass_cutoff = (int)(280.0f / hz_per_bin) + 1;   /* ~280 Hz */

        for (int k = 1; k < FFT_SIZE / 2; k++) {
            float re  = fft_buf[k * 2];
            float im  = fft_buf[k * 2 + 1];
            float mag = sqrtf(re * re + im * im);
            float hz  = k * hz_per_bin;

            cen_num += hz  * mag;
            cen_den +=        mag;

            if (k <= bass_cutoff) bass_energy += mag;

            float delta = mag - prev_mag[k];
            if (delta > 0.0f) flux += delta;
            prev_mag[k] = mag;
        }
        if (bass_cutoff > 0) bass_energy /= bass_cutoff;

        /* Centroid: very slow update — tracks song mood, not individual notes.
         * Maps CENTROID_LO_HZ–CENTROID_HI_HZ → [0, 1] → hue [0°, HUE_RANGE°] */
        if (cen_den > 1.0f) {
            float hz = cen_num / cen_den;
            float cn = (hz - CENTROID_LO_HZ) / (CENTROID_HI_HZ - CENTROID_LO_HZ);
            if (cn < 0.0f) cn = 0.0f;
            if (cn > 1.0f) cn = 1.0f;
            centroid_norm = 0.985f * centroid_norm + 0.015f * cn;
        }

        /* --- Beat detection: bass energy OR spectral flux spike --- */
        bass_avg = 0.05f * bass_energy + 0.95f * bass_avg;
        flux_avg = 0.05f * flux        + 0.95f * flux_avg;

        bool beat = (now - last_beat_tick) > pdMS_TO_TICKS(BEAT_MIN_GAP_MS) &&
                    (bass_energy > bass_avg * BEAT_RATIO ||
                     flux        > flux_avg * BEAT_FLUX_RATIO);

        if (beat) {
            uint32_t interval = (uint32_t)(pdTICKS_TO_MS(now - last_beat_tick));
            /* Update beat period estimate only for musically plausible intervals */
            if (interval > 250 && interval < 1500) {
                beat_period_ms = (beat_period_ms * 7 + interval) >> 3;
            }
            last_beat_tick = now;
            /* Reshuffle hues in arc centred on current mood colour */
            shuffle_hues_mood(led_hue, centroid_norm * HUE_RANGE);
        }

        /* --- Beat phase: 0.0 at beat → 1.0 at expected next beat --- */
        uint32_t ms_since = (uint32_t)(pdTICKS_TO_MS(now - last_beat_tick));
        float phase = (float)ms_since / (float)beat_period_ms;
        if (phase > 1.0f) phase = 1.0f;

        /* --- Brightness: beat flash decays exponentially, ambient from RMS ---
         *   At phase=0 (beat):  beat_v=1.0  →  v nearly 1.0, sat near-white
         *   At phase=0.5:       beat_v≈0.08 →  mostly ambient colour glow
         *   At phase=1.0:       beat_v≈0.007 → just ambient, waiting for beat  */
        float beat_v  = expf(-BEAT_DECAY_K * phase);
        float ambient = rms_display * AMBIENT_FACTOR;
        float v       = ambient + beat_v * (1.0f - ambient);
        if (v > 1.0f) v = 1.0f;

        /* Saturation: 0 (white) at beat instant, rapidly recovers to full colour */
        float sat = 1.0f - beat_v * (1.0f - FLASH_SAT_MIN);

        /* --- Render --- */
        for (int i = 0; i < LED_COUNT; i++) {
            uint8_t r, g, b;
            hsv_to_rgb(led_hue[i], sat, v, &r, &g, &b);
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
    /* KEY1 (GPIO36) — input-only pin, board has external pull-up, no internal */
    gpio_config_t pause_io = {
        .pin_bit_mask = (1ULL << KEY_PAUSE_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&pause_io);

    /* KEY3–KEY6 — normal GPIOs with internal pull-up */
    gpio_config_t nav_io = {
        .pin_bit_mask = (1ULL << KEY_PREV_GPIO) | (1ULL << KEY_NEXT_GPIO) |
                        (1ULL << KEY_VOLD_GPIO) | (1ULL << KEY_VOLU_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&nav_io);

    int  pause_stable = 0;
    int  prev_stable  = 0;
    int  next_stable  = 0;
    int  vold_stable  = 0;
    int  volu_stable  = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));

        /* --- PAUSE/RESUME key (KEY1, GPIO36) ---
         * Pause:  mute amp → stop player → main loop blocks on s_resume_sem.
         * Resume: unblock main loop → main loop replays same track → codec
         *         re-opens and unmutes amp automatically via player_event_cb. */
        if (gpio_get_level(KEY_PAUSE_GPIO) == 0) { pause_stable++; } else { pause_stable = 0; }
        if (pause_stable == 2) {
            pause_stable = 99;
            if (!s_is_paused) {
                gpio_set_level(PA_ENABLE_PIN, 0);   /* silence amp immediately  */
                vTaskDelay(pdMS_TO_TICKS(20));
                s_is_paused = true;
                audio_player_stop();                /* IDLE fires → s_track_done */
                ESP_LOGI(TAG, "KEY PAUSE → paused");
            } else {
                s_is_paused = false;
                xSemaphoreGive(s_resume_sem);       /* wake main loop to replay  */
                ESP_LOGI(TAG, "KEY PAUSE → resumed");
            }
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            pause_stable = 0;
        }

        /* --- PREV key (KEY3) --- */
        if (gpio_get_level(KEY_PREV_GPIO) == 0) { prev_stable++; } else { prev_stable = 0; }
        if (prev_stable == 2) {
            prev_stable = 99;
            int target = (s_track_idx - 1 + s_track_count) % s_track_count;
            s_next_track_override = target;
            ESP_LOGI(TAG, "KEY PREV → track %d", target + 1);
            if (s_is_paused) {
                /* Already stopped — just unblock main loop */
                s_is_paused = false;
                xSemaphoreGive(s_resume_sem);
            } else {
                audio_player_stop();
            }
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            prev_stable = 0;
        }

        /* --- NEXT key (KEY4) --- */
        if (gpio_get_level(KEY_NEXT_GPIO) == 0) { next_stable++; } else { next_stable = 0; }
        if (next_stable == 2) {
            next_stable = 99;
            int target = (s_track_idx + 1) % s_track_count;
            s_next_track_override = target;
            ESP_LOGI(TAG, "KEY NEXT → track %d", target + 1);
            if (s_is_paused) {
                s_is_paused = false;
                xSemaphoreGive(s_resume_sem);
            } else {
                audio_player_stop();
            }
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            next_stable = 0;
        }

        /* --- VOL DOWN key (KEY5) --- */
        if (gpio_get_level(KEY_VOLD_GPIO) == 0) { vold_stable++; } else { vold_stable = 0; }
        if (vold_stable == 2) {
            vold_stable = 99;
            s_volume -= VOL_STEP;
            if (s_volume < 0) s_volume = 0;
            esp_codec_dev_set_out_vol(s_play_dev, s_volume);
            ESP_LOGI(TAG, "KEY VOL- → %d", s_volume);
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            vold_stable = 0;
        }

        /* --- VOL UP key (KEY6) --- */
        if (gpio_get_level(KEY_VOLU_GPIO) == 0) { volu_stable++; } else { volu_stable = 0; }
        if (volu_stable == 2) {
            volu_stable = 99;
            s_volume += VOL_STEP;
            if (s_volume > 100) s_volume = 100;
            esp_codec_dev_set_out_vol(s_play_dev, s_volume);
            ESP_LOGI(TAG, "KEY VOL+ → %d", s_volume);
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            volu_stable = 0;
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
    switch (ctx->audio_event) {
    case AUDIO_PLAYER_CALLBACK_EVENT_PLAYING:
        s_track_started = true;            /* LED task resets gain + hues       */
        gpio_set_level(PA_ENABLE_PIN, 1);  /* unmute amp — safe if already on   */
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_IDLE:
        xSemaphoreGive(s_track_done);
        break;
    default:
        break;
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
    esp_codec_dev_set_out_vol(s_play_dev, s_volume);
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
    s_resume_sem = xSemaphoreCreateBinary();
    assert(s_resume_sem);

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
            s_track_idx = (s_track_idx + 1) % s_track_count;
            continue;
        }

        /* Library owns fclose() when playback ends */
        bool was_paused = false;
        audio_player_play(fp);
        xSemaphoreTake(s_track_done, portMAX_DELAY);

        /* Paused: block here until KEY1 resume or next/prev override */
        if (s_is_paused) {
            was_paused = true;
            xSemaphoreTake(s_resume_sem, portMAX_DELAY);
            /* s_is_paused is now false — set by button_task before Give */
        }

        /* Advance track index:
         *   - next/prev override → jump to that track (works from both states)
         *   - natural end of track → advance by one
         *   - pure pause/resume → replay same track (no index change)         */
        int override = s_next_track_override;
        if (override >= 0) {
            s_next_track_override = -1;
            s_track_idx = override;
        } else if (!was_paused) {
            s_track_idx = (s_track_idx + 1) % s_track_count;
        }
        /* else: was_paused + no override → replay same track */
    }
}
