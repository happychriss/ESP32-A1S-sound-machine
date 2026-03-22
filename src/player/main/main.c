/*
 * ESP32-Audio-Kit MP3 player + WS2812B LED mood show
 *
 * SD card → MP3 decode (chmorgan/esp-audio-player) → ES8388 → speaker
 * LED show (GPIO22, 30 LEDs) — 4-layer composited mood show:
 *   Layer 1 ambient:  slow palette rotation across all LEDs, speed tracks BPM
 *   Layer 2 beat:     white flash on each beat, exponential decay (additive)
 *   Layer 3 bass:     warm Gaussian blob drifts along strip driven by sub-bass
 *   Layer 4 sparkle:  individual LEDs sparked by upper-band energy
 *   Section detector: cosine novelty between band vectors cycles palette on
 *                     verse/chorus transitions. Per-band AGC + global RMS gate.
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

/* ---------- FFT / LED show parameters ---------- */
#define FFT_SIZE              512
/* stream buffer holds 8 FFT-blocks worth of mono float samples */
#define STREAM_BUF_BYTES      (FFT_SIZE * 8 * sizeof(float))

/* 8 logarithmic frequency bands */
#define NUM_BANDS             8
static const float BAND_HZ[NUM_BANDS + 1] = {
    60.0f, 200.0f, 400.0f, 800.0f, 2000.0f, 4000.0f, 8000.0f, 14000.0f, 20000.0f
};

/* Per-band AGC — exponential peak decay (~10 s at ~86 FPS update rate) */
#define BAND_AGC_DECAY        0.9997f
#define BAND_AGC_MIN          0.0001f

/* Beat detection — spectral flux with stddev-adaptive threshold */
#define BEAT_K                1.2f   /* lower = more responsive to clear kick  */
#define BEAT_MIN_GAP_MS       220    /* ~273 BPM max                           */
#define BEAT_PERIOD_INIT_MS   500    /* starting estimate ~120 BPM (pop tempo) */

/* Section change — cosine novelty between band energy vectors */
#define NOVELTY_K             2.0f   /* threshold = novelty_mean + K × std   */
#define NOVELTY_SMOOTH        0.003f /* smooth_vec α — time constant ~4 s    */
#define NOVELTY_MIN_GAP_MS    5000   /* minimum 5 s between section changes  */

/* Layer strengths */
#define AMBIENT_BASE          0.12f  /* dim ambient — keep dark between beats */
#define BEAT_FLASH_PEAK       0.95f  /* beat-pulse additive white brightness  */
#define BEAT_DECAY_K          9.0f   /* high = very dark between beats        */
#define BASS_WAVE_PEAK        0.85f  /* bass blob peak brightness             */
#define SPARKLE_THRESHOLD     0.45f  /* upper-band norm to trigger a sparkle  */
#define SPARKLE_DECAY         0.88f  /* sparkle brightness × per frame        */
#define MAX_SPARKLES          8      /* simultaneous sparkle slots            */

/* Colour palettes: hue_start° and hue_range° (HSV, 0–360) */
#define NUM_PALETTES          5
typedef struct { float start; float range; } palette_t;
static const palette_t PALETTES[NUM_PALETTES] = {
    {  0.0f,  80.0f },   /* fire:    red → orange → yellow  */
    {180.0f,  80.0f },   /* ocean:   cyan → blue            */
    { 90.0f,  70.0f },   /* forest:  yellow-green           */
    {270.0f, 100.0f },   /* neon:    violet → magenta       */
    {  0.0f, 360.0f },   /* rainbow: full spectrum          */
};

/* Sparkle slot */
typedef struct { uint8_t pos; float hue; float bright; } sparkle_t;

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

/* Additive blend clamped to [0, 255] */
static inline uint8_t clamp8(int v) { return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v; }

/* ------------------------------------------------------------------ */
/*  LED show task — runs on core 0                                      */
/* ------------------------------------------------------------------ */

static void led_task(void *arg)
{
    /* ---- BSS work buffers ---- */
    static float     fft_buf[FFT_SIZE * 2];
    static float     pcm_buf[FFT_SIZE];
    static float     hann_win[FFT_SIZE];
    static float     prev_mag[FFT_SIZE / 2];
    static float     band_energy[NUM_BANDS];
    static float     band_peak[NUM_BANDS];
    static float     band_norm[NUM_BANDS];
    static float     smooth_vec[NUM_BANDS];   /* slow reference for novelty   */
    static sparkle_t sparkles[MAX_SPARKLES];
    static uint8_t   r_buf[LED_COUNT], g_buf[LED_COUNT], b_buf[LED_COUNT];

    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    dsps_wind_hann_f32(hann_win, FFT_SIZE);

    for (int i = 0; i < NUM_BANDS; i++) band_peak[i] = BAND_AGC_MIN;
    memset(band_energy, 0, sizeof(band_energy));
    memset(band_norm,   0, sizeof(band_norm));
    memset(smooth_vec,  0, sizeof(smooth_vec));
    memset(sparkles,    0, sizeof(sparkles));
    memset(prev_mag,    0, sizeof(prev_mag));

    /* Beat */
    float      flux_mean      = 0.0f;
    float      flux_m2        = 0.0f;
    float      beat_strength  = 0.0f;
    uint32_t   beat_period_ms = BEAT_PERIOD_INIT_MS;
    TickType_t last_beat_tick = 0;

    /* Global RMS / brightness gate */
    float global_peak  = 0.001f;
    float global_rms_s = 0.0f;   /* smoothed RMS */

    /* Section / palette */
    float      novelty_mean  = 0.0f;
    float      novelty_m2    = 0.0f;
    TickType_t last_sec_tick = 0;
    int        palette_idx   = 0;
    float      hue_offset    = 0.0f;   /* ambient layer rotation */
    float      wave_pos      = 0.0f;   /* bass blob position     */

    while (1) {

        /* ---- New track: full state reset ---- */
        if (s_track_started) {
            s_track_started = false;
            for (int i = 0; i < NUM_BANDS; i++) {
                band_peak[i]  = BAND_AGC_MIN;
                band_norm[i]  = 0.0f;
                smooth_vec[i] = 0.0f;
            }
            memset(prev_mag, 0, sizeof(prev_mag));
            memset(sparkles, 0, sizeof(sparkles));
            flux_mean = 0.0f; flux_m2 = 0.0f;
            beat_strength  = 0.0f;
            beat_period_ms = BEAT_PERIOD_INIT_MS;
            last_beat_tick = xTaskGetTickCount();
            global_peak    = 0.001f;
            global_rms_s   = 0.0f;
            novelty_mean   = 0.0f; novelty_m2 = 0.0f;
            last_sec_tick  = xTaskGetTickCount();
        }

        /* ---- Wait for PCM frame (up to 50 ms) ---- */
        size_t got = xStreamBufferReceive(s_pcm_stream, pcm_buf,
                                          FFT_SIZE * sizeof(float),
                                          pdMS_TO_TICKS(50));
        TickType_t now = xTaskGetTickCount();

        if (got < FFT_SIZE * sizeof(float)) {
            /* Idle / silence: clear strip and decay state */
            beat_strength *= 0.7f;
            global_rms_s  *= 0.9f;
            led_strip_clear(s_led_strip);
            led_strip_refresh(s_led_strip);
            continue;
        }

        /* ---- Global RMS brightness gate ---- */
        float rms_sq = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) rms_sq += pcm_buf[i] * pcm_buf[i];
        float rms = sqrtf(rms_sq / FFT_SIZE);
        global_rms_s = 0.15f * rms + 0.85f * global_rms_s;
        if (rms > global_peak) global_peak = rms;
        else                   global_peak *= 0.9997f;
        if (global_peak < 0.001f) global_peak = 0.001f;
        float global_norm = global_rms_s / global_peak;
        if (global_norm > 1.0f) global_norm = 1.0f;

        /* ---- FFT ---- */
        for (int i = 0; i < FFT_SIZE; i++) {
            fft_buf[i * 2]     = pcm_buf[i] * hann_win[i];
            fft_buf[i * 2 + 1] = 0.0f;
        }
        dsps_fft2r_fc32(fft_buf, FFT_SIZE);
        dsps_bit_rev2r_fc32(fft_buf, FFT_SIZE);

        /* ---- Band energies + spectral flux (single pass) ---- */
        float hz_per_bin = (float)s_sample_rate / FFT_SIZE;
        float flux = 0.0f;
        float band_count[NUM_BANDS] = {0};
        memset(band_energy, 0, sizeof(band_energy));

        for (int k = 1; k < FFT_SIZE / 2; k++) {
            float re  = fft_buf[k * 2], im = fft_buf[k * 2 + 1];
            float mag = sqrtf(re * re + im * im);

            float d = mag - prev_mag[k];
            if (d > 0.0f) flux += d;
            prev_mag[k] = mag;

            float hz = k * hz_per_bin;
            for (int b = 0; b < NUM_BANDS; b++) {
                if (hz >= BAND_HZ[b] && hz < BAND_HZ[b + 1]) {
                    band_energy[b] += mag;
                    band_count[b]  += 1.0f;
                    break;
                }
            }
        }

        /* Per-band AGC normalisation */
        for (int b = 0; b < NUM_BANDS; b++) {
            if (band_count[b] > 0.0f) band_energy[b] /= band_count[b];
            if (band_energy[b] > band_peak[b]) band_peak[b] = band_energy[b];
            else                               band_peak[b] *= BAND_AGC_DECAY;
            if (band_peak[b] < BAND_AGC_MIN)   band_peak[b]  = BAND_AGC_MIN;
            band_norm[b] = band_energy[b] / band_peak[b];
        }

        /* ---- Beat detection — flux with stddev-adaptive threshold ---- */
        float fd = flux - flux_mean;
        flux_mean += 0.05f * fd;
        flux_m2    = 0.95f * flux_m2 + 0.05f * fd * fd;
        float flux_std = (flux_m2 > 0.0f) ? sqrtf(flux_m2) : 0.0f;

        bool beat = (pdTICKS_TO_MS(now - last_beat_tick) > BEAT_MIN_GAP_MS) &&
                    (flux > flux_mean + BEAT_K * flux_std)                   &&
                    (flux > 0.001f);

        if (beat) {
            uint32_t interval = (uint32_t)pdTICKS_TO_MS(now - last_beat_tick);
            if (interval > 250 && interval < 1500)
                beat_period_ms = (beat_period_ms * 7 + interval) >> 3;
            last_beat_tick = now;
            beat_strength  = 1.0f;
            /* Disco snap: jump hue_offset 40–100° so all LED colors shift
             * to new positions on the beat — "lights between LEDs changing" */
            hue_offset = fmodf(hue_offset + 40.0f + (float)(esp_random() % 60), 360.0f);
        }
        /* Beat-phase exponential decay */
        float phase = (float)pdTICKS_TO_MS(now - last_beat_tick) / (float)beat_period_ms;
        if (phase > 1.0f) phase = 1.0f;
        beat_strength = expf(-BEAT_DECAY_K * phase);

        /* ---- Section change — cosine novelty on band_norm vector ---- */
        float dot = 0.0f, mag_a = 0.0f, mag_b = 0.0f;
        for (int b = 0; b < NUM_BANDS; b++) {
            dot   += band_norm[b] * smooth_vec[b];
            mag_a += band_norm[b] * band_norm[b];
            mag_b += smooth_vec[b] * smooth_vec[b];
        }
        float novelty = 0.0f;
        if (mag_a > 0.0001f && mag_b > 0.0001f)
            novelty = 1.0f - dot / (sqrtf(mag_a) * sqrtf(mag_b));

        /* Slowly update reference vector */
        for (int b = 0; b < NUM_BANDS; b++)
            smooth_vec[b] += NOVELTY_SMOOTH * (band_norm[b] - smooth_vec[b]);

        /* Running stats on novelty for adaptive threshold */
        float nd = novelty - novelty_mean;
        novelty_mean += 0.01f * nd;
        novelty_m2    = 0.99f * novelty_m2 + 0.01f * nd * nd;
        float novelty_std = (novelty_m2 > 0.0f) ? sqrtf(novelty_m2) : 0.0f;

        if (pdTICKS_TO_MS(now - last_sec_tick) > NOVELTY_MIN_GAP_MS &&
            novelty > novelty_mean + NOVELTY_K * novelty_std          &&
            novelty > 0.05f) {
            last_sec_tick = now;
            palette_idx   = (palette_idx + 1) % NUM_PALETTES;
            ESP_LOGI(TAG, "Section change → palette %d", palette_idx);
        }

        /* ---- Palette + ambient rotation speed (BPM-relative) ---- */
        float bpm       = 60000.0f / (float)beat_period_ms;
        float pal_start = PALETTES[palette_idx].start;
        float pal_range = PALETTES[palette_idx].range;
        hue_offset = fmodf(hue_offset + 0.5f * (bpm / 120.0f), 360.0f);

        /* ==== Layer 1: Ambient base (palette spread + rotation) ==== */
        float amb_v = AMBIENT_BASE * global_norm;
        for (int i = 0; i < LED_COUNT; i++) {
            float hue = fmodf(pal_start + hue_offset +
                              pal_range * i / (LED_COUNT - 1), 360.0f);
            hsv_to_rgb(hue, 1.0f, amb_v, &r_buf[i], &g_buf[i], &b_buf[i]);
        }

        /* ==== Layer 2: Beat pulse (additive white flash) ==== */
        uint8_t beat_add = (uint8_t)(beat_strength * BEAT_FLASH_PEAK * 255.0f);
        if (beat_add > 0) {
            for (int i = 0; i < LED_COUNT; i++) {
                r_buf[i] = clamp8((int)r_buf[i] + beat_add);
                g_buf[i] = clamp8((int)g_buf[i] + beat_add);
                b_buf[i] = clamp8((int)b_buf[i] + beat_add);
            }
        }

        /* ==== Layer 3: Bass wave (warm Gaussian blob drifting along strip) ==== */
        float bass_level = (band_norm[0] + band_norm[1]) * 0.5f * global_norm;
        wave_pos = fmodf(wave_pos + 0.1f + band_norm[0] * 0.4f, (float)LED_COUNT);
        float wave_hue = fmodf(pal_start + 15.0f, 360.0f);
        for (int i = 0; i < LED_COUNT; i++) {
            float dist = fabsf((float)i - wave_pos);
            if (dist > LED_COUNT * 0.5f) dist = LED_COUNT - dist;  /* wrap */
            float glow = bass_level * BASS_WAVE_PEAK * expf(-0.25f * dist * dist);
            if (glow < 0.01f) continue;
            uint8_t r, g, b;
            hsv_to_rgb(wave_hue, 0.85f, glow, &r, &g, &b);
            r_buf[i] = clamp8((int)r_buf[i] + r);
            g_buf[i] = clamp8((int)g_buf[i] + g);
            b_buf[i] = clamp8((int)b_buf[i] + b);
        }

        /* ==== Layer 4: Sparkles (upper-band energy) ==== */
        float upper = (band_norm[4] + band_norm[5] + band_norm[6] + band_norm[7]) * 0.25f;
        if (upper > SPARKLE_THRESHOLD && global_norm > 0.1f && (esp_random() % 4 == 0)) {
            for (int s = 0; s < MAX_SPARKLES; s++) {
                if (sparkles[s].bright < 0.02f) {
                    sparkles[s].pos    = (uint8_t)(esp_random() % LED_COUNT);
                    sparkles[s].hue    = fmodf(pal_start + pal_range * 0.7f +
                                               (float)(esp_random() % 60) - 30.0f, 360.0f);
                    sparkles[s].bright = upper * global_norm;
                    break;
                }
            }
        }
        for (int s = 0; s < MAX_SPARKLES; s++) {
            if (sparkles[s].bright > 0.02f) {
                uint8_t r, g, b;
                hsv_to_rgb(sparkles[s].hue, 0.5f, sparkles[s].bright, &r, &g, &b);
                int p = sparkles[s].pos;
                r_buf[p] = clamp8((int)r_buf[p] + r);
                g_buf[p] = clamp8((int)g_buf[p] + g);
                b_buf[p] = clamp8((int)b_buf[p] + b);
                sparkles[s].bright *= SPARKLE_DECAY;
            }
        }

        /* ==== Output ==== */
        for (int i = 0; i < LED_COUNT; i++)
            led_strip_set_pixel(s_led_strip, i, r_buf[i], g_buf[i], b_buf[i]);
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
