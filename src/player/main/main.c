/*
 * ESP32-Audio-Kit MP3 player + BT A2DP sink + WS2812B LED show
 *
 * SD card → MP3 decode (chmorgan/esp-audio-player) → ES8388 → speaker
 * BT A2DP sink: long-press KEY1 (2 s) enters 30 s pairing window, phone
 *               streams audio through same codec + LED show.
 *
 * LED show (GPIO22, 30 LEDs) — 5-layer composited show:
 *   Layer 1 ambient:  dim hue gradient (180° spread) slowly rotating, BPM-relative
 *   Layer 2 beat:     strobe burst — 4 frame sides each get a distinct random hue
 *                     (80–120° forced separation). LEDs fully off between beats.
 *                     vol_tier drives saturation: quiet=pastel, mid=normal, loud=vivid.
 *   Layer 3 bass:     warm Gaussian blob drifts along strip driven by sub-bass energy
 *   Layer 4 sparkle:  individual LEDs sparked by upper-band (treble) energy
 *   Layer 5 frame:    picture-frame border always visible at dim level between beats;
 *                     vol_tier controls which sides show (quiet=1 pair, mid=alternating,
 *                     loud=all 4). Each side has its own random hue from side_hues[].
 *
 * Beat detection: dual-EMA sub-bass ratio (kick_fast/kick_slow > KICK_RATIO).
 * Strobe gate: LEDs fully off when beat_strength < 0.04 (clean on/off contrast).
 * Side hues regenerated on each disco snap (~every beat) with ~80-120° separation.
 * No-blank fix: stream-dry holds last frame (prevents DMA-timing flicker).
 *
 * Keys (active LOW):
 *   KEY1  GPIO36 = short=pause/resume, long(2s)=BT mode toggle
 *   KEY3  GPIO19 = Previous track
 *   KEY4  GPIO23 = Next track
 *   KEY5  GPIO18 = Volume down
 *   KEY6  GPIO5  = Volume up
 *   (KEY2/GPIO13 skipped — SD D3 conflict)
 *
 * Pins:
 *   I2C   SCL=GPIO32  SDA=GPIO33          (ES8388 ctrl)
 *   I2S   MCLK=GPIO0  BCLK=GPIO27  WS=GPIO25  DOUT=GPIO26
 *   PA    GPIO21 HIGH = NS4150 amp enable
 *   SD    CLK=GPIO14  CMD=GPIO15  D0=GPIO2   (SDMMC slot 1, 1-bit)
 *   LED   GPIO22 = WS2812B data (30 LEDs, external 5V power)
 */

#include <stdio.h>
#include <stdlib.h>
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
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

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
#define BTN_DEBOUNCE_MS      40
#define BTN_LOCKOUT_MS      400
#define VOL_STEP              8       /* volume change per press (0–100 scale)   */
#define VOL_INIT             70
#define KEY_PAUSE_LONG_MS  2000       /* hold KEY1 this long to toggle BT mode   */

/* Bluetooth */
#define BT_DEVICE_NAME  "Dietmars-Soundbox"
#define BT_PAIRING_MS   30000         /* discoverable window in ms               */

/* ---------- FFT / LED show parameters ---------- */
#define FFT_SIZE              512
/* stream buffer holds 4 FFT-blocks worth of mono float samples */
#define STREAM_BUF_BYTES      (FFT_SIZE * 4 * sizeof(float))

/* 8 logarithmic frequency bands */
#define NUM_BANDS             8
static const float BAND_HZ[NUM_BANDS + 1] = {
    60.0f, 200.0f, 400.0f, 800.0f, 2000.0f, 4000.0f, 8000.0f, 14000.0f, 20000.0f
};

/* Per-band AGC — exponential peak decay (~10 s at ~86 FPS update rate) */
#define BAND_AGC_DECAY        0.9997f
#define BAND_AGC_MIN          0.0001f

/* Beat detection — dual-EMA sub-bass ratio (kick drum focused, 60–400 Hz)
 *   kick_fast tracks attack (τ ≈ 30 ms), kick_slow tracks background (τ ≈ 650 ms).
 *   A beat fires when the fast EMA exceeds KICK_RATIO × the slow background,
 *   meaning a genuine transient spike rather than a sustained level change.    */
#define KICK_ALPHA_FAST       0.60f  /* fast EMA α  — attack time constant (~8 ms) */
#define KICK_ALPHA_SLOW       0.015f /* slow EMA α  — background time constant     */
#define KICK_RATIO            1.3f   /* lower threshold — catches more beats, fewer misses  */
#define KICK_ABS_MIN          0.0002f/* slow EMA floor — noise guard                       */
#define BEAT_MIN_GAP_MS       200    /* 200 ms floor = max 5/s — allows 2–3 flashes/beat   */
#define DISCO_SNAP_GAP_MS     350    /* snap on nearly every beat — drives colour energy */
#define BEAT_PERIOD_INIT_MS   500    /* initial BPM estimate ~120 BPM           */

/* Silence gate — absolute RMS floor below which we skip all processing */
#define SILENCE_FLOOR         0.0005f

/* Frame layer — beat-driven phase flip minimum gap (prevents frame flicker) */
#define FRAME_BEAT_GAP_MS     500    /* at most ~120 flips/min — suits 100–130 BPM Schlager/rock */

/* Volume tier smoothing for quiet/mid/loud classification */
#define VOL_SMOOTH_ALPHA      0.03f  /* EMA α — τ ≈ 400 ms, smooths tier transitions */

/* Section change — cosine novelty between band energy vectors */
#define NOVELTY_K             2.0f   /* threshold = novelty_mean + K × std   */
#define NOVELTY_SMOOTH        0.003f /* smooth_vec α — time constant ~4 s    */
#define NOVELTY_MIN_GAP_MS    5000   /* minimum 5 s between section changes  */

/* Layer strengths */
#define AMBIENT_BASE          0.06f  /* near-dark between beats — maximum contrast     */
#define BEAT_FLASH_PEAK       1.00f  /* beat-pulse peak — full brightness on kick      */
#define BEAT_DECAY_K          10.0f  /* very fast decay — beat dies in ~100 ms         */
#define BASS_WAVE_PEAK        0.85f  /* bass blob peak brightness             */
#define SPARKLE_THRESHOLD     0.25f  /* lower threshold — more frequent sparkles      */
#define SPARKLE_DECAY         0.82f  /* faster decay — crisp bright pop               */
#define MAX_SPARKLES          14     /* more simultaneous sparkles                    */

/* Welcome animation */
#define WELCOME_RAMP_MS      10000   /* speed ramp from slow→fast (ms)        */
#define WELCOME_PASS_MS_MAX   3000   /* slowest pass: one sweep takes 3 s     */
#define WELCOME_PASS_MS_MIN    300   /* fastest pass: one sweep takes 0.3 s   */
#define WELCOME_TIMEOUT_MS   30000   /* safety cut-off waiting for audio end  */

/* Picture frame LED segments (0-indexed)
 *   bottom-left:  LEDs  0– 5   right: LEDs  5–12 (LED 5 = shared corner)
 *   top:          LEDs 13–18   left:  LEDs 19–24
 *   bottom-right: LEDs 25–29                                                 */
#define FRAME_BL_START   0
#define FRAME_BL_END     5
#define FRAME_R_START    5
#define FRAME_R_END     12
#define FRAME_T_START   13
#define FRAME_T_END     18
#define FRAME_L_START   19
#define FRAME_L_END     24
#define FRAME_BR_START  25
#define FRAME_BR_END    29
/* Volume thresholds (relative to song's own peak via global_norm) */
#define FRAME_QUIET_THR  0.35f   /* below → quiet: one pair at a time */
#define FRAME_LOUD_THR   0.65f   /* above → loud:  all four sides     */


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

/* Bluetooth state */
static volatile bool       s_bt_mode            = false; /* in BT audio mode        */
static volatile bool       s_bt_pairing         = false; /* discoverable window open */
static volatile bool       s_bt_connected       = false; /* A2DP link up             */
static volatile TickType_t s_pairing_start_tick = 0;
static esp_bd_addr_t       s_bt_peer_addr;               /* last connected peer      */

/* BT→I2S decoupling: A2DP callback fills this, bt_audio_task drains to codec.
 * Prevents blocking the BT task on I2S DMA waits (which drops the link).    */
static StreamBufferHandle_t s_bt_i2s_stream;
#define BT_I2S_BUF_BYTES  (8 * 1024)    /* ~45 ms at 44100 Hz stereo 16-bit */

/* Set false on STARTED/STOPPED; bt_audio_task sets true + unmutes PA on first
 * real data frame — prevents stale-DMA blurp between connect and first play. */
static volatile bool s_bt_audio_active = false;

/* Welcome sequence (SD boot only) */
static volatile bool       s_welcome_mode       = false;
static volatile TickType_t s_welcome_start_tick = 0;

/* BT connect: 3 short blue blinks then straight into normal show */
static volatile bool       s_bt_just_connected  = false;

/* ------------------------------------------------------------------ */
/*  Bluetooth A2DP sink                                                 */
/* ------------------------------------------------------------------ */

/* A2DP data callback — runs in BT task context (time-critical).
 * Must NOT block on I2S DMA — push to buffers with timeout=0 and return fast. */
static void a2dp_data_cb(const uint8_t *buf, uint32_t len)
{
    if (!s_bt_mode) return;

    /* Push raw PCM to I2S write task (non-blocking — never stall BT timing) */
    if (s_bt_i2s_stream)
        xStreamBufferSend(s_bt_i2s_stream, buf, len, 0);

    /* Push mono-float to LED stream buffer for the visualiser */
    if (s_pcm_stream) {
        const int16_t *src    = (const int16_t *)buf;
        int            frames = (int)(len / 4);
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
}

/* BT audio write task — drains s_bt_i2s_stream to codec on Core 1.
 * Keeps I2S DMA waits out of the BT task completely.
 * Unmutes the PA on the first real data frame to avoid blurp noise. */
static void bt_audio_task(void *arg)
{
    static uint8_t i2s_buf[1024];
    while (1) {
        if (!s_bt_mode || !s_bt_i2s_stream) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        size_t got = xStreamBufferReceive(s_bt_i2s_stream, i2s_buf,
                                          sizeof(i2s_buf), pdMS_TO_TICKS(100));
        if (got > 0) {
            if (!s_bt_audio_active) {
                s_bt_audio_active = true;
                gpio_set_level(PA_ENABLE_PIN, 1);   /* unmute on first real frame */
            }
            esp_codec_dev_write(s_play_dev, i2s_buf, (int)got);
        }
    }
}

/* A2DP connection / audio-state events */
static void a2dp_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            memcpy(s_bt_peer_addr, p->conn_stat.remote_bda, sizeof(esp_bd_addr_t));
            s_bt_connected      = true;
            s_bt_pairing        = false;   /* stop blinking */
            s_bt_just_connected = true;    /* led_task: 3 short blue blinks */
            ESP_LOGI(TAG, "BT A2DP connected");
        } else if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            s_bt_connected = false;
            if (s_bt_mode) {
                s_bt_mode = false;    /* phone hung up → return to SD */
                ESP_LOGI(TAG, "BT disconnected → SD mode");
            }
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        if (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
            s_track_started = true;
            if (s_bt_i2s_stream) xStreamBufferReset(s_bt_i2s_stream);
            s_bt_audio_active = false;  /* bt_audio_task unmutes on first frame */
            ESP_LOGI(TAG, "BT audio started");
        } else if (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED) {
            s_bt_audio_active = false;
            gpio_set_level(PA_ENABLE_PIN, 0);
        }
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        /* Reconfigure codec for negotiated sample rate.
         * SBC CIE byte 0 sampling-frequency bits are a one-hot bitmask:
         *   bit7=16k  bit6=32k  bit5=44.1k  bit4=48k                  */
        if (p->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            uint8_t sf = p->audio_cfg.mcc.cie.sbc[0];
            int rate;
            if      (sf & (1 << 5)) rate = 44100;
            else if (sf & (1 << 4)) rate = 48000;
            else if (sf & (1 << 6)) rate = 32000;
            else                    rate = 16000;
            ESP_LOGI(TAG, "BT SBC rate: %d Hz", rate);
            s_sample_rate = rate;
            esp_codec_dev_close(s_play_dev);
            esp_codec_dev_sample_info_t fs = {
                .sample_rate = rate, .channel = 2, .bits_per_sample = 16
            };
            esp_codec_dev_open(s_play_dev, &fs);
            /* esp_codec_dev_open re-enables pa_pin — force mute;
             * bt_audio_task will unmute when first audio frame arrives */
            gpio_set_level(PA_ENABLE_PIN, 0);
        }
        break;
    default:
        break;
    }
}

/* GAP callback — auto-confirm SSP (Just Works, no PIN display needed) */
static void gap_event_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *p)
{
    switch (event) {
    case ESP_BT_GAP_CFM_REQ_EVT:
        esp_bt_gap_ssp_confirm_reply(p->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (p->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "BT paired: %s", p->auth_cmpl.device_name);
        else
            ESP_LOGW(TAG, "BT auth failed: %d", p->auth_cmpl.stat);
        break;
    default:
        break;
    }
}

/* Enter BT pairing mode — call from button_task */
static void bt_enter_pairing(void)
{
    ESP_LOGI(TAG, "BT pairing mode — %d s window", BT_PAIRING_MS / 1000);
    gpio_set_level(PA_ENABLE_PIN, 0);     /* mute amp — silence MP3 flush tail */
    s_bt_mode            = true;
    s_bt_pairing         = true;
    s_pairing_start_tick = xTaskGetTickCount();
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
}

/* Exit BT mode — call from button_task or timeout handler */
static void bt_exit_mode(void)
{
    ESP_LOGI(TAG, "BT mode off → SD");
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (s_bt_connected)
        esp_a2d_sink_disconnect(s_bt_peer_addr);
    s_bt_pairing   = false;
    s_bt_connected = false;
    s_bt_mode      = false;   /* main loop sees this and restarts SD */
}

/* One-time BT stack init — call after audio_init() */
static void bt_init(void)
{
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* IO capability: no input/no output → "Just Works" SSP, no PIN needed */
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_event_cb));
    ESP_ERROR_CHECK(esp_a2d_register_callback(a2dp_event_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(a2dp_data_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_init());

    esp_bt_gap_set_device_name(BT_DEVICE_NAME);
    /* Start non-discoverable; long-press KEY1 enables pairing */
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    ESP_LOGI(TAG, "BT A2DP sink ready — long-press KEY1 (2 s) to pair");
}

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
/*  Welcome sequence — plays _welcome.mp3 (if found) + LED animation   */
/*  Call from app_main (SD mode) or trigger via flags (BT mode).       */
/* ------------------------------------------------------------------ */

static void play_welcome_sequence(void)
{
    s_welcome_start_tick = xTaskGetTickCount();
    s_welcome_mode       = true;

    char path[64];
    snprintf(path, sizeof(path), "%s/_welcome.mp3", MOUNT_POINT);
    FILE *fp = fopen(path, "rb");
    if (fp) {
        ESP_LOGI(TAG, "Welcome: playing %s", path);
        audio_player_play(fp);
        /* Block until track ends (player_event_cb gives s_track_done on IDLE) */
        xSemaphoreTake(s_track_done, pdMS_TO_TICKS(WELCOME_TIMEOUT_MS));
    } else {
        ESP_LOGI(TAG, "Welcome: no _welcome.mp3 — LED only (5 s)");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    xStreamBufferReset(s_pcm_stream);
    s_welcome_mode = false;
}

/* ------------------------------------------------------------------ */
/*  LED show task — runs on core 0                                      */
/* ------------------------------------------------------------------ */

static void led_task(void *arg)
{
    /* ---- BSS work buffers ---- */
    static float     fft_buf[FFT_SIZE * 2];
    static float     pcm_buf[FFT_SIZE];
    static float     hann_win[FFT_SIZE];
    static float     band_energy[NUM_BANDS];
    static float     band_peak[NUM_BANDS];
    static float     band_norm[NUM_BANDS];
    static float     smooth_vec[NUM_BANDS];   /* slow reference for novelty   */
    static sparkle_t sparkles[MAX_SPARKLES];
    static uint8_t   r_buf[LED_COUNT], g_buf[LED_COUNT], b_buf[LED_COUNT];

    float      welcome_pos      = 0.0f;   /* welcome sweep position                   */
    bool       welcome_prev     = false;  /* detects welcome mode entry               */
    bool       frame_phase      = false;  /* flips on beat — drives frame animation   */
    TickType_t last_frame_tick  = 0;      /* guards frame phase min gap               */
    float      vol_smooth       = 0.0f;   /* slow EMA of global_norm for tier logic   */
    int        vol_tier         = 0;      /* 0=quiet  1=mid  2=loud (with hysteresis) */
    TickType_t last_snap_tick   = 0;      /* guards disco hue snap min gap            */
    int        dbg_ctr          = 0;      /* diagnostic log counter                   */
    uint32_t   dbg_beats        = 0;      /* beats since last log line                */

    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, FFT_SIZE));
    dsps_wind_hann_f32(hann_win, FFT_SIZE);

    for (int i = 0; i < NUM_BANDS; i++) band_peak[i] = BAND_AGC_MIN;
    memset(band_energy, 0, sizeof(band_energy));
    memset(band_norm,   0, sizeof(band_norm));
    memset(smooth_vec,  0, sizeof(smooth_vec));
    memset(sparkles,    0, sizeof(sparkles));

    /* Beat */
    float      kick_fast      = 0.0f;    /* fast EMA of sub-bass energy  */
    float      kick_slow      = 0.001f;  /* slow EMA — background level  */
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
    float      hue_offset    = 0.0f;   /* ambient layer rotation */
    float      side_hues[4]  = {0.0f, 90.0f, 180.0f, 270.0f}; /* bottom/right/top/left — randomised on beat */
    float      wave_pos      = 0.0f;   /* bass blob position     */

    while (1) {

        /* ---- BT pairing mode: all LEDs blink blue, skip audio analysis ---- */
        if (s_bt_pairing) {
            uint32_t ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
            uint8_t  v  = ((ms / 500u) & 1u) ? 0u : 40u;   /* 1 Hz blink */
            for (int i = 0; i < LED_COUNT; i++)
                led_strip_set_pixel(s_led_strip, i, 0, 0, v);
            led_strip_refresh(s_led_strip);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* ---- Welcome animation: single LED sweep, speed ramps up over 10 s ---- */
        if (s_welcome_mode) {
            /* Detect entry → reset sweep position */
            if (!welcome_prev) {
                welcome_pos  = 0.0f;
                welcome_prev = true;
            }

            uint32_t wel_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - s_welcome_start_tick);

            /* Speed ramp: 3000 ms/pass → 300 ms/pass over WELCOME_RAMP_MS */
            float t = (float)wel_ms / (float)WELCOME_RAMP_MS;
            if (t > 1.0f) t = 1.0f;
            float pass_ms = WELCOME_PASS_MS_MAX - t * (WELCOME_PASS_MS_MAX - WELCOME_PASS_MS_MIN);
            welcome_pos += (25.0f / pass_ms) * 20.0f;  /* advance per ~20 ms frame */
            if (welcome_pos >= 25.0f) welcome_pos -= 25.0f;

            float hue = 330.0f;   /* pink */
            led_strip_clear(s_led_strip);
            int head = (int)welcome_pos;
            const float trail[4] = { 1.0f, 0.50f, 0.20f, 0.07f };
            for (int tr = 0; tr < 4; tr++) {
                int pos = (head - tr + 25) % 25;  /* wrap within LEDs 0–24 */
                uint8_t r, g, b;
                hsv_to_rgb(hue, 1.0f, trail[tr], &r, &g, &b);
                led_strip_set_pixel(s_led_strip, pos, r, g, b);
            }
            led_strip_refresh(s_led_strip);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        welcome_prev = false;   /* reset detector when not in welcome mode */

        /* ---- BT connected: 3 short blue blinks then straight to normal show ---- */
        if (s_bt_just_connected) {
            s_bt_just_connected = false;
            for (int b = 0; b < 3; b++) {
                for (int i = 0; i < LED_COUNT; i++)
                    led_strip_set_pixel(s_led_strip, i, 0, 0, 40);
                led_strip_refresh(s_led_strip);
                vTaskDelay(pdMS_TO_TICKS(80));
                led_strip_clear(s_led_strip);
                led_strip_refresh(s_led_strip);
                vTaskDelay(pdMS_TO_TICKS(80));
            }
            xStreamBufferReset(s_pcm_stream);
            continue;
        }

        /* ---- New track: full state reset ---- */
        if (s_track_started) {
            s_track_started = false;
            for (int i = 0; i < NUM_BANDS; i++) {
                band_peak[i]  = BAND_AGC_MIN;
                band_norm[i]  = 0.0f;
                smooth_vec[i] = 0.0f;
            }
            memset(sparkles, 0, sizeof(sparkles));
            kick_fast      = 0.0f;
            kick_slow      = 0.001f;
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
            /* Stream momentarily dry — hold last rendered frame, don't blank */
            beat_strength *= 0.7f;
            global_rms_s  *= 0.9f;
            continue;
        }

        /* ---- Global RMS brightness gate ---- */
        float rms_sq = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) rms_sq += pcm_buf[i] * pcm_buf[i];
        float rms = sqrtf(rms_sq / FFT_SIZE);

        /* Silence floor: BT codec and quiet streams produce near-zero noise.
         * Below this level there is nothing meaningful to visualise.          */
        if (rms < SILENCE_FLOOR) {
            /* True silence — blank strip and decay state */
            beat_strength *= 0.7f;
            global_rms_s  *= 0.9f;
            led_strip_clear(s_led_strip);
            led_strip_refresh(s_led_strip);
            continue;
        }

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

        /* ---- Band energies (single pass over FFT bins) ---- */
        float hz_per_bin = (float)s_sample_rate / FFT_SIZE;
        float band_count[NUM_BANDS] = {0};
        memset(band_energy, 0, sizeof(band_energy));

        for (int k = 1; k < FFT_SIZE / 2; k++) {
            float re  = fft_buf[k * 2], im = fft_buf[k * 2 + 1];
            float mag = sqrtf(re * re + im * im);
            float hz  = k * hz_per_bin;
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

        /* ---- Beat detection — dual-EMA sub-bass ratio ----
         * Sub-bass energy (bands 0+1, 60–400 Hz) contains the kick drum.
         * kick_fast tracks the current attack; kick_slow tracks the background.
         * A beat fires when the ratio kick_fast/kick_slow exceeds KICK_RATIO,
         * meaning a genuine transient above the song's running bass level.     */
        float kick = band_energy[0] + band_energy[1];
        kick_fast = KICK_ALPHA_FAST * kick + (1.0f - KICK_ALPHA_FAST) * kick_fast;
        kick_slow = KICK_ALPHA_SLOW * kick + (1.0f - KICK_ALPHA_SLOW) * kick_slow;

        bool beat = (pdTICKS_TO_MS(now - last_beat_tick) > BEAT_MIN_GAP_MS) &&
                    (kick_fast > kick_slow * KICK_RATIO)                      &&
                    (kick_slow > KICK_ABS_MIN);

        if (beat) {
            dbg_beats++;
            uint32_t interval = (uint32_t)pdTICKS_TO_MS(now - last_beat_tick);
            if (interval > 250 && interval < 1500)
                beat_period_ms = (beat_period_ms * 7 + interval) >> 3;
            last_beat_tick = now;
            beat_strength  = 1.0f;
            /* On each snap: assign 4 random hues with ~80-120° separation so
             * all four frame sides are always visually distinct.             */
            if (pdTICKS_TO_MS(now - last_snap_tick) >= DISCO_SNAP_GAP_MS) {
                float base      = (float)(esp_random() % 360);
                side_hues[0]    = base;
                side_hues[1]    = fmodf(base +  80.0f + (float)(esp_random() % 40), 360.0f);
                side_hues[2]    = fmodf(base + 170.0f + (float)(esp_random() % 40), 360.0f);
                side_hues[3]    = fmodf(base + 255.0f + (float)(esp_random() % 40), 360.0f);
                hue_offset      = base;
                last_snap_tick  = now;
            }
            /* Frame phase: flip only if enough time has passed since last flip */
            if (pdTICKS_TO_MS(now - last_frame_tick) >= FRAME_BEAT_GAP_MS) {
                frame_phase    = !frame_phase;
                last_frame_tick = now;
            }
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
            ESP_LOGI(TAG, "Section change detected");
        }

        /* ---- Ambient rotation speed (BPM-relative) ---- */
        float bpm = 60000.0f / (float)beat_period_ms;
        hue_offset = fmodf(hue_offset + 0.5f * (bpm / 120.0f), 360.0f);

        /* ==== Layer 1: Ambient base (180° spread across strip) ==== */
        float amb_v = AMBIENT_BASE * global_norm;
        for (int i = 0; i < LED_COUNT; i++) {
            float hue = fmodf(hue_offset + 180.0f * i / (LED_COUNT - 1), 360.0f);
            hsv_to_rgb(hue, 1.0f, amb_v, &r_buf[i], &g_buf[i], &b_buf[i]);
        }

        /* ==== Layer 2: Beat pulse — random hue per side ====
         * side_hues[4] are randomised on each disco snap with ~80-120°
         * separation → 4 clearly distinct colours every beat.
         * vol_tier drives saturation: quiet=pastel, mid=normal, loud=vivid. */
        if (beat_strength > 0.01f) {
            float sat = (vol_tier == 0) ? 0.45f : (vol_tier == 1) ? 0.80f : 1.00f;
            float vsc = (vol_tier == 0) ? 0.60f : (vol_tier == 1) ? 0.85f : 1.00f;
            float v   = beat_strength * BEAT_FLASH_PEAK * vsc;
            uint8_t rc[4], gc[4], bc[4];
            for (int s = 0; s < 4; s++) hsv_to_rgb(side_hues[s], sat, v, &rc[s], &gc[s], &bc[s]);

#define BEAT_SIDE(seg_start, seg_end, ci) \
    for (int k = (seg_start); k <= (seg_end); k++) { \
        r_buf[k] = clamp8((int)r_buf[k] + rc[ci]); \
        g_buf[k] = clamp8((int)g_buf[k] + gc[ci]); \
        b_buf[k] = clamp8((int)b_buf[k] + bc[ci]); }

            BEAT_SIDE(FRAME_BL_START, FRAME_BL_END, 0)   /* bottom */
            BEAT_SIDE(FRAME_BR_START, FRAME_BR_END, 0)   /* bottom */
            BEAT_SIDE(FRAME_R_START,  FRAME_R_END,  1)   /* right  */
            BEAT_SIDE(FRAME_T_START,  FRAME_T_END,  2)   /* top    */
            BEAT_SIDE(FRAME_L_START,  FRAME_L_END,  3)   /* left   */
#undef BEAT_SIDE
        }

        /* ==== Layer 3: Bass wave (warm Gaussian blob drifting along strip) ==== */
        float bass_level = (band_norm[0] + band_norm[1]) * 0.5f * global_norm;
        wave_pos = fmodf(wave_pos + 0.1f + band_norm[0] * 0.4f, (float)LED_COUNT);
        float wave_hue = fmodf(hue_offset + 15.0f, 360.0f);
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
                    sparkles[s].hue    = fmodf(hue_offset + 120.0f +
                                               (float)(esp_random() % 120) - 60.0f, 360.0f);
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

        /* Frame brightness kept outside block so output section can use them */
        float frame_blr = 0.0f, frame_btb = 0.0f;

        /* ==== Layer 5: Picture frame — beat-driven, volume-relative ====
         *  vol_smooth is a slow EMA of global_norm (τ≈400 ms) so tier transitions
         *  are gradual. Hysteresis prevents rapid oscillation at tier boundaries.
         *  Quiet: one pair (L+R or T+B) swaps each beat.
         *  Mid:   both pairs on, brightness alternates each beat.
         *  Loud:  all four sides full.                                              */
        {
            /* Update slow volume EMA */
            vol_smooth += VOL_SMOOTH_ALPHA * (global_norm - vol_smooth);

            /* Hysteresis state machine — different thresholds for up/down transitions.
             * global_norm = smoothed_RMS / peak_RMS, typical range 0.05–0.40 in music.
             * Thresholds tuned to that range so all three tiers are reachable.        */
            switch (vol_tier) {
            case 0: if (vol_smooth > 0.22f) vol_tier = 1; break;  /* quiet → mid  */
            case 1: if      (vol_smooth > 0.40f) vol_tier = 2;    /* mid   → loud */
                    else if (vol_smooth < 0.12f) vol_tier = 0;    /* mid   → quiet */
                    break;
            case 2: if (vol_smooth < 0.28f) vol_tier = 1; break;  /* loud  → mid  */
            }

            /* Diagnostic log — every ~2 s (≈170 FFT frames at 44100/512) */
            if (++dbg_ctr >= 170) {
                dbg_ctr = 0;
                ESP_LOGI(TAG, "DBG norm=%.3f vol_s=%.3f tier=%d "
                              "kf=%.4f ks=%.4f ratio=%.1f beats=%u",
                         global_norm, vol_smooth, vol_tier,
                         kick_fast, kick_slow,
                         (kick_slow > 0.0001f) ? kick_fast / kick_slow : 0.0f,
                         dbg_beats);
                dbg_beats = 0;
            }

            float bright_lr, bright_tb;
            if (vol_tier == 2) {
                bright_lr = bright_tb = 0.85f;
            } else if (vol_tier == 1) {
                bright_lr = frame_phase ? 0.85f : 0.30f;
                bright_tb = frame_phase ? 0.30f : 0.85f;
            } else {
                bright_lr = frame_phase ? 0.0f  : 0.85f;
                bright_tb = frame_phase ? 0.85f : 0.0f;
            }

            /* Per-side hues from side_hues[]: bottom=0, right=1, top=2, left=3 */
            frame_blr = bright_lr;
            frame_btb = bright_tb;
            float f_sat = (vol_tier == 0) ? 0.50f : (vol_tier == 1) ? 0.80f : 1.00f;
            uint8_t fs_r[4], fs_g[4], fs_b[4];
            float frame_bright[4] = { bright_tb, bright_lr, bright_tb, bright_lr };
            for (int s = 0; s < 4; s++)
                hsv_to_rgb(side_hues[s], f_sat, frame_bright[s], &fs_r[s], &fs_g[s], &fs_b[s]);
            uint8_t frl = fs_r[3], fgl = fs_g[3], fbl = fs_b[3]; /* left   */
            uint8_t frl_r = fs_r[1], fgl_r = fs_g[1], fbl_r = fs_b[1]; /* right  */
            uint8_t frt = fs_r[2], fgt = fs_g[2], fbt = fs_b[2]; /* top    */

            /* Each side: its own colour from side_hues[] */
            for (int k = FRAME_R_START;  k <= FRAME_R_END;  k++) { r_buf[k]=clamp8(r_buf[k]+frl_r); g_buf[k]=clamp8(g_buf[k]+fgl_r); b_buf[k]=clamp8(b_buf[k]+fbl_r); }
            for (int k = FRAME_L_START;  k <= FRAME_L_END;  k++) { r_buf[k]=clamp8(r_buf[k]+frl);   g_buf[k]=clamp8(g_buf[k]+fgl);   b_buf[k]=clamp8(b_buf[k]+fbl);   }
            for (int k = FRAME_BL_START; k <= FRAME_BL_END; k++) { r_buf[k]=clamp8(r_buf[k]+fs_r[0]); g_buf[k]=clamp8(g_buf[k]+fs_g[0]); b_buf[k]=clamp8(b_buf[k]+fs_b[0]); }
            for (int k = FRAME_T_START;  k <= FRAME_T_END;  k++) { r_buf[k]=clamp8(r_buf[k]+frt);   g_buf[k]=clamp8(g_buf[k]+fgt);   b_buf[k]=clamp8(b_buf[k]+fbt);   }
            for (int k = FRAME_BR_START; k <= FRAME_BR_END; k++) { r_buf[k]=clamp8(r_buf[k]+fs_r[0]); g_buf[k]=clamp8(g_buf[k]+fs_g[0]); b_buf[k]=clamp8(b_buf[k]+fs_b[0]); }
        }

        /* ==== Volume-driven brightness scale: 0–100 % (no floor) ====
         * No minimum — strips go dark between beats naturally.          */
        {
            float bs = global_norm;
            for (int i = 0; i < LED_COUNT; i++) {
                r_buf[i] = (uint8_t)(r_buf[i] * bs);
                g_buf[i] = (uint8_t)(g_buf[i] * bs);
                b_buf[i] = (uint8_t)(b_buf[i] * bs);
            }
        }

        /* ==== Output — strobe gate ====
         * Beat burst (layers 1-4) strobes on/off.
         * Frame layer (layer 5) stays visible at dim level between beats,
         * showing active sides based on vol_tier.                         */
        if (beat_strength < 0.04f) {
            /* Between beats: frame-only, per-side random hues at dim level */
            led_strip_clear(s_led_strip);
            float dim = global_norm * 0.35f;
            float f_sat_d = (vol_tier == 0) ? 0.50f : (vol_tier == 1) ? 0.80f : 1.00f;
            uint8_t ds_r[4], ds_g[4], ds_b[4];
            float db[4] = { frame_btb * dim, frame_blr * dim, frame_btb * dim, frame_blr * dim };
            for (int s = 0; s < 4; s++) hsv_to_rgb(side_hues[s], f_sat_d, db[s], &ds_r[s], &ds_g[s], &ds_b[s]);
            for (int k = FRAME_BL_START; k <= FRAME_BL_END; k++) led_strip_set_pixel(s_led_strip, k, ds_r[0], ds_g[0], ds_b[0]);
            for (int k = FRAME_BR_START; k <= FRAME_BR_END; k++) led_strip_set_pixel(s_led_strip, k, ds_r[0], ds_g[0], ds_b[0]);
            for (int k = FRAME_R_START;  k <= FRAME_R_END;  k++) led_strip_set_pixel(s_led_strip, k, ds_r[1], ds_g[1], ds_b[1]);
            for (int k = FRAME_T_START;  k <= FRAME_T_END;  k++) led_strip_set_pixel(s_led_strip, k, ds_r[2], ds_g[2], ds_b[2]);
            for (int k = FRAME_L_START;  k <= FRAME_L_END;  k++) led_strip_set_pixel(s_led_strip, k, ds_r[3], ds_g[3], ds_b[3]);
        } else {
            for (int i = 0; i < LED_COUNT; i++)
                led_strip_set_pixel(s_led_strip, i, r_buf[i], g_buf[i], b_buf[i]);
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

    int  pause_stable    = 0;
    bool pause_long_done = false;
    int  prev_stable     = 0;
    int  next_stable     = 0;
    int  vold_stable     = 0;
    int  volu_stable     = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));

        /* --- BT pairing window timeout --- */
        if (s_bt_pairing && !s_bt_connected) {
            if (pdTICKS_TO_MS(xTaskGetTickCount() - s_pairing_start_tick) >= BT_PAIRING_MS) {
                ESP_LOGI(TAG, "BT pairing timeout — back to SD");
                bt_exit_mode();
            }
        }

        /* --- KEY1 (GPIO36): short = pause/resume, long (2 s) = BT mode toggle --- */
        if (gpio_get_level(KEY_PAUSE_GPIO) == 0) {
            pause_stable++;
            if (pause_stable == (KEY_PAUSE_LONG_MS / 20) && !pause_long_done) {
                pause_long_done = true;
                if (!s_bt_mode) {
                    bt_enter_pairing();
                    if (s_is_paused) { s_is_paused = false; xSemaphoreGive(s_resume_sem); }
                    else             { audio_player_stop(); }
                } else {
                    bt_exit_mode();
                }
                ESP_LOGI(TAG, "KEY1 long → BT %s", s_bt_mode ? "ON" : "OFF");
                vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            }
        } else {
            if (pause_stable >= 2 && !pause_long_done && !s_bt_mode) {
                if (!s_is_paused) {
                    gpio_set_level(PA_ENABLE_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(20));
                    s_is_paused = true;
                    audio_player_stop();
                    ESP_LOGI(TAG, "KEY PAUSE → paused");
                } else {
                    s_is_paused = false;
                    xSemaphoreGive(s_resume_sem);
                    ESP_LOGI(TAG, "KEY PAUSE → resumed");
                }
                vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            }
            pause_stable    = 0;
            pause_long_done = false;
        }

        /* --- PREV key (KEY3) — SD mode only --- */
        if (gpio_get_level(KEY_PREV_GPIO) == 0) { prev_stable++; } else { prev_stable = 0; }
        if (prev_stable == 2 && !s_bt_mode) {
            prev_stable = 99;
            int target = (s_track_idx - 1 + s_track_count) % s_track_count;
            s_next_track_override = target;
            ESP_LOGI(TAG, "KEY PREV → track %d", target + 1);
            if (s_is_paused) { s_is_paused = false; xSemaphoreGive(s_resume_sem); }
            else              { audio_player_stop(); }
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            prev_stable = 0;
        }

        /* --- NEXT key (KEY4) — SD mode only --- */
        if (gpio_get_level(KEY_NEXT_GPIO) == 0) { next_stable++; } else { next_stable = 0; }
        if (next_stable == 2 && !s_bt_mode) {
            next_stable = 99;
            int target = (s_track_idx + 1) % s_track_count;
            s_next_track_override = target;
            ESP_LOGI(TAG, "KEY NEXT → track %d", target + 1);
            if (s_is_paused) { s_is_paused = false; xSemaphoreGive(s_resume_sem); }
            else              { audio_player_stop(); }
            vTaskDelay(pdMS_TO_TICKS(BTN_LOCKOUT_MS));
            next_stable = 0;
        }

        /* --- VOL DOWN key (KEY5) — works in both modes --- */
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

        /* --- VOL UP key (KEY6) — works in both modes --- */
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
        if (ent->d_name[0] == '_') continue;   /* skip reserved files (_welcome.mp3 etc.) */
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

    /* Sort playlist alphabetically so track order on card matches filename order */
    if (s_track_count > 1) {
        qsort(s_tracks, s_track_count, sizeof(s_tracks[0]),
              (int (*)(const void *, const void *))strcmp);
    }

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

    /* NVS required by BT stack for bonding info */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

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

    /* BT→I2S decoupling buffer — drained by bt_audio_task on Core 1 */
    s_bt_i2s_stream = xStreamBufferCreate(BT_I2S_BUF_BYTES, 512);
    assert(s_bt_i2s_stream);

    audio_init();

    /* LED show task on core 0, priority below audio decode (5) */
    xTaskCreatePinnedToCore(led_task, "led_fft", 3072, NULL, 3, NULL, 0);

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

    bt_init();

    ESP_LOGI(TAG, "Starting playlist (%d tracks)", s_track_count);

    /* BT audio write task on Core 1 (same core as MP3 decode, away from BT stack) */
    xTaskCreatePinnedToCore(bt_audio_task, "bt_audio", 2048, NULL, 4, NULL, 1);

    /* Button task on core 0, lowest priority */
    xTaskCreatePinnedToCore(button_task, "buttons", 2048, NULL, 2, NULL, 0);

    /* Welcome sequence: play _welcome.mp3 (if on SD) + pink LED sweep */
    play_welcome_sequence();

    while (1) {
        /* ---- BT mode: A2DP data callback drives audio, spin here ---- */
        while (s_bt_mode) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }

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
