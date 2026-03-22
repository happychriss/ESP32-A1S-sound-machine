# chmorgan/esp-audio-player

Component registry: `chmorgan/esp-audio-player`
Version confirmed working: **1.1.0** (pulls `chmorgan/esp-libhelix-mp3` 1.0.3 automatically)
Platform: ESP32, ESP-IDF ≥ 5.0

Working reference: `/workspace/src/player/`

---

## What it does

Decodes MP3 files from a `FILE *` using the Helix MP3 decoder. Calls user-supplied callbacks for PCM output, clock reconfiguration, and mute. Runs in its own FreeRTOS task.

---

## idf_component.yml

```yaml
dependencies:
  chmorgan/esp-audio-player: '*'
```

`libhelix-mp3` is pulled automatically as a transitive dependency — do not add it manually.

---

## API

### Config struct

```c
typedef struct {
    audio_player_mute_fn        mute_fn;      // optional: NULL if unused
    audio_reconfig_std_clock    clk_set_fn;   // called on sample-rate change
    audio_player_write_fn       write_fn;     // called with decoded PCM frames
    UBaseType_t                 priority;     // FreeRTOS task priority
    BaseType_t                  coreID;       // ESP32 core (0 or 1)
    bool                        force_stereo; // upmix mono → stereo
    audio_player_write_fn2      write_fn2;    // alternative write fn with ctx
    void                       *write_ctx;    // ctx for write_fn2
} audio_player_config_t;
```

### Callback signatures (exact — must match)

```c
// PCM output
typedef esp_err_t (*audio_player_write_fn)(
    void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);

// Clock reconfiguration — ch is i2s_slot_mode_t, NOT uint32_t
typedef esp_err_t (*audio_reconfig_std_clock)(
    uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

// Mute
typedef esp_err_t (*audio_player_mute_fn)(AUDIO_PLAYER_MUTE_SETTING setting);
```

### Key functions

```c
esp_err_t audio_player_new(audio_player_config_t config);
esp_err_t audio_player_play(FILE *fp);   // see note on fclose below
esp_err_t audio_player_pause(void);
esp_err_t audio_player_resume(void);
esp_err_t audio_player_stop(void);
esp_err_t audio_player_delete(void);
esp_err_t audio_player_callback_register(audio_player_cb_t cb, void *user_ctx);
audio_player_state_t audio_player_get_state(void);
```

### Events (audio_player_callback_event_t)

| Event | Meaning |
|-------|---------|
| `AUDIO_PLAYER_CALLBACK_EVENT_IDLE` | Playback finished or error — safe to play next track |
| `AUDIO_PLAYER_CALLBACK_EVENT_PLAYING` | Playback started |
| `AUDIO_PLAYER_CALLBACK_EVENT_PAUSE` | Paused |
| `AUDIO_PLAYER_CALLBACK_EVENT_COMPLETED_PLAYING_NEXT` | Track transition |
| `AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN_FILE_TYPE` | Unrecognised file |

---

## Critical notes

### fclose ownership
`audio_player_play(fp)` takes ownership of `fp`. The library calls `fclose()` when playback ends (including on error). **The caller must NOT call `fclose()` after `audio_player_play()`.**

### clk_set_fn ch parameter
The third parameter is `i2s_slot_mode_t`, not `uint32_t`. Use:
```c
int channels = (ch == I2S_SLOT_MODE_STEREO) ? 2 : 1;
```
Inside `clk_set_fn`, call `esp_codec_dev_close()` then `esp_codec_dev_open()` with the new rate/bits/channels.

### force_stereo
Set `true` for ES8388 — it expects stereo data. Without this, mono MP3s play at double speed on one channel.

### Track completion
Use a semaphore: register a callback, `xSemaphoreGive` on `AUDIO_PLAYER_CALLBACK_EVENT_IDLE`, `xSemaphoreTake(portMAX_DELAY)` in the playlist loop.

---

## Minimal integration (ES8388)

```c
audio_player_config_t cfg = {
    .mute_fn      = NULL,
    .write_fn     = my_write_cb,      // calls esp_codec_dev_write()
    .clk_set_fn   = my_clk_set_cb,    // calls esp_codec_dev_close/open
    .priority     = 5,
    .coreID       = 1,
    .force_stereo = true,
};
ESP_ERROR_CHECK(audio_player_new(cfg));
audio_player_callback_register(my_event_cb, NULL);

// play a file
FILE *fp = fopen("/sdcard/song.mp3", "rb");
audio_player_play(fp);  // do NOT fclose(fp)
```
