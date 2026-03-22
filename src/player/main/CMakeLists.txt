idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        fatfs
        log
        esp_timer
        nvs_flash
        bt
        espressif__esp_codec_dev
        chmorgan__esp-audio-player
        espressif__led_strip
)
