idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver log espressif__esp_codec_dev
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
