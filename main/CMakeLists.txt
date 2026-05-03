idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt esp_hid driver esp_timer nvs_flash)
