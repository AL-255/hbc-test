idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_rmt esp_driver_gpio esp_driver_ledc
                                     esp_driver_gptimer esp_timer esp_system esp_hw_support soc)
