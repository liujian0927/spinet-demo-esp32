idf_component_register(SRCS   "spi_eth_modem/spi_eth_modem.cc"
                             "main.cc"
                    PRIV_REQUIRES mbedtls esp_driver_spi esp_driver_gpio esp_psram esp_mm esp_pm esp_http_client spi_flash esp_app_format esp_timer app_update
                    INCLUDE_DIRS "." "spi_eth_modem")
