# ==================== main/CMakeLists.txt ====================

idf_component_register(
    SRCS 
        "main.c"
        "nvs_storage.c"
        "wifi_manager.c"
        "mqtt_client_own.c"
        "relay_control.c"
        "energy_meter.c"      
    INCLUDE_DIRS 
        "."
    REQUIRES 
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        esp-tls
        mqtt
        json
        driver
        esp_adc_cal
        esp_timer
        freertos
        log
        esp_system
        mbedtls
        lwip
        app_update
        bootloader_support
        esp_hw_support
        hal
        esp_common
        wpa_supplicant
        tcp_transport
        esp_ringbuf
        heap
        pthread
        esp_pm
        esp_rom        # Added for ESP-IDF v4.4
        soc            # Added for ESP-IDF v4.4
        xtensa         # Added for ESP-IDF v4.4
        newlib         # Added for ESP-IDF v4.4
        esp_ipc        # Added for ESP-IDF v4.4
        esp_phy        # Added for ESP-IDF v4.4
        efuse          # Added for ESP-IDF v4.4
        esp32          # Added for ESP-IDF v4.4
        vfs            # Added for ESP-IDF v4.4
        esp_eth        # Added for ESP-IDF v4.4
        tcpip_adapter  # Added for ESP-IDF v4.4
        app_trace      # Added for ESP-IDF v4.4
        spi_flash      # Added for ESP-IDF v4.4
        nghttp         # Added for ESP-IDF v4.4
    EMBED_FILES
        "../certs/ca_cert.crt"
        "../certs/esp32_cert.crt"
        "../certs/esp32_key.key"
)

# Auto-create certificates if they don't exist
execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/certs"
)

# Check if your custom certificates exist
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/ca_cert.crt")
    message(FATAL_ERROR "❌ ca.crt not found in certs/ directory. Please add your CA certificate.")
endif()

if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/esp32_cert.crt")
    message(FATAL_ERROR "❌ esp32.crt not found in certs/ directory. Please add your ESP32 client certificate.")
endif()

if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/esp32_key.key")
    message(FATAL_ERROR "❌ esp32.key not found in certs/ directory. Please add your ESP32 private key.")
endif()

message(STATUS "✅ All custom certificates found and will be embedded")
message(STATUS "   - CA Certificate: ${CMAKE_SOURCE_DIR}/certs/ca_cert.crt")
message(STATUS "   - ESP32 Certificate: ${CMAKE_SOURCE_DIR}/certs/esp32_cert.crt") 
message(STATUS "   - ESP32 Private Key: ${CMAKE_SOURCE_DIR}/certs/esp32_key.key")