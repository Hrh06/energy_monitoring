# ==================== main/CMakeLists.txt ====================

idf_component_register(
    SRCS 
        "main.c"
        "nvs_storage.c"
        "wifi_manager.c"
        "mqtt_client_own.c"
        "relay_control.c"
        "energy_meter.c"      
        "tft_display.c"
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

# Check if custom certificates exist with detailed error messages
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/ca_cert.crt")
    message(FATAL_ERROR "❌ CA certificate not found!")
    message(FATAL_ERROR "   Expected: ${CMAKE_SOURCE_DIR}/certs/ca_cert.crt")
    message(FATAL_ERROR "   Please add your CA certificate to verify the MQTT broker.")
endif()

if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/esp32_cert.crt")
    message(FATAL_ERROR "❌ ESP32 client certificate not found!")
    message(FATAL_ERROR "   Expected: ${CMAKE_SOURCE_DIR}/certs/esp32_cert.crt")
    message(FATAL_ERROR "   Please add your ESP32 client certificate for authentication.")
endif()

if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/esp32_key.key")
    message(FATAL_ERROR "❌ ESP32 private key not found!")
    message(FATAL_ERROR "   Expected: ${CMAKE_SOURCE_DIR}/certs/esp32_key.key")
    message(FATAL_ERROR "   Please add your ESP32 private key for encryption.")
endif()

# Validate certificate file sizes (basic check)
file(SIZE "${CMAKE_SOURCE_DIR}/certs/ca_cert.crt" CA_CERT_SIZE)
file(SIZE "${CMAKE_SOURCE_DIR}/certs/esp32_cert.crt" ESP32_CERT_SIZE)
file(SIZE "${CMAKE_SOURCE_DIR}/certs/esp32_key.key" ESP32_KEY_SIZE)

if(CA_CERT_SIZE LESS 100)
    message(FATAL_ERROR "❌ CA certificate file is too small (${CA_CERT_SIZE} bytes) - possibly empty or corrupted")
endif()

if(ESP32_CERT_SIZE LESS 100)
    message(FATAL_ERROR "❌ ESP32 certificate file is too small (${ESP32_CERT_SIZE} bytes) - possibly empty or corrupted")
endif()

if(ESP32_KEY_SIZE LESS 100)
    message(FATAL_ERROR "❌ ESP32 private key file is too small (${ESP32_KEY_SIZE} bytes) - possibly empty or corrupted")
endif()

message(STATUS "✅ All custom certificates found and will be embedded")
message(STATUS "   - CA Certificate: ${CMAKE_SOURCE_DIR}/certs/ca_cert.crt")
message(STATUS "   - ESP32 Certificate: ${CMAKE_SOURCE_DIR}/certs/esp32_cert.crt") 
message(STATUS "   - ESP32 Private Key: ${CMAKE_SOURCE_DIR}/certs/esp32_key.key")

# Additional validation - check for PEM format markers
file(READ "${CMAKE_SOURCE_DIR}/certs/ca_cert.crt" CA_CERT_CONTENT)
file(READ "${CMAKE_SOURCE_DIR}/certs/esp32_cert.crt" ESP32_CERT_CONTENT)
file(READ "${CMAKE_SOURCE_DIR}/certs/esp32_key.key" ESP32_KEY_CONTENT)

string(FIND "${CA_CERT_CONTENT}" "-----BEGIN CERTIFICATE-----" CA_BEGIN_POS)
string(FIND "${CA_CERT_CONTENT}" "-----END CERTIFICATE-----" CA_END_POS)
string(FIND "${ESP32_CERT_CONTENT}" "-----BEGIN CERTIFICATE-----" ESP32_BEGIN_POS)
string(FIND "${ESP32_CERT_CONTENT}" "-----END CERTIFICATE-----" ESP32_END_POS)
string(FIND "${ESP32_KEY_CONTENT}" "-----BEGIN PRIVATE KEY-----" KEY_BEGIN_POS)
string(FIND "${ESP32_KEY_CONTENT}" "-----END PRIVATE KEY-----" KEY_END_POS)

if(CA_BEGIN_POS EQUAL -1 OR CA_END_POS EQUAL -1)
    message(FATAL_ERROR "❌ CA certificate is not in valid PEM format (missing BEGIN/END markers)")
endif()

if(ESP32_BEGIN_POS EQUAL -1 OR ESP32_END_POS EQUAL -1)
    message(FATAL_ERROR "❌ ESP32 certificate is not in valid PEM format (missing BEGIN/END markers)")
endif()

if(KEY_BEGIN_POS EQUAL -1 OR KEY_END_POS EQUAL -1)
    message(FATAL_ERROR "❌ ESP32 private key is not in valid PEM format (missing BEGIN/END markers)")
endif()

message(STATUS "✅ Certificate format validation passed - all files are valid PEM format")
message(STATUS "🔒 SSL/TLS encryption will be enabled with custom certificates")