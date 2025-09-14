# ==================== main/CMakeLists.txt ====================

idf_component_register(
    SRCS 
        "main.c"
        "nvs_storage.c"
        "wifi_manager.c"
        "mqtt_client_own.c"
        "relay_control.c"
        "energy_meter.c"
        "https_server.c"        
    INCLUDE_DIRS 
        "."
    REQUIRES 
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        esp_https_server
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
        "../certs/mqtt_broker_cert.pem"
        "../certs/server_cert.pem"
        "../certs/server_key.pem"
)

# Auto-create certificates if they don't exist
execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/certs"
)

# Create dummy MQTT certificate
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/mqtt_broker_cert.pem")
    file(WRITE "${CMAKE_SOURCE_DIR}/certs/mqtt_broker_cert.pem"
"-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIuJruydjsw2hUwsqdnksL6GjubTOB+Ureo0SKHM6v8P
-----END CERTIFICATE-----")
endif()

# Create dummy server certificate
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/server_cert.pem")
    file(WRITE "${CMAKE_SOURCE_DIR}/certs/server_cert.pem"
"-----BEGIN CERTIFICATE-----
MIIDXTCCAkWgAwIBAgIJAKoK/heBjcOuMA0GCSqGSIb3DQEBBQUAMEUxCzAJBgNV
BAYTAkFVMRMwEQYDVQQIDApTb21lLVN0YXRlMSEwHwYDVQQKDBhJbnRlcm5ldCBX
aWRnaXRzIFB0eSBMdGQwHhcNMTgxMjI4MjI0NTM4WhcNMjgxMjI1MjI0NTM4WjBF
MQswCQYDVQQGEwJBVTETMBEGA1UECAwKU29tZS1TdGF0ZTEhMB8GA1UECgwYSW50
ZXJuZXQgV2lkZ2l0cyBQdHkgTHRkMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB
CgKCAQEAy8dbwAD1VfpA5EWVHlzQ2v2jmfb5ZbZHgOSJ7pOaQJiV6xN7kZHjGqxE
K6xN7kZHjGqxEK6xN7kZHjGqxEK6xN7kZHjGqxEK6xN7kZHjGqxEK6xN7kZHjGqx
-----END CERTIFICATE-----")
endif()

# Create dummy server key
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/certs/server_key.pem")
    file(WRITE "${CMAKE_SOURCE_DIR}/certs/server_key.pem"
"-----BEGIN PRIVATE KEY-----
MIIEvwIBADANBgkqhkiG9w0BAQEFAASCBKkwggSlAgEAAoIBAQDLx1vAAPVV+kDk
RZUeXNDa/aOZ9vlltkeA5Inuk5pAmJXrE3uRkeMarESKz7w5xgxZm3QnZoZYZgP8
K6xN7kZHjGqxEK6xN7kZHjGqxEK6xN7kZHjGqxEK6xN7kZHjGqxEK6xN7kZHjGqx
-----END PRIVATE KEY-----")
endif()