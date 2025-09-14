#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "mqtt_client.h"  // ESP-IDF MQTT client - UNCOMMENTED
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_https_server.h"  // Added for HTTPS support
#include "esp_tls.h"
#include "esp_crt_bundle.h"
// ==================== CONFIGURATION ====================

// System Configuration
#define NUM_CHANNELS 8
#define NUM_RELAYS 8
#define SAMPLES_PER_CYCLE 200
#define CYCLES_PER_PACKET 7
#define SYSTEM_FREQUENCY 50.0
#define MAX_VOLTAGE 240.0
#define MAX_CURRENT 100.0

// Hardware Pin Definitions (Updated as requested)
#define VOLTAGE_SENSOR_PIN ADC1_CHANNEL_6  // GPIO 34 (input-only)
#define CURRENT_SENSOR_PIN ADC1_CHANNEL_7  // GPIO 35 (input-only)
#define RESET_BUTTON_PIN 0
#define STATUS_LED_PIN 2
#define SHIFT_REG_DATA_PIN 4     // DS pin
#define SHIFT_REG_CLOCK_PIN 5    // SH_CP pin  
#define SHIFT_REG_LATCH_PIN 18   // ST_CP pin
#define UART_NUM UART_NUM_2
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BUF_SIZE 2048

// WiFi Configuration
#define MAX_STA_CONN 4
#define HOTSPOT_SSID "ESP32_Energy_Meter"
#define HOTSPOT_PASS "energy123"
#define PROVISIONING_TIMEOUT_MS 300000

// MQTT Configuration
#define MQTT_BROKER_URL "32821620fa6640e3b91fecff79c3dcce.s1.eu.hivemq.cloud:8883"
#define MQTT_BROKER_PORT 8883
#define MQTT_USERNAME "Bitminds"
#define MQTT_PASSWORD "Bitminds@123456"
#define MQTT_CLIENT_ID "esp32_energy_meter"

// MQTT Topics
#define MQTT_TOPIC_DATA "energy_meter/data"
#define MQTT_TOPIC_RELAY_CONTROL "energy_meter/relay_control"
#define MQTT_TOPIC_RELAY_STATUS "energy_meter/relay_status"
#define MQTT_TOPIC_REALTIME "energy_meter/realtime"
#define MQTT_TOPIC_SYSTEM_STATUS "energy_meter/system_status"
#define MQTT_TOPIC_COMMANDS "energy_meter/commands"

// Button timing (Updated as requested)
#define BUTTON_DEBOUNCE_MS 50
#define WIFI_RESET_HOLD_TIME_MS 3000    // 3 seconds for WiFi reset
#define HOTSPOT_HOLD_TIME_MS 7000       // 7 seconds for hotspot mode

// Update intervals
#define RELAY_STATUS_PUBLISH_INTERVAL_MS 1000  // 1 second for responsive control
#define REALTIME_DATA_INTERVAL_MS 1000         // 1 second for real-time data

// ==================== DATA STRUCTURES ====================

typedef struct {
    bool sensor_connected;
    float current_rms;
    float voltage_rms;
    float power_real;
    float power_apparent;
    float power_reactive;
    float power_factor;
    float energy_consumed;
    float frequency;
    int64_t timestamp;
} channel_data_t;

typedef struct {
    bool data_valid;
    float solar_voltage;
    float solar_current;
    float solar_power;
    float battery_voltage;
    float battery_current;
    float battery_soc;
    int64_t timestamp;
} solar_data_t;

typedef struct {
    uint32_t packet_id;
    int64_t timestamp;
    float system_voltage;
    channel_data_t channels[NUM_CHANNELS];
    solar_data_t solar;
} complete_data_packet_t;

typedef struct {
    float voltage_samples[SAMPLES_PER_CYCLE];
    float current_samples[SAMPLES_PER_CYCLE];
    int sample_count;
    int64_t cycle_start_time;
} cycle_data_t;

typedef struct {
    bool relay_states[NUM_RELAYS];
    bool manual_mode[NUM_RELAYS];
    bool auto_enabled[NUM_RELAYS];
    float auto_voltage_threshold_high[NUM_RELAYS];
    float auto_voltage_threshold_low[NUM_RELAYS];
    float auto_current_threshold_high[NUM_RELAYS];
    float auto_current_threshold_low[NUM_RELAYS];
    bool power_failure_mode;
    int64_t last_update_time;
} relay_control_t;

typedef struct {
    float voltage_rms;
    float current_rms;
    float power_real;
    float power_apparent;
    float power_factor;
    float frequency;
    bool sensor_connected;
    int64_t timestamp;
} realtime_data_t;

// ==================== GLOBAL VARIABLES (EXTERN) ====================

// System components
extern QueueHandle_t data_queue;
extern QueueHandle_t solar_data_queue;
extern EventGroupHandle_t wifi_event_group;
extern SemaphoreHandle_t relay_mutex;
extern esp_adc_cal_characteristics_t *adc_chars;

// System state
extern relay_control_t relay_control;
extern realtime_data_t current_realtime_data;
extern bool mqtt_connected;
extern bool wifi_connected;
extern bool provisioning_mode;
extern uint32_t packet_counter;
extern float current_voltage_rms;

// MQTT client handle
extern esp_mqtt_client_handle_t mqtt_client;
extern httpd_handle_t server;

// Task handles
extern TaskHandle_t sampling_task_handle;
extern TaskHandle_t processing_task_handle;
extern TaskHandle_t communication_task_handle;
extern TaskHandle_t button_task_handle;

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// SSL Certificates (extern references)
extern const uint8_t mqtt_broker_cert_pem_start[] asm("_binary_mqtt_broker_cert_pem_start");
extern const uint8_t mqtt_broker_cert_pem_end[]   asm("_binary_mqtt_broker_cert_pem_end");
extern const uint8_t server_cert_pem_start[] asm("_binary_server_cert_pem_start");
extern const uint8_t server_cert_pem_end[]   asm("_binary_server_cert_pem_end");
extern const uint8_t server_key_pem_start[] asm("_binary_server_key_pem_start");
extern const uint8_t server_key_pem_end[]   asm("_binary_server_key_pem_end");

// ==================== FUNCTION PROTOTYPES ====================

// NVS Storage functions
esp_err_t nvs_storage_init(void);
void save_wifi_credentials(const char* ssid, const char* password);
bool load_wifi_credentials(char* ssid, char* password);
void clear_wifi_credentials(void);
void save_relay_states_to_nvs(void);
bool load_relay_states_from_nvs(void);
void save_certificates_to_nvs(void);
void restore_relay_states_from_nvs(void);

// WiFi Manager functions
esp_err_t wifi_manager_init(void);
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void start_provisioning_mode(void);
void stop_provisioning_mode(void);
esp_err_t root_handler(httpd_req_t *req);
esp_err_t wifi_credentials_handler(httpd_req_t *req);

// MQTT Client functions
esp_err_t mqtt_client_init(void);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_client_start(void);
void publish_sensor_data(complete_data_packet_t *packet);
void publish_relay_status(void);
void publish_realtime_data(void);
void publish_system_status(void);
void handle_mqtt_relay_command(const char* command_data, int data_len);
bool verify_mqtt_connection_security(void);

// Relay Control functions
esp_err_t relay_control_init(void);
void shift_register_write(uint8_t data);
void set_relay_state(int relay_num, bool state);
void set_all_relays(uint8_t relay_mask);
void toggle_relay(int relay_num);
uint8_t get_relay_mask(void);
void handle_power_failure_relays(void);
void process_automatic_relay_control(complete_data_packet_t *packet);

// Energy Meter functions
esp_err_t energy_meter_init(void);
void sampling_task(void *pvParameters);
void processing_task(void *pvParameters);
void communication_task(void *pvParameters);
void button_task(void *pvParameters);

#endif // COMMON_H