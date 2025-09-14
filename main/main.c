#include "common.h"
#include "nvs_storage.h"
#include "wifi_manager.h"
#include "mqtt_client_own.h"
#include "relay_control.h"
#include "energy_meter.h"

// ==================== GLOBAL VARIABLE DEFINITIONS ====================

// System components
QueueHandle_t data_queue = NULL;
QueueHandle_t solar_data_queue = NULL;
EventGroupHandle_t wifi_event_group = NULL;
SemaphoreHandle_t relay_mutex = NULL;
esp_adc_cal_characteristics_t *adc_chars = NULL;

// System state
relay_control_t relay_control = {0};
realtime_data_t current_realtime_data = {0};
bool mqtt_connected = false;
bool wifi_connected = false;
bool provisioning_mode = false;
uint32_t packet_counter = 0;
float current_voltage_rms = 0.0;

// Handles
esp_mqtt_client_handle_t mqtt_client = NULL;
httpd_handle_t server = NULL;

// Task handles
TaskHandle_t sampling_task_handle = NULL;
TaskHandle_t processing_task_handle = NULL;
TaskHandle_t communication_task_handle = NULL;
TaskHandle_t button_task_handle = NULL;

// ==================== SYSTEM INITIALIZATION ====================

static const char* MAIN_TAG = "MAIN";

static esp_err_t init_system(void)
{
    ESP_LOGI(MAIN_TAG, "Initializing system components...");
    
    // Initialize NVS
    esp_err_t err = nvs_storage_init();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create queues and synchronization objects
    data_queue = xQueueCreate(10, sizeof(cycle_data_t));
    solar_data_queue = xQueueCreate(5, sizeof(solar_data_t));
    wifi_event_group = xEventGroupCreate();
    relay_mutex = xSemaphoreCreateMutex();
    
    if (!data_queue || !solar_data_queue || !wifi_event_group || !relay_mutex) {
        ESP_LOGE(MAIN_TAG, "Failed to create queues, event groups, or mutex");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(MAIN_TAG, "System components initialized successfully");
    return ESP_OK;
}

static esp_err_t init_tasks(void)
{
    ESP_LOGI(MAIN_TAG, "Creating application tasks...");
    
    // High-priority sampling task on Core 1
    BaseType_t result = xTaskCreatePinnedToCore(
        sampling_task, "sampling_task", 6144, NULL, 5, &sampling_task_handle, 1);
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create sampling task");
        return ESP_FAIL;
    }
    
    // Processing task on Core 0
    result = xTaskCreatePinnedToCore(
        processing_task, "processing_task", 10240, NULL, 4, &processing_task_handle, 0);
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create processing task");
        return ESP_FAIL;
    }
    
    // Communication task on Core 0
    result = xTaskCreatePinnedToCore(
        communication_task, "communication_task", 10240, NULL, 3, &communication_task_handle, 0);
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create communication task");
        return ESP_FAIL;
    }
    
    // Button task on Core 0
    result = xTaskCreatePinnedToCore(
        button_task, "button_task", 3072, NULL, 2, &button_task_handle, 0);
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create button task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(MAIN_TAG, "All application tasks created successfully");
    return ESP_OK;
}

// ==================== MAIN APPLICATION ====================

void app_main(void)
{
    ESP_LOGI(MAIN_TAG, "=== ESP32 Multi-Core Energy Meter with SSL/TLS ===");
    ESP_LOGI(MAIN_TAG, "Version: 2.1.0 - Modular Architecture");
    ESP_LOGI(MAIN_TAG, "Features: 8CH Energy Monitor + 8 Relay Control + MQTT SSL");
    ESP_LOGI(MAIN_TAG, "GPIO Updates: Voltage=34, Current=35, Button=3s/7s timing");
    
    // Initialize system
    esp_err_t err = init_system();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "System initialization failed: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    // Initialize hardware components
    err = energy_meter_init();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Energy meter init failed: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    err = relay_control_init();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Relay control init failed: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    // Save certificates and restore relay states
    save_certificates_to_nvs();
    restore_relay_states_from_nvs();
    
    // Initialize network components
    err = wifi_manager_init();
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
        ESP_LOGE(MAIN_TAG, "WiFi manager init failed: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    err = mqtt_client_init();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "MQTT client init failed: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    // Create application tasks
    err = init_tasks();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Task creation failed");
        esp_restart();
    }
    
    ESP_LOGI(MAIN_TAG, "=== System Initialization Complete ===");
    ESP_LOGI(MAIN_TAG, "MQTT Topics:");
    ESP_LOGI(MAIN_TAG, "  Control:   %s", MQTT_TOPIC_RELAY_CONTROL);
    ESP_LOGI(MAIN_TAG, "  Relay:     %s (1s updates)", MQTT_TOPIC_RELAY_STATUS);
    ESP_LOGI(MAIN_TAG, "  Real-time: %s (1s updates)", MQTT_TOPIC_REALTIME);
    ESP_LOGI(MAIN_TAG, "  Data:      %s (~7s updates)", MQTT_TOPIC_DATA);
    ESP_LOGI(MAIN_TAG, "  System:    %s (30s updates)", MQTT_TOPIC_SYSTEM_STATUS);
    ESP_LOGI(MAIN_TAG, "Button: 3s=WiFi reset, 7s=Hotspot mode");
    ESP_LOGI(MAIN_TAG, "ADC Pins: GPIO34=Voltage, GPIO35=Current");
    
    // Main monitoring loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 second status updates
        
        bool ssl_active = verify_mqtt_connection_security();
        uint8_t relay_mask = get_relay_mask();
        int active_relays = __builtin_popcount(relay_mask);
        
        ESP_LOGI(MAIN_TAG, "Status: MQTT=%s SSL=%s WiFi=%s V=%.1fV I=%.2fA P=%.1fW Relays=%d/8", 
                 mqtt_connected ? "✓" : "✗", 
                 ssl_active ? "✓" : "✗",
                 wifi_connected ? "✓" : "✗",
                 current_realtime_data.voltage_rms,
                 current_realtime_data.current_rms,
                 current_realtime_data.power_real,
                 active_relays);
    }
}

void debug_task_stacks(void)
{
    if (sampling_task_handle) {
        ESP_LOGI("DEBUG", "Sampling task high water mark: %d", uxTaskGetStackHighWaterMark(sampling_task_handle));
    }
    if (processing_task_handle) {
        ESP_LOGI("DEBUG", "Processing task high water mark: %d", uxTaskGetStackHighWaterMark(processing_task_handle));
    }
    if (communication_task_handle) {
        ESP_LOGI("DEBUG", "Communication task high water mark: %d", uxTaskGetStackHighWaterMark(communication_task_handle));
    }
    if (button_task_handle) {
        ESP_LOGI("DEBUG", "Button task high water mark: %d", uxTaskGetStackHighWaterMark(button_task_handle));
    }
}