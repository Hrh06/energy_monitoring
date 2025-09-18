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

// MQTT initialization flag
bool mqtt_initialized = false;

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

// ==================== ULTRA PRIORITY TASK CREATION ====================

static esp_err_t init_ultra_priority_tasks(void)
{
    ESP_LOGI(MAIN_TAG, "Creating ULTRA HIGH PRIORITY application tasks...");
    
    // ULTRA HIGH PRIORITY: Button task - CANNOT BE BLOCKED
    // Priority 9 = Maximum priority (higher than any other task)
    BaseType_t result = xTaskCreatePinnedToCore(
        button_task, 
        "ULTRA_BUTTON",             // Clear name to identify in task list
        ULTRA_BUTTON_TASK_STACK_SIZE, // Large stack for safety 
        NULL, 
        ULTRA_BUTTON_TASK_PRIORITY, // MAXIMUM PRIORITY - CANNOT BE BLOCKED
        &button_task_handle, 
        ULTRA_BUTTON_TASK_CORE      // Dedicated core
    );
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "CRITICAL: Failed to create ULTRA PRIORITY button task");
        return ESP_FAIL;
    }
    ESP_LOGI(MAIN_TAG, "✅ ULTRA HIGH PRIORITY button task created (Priority %d, Core %d)", ULTRA_BUTTON_TASK_PRIORITY, ULTRA_BUTTON_TASK_CORE);
    
    // High-priority sampling task on Core 1
    result = xTaskCreatePinnedToCore(
        sampling_task, 
        "sampling_task", 
        4096, 
        NULL, 
        5,                  // Priority 5 (high for real-time sampling)
        &sampling_task_handle, 
        1
    );
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create sampling task");
        return ESP_FAIL;
    }
    ESP_LOGI(MAIN_TAG, "✅ Sampling task created (Priority 5, Core 1)");
    
    // Processing task on Core 0
    result = xTaskCreatePinnedToCore(
        processing_task, 
        "processing_task", 
        8192, 
        NULL, 
        4,                  // Priority 4 (medium-high)
        &processing_task_handle, 
        0
    );
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create processing task");
        return ESP_FAIL;
    }
    ESP_LOGI(MAIN_TAG, "✅ Processing task created (Priority 4, Core 0)");
    
    // ⚠️ LOWEST PRIORITY: Communication task - Can be blocked without affecting button
    result = xTaskCreatePinnedToCore(
        communication_task, 
        "comm_task", 
        8192, 
        NULL, 
        1,                  // LOWEST PRIORITY - Can be blocked by MQTT issues
        &communication_task_handle, 
        0
    );
    if (result != pdPASS) {
        ESP_LOGE(MAIN_TAG, "Failed to create communication task");
        return ESP_FAIL;
    }
    ESP_LOGI(MAIN_TAG, "✅ Communication task created (Priority 1 - Lowest, Core 0)");
    
    ESP_LOGI(MAIN_TAG, "🎯 Task Priority Hierarchy:");
    ESP_LOGI(MAIN_TAG, "  1. Button Task (Priority 9) - ULTRA HIGH - CANNOT BE BLOCKED");
    ESP_LOGI(MAIN_TAG, "  2. Sampling Task (Priority 5) - High priority real-time");
    ESP_LOGI(MAIN_TAG, "  3. Processing Task (Priority 4) - Medium-high priority");
    ESP_LOGI(MAIN_TAG, "  4. Communication Task (Priority 1) - LOW - Can be blocked");
    
    return ESP_OK;
}

// ==================== MQTT TIMEOUT MONITORING TASK ====================

static void mqtt_timeout_monitor_task(void *param) 
{
    ESP_LOGI(MAIN_TAG, "MQTT timeout monitor started - 15 second timeout");
    vTaskDelay(pdMS_TO_TICKS(15000)); // 15 second timeout
    
    if (!mqtt_initialized) {
        ESP_LOGW(MAIN_TAG, "🚨 MQTT initialization timeout - System continues without MQTT");
        ESP_LOGW(MAIN_TAG, "Button functionality remains active regardless of MQTT status");
    } else {
        ESP_LOGI(MAIN_TAG, "MQTT initialized within timeout period");
    }
    
    ESP_LOGI(MAIN_TAG, "MQTT timeout monitor task completed");
    vTaskDelete(NULL); // Delete this timeout task
}

// ==================== NON-BLOCKING MQTT INITIALIZATION ====================

esp_err_t initialize_mqtt_with_timeout_protection(void)
{
    if (mqtt_initialized) {
        ESP_LOGW(MAIN_TAG, "MQTT already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(MAIN_TAG, "WiFi connected - Initializing MQTT with timeout protection...");
    
    // Create timeout monitoring task
    TaskHandle_t mqtt_timeout_task = NULL;
    BaseType_t task_result = xTaskCreate(
        mqtt_timeout_monitor_task, 
        "mqtt_timeout", 
        2048, 
        NULL, 
        2, 
        &mqtt_timeout_task
    );
    
    if (task_result != pdPASS) {
        ESP_LOGW(MAIN_TAG, "Failed to create MQTT timeout task - continuing without timeout protection");
    }

    // Try to initialize MQTT (non-blocking)
    esp_err_t err = mqtt_client_init();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "MQTT client initialization failed: %s", esp_err_to_name(err));
        ESP_LOGW(MAIN_TAG, "System continues running - Button remains fully functional");
        return err;
    }
    
    mqtt_initialized = true;
    ESP_LOGI(MAIN_TAG, "✅ MQTT client initialized successfully");
    
    // Delete timeout task if MQTT initialized successfully
    if (mqtt_timeout_task) {
        vTaskDelete(mqtt_timeout_task);
    }
    
    // Start MQTT client (also non-blocking)
    mqtt_client_start();
    
    return ESP_OK;
}

// ==================== MAIN APPLICATION ====================

void app_main(void)
{
    ESP_LOGI(MAIN_TAG, "=== ESP32 Multi-Core Energy Meter with SSL/TLS ===");
    ESP_LOGI(MAIN_TAG, "Version: 2.1.0 - Modular Architecture");
    ESP_LOGI(MAIN_TAG, "Features: UNBLOCKABLE Button + 8CH Monitor + 8 Relay + MQTT");
    ESP_LOGI(MAIN_TAG, "UPDATED Button: GPIO%d (HIGHEST PRIORITY - Cannot be blocked by ANY task)", RESET_BUTTON_PIN);
    ESP_LOGI(MAIN_TAG, "UPDATED Shift Register: Data=GPIO%d, Clock=GPIO%d, Latch=GPIO%d", 
             SHIFT_REG_DATA_PIN, SHIFT_REG_CLOCK_PIN, SHIFT_REG_LATCH_PIN);
    
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
    ESP_LOGI(MAIN_TAG, "ULTRA HIGH PRIORITY button system active");
    
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
    
    // Create application tasks
    err = init_ultra_priority_tasks();
    if (err != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "CRITICAL: Ultra priority task creation failed");
        esp_restart();
    }
    
    ESP_LOGI(MAIN_TAG, "=== ULTRA PRIORITY SYSTEM READY ===");
    ESP_LOGI(MAIN_TAG, "Button Status: ULTRA HIGH PRIORITY (Cannot be blocked)");
    ESP_LOGI(MAIN_TAG, "Emergency Actions:");
    ESP_LOGI(MAIN_TAG, "   • 3-7s press = WiFi Reset (IMMEDIATE)");
    ESP_LOGI(MAIN_TAG, "   • 7s+ press = Factory Reset (IMMEDIATE)");
    ESP_LOGI(MAIN_TAG, "   • 10s+ press = Emergency Restart (FORCE)");
    ESP_LOGI(MAIN_TAG, "Button works even if MQTT/WiFi/Tasks are frozen");

    ESP_LOGI(MAIN_TAG, "MQTT Topics (When connected):");
    ESP_LOGI(MAIN_TAG, "  Control:   %s", MQTT_TOPIC_RELAY_CONTROL);
    ESP_LOGI(MAIN_TAG, "  Relay:     %s (1s updates)", MQTT_TOPIC_RELAY_STATUS);
    ESP_LOGI(MAIN_TAG, "  Real-time: %s (1s updates)", MQTT_TOPIC_REALTIME);
    ESP_LOGI(MAIN_TAG, "  Data:      %s (~7s updates)", MQTT_TOPIC_DATA);
    ESP_LOGI(MAIN_TAG, "  System:    %s (30s updates)", MQTT_TOPIC_SYSTEM_STATUS);
    
    ESP_LOGI(MAIN_TAG, "Hardware Configuration:");
    ESP_LOGI(MAIN_TAG, "  Button:     GPIO%d (Ultra Priority)", RESET_BUTTON_PIN);
    ESP_LOGI(MAIN_TAG, "  Status LED: GPIO%d", STATUS_LED_PIN);
    ESP_LOGI(MAIN_TAG, "  Shift Reg:  Data=GPIO%d, Clock=GPIO%d, Latch=GPIO%d", 
             SHIFT_REG_DATA_PIN, SHIFT_REG_CLOCK_PIN, SHIFT_REG_LATCH_PIN);
    ESP_LOGI(MAIN_TAG, "  ADC Pins:   Voltage=GPIO34, Current=GPIO35");
    
    // Main monitoring loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 second status updates
        
        bool ssl_active = mqtt_initialized ? verify_mqtt_connection_security() : false;
        uint8_t relay_mask = get_relay_mask();
        int active_relays = __builtin_popcount(relay_mask);
        
        ESP_LOGI(MAIN_TAG, "Button: %s | MQTT: %s | SSL: %s | WiFi: %s | Relays: %d/8", 
                 "ULTRA-PRIORITY",
                 mqtt_connected ? "✅" : "❌",
                 ssl_active ? "✅" : "❌", 
                 wifi_connected ? "✅" : "❌",
                 active_relays);
        
        ESP_LOGI(MAIN_TAG, "V=%.1fV | I=%.2fA | P=%.1fW | Heap=%dKB", 
                 current_realtime_data.voltage_rms,
                 current_realtime_data.current_rms,
                 current_realtime_data.power_real,
                 esp_get_free_heap_size()/1024);
        
        // Emergency system status check
        emergency_system_status();

        // Log custom certificate status every 60 seconds
        static int cert_status_counter = 0;
        if (++cert_status_counter >= 6) { // 6 * 10s = 60s
            if (mqtt_connected) {
                ESP_LOGI(MAIN_TAG, "🔐 Custom Certificate Status:");
                ESP_LOGI(MAIN_TAG, "   📋 CA Certificate: %d bytes (verifies HiveMQ)", ca_cert_end - ca_cert_start);
                ESP_LOGI(MAIN_TAG, "   🆔 ESP32 Certificate: %d bytes (authenticates device)", esp32_cert_end - esp32_cert_start);
                ESP_LOGI(MAIN_TAG, "   🔑 Private Key: %d bytes (encrypts communication)", esp32_key_end - esp32_key_start);
                ESP_LOGI(MAIN_TAG, "   ✅ All certificates loaded and working");
            }
            cert_status_counter = 0;
        }
    }
}

// ==================== EMERGENCY BUTTON TEST FUNCTION ====================

void test_ultra_priority_button(void)
{
    ESP_LOGI(MAIN_TAG, "Testing ULTRA HIGH PRIORITY button system...");
    
    // Simulate system freeze by creating blocking task
    void system_freeze_simulation(void *param) {
        ESP_LOGW(MAIN_TAG, "Simulating system freeze - Button should still work!");
        
        // Block this task indefinitely
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
    
    // Create a blocking task to test button independence
    xTaskCreate(system_freeze_simulation, "freeze_test", 2048, NULL, 8, NULL);
    
    ESP_LOGI(MAIN_TAG, "Button test: Press GPIO4 for 3s/7s - Should work even with system frozen!");
}

// ==================== DEBUG FUNCTIONS ====================

void debug_task_stacks(void)
{
    ESP_LOGI("DEBUG", "=== TASK STACK ANALYSIS ===");
    
    if (button_task_handle) {
        UBaseType_t button_priority = uxTaskPriorityGet(button_task_handle);
        ESP_LOGI("DEBUG", "🔘 ULTRA Button: Priority=%lu, Stack=%lu bytes free, Core=%d", 
                 (unsigned long)button_priority, 
                 (unsigned long)(uxTaskGetStackHighWaterMark(button_task_handle) * 4),
                 ULTRA_BUTTON_TASK_CORE);
        
        // CRITICAL: Verify button task still has maximum priority
        if (button_priority != ULTRA_BUTTON_TASK_PRIORITY) {
            ESP_LOGW("DEBUG", "🚨 WARNING: Button task priority degraded! Restoring to maximum");
            vTaskPrioritySet(button_task_handle, ULTRA_BUTTON_TASK_PRIORITY);
        }
    }
    
    if (sampling_task_handle) {
        UBaseType_t sampling_priority = uxTaskPriorityGet(sampling_task_handle);
        ESP_LOGI("DEBUG", "📊 Sampling: Priority=%lu, Stack=%lu bytes free", 
                 (unsigned long)sampling_priority, 
                 (unsigned long)(uxTaskGetStackHighWaterMark(sampling_task_handle) * 4));
    }
    
    if (processing_task_handle) {
        UBaseType_t processing_priority = uxTaskPriorityGet(processing_task_handle);
        ESP_LOGI("DEBUG", "⚙️ Processing: Priority=%lu, Stack=%lu bytes free", 
                 (unsigned long)processing_priority, 
                 (unsigned long)(uxTaskGetStackHighWaterMark(processing_task_handle) * 4));
    }
    
    if (communication_task_handle) {
        UBaseType_t comm_priority = uxTaskPriorityGet(communication_task_handle);
        ESP_LOGI("DEBUG", "📡 Communication: Priority=%lu, Stack=%lu bytes free", 
                 (unsigned long)comm_priority, 
                 (unsigned long)(uxTaskGetStackHighWaterMark(communication_task_handle) * 4));
    }
    
    ESP_LOGI("DEBUG", "💾 System: Free heap=%ldKB, Min free=%ldKB", 
             (long)(esp_get_free_heap_size()/1024), 
             (long)(esp_get_minimum_free_heap_size()/1024));
    
    ESP_LOGI("DEBUG", "🔘 Button interrupt statistics and priority verification complete");
}

// ==================== SYSTEM RECOVERY FUNCTIONS ====================

void force_button_task_priority_restore(void)
{
    if (button_task_handle) {
        UBaseType_t current_priority = uxTaskPriorityGet(button_task_handle);
        if (current_priority != ULTRA_BUTTON_TASK_PRIORITY) {
            ESP_LOGW(MAIN_TAG, "🚨 CRITICAL: Button task priority was %lu, restoring to %d", 
                     (unsigned long)current_priority, ULTRA_BUTTON_TASK_PRIORITY);
            vTaskPrioritySet(button_task_handle, ULTRA_BUTTON_TASK_PRIORITY);
            ESP_LOGI(MAIN_TAG, "✅ Button task priority restored to maximum");
        }
    }
}

void monitor_button_task_health(void)
{
    static TickType_t last_health_check = 0;
    TickType_t current_time = xTaskGetTickCount();
    
    // Check button task health every 60 seconds
    if ((current_time - last_health_check) >= pdMS_TO_TICKS(60000)) {
        if (button_task_handle) {
            eTaskState task_state = eTaskGetState(button_task_handle);
            
            switch (task_state) {
                case eReady:
                case eRunning:
                case eBlocked:
                    ESP_LOGI(MAIN_TAG, "✅ Button task health: GOOD (State: %s)", 
                             task_state == eReady ? "Ready" : 
                             task_state == eRunning ? "Running" : "Blocked");
                    break;
                case eSuspended:
                    ESP_LOGW(MAIN_TAG, "🚨 Button task is SUSPENDED - This should not happen!");
                    vTaskResume(button_task_handle);
                    break;
                case eDeleted:
                    ESP_LOGE(MAIN_TAG, "🚨 CRITICAL: Button task was DELETED - System restart required!");
                    esp_restart();
                    break;
                default:
                    ESP_LOGW(MAIN_TAG, "🚨 Button task in unknown state: %d", task_state);
                    break;
            }
            
            // Verify and restore priority if needed
            force_button_task_priority_restore();
        } else {
            ESP_LOGE(MAIN_TAG, "🚨 CRITICAL: Button task handle is NULL - System restart required!");
            esp_restart();
        }
        
        last_health_check = current_time;
    }
}

// ==================== INITIALIZE MQTT AFTER WIFI ====================

esp_err_t initialize_mqtt_after_wifi_connection(void)
{
    return initialize_mqtt_with_timeout_protection();
}