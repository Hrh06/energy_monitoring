#include "relay_control.h"
#include "nvs_storage.h"

static const char* RELAY_TAG = "RELAY_CONTROL";

// ==================== RELAY CONTROL INITIALIZATION ====================

esp_err_t relay_control_init(void)
{
    ESP_LOGI(RELAY_TAG, "Initializing relay control system...");
    
    // Configure shift register GPIO pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SHIFT_REG_DATA_PIN) | 
                        (1ULL << SHIFT_REG_CLOCK_PIN) | 
                        (1ULL << SHIFT_REG_LATCH_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(RELAY_TAG, "Failed to configure shift register GPIO pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize relay control structure with default values
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        memset(&relay_control, 0, sizeof(relay_control_t));
        relay_control.last_update_time = esp_timer_get_time();
        
        // Set default thresholds for automatic control
        for (int i = 0; i < NUM_RELAYS; i++) {
            relay_control.auto_voltage_threshold_high[i] = 250.0; // Turn off above 250V
            relay_control.auto_voltage_threshold_low[i] = 200.0;  // Turn on below 200V
            relay_control.auto_current_threshold_high[i] = 50.0;  // Turn off above 50A
            relay_control.auto_current_threshold_low[i] = 5.0;    // Turn on below 5A
            relay_control.manual_mode[i] = true;                  // Start in manual mode
            relay_control.auto_enabled[i] = false;                // Auto control disabled by default
            relay_control.relay_states[i] = false;                // All relays OFF initially
        }
        
        relay_control.power_failure_mode = false;
        
        xSemaphoreGive(relay_mutex);
    } else {
        ESP_LOGE(RELAY_TAG, "Failed to acquire relay mutex during initialization");
        return ESP_ERR_TIMEOUT;
    }
    
    // Initialize all relays to OFF state
    shift_register_write(0x00);
    
    ESP_LOGI(RELAY_TAG, "Relay control system initialized - 8 relays available");
    ESP_LOGI(RELAY_TAG, "Shift register pins: Data=%d, Clock=%d, Latch=%d", 
             SHIFT_REG_DATA_PIN, SHIFT_REG_CLOCK_PIN, SHIFT_REG_LATCH_PIN);
    
    return ESP_OK;
}

// ==================== SHIFT REGISTER CONTROL ====================

void shift_register_write(uint8_t data)
{
    // Set latch low to prepare for data transmission
    gpio_set_level(SHIFT_REG_LATCH_PIN, 0);
    
    // Send 8 bits of data, MSB first
    for (int i = 7; i >= 0; i--) {
        // Set data pin
        gpio_set_level(SHIFT_REG_DATA_PIN, (data >> i) & 1);
        
        // Clock pulse
        gpio_set_level(SHIFT_REG_CLOCK_PIN, 1);
        ets_delay_us(1); // Small delay for setup time
        gpio_set_level(SHIFT_REG_CLOCK_PIN, 0);
        ets_delay_us(1); // Small delay for hold time
    }
    
    // Latch the data to output
    gpio_set_level(SHIFT_REG_LATCH_PIN, 1);
    ets_delay_us(1);
    gpio_set_level(SHIFT_REG_LATCH_PIN, 0);
    
    ESP_LOGD(RELAY_TAG, "Shift register updated: 0x%02X", data);
}

// ==================== RELAY STATE CONTROL ====================

void set_relay_state(int relay_num, bool state)
{
    if (relay_num < 0 || relay_num >= NUM_RELAYS) {
        ESP_LOGW(RELAY_TAG, "Invalid relay number: %d", relay_num);
        return;
    }
    
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        bool previous_state = relay_control.relay_states[relay_num];
        relay_control.relay_states[relay_num] = state;
        relay_control.last_update_time = esp_timer_get_time();
        
        // Build relay mask and update hardware
        uint8_t relay_mask = get_relay_mask();
        shift_register_write(relay_mask);
        
        // Save to NVS for power failure recovery
        save_relay_states_to_nvs();
        
        xSemaphoreGive(relay_mutex);
        
        if (previous_state != state) {
            ESP_LOGI(RELAY_TAG, "Relay %d: %s -> %s", relay_num, 
                     previous_state ? "ON" : "OFF", state ? "ON" : "OFF");
        }
    } else {
        ESP_LOGW(RELAY_TAG, "Failed to acquire relay mutex for relay %d", relay_num);
    }
}

void set_all_relays(uint8_t relay_mask)
{
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        uint8_t previous_mask = get_relay_mask();
        
        // Update individual relay states
        for (int i = 0; i < NUM_RELAYS; i++) {
            relay_control.relay_states[i] = (relay_mask >> i) & 1;
        }
        relay_control.last_update_time = esp_timer_get_time();
        
        // Update hardware
        shift_register_write(relay_mask);
        
        // Save to NVS
        save_relay_states_to_nvs();
        
        xSemaphoreGive(relay_mutex);
        
        if (previous_mask != relay_mask) {
            ESP_LOGI(RELAY_TAG, "All relays updated: 0x%02X -> 0x%02X", previous_mask, relay_mask);
        }
    } else {
        ESP_LOGW(RELAY_TAG, "Failed to acquire relay mutex for set_all_relays");
    }
}

void toggle_relay(int relay_num)
{
    if (relay_num < 0 || relay_num >= NUM_RELAYS) {
        ESP_LOGW(RELAY_TAG, "Invalid relay number for toggle: %d", relay_num);
        return;
    }
    
    bool current_state = false;
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_state = relay_control.relay_states[relay_num];
        xSemaphoreGive(relay_mutex);
    }
    
    set_relay_state(relay_num, !current_state);
    ESP_LOGI(RELAY_TAG, "Relay %d toggled to %s", relay_num, !current_state ? "ON" : "OFF");
}

// ==================== RELAY STATE QUERIES ====================

uint8_t get_relay_mask(void)
{
    uint8_t mask = 0;
    
    // Note: This function is called from within mutex-protected sections,
    // so we don't take the mutex here to avoid deadlock
    for (int i = 0; i < NUM_RELAYS; i++) {
        if (relay_control.relay_states[i]) {
            mask |= (1 << i);
        }
    }
    
    return mask;
}

bool get_relay_state(int relay_num)
{
    if (relay_num < 0 || relay_num >= NUM_RELAYS) {
        ESP_LOGW(RELAY_TAG, "Invalid relay number for get_state: %d", relay_num);
        return false;
    }
    
    bool state = false;
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = relay_control.relay_states[relay_num];
        xSemaphoreGive(relay_mutex);
    }
    
    return state;
}

// ==================== POWER FAILURE HANDLING ====================

void handle_power_failure_relays(void)
{
    ESP_LOGI(RELAY_TAG, "Handling power failure - switching to safe mode");
    
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        relay_control.power_failure_mode = true;
        
        // Switch all relays to manual mode for safety
        for (int i = 0; i < NUM_RELAYS; i++) {
            relay_control.manual_mode[i] = true;
            relay_control.auto_enabled[i] = false;
        }
        
        // Turn all relays ON for manual control during power failure
        for (int i = 0; i < NUM_RELAYS; i++) {
            relay_control.relay_states[i] = true;
        }
        
        relay_control.last_update_time = esp_timer_get_time();
        
        // Update hardware - all relays ON
        shift_register_write(0xFF);
        
        // Save state to NVS
        save_relay_states_to_nvs();
        
        xSemaphoreGive(relay_mutex);
        
        ESP_LOGI(RELAY_TAG, "Power failure mode: All relays ON for manual control");
    } else {
        ESP_LOGE(RELAY_TAG, "Failed to acquire mutex during power failure handling");
    }
}

// ==================== AUTOMATIC CONTROL ====================

void process_automatic_relay_control(complete_data_packet_t *packet)
{
    if (!packet) {
        return;
    }
    
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool changes_made = false;
        
        for (int i = 0; i < NUM_RELAYS; i++) {
            // Skip if relay is in manual mode or auto is disabled
            if (relay_control.manual_mode[i] || !relay_control.auto_enabled[i]) {
                continue;
            }
            
            bool new_state = relay_control.relay_states[i];
            bool state_changed = false;
            
            // Voltage-based control
            if (packet->system_voltage > relay_control.auto_voltage_threshold_high[i]) {
                if (new_state) {
                    new_state = false; // Turn OFF relay if voltage too high
                    state_changed = true;
                    ESP_LOGI(RELAY_TAG, "Auto: Relay %d OFF (voltage %.1fV > %.1fV)", 
                             i, packet->system_voltage, relay_control.auto_voltage_threshold_high[i]);
                }
            } else if (packet->system_voltage < relay_control.auto_voltage_threshold_low[i]) {
                if (!new_state) {
                    new_state = true;  // Turn ON relay if voltage too low
                    state_changed = true;
                    ESP_LOGI(RELAY_TAG, "Auto: Relay %d ON (voltage %.1fV < %.1fV)", 
                             i, packet->system_voltage, relay_control.auto_voltage_threshold_low[i]);
                }
            }
            
            // Current-based control for connected sensors
            if (i < NUM_CHANNELS && packet->channels[i].sensor_connected) {
                if (packet->channels[i].current_rms > relay_control.auto_current_threshold_high[i]) {
                    if (new_state) {
                        new_state = false; // Turn OFF relay if current too high
                        state_changed = true;
                        ESP_LOGI(RELAY_TAG, "Auto: Relay %d OFF (current %.2fA > %.2fA)", 
                                 i, packet->channels[i].current_rms, relay_control.auto_current_threshold_high[i]);
                    }
                }
            }
            
            // Apply state change if needed
            if (state_changed && new_state != relay_control.relay_states[i]) {
                relay_control.relay_states[i] = new_state;
                changes_made = true;
            }
        }
        
        if (changes_made) {
            // Update hardware
            uint8_t relay_mask = get_relay_mask();
            shift_register_write(relay_mask);
            
            // Update timestamp and save to NVS
            relay_control.last_update_time = esp_timer_get_time();
            save_relay_states_to_nvs();
            
            ESP_LOGI(RELAY_TAG, "Automatic control updated relay states: 0x%02X", relay_mask);
        }
        
        xSemaphoreGive(relay_mutex);
    }
}

// ==================== CONFIGURATION FUNCTIONS ====================

void set_relay_auto_params(int relay_num, bool auto_enabled, 
                          float voltage_high, float voltage_low,
                          float current_high, float current_low)
{
    if (relay_num < 0 || relay_num >= NUM_RELAYS) {
        ESP_LOGW(RELAY_TAG, "Invalid relay number for auto params: %d", relay_num);
        return;
    }
    
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        relay_control.auto_enabled[relay_num] = auto_enabled;
        relay_control.auto_voltage_threshold_high[relay_num] = voltage_high;
        relay_control.auto_voltage_threshold_low[relay_num] = voltage_low;
        relay_control.auto_current_threshold_high[relay_num] = current_high;
        relay_control.auto_current_threshold_low[relay_num] = current_low;
        
        // Save to NVS
        save_relay_states_to_nvs();
        
        xSemaphoreGive(relay_mutex);
        
        ESP_LOGI(RELAY_TAG, "Relay %d auto params: %s, V[%.1f-%.1f], I[%.1f-%.1f]", 
                 relay_num, auto_enabled ? "ENABLED" : "DISABLED",
                 voltage_low, voltage_high, current_low, current_high);
    } else {
        ESP_LOGW(RELAY_TAG, "Failed to acquire mutex for auto params");
    }
}

void set_relay_mode(int relay_num, bool manual_mode)
{
    if (relay_num < 0 || relay_num >= NUM_RELAYS) {
        ESP_LOGW(RELAY_TAG, "Invalid relay number for mode setting: %d", relay_num);
        return;
    }
    
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        bool previous_mode = relay_control.manual_mode[relay_num];
        relay_control.manual_mode[relay_num] = manual_mode;
        
        // If switching to manual mode, disable auto control
        if (manual_mode) {
            relay_control.auto_enabled[relay_num] = false;
        }
        
        // Save to NVS
        save_relay_states_to_nvs();
        
        xSemaphoreGive(relay_mutex);
        
        if (previous_mode != manual_mode) {
            ESP_LOGI(RELAY_TAG, "Relay %d mode: %s -> %s", relay_num,
                     previous_mode ? "MANUAL" : "AUTO", manual_mode ? "MANUAL" : "AUTO");
        }
    } else {
        ESP_LOGW(RELAY_TAG, "Failed to acquire mutex for mode setting");
    }
}

// ==================== STATISTICS AND DIAGNOSTICS ====================

void get_relay_stats(int relay_num, void* stats)
{
    if (relay_num < 0 || relay_num >= NUM_RELAYS || !stats) {
        ESP_LOGW(RELAY_TAG, "Invalid parameters for relay stats");
        return;
    }
    
    // This is a placeholder for future statistics implementation
    // Could include: switch count, uptime, power consumption, etc.
    ESP_LOGD(RELAY_TAG, "Relay %d stats requested (not implemented)", relay_num);
}