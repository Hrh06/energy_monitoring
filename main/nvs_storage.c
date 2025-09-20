#include "nvs_storage.h"

static const char* NVS_TAG = "NVS_STORAGE";

// ==================== NVS INITIALIZATION ====================

esp_err_t nvs_storage_init(void)
{
    ESP_LOGI(NVS_TAG, "Initializing NVS storage...");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(NVS_TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(NVS_TAG, "NVS storage initialized successfully");
    return ESP_OK;
}

// ==================== WiFi CREDENTIALS ====================

void save_wifi_credentials(const char* ssid, const char* password)
{
    if (!ssid || !password) {
        ESP_LOGE(NVS_TAG, "Invalid WiFi credentials");
        return;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error opening NVS handle for WiFi config: %s", esp_err_to_name(err));
        return;
    }
    
    err = nvs_set_str(nvs_handle, "ssid", ssid);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error saving SSID: %s", esp_err_to_name(err));
    }
    
    err = nvs_set_str(nvs_handle, "password", password);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error saving password: %s", esp_err_to_name(err));
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error committing WiFi credentials: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(NVS_TAG, "WiFi credentials saved successfully: SSID=%s", ssid);
}

bool load_wifi_credentials(char* ssid, char* password)
{
    if (!ssid || !password) {
        ESP_LOGE(NVS_TAG, "Invalid buffers for WiFi credentials");
        return false;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGD(NVS_TAG, "No WiFi config found in NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    size_t ssid_len = 32;
    size_t password_len = 64;
    
    err = nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGD(NVS_TAG, "No SSID found in NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_get_str(nvs_handle, "password", password, &password_len);
    if (err != ESP_OK) {
        ESP_LOGD(NVS_TAG, "No password found in NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(NVS_TAG, "WiFi credentials loaded successfully: SSID=%s", ssid);
    return true;
}

void clear_wifi_credentials(void)
{
    ESP_LOGI(NVS_TAG, "Clearing WiFi credentials...");
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error opening NVS handle for clearing WiFi config: %s", esp_err_to_name(err));
        return;
    }
    
    nvs_erase_key(nvs_handle, "ssid");
    nvs_erase_key(nvs_handle, "password");
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(NVS_TAG, "WiFi credentials cleared successfully");
}

// ==================== RELAY STATES ====================

void save_relay_states_to_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("relay_control", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error opening NVS handle for relay control: %s", esp_err_to_name(err));
        return;
    }
    
    // Save relay control structure
    err = nvs_set_blob(nvs_handle, "relay_config", &relay_control, sizeof(relay_control_t));
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error saving relay states: %s", esp_err_to_name(err));
    }
    
    // Save timestamp
    int64_t timestamp = esp_timer_get_time();
    err = nvs_set_i64(nvs_handle, "save_time", timestamp);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error saving timestamp: %s", esp_err_to_name(err));
    }
    
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error committing relay states: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGD(NVS_TAG, "Relay states saved to NVS successfully");
}

bool load_relay_states_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("relay_control", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGD(NVS_TAG, "No relay config found in NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    size_t required_size = sizeof(relay_control_t);
    err = nvs_get_blob(nvs_handle, "relay_config", &relay_control, &required_size);
    if (err != ESP_OK || required_size != sizeof(relay_control_t)) {
        ESP_LOGD(NVS_TAG, "No valid relay config found: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    // Load timestamp
    int64_t save_time = 0;
    err = nvs_get_i64(nvs_handle, "save_time", &save_time);
    if (err == ESP_OK) {
        int64_t current_time = esp_timer_get_time();
        int64_t time_diff = (current_time - save_time) / 1000000; // Convert to seconds
        ESP_LOGI(NVS_TAG, "Relay states were saved %lld seconds ago", time_diff);
    }
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(NVS_TAG, "Relay states loaded from NVS successfully");
    return true;
}

void restore_relay_states_from_nvs(void)
{
    ESP_LOGI(NVS_TAG, "Restoring relay states from NVS after power recovery...");
    
    if (load_relay_states_from_nvs()) {
        // Clear power failure mode
        if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            relay_control.power_failure_mode = false;
            relay_control.last_update_time = esp_timer_get_time();
            xSemaphoreGive(relay_mutex);
        }
        
        // Build relay mask from restored states
        uint8_t relay_mask = 0;
        for (int i = 0; i < NUM_RELAYS; i++) {
            if (relay_control.relay_states[i]) {
                relay_mask |= (1 << i);
            }
        }
        
        // Restore hardware state
        shift_register_write(relay_mask);
        
        ESP_LOGI(NVS_TAG, "Relay states restored successfully: 0x%02X", relay_mask);
    } else {
        ESP_LOGI(NVS_TAG, "No saved relay states found, initializing to default OFF state");
        
        // Initialize with default state
        if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            memset(&relay_control, 0, sizeof(relay_control_t));
            
            // Set default thresholds
            for (int i = 0; i < NUM_RELAYS; i++) {
                relay_control.auto_voltage_threshold_high[i] = 250.0;
                relay_control.auto_voltage_threshold_low[i] = 200.0;
                relay_control.auto_current_threshold_high[i] = 50.0;
                relay_control.auto_current_threshold_low[i] = 5.0;
                relay_control.manual_mode[i] = true;
                relay_control.auto_enabled[i] = false;
                relay_control.relay_states[i] = false;
            }
            
            relay_control.power_failure_mode = false;
            relay_control.last_update_time = esp_timer_get_time();
            
            xSemaphoreGive(relay_mutex);
        }
        
        // Initialize hardware to OFF state
        shift_register_write(0x00);
    }
}

// ==================== CERTIFICATES ====================

void save_certificates_to_nvs(void)
{
    ESP_LOGI(NVS_TAG, "Saving SSL/TLS certificates to NVS...");
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("certificates", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error opening NVS handle for certificates: %s", esp_err_to_name(err));
        return;
    }
    
    size_t ca_cert_len = ca_cert_end - ca_cert_start;
    size_t esp32_cert_len = esp32_cert_end - esp32_cert_start;
    size_t esp32_key_len = esp32_key_end - esp32_key_start;

    // Store MQTT broker certificate
    if (ca_cert_len > 0) {
        err = nvs_set_blob(nvs_handle, "ca_cert_info", &ca_cert_len, sizeof(ca_cert_len));
        if (err != ESP_OK) {
            ESP_LOGE(NVS_TAG, "Error saving CA certificate: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(NVS_TAG, "MQTT certificate saved (%d bytes)", ca_cert_len);
        }
    }
    
    // Store ESP32 client certificate info
    if (esp32_cert_len > 0) {
        err = nvs_set_blob(nvs_handle, "esp32_cert_info", &esp32_cert_len, sizeof(esp32_cert_len));
        if (err != ESP_OK) {
            ESP_LOGE(NVS_TAG, "Error saving ESP32 certificate info: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(NVS_TAG, "ESP32 certificate info saved (%d bytes)", esp32_cert_len);
        }
    }

    // Store private key info
    if (esp32_key_len > 0) {
        err = nvs_set_blob(nvs_handle, "esp32_key_info", &esp32_key_len, sizeof(esp32_key_len));
        if (err != ESP_OK) {
            ESP_LOGE(NVS_TAG, "Error saving ESP32 private key info: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(NVS_TAG, "ESP32 private key info saved (%d bytes)", esp32_key_len);
        }
    }

    // Store certificate file names for reference
    const char* cert_files_info = "ca_cert.crt|esp32_cert.crt|esp32_key.key";
    err = nvs_set_str(nvs_handle, "cert_files", cert_files_info);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error saving certificate file names: %s", esp_err_to_name(err));
    }

    // Store timestamp
    int64_t timestamp = esp_timer_get_time();
    err = nvs_set_i64(nvs_handle, "cert_save_time", timestamp);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error saving certificate timestamp: %s", esp_err_to_name(err));
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(NVS_TAG, "Error committing certificates: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(NVS_TAG, "✅ Certificate information saved to NVS successfully");
    ESP_LOGI(NVS_TAG, "   📋 CA Certificate: %d bytes", ca_cert_len);
    ESP_LOGI(NVS_TAG, "   🆔 ESP32 Certificate: %d bytes", esp32_cert_len);
    ESP_LOGI(NVS_TAG, "   🔑 Private Key: %d bytes", esp32_key_len);
}