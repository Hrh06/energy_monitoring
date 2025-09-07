#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include "common.h"

// ==================== NVS STORAGE FUNCTIONS ====================

/**
 * @brief Initialize NVS storage system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t nvs_storage_init(void);

/**
 * @brief Save WiFi credentials to NVS
 * @param ssid WiFi SSID string
 * @param password WiFi password string
 */
void save_wifi_credentials(const char* ssid, const char* password);

/**
 * @brief Load WiFi credentials from NVS
 * @param ssid Buffer to store SSID (32 bytes)
 * @param password Buffer to store password (64 bytes)
 * @return true if credentials loaded successfully, false otherwise
 */
bool load_wifi_credentials(char* ssid, char* password);

/**
 * @brief Clear WiFi credentials from NVS
 */
void clear_wifi_credentials(void);

/**
 * @brief Save relay states to NVS
 */
void save_relay_states_to_nvs(void);

/**
 * @brief Load relay states from NVS
 * @return true if states loaded successfully, false otherwise
 */
bool load_relay_states_from_nvs(void);

/**
 * @brief Save SSL/TLS certificates to NVS
 */
void save_certificates_to_nvs(void);

/**
 * @brief Restore relay states from NVS after power recovery
 */
void restore_relay_states_from_nvs(void);

#endif // NVS_STORAGE_H