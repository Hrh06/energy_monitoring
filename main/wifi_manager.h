#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "common.h"

// ==================== WIFI MANAGER FUNCTIONS ====================

/**
 * @brief Initialize WiFi manager
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no saved credentials
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief WiFi event handler
 * @param arg User argument
 * @param event_base Event base
 * @param event_id Event ID
 * @param event_data Event data
 * @return ESP_OK on success
 */
esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/**
 * @brief Start WiFi provisioning mode (hotspot + web server)
 */
void start_provisioning_mode(void);

/**
 * @brief Stop WiFi provisioning mode
 */
void stop_provisioning_mode(void);

/**
 * @brief HTTP handler for root page (WiFi configuration form)
 * @param req HTTP request
 * @return ESP_OK on success
 */
esp_err_t root_handler(httpd_req_t *req);

/**
 * @brief HTTP handler for WiFi credentials submission
 * @param req HTTP request with form data
 * @return ESP_OK on success
 */
esp_err_t wifi_credentials_handler(httpd_req_t *req);

/**
 * @brief Connect to WiFi with given credentials
 * @param ssid WiFi network name
 * @param password WiFi password
 * @return ESP_OK on success
 */
esp_err_t wifi_connect(const char* ssid, const char* password);

/**
 * @brief Check if WiFi is connected
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * @brief Get current WiFi IP address
 * @param ip_str Buffer to store IP address string (16 bytes minimum)
 * @return true if IP retrieved successfully
 */
bool wifi_get_ip_address(char* ip_str);

#endif // WIFI_MANAGER_H