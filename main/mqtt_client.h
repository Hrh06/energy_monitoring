#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "common.h"
#include "mqtt_client.h"

// ==================== MQTT CLIENT FUNCTIONS ====================

/**
 * @brief Initialize MQTT client with SSL/TLS configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mqtt_client_init(void);

/**
 * @brief Start MQTT client connection
 */
void mqtt_client_start(void);

/**
 * @brief Stop MQTT client connection
 */
void mqtt_client_stop(void);

/**
 * @brief MQTT event handler (ESP-IDF v4.4 compatible)
 * @param event MQTT event handle
 * @return ESP_OK on success
 */
esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);

/**
 * @brief Publish complete sensor data packet
 * @param packet Complete data packet with all measurements
 */
void publish_sensor_data(complete_data_packet_t *packet);

/**
 * @brief Publish relay status for real-time updates
 */
void publish_relay_status(void);

/**
 * @brief Publish real-time sensor data (1-second updates)
 */
void publish_realtime_data(void);

/**
 * @brief Publish system status information
 */
void publish_system_status(void);

/**
 * @brief Handle incoming MQTT relay control commands
 * @param command_data JSON command string
 * @param data_len Length of command data
 */
void handle_mqtt_relay_command(const char* command_data, int data_len);

/**
 * @brief Verify MQTT connection security status
 * @return true if SSL/TLS active, false otherwise
 */
bool verify_mqtt_connection_security(void);

/**
 * @brief Check if MQTT client is connected
 * @return true if connected, false otherwise
 */
bool mqtt_is_connected(void);

/**
 * @brief Get MQTT connection statistics
 * @param stats Pointer to store statistics
 */
void mqtt_get_stats(void* stats);

#endif // MQTT_CLIENT_H