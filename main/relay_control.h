#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include "common.h"

// ==================== RELAY CONTROL FUNCTIONS ====================

/**
 * @brief Initialize relay control system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t relay_control_init(void);

/**
 * @brief Write data to shift register
 * @param data 8-bit data to write to shift register
 */
void shift_register_write(uint8_t data);

/**
 * @brief Set individual relay state
 * @param relay_num Relay number (0-7)
 * @param state true for ON, false for OFF
 */
void set_relay_state(int relay_num, bool state);

/**
 * @brief Set all relays using bitmask
 * @param relay_mask 8-bit mask (bit 0 = relay 0, etc.)
 */
void set_all_relays(uint8_t relay_mask);

/**
 * @brief Toggle individual relay state
 * @param relay_num Relay number (0-7)
 */
void toggle_relay(int relay_num);

/**
 * @brief Get current relay mask
 * @return 8-bit mask representing current relay states
 */
uint8_t get_relay_mask(void);

/**
 * @brief Get individual relay state
 * @param relay_num Relay number (0-7)
 * @return true if relay is ON, false if OFF
 */
bool get_relay_state(int relay_num);

/**
 * @brief Handle power failure relay management
 */
void handle_power_failure_relays(void);

/**
 * @brief Process automatic relay control based on sensor data
 * @param packet Complete data packet with sensor measurements
 */
void process_automatic_relay_control(complete_data_packet_t *packet);

/**
 * @brief Set automatic control parameters for a relay
 * @param relay_num Relay number (0-7)
 * @param auto_enabled Enable automatic control
 * @param voltage_high High voltage threshold
 * @param voltage_low Low voltage threshold
 * @param current_high High current threshold
 * @param current_low Low current threshold
 */
void set_relay_auto_params(int relay_num, bool auto_enabled, 
                          float voltage_high, float voltage_low,
                          float current_high, float current_low);

/**
 * @brief Set relay to manual or automatic mode
 * @param relay_num Relay number (0-7)
 * @param manual_mode true for manual mode, false for automatic
 */
void set_relay_mode(int relay_num, bool manual_mode);

/**
 * @brief Get relay control statistics
 * @param relay_num Relay number (0-7)
 * @param stats Pointer to store statistics
 */
void get_relay_stats(int relay_num, void* stats);

#endif // RELAY_CONTROL_H