#ifndef ENERGY_METER_H
#define ENERGY_METER_H

#include "common.h"

// ==================== ENERGY METER FUNCTIONS ====================

/**
 * @brief Initialize energy meter hardware and calibration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t energy_meter_init(void);

/**
 * @brief Deinitialize energy meter (cleanup)
 */
void energy_meter_deinit(void);

/**
 * @brief High-frequency sampling task (Core 1)
 * Samples voltage and current at ~10kHz for accurate measurements
 * @param pvParameters Task parameters (unused)
 */
void sampling_task(void *pvParameters);

/**
 * @brief Data processing task (Core 0)
 * Processes raw samples into RMS values and power calculations
 * @param pvParameters Task parameters (unused)
 */
void processing_task(void *pvParameters);

/**
 * @brief Communication task (Core 0)
 * Handles MQTT publishing and solar data communication
 * @param pvParameters Task parameters (unused)
 */
void communication_task(void *pvParameters);

/**
 * @brief Button monitoring task (Core 0)
 * Monitors reset button for WiFi reset (3s) and factory reset (7s)
 * @param pvParameters Task parameters (unused)
 */
void button_task(void *pvParameters);

/**
 * @brief Calculate RMS value from samples
 * @param samples Array of sample values
 * @param count Number of samples
 * @return RMS value
 */
float calculate_rms(const float* samples, int count);

/**
 * @brief Calculate real power from voltage and current samples
 * @param voltage_samples Array of voltage samples
 * @param current_samples Array of current samples
 * @param count Number of samples
 * @return Real power in watts
 */
float calculate_real_power(const float* voltage_samples, const float* current_samples, int count);

/**
 * @brief Calculate apparent power
 * @param voltage_rms RMS voltage
 * @param current_rms RMS current
 * @return Apparent power in VA
 */
float calculate_apparent_power(float voltage_rms, float current_rms);

/**
 * @brief Calculate reactive power
 * @param apparent_power Apparent power in VA
 * @param real_power Real power in watts
 * @return Reactive power in VAR
 */
float calculate_reactive_power(float apparent_power, float real_power);

/**
 * @brief Calculate power factor
 * @param real_power Real power in watts
 * @param apparent_power Apparent power in VA
 * @return Power factor (0.0 to 1.0)
 */
float calculate_power_factor(float real_power, float apparent_power);

/**
 * @brief Parse solar charge controller data
 * @param data_buffer Raw UART data buffer
 * @param solar_data Output structure for parsed data
 * @return true if parsing successful, false otherwise
 */
bool parse_solar_data(const char* data_buffer, solar_data_t* solar_data);

/**
 * @brief Update real-time data structure
 * @param voltage_rms Current voltage RMS
 * @param current_rms Current current RMS
 * @param power_real Current real power
 * @param power_apparent Current apparent power
 * @param power_factor Current power factor
 * @param frequency Current frequency
 * @param sensor_connected Sensor connection status
 */
void update_realtime_data(float voltage_rms, float current_rms, float power_real,
                         float power_apparent, float power_factor, float frequency,
                         bool sensor_connected);

/**
 * @brief Get current energy meter statistics
 * @param stats Pointer to store statistics
 */
void get_energy_meter_stats(void* stats);

/**
 * @brief Calibrate ADC readings
 * @param raw_adc Raw ADC value
 * @param channel ADC channel number
 * @return Calibrated voltage in mV
 */
uint32_t calibrate_adc_reading(uint32_t raw_adc, adc1_channel_t channel);

/**
 * @brief Check if sensor is connected on channel
 * @param channel_num Channel number (0-7)
 * @return true if sensor detected, false otherwise
 */
bool is_sensor_connected(int channel_num);

/**
 * @brief Convert ADC reading to actual voltage/current value
 * @param adc_mv ADC reading in millivolts
 * @param sensor_type 0 for voltage, 1 for current
 * @return Converted value (volts or amps)
 */
float convert_adc_to_measurement(uint32_t adc_mv, int sensor_type);

#endif // ENERGY_METER_H