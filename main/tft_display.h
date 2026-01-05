#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include "common.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>

// ==================== TFT DISPLAY CONFIGURATION ====================

// Display dimensions
#define TFT_WIDTH  240
#define TFT_HEIGHT 240

// Display pins (ST7789 driver)
#define TFT_CS    -1    // Not used
#define TFT_RST   15    // Reset
#define TFT_DC    2     // Data/Command
#define TFT_MOSI  23    // SDA (Hardware MOSI)
#define TFT_SCLK  18    // SCL (Hardware SCLK)
#define TFT_MISO  19    // Not used
#define TFT_BL    22    // LED backlight

// SPI settings
#define TFT_SPI_FREQUENCY  27000000  // 27 MHz

// Display rotation (0-3)
#define TFT_ROTATION 2  // Portrait mode

// ==================== COLOR DEFINITIONS ====================

// RGB565 color definitions
#define TFT_BLACK       0x0000
#define TFT_NAVY        0x000F
#define TFT_DARKGREEN   0x03E0
#define TFT_DARKCYAN    0x03EF
#define TFT_MAROON      0x7800
#define TFT_PURPLE      0x780F
#define TFT_OLIVE       0x7BE0
#define TFT_LIGHTGREY   0xD69A
#define TFT_DARKGREY    0x7BEF
#define TFT_BLUE        0x001F
#define TFT_GREEN       0x07E0
#define TFT_CYAN        0x07FF
#define TFT_RED         0xF800
#define TFT_MAGENTA     0xF81F
#define TFT_YELLOW      0xFFE0
#define TFT_WHITE       0xFFFF
#define TFT_ORANGE      0xFDA0
#define TFT_GREENYELLOW 0xB7E0
#define TFT_PINK        0xFE19

// ==================== DISPLAY STATES ====================

typedef enum {
    DISPLAY_STATE_STARTUP,       // Show BITMINDS logo
    DISPLAY_STATE_WIFI_CONNECTING,
    DISPLAY_STATE_WIFI_CONNECTED,
    DISPLAY_STATE_HW_CONFIG,
    DISPLAY_STATE_HW_READY,
    DISPLAY_STATE_SENSOR_DATA,   // Show individual sensor data
    DISPLAY_STATE_TOTAL_DATA,    // Show total/summary data
    DISPLAY_STATE_ERROR
} display_state_t;

// ==================== DISPLAY DATA STRUCTURE ====================

typedef struct {
    int sensor_number;     // 1-8 for individual sensors, 0 for total
    float voltage;         // Voltage in V
    float current;         // Current in A
    float power;          // Power in W
    float energy;         // Energy in kWh (only for total)
    bool sensor_connected; // Sensor connection status
} tft_display_data_t;

// ==================== TFT DISPLAY FUNCTIONS ====================

/**
 * @brief Initialize TFT display hardware and SPI
 * @return ESP_OK on success, error code on failure
 */
esp_err_t tft_display_init(void);

/**
 * @brief Deinitialize TFT display
 */
void tft_display_deinit(void);

/**
 * @brief Display task - handles all screen updates
 * @param pvParameters Task parameters (unused)
 */
void tft_display_task(void *pvParameters);

/**
 * @brief Show startup message (BITMINDS)
 * @param line1 First line of text
 * @param line2 Second line of text
 * @param duration_ms Display duration in milliseconds
 */
void tft_show_startup_message(const char* line1, const char* line2, uint32_t duration_ms);

/**
 * @brief Show two-line message (WiFi status, etc.)
 * @param line1 First line of text
 * @param line2 Second line of text
 * @param duration_ms Display duration in milliseconds
 */
void tft_show_two_line_message(const char* line1, const char* line2, uint32_t duration_ms);

/**
 * @brief Show individual sensor data
 * @param data Pointer to sensor data structure
 * @param duration_ms Display duration in milliseconds
 */
void tft_show_sensor_data(tft_display_data_t* data, uint32_t duration_ms);

/**
 * @brief Show total/summary data
 * @param data Pointer to total data structure
 * @param duration_ms Display duration in milliseconds
 */
void tft_show_total_data(tft_display_data_t* data, uint32_t duration_ms);

/**
 * @brief Update display with current system state
 * Called from energy meter processing task
 */
void tft_update_display_from_sensor_data(void);

/**
 * @brief Set display state
 * @param state New display state
 */
void tft_set_display_state(display_state_t state);

/**
 * @brief Get current display state
 * @return Current display state
 */
display_state_t tft_get_display_state(void);

/**
 * @brief Clear display screen
 */
void tft_clear_screen(void);

/**
 * @brief Set backlight brightness
 * @param brightness 0-255 (0=off, 255=full brightness)
 */
void tft_set_backlight(uint8_t brightness);

// ==================== HELPER FUNCTIONS ====================

/**
 * @brief Generate simulated sensor data for testing
 * @param sensor_num Sensor number (1-8, or 0 for total)
 * @param data Output data structure
 */
void tft_generate_test_data(int sensor_num, tft_display_data_t* data);

#endif // TFT_DISPLAY_H