#include "tft_display.h"
#include <math.h>

static const char* TFT_TAG = "TFT_DISPLAY";

// ==================== GLOBAL VARIABLES ====================

static display_state_t current_display_state = DISPLAY_STATE_STARTUP;
static SemaphoreHandle_t display_mutex = NULL;
static TaskHandle_t display_task_handle = NULL;

// Global voltage that changes every 10 seconds
static float global_voltage = 0.0;
static uint64_t last_voltage_update = 0;
#define VOLTAGE_UPDATE_INTERVAL_MS 10000  // 10 seconds

// SPI device handle
static spi_device_handle_t spi_device = NULL;

// Display buffer (optional - for faster updates)
static uint16_t* display_buffer = NULL;
#define DISPLAY_BUFFER_SIZE (TFT_WIDTH * 20)  // 20 lines buffer

// ==================== LOW-LEVEL SPI FUNCTIONS ====================

// Send command to TFT
static void tft_send_command(uint8_t cmd)
{
    gpio_set_level(TFT_DC, 0);  // Command mode
    
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &cmd,
        .flags = 0
    };
    
    spi_device_polling_transmit(spi_device, &trans);
}

// Send data byte to TFT
static void tft_send_data(uint8_t data)
{
    gpio_set_level(TFT_DC, 1);  // Data mode
    
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &data,
        .flags = 0
    };
    
    spi_device_polling_transmit(spi_device, &trans);
}

// Send data buffer to TFT
static void tft_send_data_buffer(const uint8_t* data, size_t len)
{
    if (!data || len == 0) return;
    
    gpio_set_level(TFT_DC, 1);  // Data mode
    
    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = data,
        .flags = 0
    };
    
    spi_device_polling_transmit(spi_device, &trans);
}

// ==================== TFT HARDWARE INITIALIZATION ====================

esp_err_t tft_display_init(void)
{
    ESP_LOGI(TFT_TAG, "Initializing TFT display (ST7789, 240x240)...");
    
    // Create display mutex
    display_mutex = xSemaphoreCreateMutex();
    if (!display_mutex) {
        ESP_LOGE(TFT_TAG, "Failed to create display mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate display buffer
    display_buffer = (uint16_t*)heap_caps_malloc(DISPLAY_BUFFER_SIZE * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!display_buffer) {
        ESP_LOGW(TFT_TAG, "Failed to allocate display buffer - will work without buffering");
    }
    
    // Configure GPIO pins
    gpio_config_t io_conf = {0};
    
    // Configure backlight as output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << TFT_BL);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(TFT_BL, 1);  // Turn on backlight
    
    // Configure DC pin as output
    io_conf.pin_bit_mask = (1ULL << TFT_DC);
    gpio_config(&io_conf);
    
    // Configure RST pin as output
    io_conf.pin_bit_mask = (1ULL << TFT_RST);
    gpio_config(&io_conf);
    
    // Hardware reset sequence
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = TFT_MISO,
        .mosi_io_num = TFT_MOSI,
        .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_WIDTH * TFT_HEIGHT * 2
    };
    
    // Initialize SPI bus (HSPI)
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TFT_TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = TFT_SPI_FREQUENCY,
        .mode = 0,  // SPI mode 0
        .spics_io_num = TFT_CS,  // -1 if not used
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TFT_TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // ST7789 initialization sequence
    ESP_LOGI(TFT_TAG, "Initializing ST7789 display driver...");
    
    // Software reset
    tft_send_command(0x01);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Sleep out
    tft_send_command(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Color mode - 16-bit color (RGB565)
    tft_send_command(0x3A);
    tft_send_data(0x05);
    
    // Memory access control (rotation)
    tft_send_command(0x36);
    tft_send_data(0x00);  // No rotation
    
    // Display inversion on
    tft_send_command(0x21);
    
    // Normal display mode
    tft_send_command(0x13);
    
    // Display on
    tft_send_command(0x29);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Clear screen to black
    tft_clear_screen();
    
    // Initialize random voltage
    global_voltage = (float)(esp_random() % 200 + 200) / 100.0f;  // 2.00-4.00V
    last_voltage_update = esp_timer_get_time();
    
    ESP_LOGI(TFT_TAG, "TFT display initialized successfully");
    ESP_LOGI(TFT_TAG, "Display: 240x240, ST7789, 16-bit RGB565");
    
    return ESP_OK;
}

void tft_display_deinit(void)
{
    ESP_LOGI(TFT_TAG, "Deinitializing TFT display...");
    
    if (display_task_handle) {
        vTaskDelete(display_task_handle);
        display_task_handle = NULL;
    }
    
    if (spi_device) {
        spi_bus_remove_device(spi_device);
        spi_device = NULL;
    }
    
    spi_bus_free(HSPI_HOST);
    
    if (display_buffer) {
        free(display_buffer);
        display_buffer = NULL;
    }
    
    if (display_mutex) {
        vSemaphoreDelete(display_mutex);
        display_mutex = NULL;
    }
    
    gpio_set_level(TFT_BL, 0);  // Turn off backlight
    
    ESP_LOGI(TFT_TAG, "TFT display deinitialized");
}

// ==================== DRAWING FUNCTIONS ====================

// Set drawing window
static void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    // Column address set
    tft_send_command(0x2A);
    tft_send_data(x0 >> 8);
    tft_send_data(x0 & 0xFF);
    tft_send_data(x1 >> 8);
    tft_send_data(x1 & 0xFF);
    
    // Row address set
    tft_send_command(0x2B);
    tft_send_data(y0 >> 8);
    tft_send_data(y0 & 0xFF);
    tft_send_data(y1 >> 8);
    tft_send_data(y1 & 0xFF);
    
    // Write to RAM
    tft_send_command(0x2C);
}

// Fill screen with color
void tft_clear_screen(void)
{
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    tft_set_addr_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
    
    gpio_set_level(TFT_DC, 1);  // Data mode
    
    // Send black pixels
    uint16_t black = 0x0000;
    uint8_t black_bytes[2] = {(black >> 8) & 0xFF, black & 0xFF};
    
    for (int i = 0; i < TFT_WIDTH * TFT_HEIGHT; i++) {
        tft_send_data_buffer(black_bytes, 2);
    }
    
    xSemaphoreGive(display_mutex);
}

// Fill rectangle with color
static void tft_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT) return;
    if (x + w > TFT_WIDTH) w = TFT_WIDTH - x;
    if (y + h > TFT_HEIGHT) h = TFT_HEIGHT - y;
    
    tft_set_addr_window(x, y, x + w - 1, y + h - 1);
    
    gpio_set_level(TFT_DC, 1);  // Data mode
    
    uint8_t color_bytes[2] = {(color >> 8) & 0xFF, color & 0xFF};
    
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        tft_send_data_buffer(color_bytes, 2);
    }
}

// Draw horizontal line
static void tft_draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    tft_fill_rect(x, y, w, 1, color);
}

// Simple 8x16 font character drawing (simplified)
static void tft_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t size)
{
    // This is a simplified implementation
    // In production, you would use the TFT_eSPI library's font rendering
    // For now, we'll just draw a simple rectangle for each character
    uint16_t char_width = 6 * size;
    uint16_t char_height = 8 * size;
    
    if (x + char_width > TFT_WIDTH || y + char_height > TFT_HEIGHT) return;
    
    // Draw background
    tft_fill_rect(x, y, char_width, char_height, bg_color);
    
    // Simple character rendering would go here
    // For this example, we'll just show the space
}

// Draw string
static void tft_draw_string(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg_color, uint8_t size)
{
    if (!str) return;
    
    uint16_t cursor_x = x;
    uint16_t char_width = 6 * size;
    
    while (*str) {
        if (cursor_x + char_width > TFT_WIDTH) break;
        tft_draw_char(cursor_x, y, *str, color, bg_color, size);
        cursor_x += char_width;
        str++;
    }
}

// ==================== HIGH-LEVEL DISPLAY FUNCTIONS ====================

void tft_show_startup_message(const char* line1, const char* line2, uint32_t duration_ms)
{
    if (!line1 || !line2) return;
    
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    ESP_LOGI(TFT_TAG, "Showing startup: %s | %s", line1, line2);
    
    // Clear screen
    tft_fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
    
    // Draw centered text (simplified - using rectangles as placeholders)
    uint16_t y_center = TFT_HEIGHT / 2;
    
    // Line 1 - BITMINDS (larger, cyan)
    tft_fill_rect(60, y_center - 30, 120, 20, TFT_CYAN);
    
    // Line 2 - SMART ENERGY (smaller, cyan)
    tft_fill_rect(50, y_center + 10, 140, 15, TFT_CYAN);
    
    xSemaphoreGive(display_mutex);
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

void tft_show_two_line_message(const char* line1, const char* line2, uint32_t duration_ms)
{
    if (!line1 || !line2) return;
    
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    ESP_LOGI(TFT_TAG, "Showing message: %s | %s", line1, line2);
    
    // Clear screen
    tft_fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
    
    // Draw centered text boxes
    uint16_t y_center = TFT_HEIGHT / 2;
    
    // Line 1
    tft_fill_rect(40, y_center - 25, 160, 18, TFT_WHITE);
    
    // Line 2
    tft_fill_rect(40, y_center + 10, 160, 18, TFT_WHITE);
    
    xSemaphoreGive(display_mutex);
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

void tft_show_sensor_data(tft_display_data_t* data, uint32_t duration_ms)
{
    if (!data) return;
    
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    ESP_LOGI(TFT_TAG, "Sensor %d: V=%.2fV, I=%.2fA, P=%.2fW", 
             data->sensor_number, data->voltage, data->current, data->power);
    
    // Clear screen
    tft_fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
    
    // Header - SENSOR X
    tft_fill_rect(10, 5, 220, 35, TFT_YELLOW);
    
    // Separator line
    tft_draw_hline(10, 45, 220, TFT_WHITE);
    
    // Current display (60-90)
    tft_fill_rect(5, 60, 80, 20, TFT_WHITE);      // Label
    tft_fill_rect(90, 60, 110, 20, TFT_GREEN);    // Value
    tft_fill_rect(205, 60, 30, 20, TFT_WHITE);    // Unit
    
    // Voltage display (95-125)
    tft_fill_rect(5, 95, 80, 20, TFT_WHITE);      // Label
    tft_fill_rect(90, 95, 110, 20, TFT_BLUE);     // Value
    tft_fill_rect(205, 95, 30, 20, TFT_WHITE);    // Unit
    
    // Power display (130-160)
    tft_fill_rect(5, 130, 80, 20, TFT_WHITE);     // Label
    tft_fill_rect(90, 130, 110, 20, TFT_RED);     // Value
    tft_fill_rect(205, 130, 30, 20, TFT_WHITE);   // Unit
    
    xSemaphoreGive(display_mutex);
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

void tft_show_total_data(tft_display_data_t* data, uint32_t duration_ms)
{
    if (!data) return;
    
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    ESP_LOGI(TFT_TAG, "TOTAL: V=%.2fV, I=%.2fA, P=%.2fW, E=%.3fkWh", 
             data->voltage, data->current, data->power, data->energy);
    
    // Clear screen
    tft_fill_rect(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_BLACK);
    
    // Header - TOTAL
    tft_fill_rect(10, 5, 220, 35, TFT_MAGENTA);
    
    // Separator line
    tft_draw_hline(10, 45, 220, TFT_WHITE);
    
    // Current display (60-90)
    tft_fill_rect(5, 60, 80, 20, TFT_WHITE);
    tft_fill_rect(90, 60, 110, 20, TFT_GREEN);
    tft_fill_rect(205, 60, 30, 20, TFT_WHITE);
    
    // Voltage display (95-125)
    tft_fill_rect(5, 95, 80, 20, TFT_WHITE);
    tft_fill_rect(90, 95, 110, 20, TFT_BLUE);
    tft_fill_rect(205, 95, 30, 20, TFT_WHITE);
    
    // Power display (135-165)
    tft_fill_rect(5, 135, 80, 20, TFT_WHITE);
    tft_fill_rect(90, 135, 110, 20, TFT_RED);
    tft_fill_rect(205, 135, 30, 20, TFT_WHITE);
    
    // Energy display (170-200)
    tft_fill_rect(5, 170, 80, 20, TFT_WHITE);
    tft_fill_rect(90, 170, 110, 20, TFT_ORANGE);
    tft_fill_rect(205, 170, 30, 20, TFT_WHITE);
    
    xSemaphoreGive(display_mutex);
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

// ==================== DISPLAY TASK ====================

void tft_display_task(void *pvParameters)
{
    ESP_LOGI(TFT_TAG, "Display task started (Priority %d)", uxTaskPriorityGet(NULL));
    
    tft_display_data_t sensor_data = {0};
    
    while (1) {
        // Update global voltage every 10 seconds
        uint64_t current_time = esp_timer_get_time();
        if ((current_time - last_voltage_update) >= (VOLTAGE_UPDATE_INTERVAL_MS * 1000)) {
            global_voltage = (float)(esp_random() % 200 + 200) / 100.0f;  // 2.00-4.00V
            last_voltage_update = current_time;
            ESP_LOGD(TFT_TAG, "Updated global voltage: %.2fV", global_voltage);
        }
        
        // Startup sequence
        tft_show_startup_message("BITMINDS", "SMART ENERGY", 5000);
        
        tft_show_two_line_message("WiFi", "Reconnecting...", 3000);
        
        tft_show_two_line_message("WiFi", "Connected", 2000);
        
        tft_show_two_line_message("Configuring", "Hardware", 5000);
        
        tft_show_two_line_message("Hardware", "Configured", 2000);
        
        // Show sensor data for each of 8 sensors
        for (int sensor = 1; sensor <= 8; sensor++) {
            tft_generate_test_data(sensor, &sensor_data);
            sensor_data.voltage = global_voltage;  // Use global voltage
            tft_show_sensor_data(&sensor_data, 2000);
        }
        
        // Show total data
        tft_generate_test_data(0, &sensor_data);
        sensor_data.voltage = global_voltage;  // Use global voltage
        tft_show_total_data(&sensor_data, 3000);
        
        // Small delay before repeating
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== STATE MANAGEMENT ====================

void tft_set_display_state(display_state_t state)
{
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_display_state = state;
        xSemaphoreGive(display_mutex);
        ESP_LOGD(TFT_TAG, "Display state changed to: %d", state);
    }
}

display_state_t tft_get_display_state(void)
{
    display_state_t state = DISPLAY_STATE_STARTUP;
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        state = current_display_state;
        xSemaphoreGive(display_mutex);
    }
    return state;
}

// ==================== BACKLIGHT CONTROL ====================

void tft_set_backlight(uint8_t brightness)
{
    // Simple on/off control
    if (brightness > 128) {
        gpio_set_level(TFT_BL, 1);
    } else {
        gpio_set_level(TFT_BL, 0);
    }
}

// ==================== TEST DATA GENERATION ====================

void tft_generate_test_data(int sensor_num, tft_display_data_t* data)
{
    if (!data) return;
    
    data->sensor_number = sensor_num;
    
    if (sensor_num == 0) {
        // Total data
        data->current = (float)(esp_random() % 40 + 10) / 100.0f;  // 0.10-0.50A
        data->voltage = global_voltage;
        data->power = data->current * data->voltage;
        data->energy = (float)(esp_random() % 3 + 1) / 100.0f;  // 0.01-0.04kWh
        data->sensor_connected = true;
    } else {
        // Individual sensor data
        data->current = (float)(esp_random() % 40 + 10) / 100.0f;  // 0.10-0.50A
        data->voltage = global_voltage;
        data->power = data->current * data->voltage;
        data->energy = 0.0f;  // Not used for individual sensors
        data->sensor_connected = true;
    }
}

// ==================== INTEGRATION WITH ENERGY METER ====================

void tft_update_display_from_sensor_data(void)
{
    // This function can be called from the processing task to update display
    // with real sensor data instead of simulated data
    
    // Example: Update display with real voltage and current
    // extern float current_voltage_rms;
    // extern realtime_data_t current_realtime_data;
    
    // You can add logic here to fetch real data and update display
    ESP_LOGD(TFT_TAG, "Display update request received");
}