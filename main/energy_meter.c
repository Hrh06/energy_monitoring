#include "energy_meter.h"
#include "relay_control.h"
#include "mqtt_client.h"
#include "nvs_storage.h"

static const char* ENERGY_TAG = "ENERGY_METER";

// Button interrupt handling variables
static volatile bool button_interrupt_flag = false;
static volatile int64_t button_press_start_time = 0;
static volatile int64_t button_release_time = 0;
static volatile bool button_currently_pressed = false;

// Button interrupt handler (IRAM_ATTR for fast execution)
static void IRAM_ATTR button_interrupt_handler(void* arg)
{
    int64_t current_time = esp_timer_get_time();
    int button_state = gpio_get_level(RESET_BUTTON_PIN);
    
    if (button_state == 0 && !button_currently_pressed) {
        // Button pressed (falling edge)
        button_currently_pressed = true;
        button_press_start_time = current_time;
        button_interrupt_flag = true;
    } else if (button_state == 1 && button_currently_pressed) {
        // Button released (rising edge)
        button_currently_pressed = false;
        button_release_time = current_time;
        button_interrupt_flag = true;
    }
}

// ==================== ENERGY METER INITIALIZATION ====================

esp_err_t energy_meter_init(void)
{
    ESP_LOGI(ENERGY_TAG, "Initializing energy meter hardware...");
    
    // Initialize ADC
    esp_err_t ret = adc1_config_width(ADC_WIDTH_BIT_12);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to configure ADC width: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Allocate ADC calibration characteristics
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    if (!adc_chars) {
        ESP_LOGE(ENERGY_TAG, "Failed to allocate memory for ADC calibration");
        return ESP_ERR_NO_MEM;
    }
    
    // Characterize ADC - FIXED: Use ADC_ATTEN_DB_12 instead of deprecated ADC_ATTEN_DB_11
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, adc_chars);
    
    // Configure voltage sensor pin (GPIO 34) - FIXED: Use ADC_ATTEN_DB_12
    ret = adc1_config_channel_atten(VOLTAGE_SENSOR_PIN, ADC_ATTEN_DB_12);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to configure voltage sensor ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure current sensor pin (GPIO 35) - FIXED: Use ADC_ATTEN_DB_12
    ret = adc1_config_channel_atten(CURRENT_SENSOR_PIN, ADC_ATTEN_DB_12);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to configure current sensor ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure additional channels for multi-channel monitoring - FIXED: Use ADC_ATTEN_DB_12
    for (int i = 0; i < NUM_CHANNELS && i < 8; i++) {
        adc1_channel_t channel = ADC1_CHANNEL_0 + i;
        if (channel <= ADC1_CHANNEL_7) {
            adc1_config_channel_atten(channel, ADC_ATTEN_DB_12);
        }
    }
    
    // Initialize UART for solar data communication
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,  // FIXED: Use UART_SCLK_APB instead of UART_SCLK_DEFAULT
    };
    
    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize GPIO pins for button and LED
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STATUS_LED_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to configure LED GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure reset button (GPIO 0)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RESET_BUTTON_PIN);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to configure button GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Install GPIO interrupt handler with HIGH PRIORITY
    ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(ENERGY_TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add interrupt handler for button
    ret = gpio_isr_handler_add(RESET_BUTTON_PIN, button_interrupt_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(ENERGY_TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize real-time data structure
    memset(&current_realtime_data, 0, sizeof(realtime_data_t));
    current_realtime_data.timestamp = esp_timer_get_time();
    
    ESP_LOGI(ENERGY_TAG, "Energy meter hardware initialized successfully");
    ESP_LOGI(ENERGY_TAG, "ADC Configuration: GPIO34=Voltage, GPIO35=Current");
    ESP_LOGI(ENERGY_TAG, "UART Configuration: TX=%d, RX=%d, Baud=%d", UART_TX_PIN, UART_RX_PIN, 9600);
    ESP_LOGI(ENERGY_TAG, "Button Configuration: GPIO%d (3s=WiFi reset, 7s=Hotspot)", RESET_BUTTON_PIN);
    
    return ESP_OK;
}

void energy_meter_deinit(void)
{
    ESP_LOGI(ENERGY_TAG, "Deinitializing energy meter...");
    
    if (adc_chars) {
        free(adc_chars);
        adc_chars = NULL;
    }
    
    uart_driver_delete(UART_NUM);
    
    ESP_LOGI(ENERGY_TAG, "Energy meter deinitialized");
}

// ==================== SAMPLING TASK (CORE 1) ====================

void sampling_task(void *pvParameters)
{
    ESP_LOGI(ENERGY_TAG, "Sampling task started on Core 1");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const int64_t period_us = 1000000 / (SYSTEM_FREQUENCY * SAMPLES_PER_CYCLE);
    TickType_t xPeriod = pdMS_TO_TICKS(period_us / 1000); // ~100μs for 50Hz
    
    // CRITICAL: Ensure period is at least 1 tick to avoid assertion failure
    if (xPeriod == 0) {
        xPeriod = 1; // Minimum 1 tick (about 1ms on most ESP32 configs)
        ESP_LOGW(ENERGY_TAG, "Sampling period too small, using minimum 1 tick");
    }
    
    ESP_LOGI(ENERGY_TAG, "Sampling period: %d ticks (%.2f ms)", xPeriod, (double)pdTICKS_TO_MS(xPeriod));
    
    cycle_data_t cycle_data = {0};
    int sample_index = 0;
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        
        if (sample_index == 0) {
            cycle_data.cycle_start_time = esp_timer_get_time();
        }
        
        // Sample voltage and current from primary sensors (GPIO 34, 35)
        uint32_t voltage_adc = adc1_get_raw(VOLTAGE_SENSOR_PIN);
        uint32_t current_adc = adc1_get_raw(CURRENT_SENSOR_PIN);
        
        // Convert ADC readings to calibrated voltage values
        uint32_t voltage_mv = calibrate_adc_reading(voltage_adc, VOLTAGE_SENSOR_PIN);
        uint32_t current_mv = calibrate_adc_reading(current_adc, CURRENT_SENSOR_PIN);
        
        // Convert to actual voltage and current values
        cycle_data.voltage_samples[sample_index] = convert_adc_to_measurement(voltage_mv, 0); // 0 = voltage
        cycle_data.current_samples[sample_index] = convert_adc_to_measurement(current_mv, 1); // 1 = current
        
        sample_index++;
        
        if (sample_index >= SAMPLES_PER_CYCLE) {
            cycle_data.sample_count = sample_index;
            
            // Send cycle data to processing queue
            if (xQueueSend(data_queue, &cycle_data, 0) != pdTRUE) {
                ESP_LOGW(ENERGY_TAG, "Data queue full, dropping cycle");
            }
            
            sample_index = 0;
        }
        
        // Yield to other tasks occasionally
        if (sample_index % 10 == 0) {
            taskYIELD();
        }
    }
}

// ==================== PROCESSING TASK (CORE 0) ====================

void processing_task(void *pvParameters)
{
    ESP_LOGI(ENERGY_TAG, "Processing task started on Core 0");
    
    cycle_data_t cycle_data;
    static cycle_data_t packet_cycles[CYCLES_PER_PACKET];
    static int cycle_count = 0;
    
    while (1) {
        if (xQueueReceive(data_queue, &cycle_data, portMAX_DELAY)) {
            packet_cycles[cycle_count] = cycle_data;
            cycle_count++;
            
            // Update real-time data from latest cycle
            if (cycle_data.sample_count > 0) {
                float voltage_rms = calculate_rms(cycle_data.voltage_samples, cycle_data.sample_count);
                float current_rms = calculate_rms(cycle_data.current_samples, cycle_data.sample_count);
                float power_real = calculate_real_power(cycle_data.voltage_samples, cycle_data.current_samples, cycle_data.sample_count);
                float power_apparent = calculate_apparent_power(voltage_rms, current_rms);
                float power_factor = calculate_power_factor(power_real, power_apparent);
                bool sensor_connected = (current_rms > 0.1); // Threshold for sensor detection
                
                update_realtime_data(voltage_rms, current_rms, power_real, power_apparent, 
                                   power_factor, SYSTEM_FREQUENCY, sensor_connected);
                
                current_voltage_rms = voltage_rms;
            }
            
            if (cycle_count >= CYCLES_PER_PACKET) {
                complete_data_packet_t packet = {0};
                packet.packet_id = ++packet_counter;
                packet.timestamp = esp_timer_get_time();
                
                // Calculate RMS values and power metrics for each channel
                for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                    float voltage_sum_sq = 0.0, current_sum_sq = 0.0;
                    float power_sum = 0.0;
                    int total_samples = 0;
                    
                    for (int c = 0; c < CYCLES_PER_PACKET; c++) {
                        for (int s = 0; s < packet_cycles[c].sample_count; s++) {
                            float v = packet_cycles[c].voltage_samples[s];
                            float i = (ch == 0) ? packet_cycles[c].current_samples[s] : 0.0; // Only first channel for now
                            
                            voltage_sum_sq += v * v;
                            current_sum_sq += i * i;
                            power_sum += v * i;
                            total_samples++;
                        }
                    }
                    
                    if (total_samples > 0) {
                        packet.channels[ch].voltage_rms = sqrt(voltage_sum_sq / total_samples);
                        packet.channels[ch].current_rms = sqrt(current_sum_sq / total_samples);
                        packet.channels[ch].power_real = power_sum / total_samples;
                        
                        packet.channels[ch].power_apparent = calculate_apparent_power(
                            packet.channels[ch].voltage_rms, packet.channels[ch].current_rms);
                        packet.channels[ch].power_reactive = calculate_reactive_power(
                            packet.channels[ch].power_apparent, packet.channels[ch].power_real);
                        packet.channels[ch].power_factor = calculate_power_factor(
                            packet.channels[ch].power_real, packet.channels[ch].power_apparent);
                        
                        packet.channels[ch].sensor_connected = (packet.channels[ch].current_rms > 0.1);
                        packet.channels[ch].frequency = SYSTEM_FREQUENCY;
                        
                        // Energy calculation (simplified integration)
                        packet.channels[ch].energy_consumed += packet.channels[ch].power_real * 
                            (CYCLES_PER_PACKET / SYSTEM_FREQUENCY) / 3600.0; // Wh
                        
                        packet.channels[ch].timestamp = esp_timer_get_time();
                    }
                }
                
                packet.system_voltage = packet.channels[0].voltage_rms;
                
                // Try to get solar data
                solar_data_t solar_data;
                if (xQueueReceive(solar_data_queue, &solar_data, 0) == pdTRUE) {
                    packet.solar = solar_data;
                }
                
                // Process automatic relay control
                process_automatic_relay_control(&packet);
                
                // Publish sensor data via MQTT
                publish_sensor_data(&packet);
                
                cycle_count = 0;
                
                ESP_LOGD(ENERGY_TAG, "Packet processed: ID=%u, V=%.1fV, I=%.2fA, P=%.1fW", 
                    packet.packet_id, packet.system_voltage, 
                    packet.channels[0].current_rms, packet.channels[0].power_real);
            }
        }
    }
}

// ==================== COMMUNICATION TASK (CORE 0) ====================

void communication_task(void *pvParameters)
{
    ESP_LOGI(ENERGY_TAG, "Communication task started on Core 0");
    
    char solar_buffer[256];
    int solar_buffer_pos = 0;
    TickType_t last_relay_status_time = 0;
    TickType_t last_realtime_data_time = 0;
    TickType_t last_system_status_time = 0;
    
    while (1) {
        // Read solar data via UART
        uint8_t data[128];
        int length = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        
        if (length > 0) {
            data[length] = '\0';
            
            for (int i = 0; i < length; i++) {
                if (data[i] == '\n' || data[i] == '\r') {
                    if (solar_buffer_pos > 0) {
                        solar_buffer[solar_buffer_pos] = '\0';
                        
                        solar_data_t solar_data = {0};
                        if (parse_solar_data(solar_buffer, &solar_data)) {
                            if (xQueueSend(solar_data_queue, &solar_data, 0) != pdTRUE) {
                                ESP_LOGW(ENERGY_TAG, "Solar data queue full");
                            }
                        }
                        
                        solar_buffer_pos = 0;
                    }
                } else {
                    if (solar_buffer_pos < sizeof(solar_buffer) - 1) {
                        solar_buffer[solar_buffer_pos++] = data[i];
                    }
                }
            }
        }
        
        // MQTT connection maintenance
        if (wifi_connected && !mqtt_connected) {
            ESP_LOGI(ENERGY_TAG, "Attempting MQTT connection...");
            mqtt_client_start();
        }
        
        TickType_t current_time = xTaskGetTickCount();
        
        // Publish relay status every 1 second for responsive control
        if (mqtt_connected && (current_time - last_relay_status_time) >= pdMS_TO_TICKS(RELAY_STATUS_PUBLISH_INTERVAL_MS)) {
            publish_relay_status();
            last_relay_status_time = current_time;
        }
        
        // Publish real-time data every 1 second
        if (mqtt_connected && (current_time - last_realtime_data_time) >= pdMS_TO_TICKS(REALTIME_DATA_INTERVAL_MS)) {
            publish_realtime_data();
            last_realtime_data_time = current_time;
        }
        
        // Publish system status every 30 seconds
        if (mqtt_connected && (current_time - last_system_status_time) >= pdMS_TO_TICKS(30000)) {
            publish_system_status();
            last_system_status_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms loop
    }
}

// ==================== BUTTON TASK (CORE 0) ====================

void button_task(void *pvParameters)
{
    ESP_LOGI(ENERGY_TAG, "Button task started on Core 0 - 3s=WiFi reset, 7s=Hotspot mode");
    
    int led_counter = 0;
    
    while (1) {
        
        // Debounced button checking
        if (button_interrupt_flag) {
            button_interrupt_flag = false;
            
            if (!button_currently_pressed && button_release_time > button_press_start_time) { // Button pressed (active low)
                int64_t press_duration_us = button_release_time - button_press_start_time;
                int32_t press_duration_ms = press_duration_us / 1000;

                ESP_LOGI(ENERGY_TAG, "Reset button released after %u ms", press_duration_ms);

                //check for 3-second WiFi reset
                if (press_duration_ms >= WIFI_RESET_HOLD_TIME_MS && press_duration_ms < HOTSPOT_HOLD_TIME_MS) {
                    ESP_LOGI(ENERGY_TAG, "3-second WiFi reset triggered");

                    // Blink LED 6 times to indicate WiFi reset
                    for (int i = 0; i < 6; i++) {
                        gpio_set_level(STATUS_LED_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(STATUS_LED_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }

                    // Clear WiFi credentials and restart
                    clear_wifi_credentials();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                }

                else if (press_duration_ms >= HOTSPOT_HOLD_TIME_MS) {
                    ESP_LOGI(ENERGY_TAG, "7-Second hotspot mode triggered");
                
                    // Blink LED 6 times to indicate wifi reset
                    for (int i = 0; i < 10; i++) {
                        gpio_set_level(STATUS_LED_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(STATUS_LED_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }

                    // Clear WiFi credentials and restart restart
                    clear_wifi_credentials();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                }
            } else if (button_currently_pressed) {
                ESP_LOGI(ENERGY_TAG, "Reset button pressed - monitoring for hold duration");
            }
        }
        
        // Status LED management
        if (++led_counter >= 10) { // Update LED every ~1 second
            if (provisioning_mode) {
                // Fast blink in provisioning mode
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(STATUS_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
            } else if (wifi_connected && mqtt_connected) {
                // Single blink for fully connected
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(STATUS_LED_PIN, 0);
            } else if (wifi_connected) {
                // Double blink for WiFi only
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(STATUS_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(STATUS_LED_PIN, 0);
            } else {
                // Slow blink for disconnected
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(STATUS_LED_PIN, 0);
            }
            led_counter = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================== CALCULATION FUNCTIONS ====================

float calculate_rms(const float* samples, int count)
{
    if (!samples || count <= 0) {
        return 0.0;
    }
    
    float sum_squares = 0.0;
    for (int i = 0; i < count; i++) {
        sum_squares += samples[i] * samples[i];
    }
    
    return sqrt(sum_squares / count);
}

float calculate_real_power(const float* voltage_samples, const float* current_samples, int count)
{
    if (!voltage_samples || !current_samples || count <= 0) {
        return 0.0;
    }
    
    float power_sum = 0.0;
    for (int i = 0; i < count; i++) {
        power_sum += voltage_samples[i] * current_samples[i];
    }
    
    return power_sum / count;
}

float calculate_apparent_power(float voltage_rms, float current_rms)
{
    return voltage_rms * current_rms;
}

float calculate_reactive_power(float apparent_power, float real_power)
{
    if (apparent_power < real_power) {
        return 0.0;
    }
    
    float reactive_squared = (apparent_power * apparent_power) - (real_power * real_power);
    return sqrt(reactive_squared);
}

float calculate_power_factor(float real_power, float apparent_power)
{
    if (apparent_power < 0.001) { // Avoid division by zero
        return 0.0;
    }
    
    float pf = real_power / apparent_power;
    return (pf > 1.0) ? 1.0 : ((pf < 0.0) ? 0.0 : pf);
}

// ==================== UTILITY FUNCTIONS ====================

uint32_t calibrate_adc_reading(uint32_t raw_adc, adc1_channel_t channel)
{
    if (!adc_chars) {
        return raw_adc; // Fallback if calibration not available
    }
    
    return esp_adc_cal_raw_to_voltage(raw_adc, adc_chars);
}

float convert_adc_to_measurement(uint32_t adc_mv, int sensor_type)
{
    if (sensor_type == 0) { // Voltage sensor
        // Convert ADC voltage to actual AC voltage
        // Assuming voltage divider scales 240V AC to 3.3V DC
        return (adc_mv / 1000.0) * (MAX_VOLTAGE / 3.3);
    } else { // Current sensor
        // Convert ADC voltage to current
        // Assuming CT + burden resistor with 1.65V offset, ±1.65V range for ±MAX_CURRENT
        return ((adc_mv / 1000.0) - 1.65) * (MAX_CURRENT / 1.65);
    }
}

bool parse_solar_data(const char* data_buffer, solar_data_t* solar_data)
{
    if (!data_buffer || !solar_data) {
        return false;
    }
    
    memset(solar_data, 0, sizeof(solar_data_t));
    solar_data->timestamp = esp_timer_get_time();
    
    // Parse solar data format: "V:12.5,I:2.3,P:28.75,BV:12.1,BI:0.5,SOC:85"
    int parsed = sscanf(data_buffer, "V:%f,I:%f,P:%f,BV:%f,BI:%f,SOC:%f", 
                       &solar_data->solar_voltage, &solar_data->solar_current, &solar_data->solar_power,
                       &solar_data->battery_voltage, &solar_data->battery_current, &solar_data->battery_soc);
    
    if (parsed == 6) {
        solar_data->data_valid = true;
        ESP_LOGD(ENERGY_TAG, "Solar data parsed: V=%.1f, I=%.2f, P=%.1f, BV=%.1f, BI=%.2f, SOC=%.0f", 
                 solar_data->solar_voltage, solar_data->solar_current, solar_data->solar_power,
                 solar_data->battery_voltage, solar_data->battery_current, solar_data->battery_soc);
        return true;
    }
    
    ESP_LOGD(ENERGY_TAG, "Failed to parse solar data: %s", data_buffer);
    return false;
}

void update_realtime_data(float voltage_rms, float current_rms, float power_real,
                         float power_apparent, float power_factor, float frequency,
                         bool sensor_connected)
{
    current_realtime_data.voltage_rms = voltage_rms;
    current_realtime_data.current_rms = current_rms;
    current_realtime_data.power_real = power_real;
    current_realtime_data.power_apparent = power_apparent;
    current_realtime_data.power_factor = power_factor;
    current_realtime_data.frequency = frequency;
    current_realtime_data.sensor_connected = sensor_connected;
    current_realtime_data.timestamp = esp_timer_get_time();
}

bool is_sensor_connected(int channel_num)
{
    if (channel_num < 0 || channel_num >= NUM_CHANNELS) {
        return false;
    }
    
    if (channel_num == 0) {
        // Primary channel uses real-time data
        return current_realtime_data.sensor_connected;
    }
    
    // For other channels, we would need to implement multi-channel ADC sampling
    // This is a placeholder implementation
    return false;
}

void get_energy_meter_stats(void* stats)
{
    // Placeholder for energy meter statistics
    // Could include: total energy consumed, uptime, sampling rate, etc.
    ESP_LOGD(ENERGY_TAG, "Energy meter stats requested (not implemented)");
}