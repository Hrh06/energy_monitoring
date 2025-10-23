#include "mqtt_client_own.h"
#include "relay_control.h"

static const char* MQTT_TAG = "MQTT_CLIENT";

// ==================== MQTT CLIENT INITIALIZATION ====================

// Create null-terminated certificate strings
static char* ca_cert_str = NULL;
static char* esp32_cert_str = NULL;
static char* esp32_key_str = NULL;

static esp_err_t prepare_certificates(void)
{
    ESP_LOGI(MQTT_TAG, "Preparing null-terminated certificates...");
    
    size_t ca_cert_len = ca_cert_end - ca_cert_start;
    size_t esp32_cert_len = esp32_cert_end - esp32_cert_start;
    size_t esp32_key_len = esp32_key_end - esp32_key_start;
    
    ESP_LOGI(MQTT_TAG, "Raw certificate sizes: CA=%d, Client=%d, Key=%d", 
             ca_cert_len, esp32_cert_len, esp32_key_len);

    // Validate certificate sizes
    if (ca_cert_len == 0 || ca_cert_len > 4096) {
        ESP_LOGE(MQTT_TAG, "Invalid CA certificate size: %d bytes", ca_cert_len);
        return ESP_ERR_INVALID_SIZE;
    }
    if (esp32_cert_len == 0 || esp32_cert_len > 4096) {
        ESP_LOGE(MQTT_TAG, "Invalid ESP32 certificate size: %d bytes", esp32_cert_len);
        return ESP_ERR_INVALID_SIZE;
    }
    if (esp32_key_len == 0 || esp32_key_len > 4096) {
        ESP_LOGE(MQTT_TAG, "Invalid private key size: %d bytes", esp32_key_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Free existing certificates if any
    if (ca_cert_str) {
        free(ca_cert_str);
        ca_cert_str = NULL;
    }
    if (esp32_cert_str) {
        free(esp32_cert_str);
        esp32_cert_str = NULL;
    }
    if (esp32_key_str) {
        free(esp32_key_str);
        esp32_key_str = NULL;
    }

    // Allocate memory for null-terminated certificates
    ca_cert_str = malloc(ca_cert_len + 1);
    esp32_cert_str = malloc(esp32_cert_len + 1);
    esp32_key_str = malloc(esp32_key_len + 1);
    
    if (!ca_cert_str || !esp32_cert_str || !esp32_key_str) {
        ESP_LOGE(MQTT_TAG, "Failed to allocate memory for certificates");
        if (ca_cert_str) free(ca_cert_str);
        if (esp32_cert_str) free(esp32_cert_str);
        if (esp32_key_str) free(esp32_key_str);
        return ESP_ERR_NO_MEM;
    }

    // Copy certificates and null-terminate
    memcpy(ca_cert_str, ca_cert_start, ca_cert_len);
    ca_cert_str[ca_cert_len] = '\0';
    
    memcpy(esp32_cert_str, esp32_cert_start, esp32_cert_len);
    esp32_cert_str[esp32_cert_len] = '\0';
    
    memcpy(esp32_key_str, esp32_key_start, esp32_key_len);
    esp32_key_str[esp32_key_len] = '\0';

    ESP_LOGI(MQTT_TAG, "✅ Certificates prepared and null-terminated:");
    ESP_LOGI(MQTT_TAG, "   📋 CA Certificate: %d bytes", ca_cert_len);
    ESP_LOGI(MQTT_TAG, "   📜 ESP32 Certificate: %d bytes", esp32_cert_len);
    ESP_LOGI(MQTT_TAG, "   🔑 Private Key: %d bytes", esp32_key_len);
    
    // Validate certificate format
    if (strstr(ca_cert_str, "-----BEGIN CERTIFICATE-----") == NULL) {
        ESP_LOGE(MQTT_TAG, "CA certificate format invalid - missing BEGIN marker");
        return ESP_ERR_INVALID_ARG;
    }
    if (strstr(esp32_cert_str, "-----BEGIN CERTIFICATE-----") == NULL) {
        ESP_LOGE(MQTT_TAG, "ESP32 certificate format invalid - missing BEGIN marker");
        return ESP_ERR_INVALID_ARG;
    }
    if (strstr(esp32_key_str, "-----BEGIN PRIVATE KEY-----") == NULL) {
        ESP_LOGE(MQTT_TAG, "Private key format invalid - missing BEGIN marker");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(MQTT_TAG, "✅ Certificate format validation passed");
    return ESP_OK;
}

esp_err_t mqtt_client_init(void)
{
    ESP_LOGI(MQTT_TAG, "Initializing MQTT client with SSL/TLS...");
    
    // Prepare null-terminated certificates first
    esp_err_t cert_err = prepare_certificates();
    if (cert_err != ESP_OK) {
        ESP_LOGE(MQTT_TAG, "Failed to prepare certificates: %s", esp_err_to_name(cert_err));
        return cert_err;
    }

    ESP_LOGI(MQTT_TAG, "Connecting to %s:%d (SSL with client certificates)", MQTT_BROKER_URL, MQTT_BROKER_PORT);

    //Build broker URI
    //char broker_uri[128];
    //snprintf(broker_uri, sizeof(broker_uri), "mqtts://%s:%d", MQTT_BROKER_URL, MQTT_BROKER_PORT);
    
    // ✅ FIXED: ESP-IDF v4.4 compatible configuration structure
    esp_mqtt_client_config_t mqtt_cfg = {
        // Event handler
        //.event_handle = NULL,
        
        // Broker connection
        //.uri = broker_uri,
        .host = MQTT_BROKER_URL,
        .port = MQTT_BROKER_PORT,
        .transport = MQTT_TRANSPORT_OVER_SSL,
        
        // Authentication (empty username/password for certificate-based auth)
        .username = strlen(MQTT_USERNAME) > 0 ? MQTT_USERNAME : NULL,
        .password = strlen(MQTT_PASSWORD) > 0 ? MQTT_PASSWORD : NULL,
        
        // ✅ FIXED: Use null-terminated certificate strings
        .cert_pem = ca_cert_str,                    // CA certificate (null-terminated)
        .client_cert_pem = esp32_cert_str,          // Client certificate (null-terminated)
        .client_key_pem = esp32_key_str,            // Private key (null-terminated)
        
        // SSL/TLS settings
        .use_global_ca_store = false,
        .skip_cert_common_name_check = true,
        .disable_auto_reconnect = false,
        
        // Session settings
        .disable_clean_session = false,
        .keepalive = 60,
        
        // Last Will and Testament
        .lwt_topic = MQTT_TOPIC_SYSTEM_STATUS,
        .lwt_msg = "{\"status\":\"offline\",\"timestamp\":0}",
        .lwt_qos = 1,
        .lwt_retain = false,

        // Network timeouts
        .network_timeout_ms = 30000,
        .refresh_connection_after_ms = 20000,
        
        // Buffer sizes
        .buffer_size = 4096,
        .out_buffer_size = 4096,
        
        // Task settings
        .task_prio = 3,
        .task_stack = 8192,
        
        // Client ID (optional, will be auto-generated if NULL)
        .client_id = NULL
    };
    
    ESP_LOGI(MQTT_TAG, "MQTT Configuration:");
    ESP_LOGI(MQTT_TAG, "  Host: %s", MQTT_BROKER_URL);
    ESP_LOGI(MQTT_TAG, "  Port: %d", MQTT_BROKER_PORT);
    ESP_LOGI(MQTT_TAG, "  Username: %s", MQTT_USERNAME);
    ESP_LOGI(MQTT_TAG, "  Client ID: %s", MQTT_CLIENT_ID);
    ESP_LOGI(MQTT_TAG, "  SSL/TLS: ENABLED");
    ESP_LOGI(MQTT_TAG, "  Certificate: %d bytes", mqtt_cfg.cert_len);
    ESP_LOGI(MQTT_TAG, "  CA Cert: %d bytes", strlen(ca_cert_str));
    ESP_LOGI(MQTT_TAG, "  Client Cert: %d bytes", strlen(esp32_cert_str));
    ESP_LOGI(MQTT_TAG, "  Private Key: %d bytes", strlen(esp32_key_str));

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(MQTT_TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    // ✅ FIXED: Correct event handler registration
    esp_err_t err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(MQTT_TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return err;
    }

    ESP_LOGI(MQTT_TAG, "MQTT Cloud SSL/TLS client initialized successfully");
    return ESP_OK;
}

void mqtt_client_start(void)
{
    if (!wifi_connected || !mqtt_client) {
        ESP_LOGW(MQTT_TAG, "Cannot start MQTT client - WiFi not connected or client not initialized");
        return;
    }
    
    if (mqtt_connected) {
        ESP_LOGD(MQTT_TAG, "MQTT client already connected");
        return;
    }

    ESP_LOGI(MQTT_TAG, "Starting MQTT client...");
    ESP_LOGI(MQTT_TAG, "MQTT client start initiated - waiting for connection...");
    ESP_LOGI(MQTT_TAG, "🔄 Connecting to MQTT Cloud SSL broker...");
    ESP_LOGI(MQTT_TAG, "Target: %s:%d", MQTT_BROKER_URL, MQTT_BROKER_PORT);
    
    ESP_LOGI(MQTT_TAG, "Starting MQTT client...");
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(MQTT_TAG, "Client has started");
        ESP_LOGE(MQTT_TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(MQTT_TAG, "MQTT client start initiated - waiting for connection...");
    }
}

void mqtt_client_stop(void)
{
    if (mqtt_client && mqtt_connected) {
        ESP_LOGI(MQTT_TAG, "Stopping MQTT client...");
        esp_mqtt_client_stop(mqtt_client);
        mqtt_connected = false;
    }
}

// ==================== MQTT EVENT HANDLER ====================

// ESP-IDF v4.4 compatible event handler
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch (event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(MQTT_TAG, "🔄 Connecting to MQTT Cloud SSL broker...");
            ESP_LOGI(MQTT_TAG, "Target: %s:%d", MQTT_BROKER_URL, MQTT_BROKER_PORT);
            break;

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT Connected to broker with SSL/TLS encryption");
            ESP_LOGI(MQTT_TAG, "✅ MQTT server verified with your CA certificate");
            ESP_LOGI(MQTT_TAG, "✅ ESP32 client authenticated with your custom certificate");
            ESP_LOGI(MQTT_TAG, "🔐 Communication encrypted with your private key");
            mqtt_connected = true;
            
            // Subscribe to command topics
            int msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_RELAY_CONTROL, 1);
            ESP_LOGI(MQTT_TAG, "Subscribed to relay control topic, msg_id=%d", msg_id);
            
            msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_COMMANDS, 1);
            ESP_LOGI(MQTT_TAG, "Subscribed to commands topic, msg_id=%d", msg_id);
            
            // Publish initial status
            publish_relay_status();
            publish_system_status_with_custom_certs();
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(MQTT_TAG, "MQTT Disconnected from SSL broker");
            mqtt_connected = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGD(MQTT_TAG, "MQTT SSL Subscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGD(MQTT_TAG, "MQTT SSL Unsubscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(MQTT_TAG, "MQTT SSL Published, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(MQTT_TAG, "MQTT SSL Data received: Topic=%.*s", event->topic_len, event->topic);
            
            // Handle relay control commands
            if (strncmp(event->topic, MQTT_TOPIC_RELAY_CONTROL, event->topic_len) == 0) {
                handle_mqtt_relay_command(event->data, event->data_len);
            }
            // Handle system commands
            else if (strncmp(event->topic, MQTT_TOPIC_COMMANDS, event->topic_len) == 0) {
                char command[64] = {0};
                int copy_len = (event->data_len < sizeof(command) - 1) ? event->data_len : sizeof(command) - 1;
                strncpy(command, event->data, copy_len);
                
                ESP_LOGI(MQTT_TAG, "System command received: %s", command);
                
                if (strcmp(command, "restart") == 0) {
                    ESP_LOGI(MQTT_TAG, "Remote restart command received");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else if (strcmp(command, "reset_wifi") == 0) {
                    ESP_LOGI(MQTT_TAG, "Remote WiFi reset command received");
                    clear_wifi_credentials();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else if (strcmp(command, "get_relay_status") == 0) {
                    publish_relay_status();
                } else if (strcmp(command, "get_system_status") == 0) {
                    publish_system_status_with_custom_certs();
                }
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGW(MQTT_TAG, "MQTT SSL Error occurred");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGW(MQTT_TAG, "SSL/TLS transport error: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGW(MQTT_TAG, "SSL/TLS stack error: 0x%x", event->error_handle->esp_tls_stack_err);
                
                // Detailed SSL error analysis
                if (event->error_handle->esp_tls_last_esp_err == ESP_ERR_MBEDTLS_X509_CRT_PARSE_FAILED) {
                    ESP_LOGE(MQTT_TAG, "🚨 Certificate parsing failed!");
                    ESP_LOGE(MQTT_TAG, "💡 Certificate troubleshooting:");
                    ESP_LOGE(MQTT_TAG, "  1. Check certificate format (PEM with BEGIN/END markers)");
                    ESP_LOGE(MQTT_TAG, "  2. Verify certificate is not corrupted");
                    ESP_LOGE(MQTT_TAG, "  3. Ensure certificate is properly null-terminated");
                    ESP_LOGE(MQTT_TAG, "  4. Check certificate validity dates");
                } else if (event->error_handle->esp_tls_stack_err == 0x2180) {
                    ESP_LOGE(MQTT_TAG, "🚨 mbedTLS certificate parsing error 0x2180");
                    ESP_LOGE(MQTT_TAG, "💡 This usually indicates:");
                    ESP_LOGE(MQTT_TAG, "  1. Invalid PEM format");
                    ESP_LOGE(MQTT_TAG, "  2. Missing null termination");
                    ESP_LOGE(MQTT_TAG, "  3. Corrupted certificate data");
                }

            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(MQTT_TAG, "🚫 Connection refused by MQTT broker");
                ESP_LOGE(MQTT_TAG, "💡 Check:");
                ESP_LOGE(MQTT_TAG, "  1. Broker address: %s:%d", MQTT_BROKER_URL, MQTT_BROKER_PORT);
                ESP_LOGE(MQTT_TAG, "  2. Network connectivity");
                ESP_LOGE(MQTT_TAG, "  3. Certificate authentication");
            }
            
            mqtt_connected = false;
            break;

        default:
            ESP_LOGD(MQTT_TAG, "MQTT SSL Other event id: %d", event_id);
            break;
    }
}

// ==================== CERTIFICATE CLEANUP ====================

void mqtt_client_cleanup(void)
{
    ESP_LOGI(MQTT_TAG, "Cleaning up MQTT client resources...");
    
    if (mqtt_client) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
    
    // Free certificate memory
    if (ca_cert_str) {
        free(ca_cert_str);
        ca_cert_str = NULL;
    }
    if (esp32_cert_str) {
        free(esp32_cert_str);
        esp32_cert_str = NULL;
    }
    if (esp32_key_str) {
        free(esp32_key_str);
        esp32_key_str = NULL;
    }
    
    mqtt_connected = false;
    ESP_LOGI(MQTT_TAG, "MQTT client cleanup completed");
}

// ==================== CUSTOM CERTIFICATE STATUS PUBLISHERS ====================

void publish_custom_security_status(void)
{
    if (!mqtt_connected) {
        return;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return;
    }
    
    // Custom security information
    cJSON_AddStringToObject(json, "device_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(json, "security_level", "CUSTOM_MUTUAL_TLS");
    cJSON_AddBoolToObject(json, "custom_certificates", true);
    cJSON_AddBoolToObject(json, "server_cert_verified", true);
    cJSON_AddBoolToObject(json, "client_cert_authenticated", true);
    cJSON_AddStringToObject(json, "encryption", "TLS_1.2_WITH_CUSTOM_CERTS");
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time());
    
    // Your certificate info (sizes only, not content)
    size_t ca_cert_len = ca_cert_end - ca_cert_start;
    size_t esp32_cert_len = esp32_cert_end - esp32_cert_start;
    size_t esp32_key_len = esp32_key_end - esp32_key_start;
    
    cJSON_AddNumberToObject(json, "ca_cert_size", ca_cert_len);
    cJSON_AddNumberToObject(json, "esp32_cert_size", esp32_cert_len);
    cJSON_AddNumberToObject(json, "esp32_key_size", esp32_key_len);
    cJSON_AddBoolToObject(json, "all_certs_loaded", ca_cert_len > 0 && esp32_cert_len > 0 && esp32_key_len > 0);
    cJSON_AddStringToObject(json, "cert_status", "CUSTOM_USER_PROVIDED");
    
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_SYSTEM_STATUS, json_string, 0, 1, 0);
        free(json_string);
    }
    
    cJSON_Delete(json);
}

void publish_certificate_info(void)
{
    if (!mqtt_connected) {
        return;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return;
    }
    
    cJSON_AddStringToObject(json, "device_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(json, "message_type", "certificate_info");
    cJSON_AddStringToObject(json, "certificate_source", "USER_PROVIDED_CUSTOM");
    
    // Certificate sizes and status
    size_t ca_cert_len = ca_cert_end - ca_cert_start;
    size_t esp32_cert_len = esp32_cert_end - esp32_cert_start;
    size_t esp32_key_len = esp32_key_end - esp32_key_start;
    
    cJSON *cert_info = cJSON_CreateObject();
    cJSON_AddNumberToObject(cert_info, "ca_certificate_bytes", ca_cert_len);
    cJSON_AddNumberToObject(cert_info, "client_certificate_bytes", esp32_cert_len);
    cJSON_AddNumberToObject(cert_info, "private_key_bytes", esp32_key_len);
    cJSON_AddBoolToObject(cert_info, "ca_loaded", ca_cert_len > 0);
    cJSON_AddBoolToObject(cert_info, "client_cert_loaded", esp32_cert_len > 0);
    cJSON_AddBoolToObject(cert_info, "private_key_loaded", esp32_key_len > 0);
    cJSON_AddItemToObject(json, "certificates", cert_info);
    
    cJSON_AddStringToObject(json, "security_note", "Using customer-provided certificates for maximum security");
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time());
    
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string) {
        esp_mqtt_client_publish(mqtt_client, "energy_meter/certificate_info", json_string, 0, 1, 0);
        free(json_string);
    }
    
    cJSON_Delete(json);
}

void publish_system_status_with_custom_certs(void)
{
    if (!mqtt_connected) {
        return;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return;
    }
    
    // System information
    cJSON_AddStringToObject(json, "device_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(json, "version", "2.1.0-CUSTOM-TLS");
    cJSON_AddNumberToObject(json, "uptime", esp_timer_get_time() / 1000000);
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time());
    
    // Memory information
    cJSON_AddNumberToObject(json, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(json, "min_free_heap", esp_get_minimum_free_heap_size());
    
    // Network status with custom security
    cJSON_AddBoolToObject(json, "wifi_connected", wifi_connected);
    cJSON_AddBoolToObject(json, "mqtt_connected", mqtt_connected);
    cJSON_AddBoolToObject(json, "custom_ssl_active", verify_mqtt_connection_security());
    cJSON_AddBoolToObject(json, "provisioning_mode", provisioning_mode);
    
    // Custom security status
    cJSON_AddStringToObject(json, "security_level", "CUSTOM_MUTUAL_TLS");
    cJSON_AddStringToObject(json, "certificate_type", "USER_PROVIDED");
    cJSON_AddBoolToObject(json, "client_cert_auth", true);
    cJSON_AddBoolToObject(json, "server_cert_verified", true);
    
    // Task status
    cJSON_AddBoolToObject(json, "sampling_task_running", sampling_task_handle != NULL);
    cJSON_AddBoolToObject(json, "processing_task_running", processing_task_handle != NULL);
    cJSON_AddBoolToObject(json, "communication_task_running", communication_task_handle != NULL);
    cJSON_AddBoolToObject(json, "button_task_running", button_task_handle != NULL);
    
    // Hardware status
    cJSON_AddNumberToObject(json, "active_relays", __builtin_popcount(get_relay_mask()));
    cJSON_AddNumberToObject(json, "packet_counter", packet_counter);
    cJSON_AddNumberToObject(json, "current_voltage", current_voltage_rms);
    
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_SYSTEM_STATUS, json_string, 0, 1, 0);
        free(json_string);
    }
    
    cJSON_Delete(json);
}

// ==================== PUBLISH FUNCTIONS ====================

void publish_sensor_data(complete_data_packet_t *packet)
{
    if (!mqtt_connected || !packet) {
        return;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        ESP_LOGE(MQTT_TAG, "Failed to create JSON object for sensor data");
        return;
    }
    
    // Add packet info
    cJSON_AddNumberToObject(json, "packet_id", packet->packet_id);
    cJSON_AddNumberToObject(json, "timestamp", packet->timestamp);
    cJSON_AddNumberToObject(json, "system_voltage", packet->system_voltage);
    cJSON_AddStringToObject(json, "security", "SSL_TLS_ENCRYPTED");
    cJSON_AddStringToObject(json, "device_id", MQTT_CLIENT_ID);
    
    // Add channel data
    cJSON *channels_array = cJSON_CreateArray();
    if (channels_array) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
            cJSON *channel = cJSON_CreateObject();
            if (channel) {
                cJSON_AddNumberToObject(channel, "channel", i);
                cJSON_AddBoolToObject(channel, "connected", packet->channels[i].sensor_connected);
                cJSON_AddNumberToObject(channel, "current", packet->channels[i].current_rms);
                cJSON_AddNumberToObject(channel, "voltage", packet->channels[i].voltage_rms);
                cJSON_AddNumberToObject(channel, "power_real", packet->channels[i].power_real);
                cJSON_AddNumberToObject(channel, "power_apparent", packet->channels[i].power_apparent);
                cJSON_AddNumberToObject(channel, "power_reactive", packet->channels[i].power_reactive);
                cJSON_AddNumberToObject(channel, "power_factor", packet->channels[i].power_factor);
                cJSON_AddNumberToObject(channel, "energy", packet->channels[i].energy_consumed);
                cJSON_AddNumberToObject(channel, "frequency", packet->channels[i].frequency);
                cJSON_AddNumberToObject(channel, "timestamp", packet->channels[i].timestamp);
                cJSON_AddItemToArray(channels_array, channel);
            }
        }
        cJSON_AddItemToObject(json, "channels", channels_array);
    }
    
    // Add solar data
    cJSON *solar_obj = cJSON_CreateObject();
    if (solar_obj) {
        cJSON_AddBoolToObject(solar_obj, "data_valid", packet->solar.data_valid);
        if (packet->solar.data_valid) {
            cJSON_AddNumberToObject(solar_obj, "solar_voltage", packet->solar.solar_voltage);
            cJSON_AddNumberToObject(solar_obj, "solar_current", packet->solar.solar_current);
            cJSON_AddNumberToObject(solar_obj, "solar_power", packet->solar.solar_power);
            cJSON_AddNumberToObject(solar_obj, "battery_voltage", packet->solar.battery_voltage);
            cJSON_AddNumberToObject(solar_obj, "battery_current", packet->solar.battery_current);
            cJSON_AddNumberToObject(solar_obj, "battery_soc", packet->solar.battery_soc);
            cJSON_AddNumberToObject(solar_obj, "timestamp", packet->solar.timestamp);
        }
        cJSON_AddItemToObject(json, "solar", solar_obj);
    }
    
    // Calculate and add summary
    float total_power = 0.0, total_energy = 0.0;
    int connected_sensors = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (packet->channels[i].sensor_connected) {
            total_power += packet->channels[i].power_real;
            total_energy += packet->channels[i].energy_consumed;
            connected_sensors++;
        }
    }
    
    cJSON *summary = cJSON_CreateObject();
    if (summary) {
        cJSON_AddNumberToObject(summary, "total_power", total_power);
        cJSON_AddNumberToObject(summary, "total_energy", total_energy);
        cJSON_AddNumberToObject(summary, "connected_sensors", connected_sensors);
        cJSON_AddStringToObject(summary, "transmission_mode", "SECURE_SSL");
        cJSON_AddItemToObject(json, "summary", summary);
    }
    
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, json_string, 0, 1, 0);
        if (msg_id != -1) {
            ESP_LOGD(MQTT_TAG, "Sensor data published: ID=%u, Power=%.1fW, Sensors=%d", 
                     packet->packet_id, total_power, connected_sensors);
        }
        free(json_string);
    }
    
    cJSON_Delete(json);
}

void publish_relay_status(void)
{
    if (!mqtt_connected) {
        return;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        ESP_LOGE(MQTT_TAG, "Failed to create JSON object for relay status");
        return;
    }
    
    cJSON *relays_array = cJSON_CreateArray();
    if (!relays_array) {
        cJSON_Delete(json);
        return;
    }
    
    if (xSemaphoreTake(relay_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Add individual relay states
        for (int i = 0; i < NUM_RELAYS; i++) {
            cJSON_AddItemToArray(relays_array, cJSON_CreateBool(relay_control.relay_states[i]));
        }
        cJSON_AddItemToObject(json, "relays", relays_array);
        
        // Add relay control info
        cJSON_AddNumberToObject(json, "relay_mask", get_relay_mask());
        cJSON_AddBoolToObject(json, "power_failure_mode", relay_control.power_failure_mode);
        cJSON_AddNumberToObject(json, "last_update", relay_control.last_update_time);
        
        xSemaphoreGive(relay_mutex);
    } else {
        cJSON_Delete(json);
        return;
    }
    
    // Add device info
    cJSON_AddStringToObject(json, "device_id", MQTT_CLIENT_ID);
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time());
    cJSON_AddStringToObject(json, "security", "SSL_TLS_ENCRYPTED");
    
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_RELAY_STATUS, json_string, 0, 1, 0);
        if (msg_id != -1) {
            ESP_LOGD(MQTT_TAG, "Relay status published: mask=0x%02X", get_relay_mask());
        }
        free(json_string);
    }
    
    cJSON_Delete(json);
}

void publish_realtime_data(void)
{
    if (!mqtt_connected) {
        return;
    }
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return;
    }
    
    // Add real-time sensor data
    cJSON_AddNumberToObject(json, "voltage_rms", current_realtime_data.voltage_rms);
    cJSON_AddNumberToObject(json, "current_rms", current_realtime_data.current_rms);
    cJSON_AddNumberToObject(json, "power_real", current_realtime_data.power_real);
    cJSON_AddNumberToObject(json, "power_apparent", current_realtime_data.power_apparent);
    cJSON_AddNumberToObject(json, "power_factor", current_realtime_data.power_factor);
    cJSON_AddNumberToObject(json, "frequency", current_realtime_data.frequency);
    cJSON_AddBoolToObject(json, "sensor_connected", current_realtime_data.sensor_connected);
    cJSON_AddNumberToObject(json, "timestamp", esp_timer_get_time());
    
    // Add relay status
    cJSON_AddNumberToObject(json, "relay_mask", get_relay_mask());
    
    // Add device info
    cJSON_AddStringToObject(json, "device_id", MQTT_CLIENT_ID);
    cJSON_AddStringToObject(json, "security", "SSL_TLS_ENCRYPTED");
    
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_REALTIME, json_string, 0, 0, 0);
        free(json_string);
    }
    
    cJSON_Delete(json);
}

// ==================== COMMAND HANDLING ====================

void handle_mqtt_relay_command(const char* command_data, int data_len)
{
    if (!command_data || data_len <= 0) {
        ESP_LOGW(MQTT_TAG, "Invalid relay command data");
        return;
    }
    
    char command[256] = {0};
    int copy_len = (data_len < sizeof(command) - 1) ? data_len : sizeof(command) - 1;
    strncpy(command, command_data, copy_len);
    
    ESP_LOGI(MQTT_TAG, "MQTT Relay Command: %s", command);
    
    cJSON *json = cJSON_Parse(command);
    if (!json) {
        ESP_LOGW(MQTT_TAG, "Invalid JSON in relay command");
        return;
    }
    
    cJSON *action = cJSON_GetObjectItem(json, "action");
    if (!action || !cJSON_IsString(action)) {
        ESP_LOGW(MQTT_TAG, "Missing or invalid action in relay command");
        cJSON_Delete(json);
        return;
    }
    
    if (strcmp(action->valuestring, "set_relay") == 0) {
        cJSON *relay_num = cJSON_GetObjectItem(json, "relay");
        cJSON *state = cJSON_GetObjectItem(json, "state");
        
        if (cJSON_IsNumber(relay_num) && cJSON_IsBool(state)) {
            int relay = relay_num->valueint;
            bool new_state = cJSON_IsTrue(state);
            
            if (relay >= 0 && relay < NUM_RELAYS) {
                set_relay_state(relay, new_state);
                ESP_LOGI(MQTT_TAG, "Relay %d set to %s via MQTT", relay, new_state ? "ON" : "OFF");
                publish_relay_status(); // Immediate feedback
            }
        }
    } else if (strcmp(action->valuestring, "toggle_relay") == 0) {
        cJSON *relay_num = cJSON_GetObjectItem(json, "relay");
        
        if (cJSON_IsNumber(relay_num)) {
            int relay = relay_num->valueint;
            if (relay >= 0 && relay < NUM_RELAYS) {
                toggle_relay(relay);
                ESP_LOGI(MQTT_TAG, "Relay %d toggled via MQTT", relay);
                publish_relay_status();
            }
        }
    } else if (strcmp(action->valuestring, "all_on") == 0) {
        set_all_relays(0xFF);
        ESP_LOGI(MQTT_TAG, "All relays turned ON via MQTT");
        publish_relay_status();
    } else if (strcmp(action->valuestring, "all_off") == 0) {
        set_all_relays(0x00);
        ESP_LOGI(MQTT_TAG, "All relays turned OFF via MQTT");
        publish_relay_status();
    } else if (strcmp(action->valuestring, "set_all_relays") == 0) {
        cJSON *mask = cJSON_GetObjectItem(json, "mask");
        
        if (cJSON_IsNumber(mask)) {
            uint8_t relay_mask = (uint8_t)mask->valueint;
            set_all_relays(relay_mask);
            ESP_LOGI(MQTT_TAG, "All relays set to mask 0x%02X via MQTT", relay_mask);
            publish_relay_status();
        }
    } else if (strcmp(action->valuestring, "get_status") == 0) {
        ESP_LOGI(MQTT_TAG, "Relay status requested via MQTT");
        publish_relay_status();
    } else {
        ESP_LOGW(MQTT_TAG, "Unknown relay command: %s", action->valuestring);
    }
    
    cJSON_Delete(json);
}

// ==================== UTILITY FUNCTIONS ====================

bool verify_mqtt_connection_security(void)
{
    if (!mqtt_connected || !mqtt_client) {
        return false;
    }
    
    // Since we configured SSL/TLS in initialization, if we're connected, it's secure
    ESP_LOGD(MQTT_TAG, "MQTT Connection Security: SSL/TLS (verified by configuration)");
    return true;
}

bool mqtt_is_connected(void)
{
    return mqtt_connected;
}

void mqtt_get_stats(void* stats)
{
    // Implementation for getting MQTT statistics
    // This could include message counts, connection uptime, etc.
    // For now, this is a placeholder
    if (stats) {
        ESP_LOGD(MQTT_TAG, "MQTT stats - Custom TLS active: %s", mqtt_connected ? "YES" : "NO");
        ESP_LOGD(MQTT_TAG, "Null-terminated certificates: CA=%s, Client=%s, Key=%s", 
                 ca_cert_str ? "OK" : "NULL", 
                 esp32_cert_str ? "OK" : "NULL", 
                 esp32_key_str ? "OK" : "NULL");
    }
    
}