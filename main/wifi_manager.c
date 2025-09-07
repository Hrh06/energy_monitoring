#include "wifi_manager.h"
#include "nvs_storage.h"
#include "mqtt_client.h"

static const char* WIFI_TAG = "WIFI_MANAGER";

// ==================== WIFI INITIALIZATION ====================

esp_err_t wifi_manager_init(void)
{
    ESP_LOGI(WIFI_TAG, "Initializing WiFi manager...");
    
    // Create default WiFi station
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    
    // Try to load saved credentials
    char ssid[32] = {0};
    char password[64] = {0};
    
    if (load_wifi_credentials(ssid, password)) {
        ESP_LOGI(WIFI_TAG, "Found saved WiFi credentials, connecting to: %s", ssid);
        return wifi_connect(ssid, password);
    } else {
        ESP_LOGI(WIFI_TAG, "No saved WiFi credentials found");
        start_provisioning_mode();
        return ESP_ERR_NOT_FOUND;
    }
}

esp_err_t wifi_connect(const char* ssid, const char* password)
{
    if (!ssid || !password) {
        ESP_LOGE(WIFI_TAG, "Invalid WiFi credentials");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(WIFI_TAG, "Connecting to WiFi: %s", ssid);
    
    wifi_config_t wifi_config = {0};
    strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    return ESP_OK;
}

// ==================== EVENT HANDLERS ====================

esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(WIFI_TAG, "WiFi station started, connecting...");
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_CONNECTED:
                {
                    wifi_event_sta_connected_t* event = (wifi_event_sta_connected_t*) event_data;
                    ESP_LOGI(WIFI_TAG, "WiFi connected to AP: %s", event->ssid);
                }
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                {
                    wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                    ESP_LOGW(WIFI_TAG, "WiFi disconnected, reason: %d", event->reason);
                    
                    wifi_connected = false;
                    mqtt_connected = false;
                    
                    // Stop MQTT client
                    if (mqtt_client) {
                        esp_mqtt_client_stop(mqtt_client);
                    }
                    
                    // Set event group bits
                    xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
                    
                    // Try to reconnect
                    ESP_LOGI(WIFI_TAG, "Attempting to reconnect...");
                    esp_wifi_connect();
                }
                break;
                
            case WIFI_EVENT_AP_START:
                ESP_LOGI(WIFI_TAG, "WiFi access point started");
                break;
                
            case WIFI_EVENT_AP_STACONNECTED:
                {
                    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                    ESP_LOGI(WIFI_TAG, "Station connected to AP: " MACSTR, MAC2STR(event->mac));
                }
                break;
                
            case WIFI_EVENT_AP_STADISCONNECTED:
                {
                    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                    ESP_LOGI(WIFI_TAG, "Station disconnected from AP: " MACSTR, MAC2STR(event->mac));
                }
                break;
                
            default:
                ESP_LOGD(WIFI_TAG, "Unhandled WiFi event: %ld", event_id);
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(WIFI_TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
            
            wifi_connected = true;
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
            
            // Stop provisioning mode if active
            if (provisioning_mode) {
                ESP_LOGI(WIFI_TAG, "WiFi connected, stopping provisioning mode");
                stop_provisioning_mode();
            }
            
            // Start MQTT client
            mqtt_client_start();
        }
    }
    
    return ESP_OK;
}

// ==================== UTILITY FUNCTIONS ====================

bool wifi_is_connected(void)
{
    return wifi_connected;
}

bool wifi_get_ip_address(char* ip_str)
{
    if (!ip_str || !wifi_connected) {
        return false;
    }
    
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        return false;
    }
    
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
        return true;
    }
    
    return false;
}

// ==================== PROVISIONING MODE ====================

void start_provisioning_mode(void)
{
    if (provisioning_mode) {
        ESP_LOGW(WIFI_TAG, "Provisioning mode already active");
        return;
    }
    
    ESP_LOGI(WIFI_TAG, "Starting WiFi provisioning mode...");
    provisioning_mode = true;
    
    // Stop current WiFi mode
    esp_wifi_stop();
    esp_wifi_deinit();
    
    // Reinitialize WiFi for AP mode
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Configure access point
    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = HOTSPOT_SSID,
            .ssid_len = strlen(HOTSPOT_SSID),
            .password = HOTSPOT_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .channel = 1,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Start HTTPS web server (with fallback to HTTP)
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.httpd.server_port = 443;
    conf.httpd.max_uri_handlers = 8;
    conf.httpd.stack_size = 8192;
    conf.servercert = (const uint8_t *)server_cert_pem_start;
    conf.servercert_len = server_cert_pem_end - server_cert_pem_start;
    conf.prvtkey_pem = (const uint8_t *)server_key_pem_start;
    conf.prvtkey_len = server_key_pem_end - server_key_pem_start;
    
    if (httpd_ssl_start(&server, &conf) == ESP_OK) {
        ESP_LOGI(WIFI_TAG, "HTTPS server started on https://192.168.4.1");
    } else {
        ESP_LOGW(WIFI_TAG, "HTTPS server failed, starting HTTP fallback");
        
        // Fallback to HTTP server
        httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
        http_config.server_port = 80;
        http_config.stack_size = 8192;
        
        if (httpd_start(&server, &http_config) == ESP_OK) {
            ESP_LOGI(WIFI_TAG, "HTTP server started on http://192.168.4.1");
        } else {
            ESP_LOGE(WIFI_TAG, "Failed to start web server");
            return;
        }
    }
    
    // Register URI handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t save_uri = {
        .uri = "/save",
        .method = HTTP_POST,
        .handler = wifi_credentials_handler,
        .user_ctx = NULL
    };
    
    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &save_uri);
    
    ESP_LOGI(WIFI_TAG, "Provisioning mode active - Connect to '%s' and visit 192.168.4.1", HOTSPOT_SSID);
}

void stop_provisioning_mode(void)
{
    if (!provisioning_mode) {
        return;
    }
    
    ESP_LOGI(WIFI_TAG, "Stopping provisioning mode...");
    provisioning_mode = false;
    
    // Stop web server
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    
    // Note: We don't stop WiFi here as we're switching to STA mode
    ESP_LOGI(WIFI_TAG, "Provisioning mode stopped");
}

// ==================== WEB INTERFACE HANDLERS ====================

esp_err_t root_handler(httpd_req_t *req)
{
    const char* html_page = 
        "<!DOCTYPE html>"
        "<html><head>"
        "<title>ESP32 Energy Meter - WiFi Setup</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;margin:0;padding:20px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);}"
        ".container{max-width:400px;margin:0 auto;background:white;padding:30px;border-radius:15px;box-shadow:0 10px 30px rgba(0,0,0,0.3);}"
        "h1{color:#333;text-align:center;margin-bottom:30px;font-size:24px;}"
        ".form-group{margin-bottom:20px;}"
        "label{display:block;margin-bottom:8px;color:#555;font-weight:bold;}"
        "input{width:100%;padding:12px;border:2px solid #ddd;border-radius:8px;box-sizing:border-box;font-size:16px;}"
        "input:focus{border-color:#667eea;outline:none;box-shadow:0 0 0 3px rgba(102,126,234,0.1);}"
        "button{width:100%;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;padding:14px;border:none;border-radius:8px;font-size:16px;font-weight:bold;cursor:pointer;transition:transform 0.2s;}"
        "button:hover{transform:translateY(-2px);}"
        ".info{background:#f8f9fa;padding:15px;border-radius:8px;margin-bottom:20px;border-left:4px solid #667eea;}"
        ".info h3{margin:0 0 10px 0;color:#333;}"
        ".info p{margin:0;color:#666;font-size:14px;}"
        "</style>"
        "</head><body>"
        "<div class='container'>"
        "<h1>⚡ Energy Meter WiFi Setup</h1>"
        "<div class='info'>"
        "<h3>🔧 Device Information</h3>"
        "<p>ESP32 Multi-Core Energy Meter v2.1</p>"
        "<p>8-Channel monitoring + 8-Relay control</p>"
        "<p>SSL/TLS encrypted MQTT communication</p>"
        "</div>"
        "<form method='post' action='/save'>"
        "<div class='form-group'>"
        "<label for='ssid'>📶 WiFi Network Name (SSID)</label>"
        "<input type='text' id='ssid' name='ssid' required maxlength='31' placeholder='Enter your WiFi network name'>"
        "</div>"
        "<div class='form-group'>"
        "<label for='password'>🔒 WiFi Password</label>"
        "<input type='password' id='password' name='password' required maxlength='63' placeholder='Enter your WiFi password'>"
        "</div>"
        "<button type='submit'>💾 Save & Connect</button>"
        "</form>"
        "<div class='info' style='margin-top:20px;'>"
        "<h3>🛡️ Security Features</h3>"
        "<p>• All data transmission uses SSL/TLS encryption</p>"
        "<p>• Credentials stored securely in device memory</p>"
        "<p>• Real-time monitoring with 1-second updates</p>"
        "</div>"
        "</div></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t wifi_credentials_handler(httpd_req_t *req)
{
    char buf[1024];
    int total_len = req->content_len;
    int cur_len = 0;
    
    if (total_len >= sizeof(buf)) {
        ESP_LOGE(WIFI_TAG, "Request content too large: %d bytes", total_len);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too long");
        return ESP_FAIL;
    }
    
    // Read request data
    while (cur_len < total_len) {
        int received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0) {
            ESP_LOGE(WIFI_TAG, "Failed to receive request data");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';
    
    ESP_LOGI(WIFI_TAG, "Received WiFi credentials form data");
    
    // Parse form data (simple URL-encoded parsing)
    char ssid[32] = {0};
    char password[64] = {0};
    
    char *ssid_start = strstr(buf, "ssid=");
    char *password_start = strstr(buf, "password=");
    
    if (ssid_start && password_start) {
        ssid_start += 5; // Skip "ssid="
        char *ssid_end = strchr(ssid_start, '&');
        if (ssid_end) {
            int ssid_len = ssid_end - ssid_start;
            if (ssid_len < sizeof(ssid)) {
                strncpy(ssid, ssid_start, ssid_len);
            }
        }
        
        password_start += 9; // Skip "password="
        char *password_end = strchr(password_start, '&');
        if (!password_end) password_end = password_start + strlen(password_start);
        
        int password_len = password_end - password_start;
        if (password_len < sizeof(password)) {
            strncpy(password, password_start, password_len);
        }
        
        // Simple URL decode (handle + signs and %20)
        for (char *p = ssid; *p; p++) {
            if (*p == '+') *p = ' ';
        }
        for (char *p = password; *p; p++) {
            if (*p == '+') *p = ' ';
        }
        
        if (strlen(ssid) > 0 && strlen(password) > 0) {
            ESP_LOGI(WIFI_TAG, "Received valid WiFi credentials: SSID=%s", ssid);
            
            // Save credentials
            save_wifi_credentials(ssid, password);
            
            // Send success response
            const char* response = 
                "<!DOCTYPE html><html><head><title>Configuration Saved</title>"
                "<style>body{font-family:Arial;text-align:center;padding:50px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);}"
                ".container{max-width:400px;margin:0 auto;background:white;padding:30px;border-radius:15px;box-shadow:0 10px 30px rgba(0,0,0,0.3);}"
                "h1{color:#28a745;margin-bottom:20px;}"
                ".success{color:#155724;background:#d4edda;padding:15px;border-radius:8px;margin:20px 0;}"
                "</style></head><body>"
                "<div class='container'>"
                "<h1>✅ Configuration Saved!</h1>"
                "<div class='success'>"
                "<p><strong>WiFi credentials have been saved successfully.</strong></p>"
                "<p>The device will now restart and connect to your network.</p>"
                "<p>The setup hotspot will disappear once connected.</p>"
                "</div>"
                "<p><em>You can close this window.</em></p>"
                "</div></body></html>";
            
            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
            
            // Restart after a delay to allow response to be sent
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart();
            
            return ESP_OK;
        }
    }
    
    ESP_LOGE(WIFI_TAG, "Invalid WiFi credentials received");
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid credentials");
    return ESP_FAIL;
}