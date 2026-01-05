# ESP32 Multi-Core Energy Meter with SSL/TLS

## 📋 Project Overview

A professional-grade, multi-core ESP32-based energy monitoring system with 8-channel measurement capabilities, 8-relay control, and secure MQTT communication using custom SSL/TLS certificates.

### Key Features

- **8-Channel Energy Monitoring**: Simultaneous voltage, current, power, and energy measurements
- **8-Relay Control System**: Individual relay control via shift register (74HC595)
- **Secure MQTT Communication**: Custom SSL/TLS certificates with mutual authentication
- **Ultra-High Priority Button System**: Unblockable emergency controls
- **Real-time Data**: 1-second update intervals for monitoring and control
- **WiFi Provisioning**: Web-based hotspot configuration
- **Solar Integration**: UART-based solar charge controller data
- **Multi-Core Architecture**: Optimized task distribution across ESP32 cores

---

## 🔧 Hardware Requirements

### Components

- **ESP32 Development Board** (ESP32-WROOM-32 or similar)
- **Voltage Sensors**: ZMPT101B or similar (x8)
- **Current Sensors**: SCT-013 or similar (x8)
- **Shift Register**: 74HC595 (for 8-relay control)
- **Relays**: 8x relay modules (5V or 3.3V compatible)
- **Reset Button**: Momentary push button
- **Status LED**: Standard LED with current-limiting resistor
- **Solar Charge Controller** (optional): UART-compatible

### Pin Configuration

| Function | GPIO Pin | Notes |
|----------|----------|-------|
| Voltage Sensor | GPIO 34 | ADC1_CHANNEL_6 (input-only) |
| Current Sensor | GPIO 35 | ADC1_CHANNEL_7 (input-only) |
| Reset Button | GPIO 4 | Pull-up enabled, active-low |
| Status LED | GPIO 2 | Built-in LED on most boards |
| Shift Register DATA | GPIO 5 | DS pin of 74HC595 |
| Shift Register CLOCK | GPIO 18 | SH_CP pin of 74HC595 |
| Shift Register LATCH | GPIO 19 | ST_CP pin of 74HC595 |
| UART TX (Solar) | GPIO 17 | UART2 transmit |
| UART RX (Solar) | GPIO 16 | UART2 receive |

---

## 📦 Software Requirements

### ESP-IDF Version

- **ESP-IDF v4.4.x** or **v5.x**
- Tested with ESP-IDF v4.4 and v5.5

### Dependencies

All dependencies are standard ESP-IDF components:

- `nvs_flash` - Non-volatile storage
- `esp_wifi` - WiFi functionality
- `esp_netif` - Network interface
- `esp_http_server` - HTTP server for provisioning
- `esp-tls` - TLS/SSL support (**Note**: Library name is `esp-tls`, not `eps-tls`)
- `mqtt` - MQTT client
- `driver` - GPIO, ADC, UART drivers
- `freertos` - Real-time operating system
- `mbedtls` - Cryptographic library

---

## 🚀 Installation & Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd energy_meter_project
```

### 2. Install ESP-IDF

Follow the official [ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/):

```bash
# For Linux/macOS
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
. ./export.sh
```

### 3. Configure Certificates

**IMPORTANT**: The project requires custom SSL/TLS certificates for secure MQTT communication.

#### Certificate Files Required

Place the following files in the `certs/` directory:

1. **`ca_cert.crt`** - CA certificate to verify your MQTT broker
2. **`esp32_cert.crt`** - Client certificate for ESP32 authentication
3. **`esp32_key.key`** - Private key for ESP32 client certificate

#### Generate Certificates (Optional)

If you need to generate your own certificates:

```bash
# 1. Generate CA private key
openssl genrsa -out ca_key.key 2048

# 2. Generate CA certificate
openssl req -new -x509 -days 3650 -key ca_key.key -out ca_cert.crt \
  -subj "/CN=MyMQTT-CA"

# 3. Generate ESP32 private key
openssl genrsa -out esp32_key.key 2048

# 4. Generate ESP32 certificate signing request (CSR)
openssl req -new -key esp32_key.key -out esp32_cert.csr \
  -subj "/CN=mqtt-client"

# 5. Sign ESP32 certificate with CA
openssl x509 -req -days 365 -in esp32_cert.csr \
  -CA ca_cert.crt -CAkey ca_key.key -CAcreateserial \
  -out esp32_cert.crt
```

**Copy the generated files to the `certs/` directory:**

```bash
mkdir -p certs
cp ca_cert.crt certs/
cp esp32_cert.crt certs/
cp esp32_key.key certs/
```

### 4. Configure Project Settings

Edit `Kconfig.projbuild` to set your MQTT broker details:

```bash
idf.py menuconfig
```

Navigate to **"ESP32 Energy Meter Configuration"** and set:

- **MQTT Broker URL**: Your MQTT broker address
- **MQTT Username**: Your MQTT username (leave empty if using certificate-only auth)
- **MQTT Password**: Your MQTT password (leave empty if using certificate-only auth)

Or directly edit `main/common.h`:

```c
#define MQTT_BROKER_URL "your-broker-address.com"
#define MQTT_BROKER_PORT 8883
#define MQTT_USERNAME "your_username"
#define MQTT_PASSWORD "your_password"
```

### 5. Build and Flash

```bash
# Build the project
idf.py build

# Flash to ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

**Replace `/dev/ttyUSB0` with your actual serial port (e.g., `COM3` on Windows).**

---

## 📡 WiFi Configuration

### Initial Setup (Provisioning Mode)

On first boot or after WiFi reset, the device creates a hotspot:

1. **SSID**: `BitMinds_Energy_Meter`
2. **Password**: `energy123`

**Steps:**

1. Connect to the hotspot
2. Open browser and navigate to `http://192.168.4.1`
3. Enter your WiFi credentials in the web interface
4. Device will save credentials and restart
5. Device connects to your WiFi network

### Button Controls

The reset button (GPIO 4) provides emergency controls:

| Press Duration | Action |
|---------------|--------|
| 3-7 seconds | **WiFi Reset** - Clears WiFi credentials |
| 7-10 seconds | **Hotspot Mode** - Enters provisioning mode |
| 10-12 seconds | **Factory Reset** - Clears all settings |
| 12+ seconds | **Emergency Restart** - Forces immediate restart |

**LED Indicators:**

- **Fast Blink (200ms)**: Provisioning mode
- **Single Blink (1s)**: WiFi + MQTT connected
- **Double Blink (1s)**: WiFi connected only
- **Slow Blink (500ms)**: Disconnected

---

## 📊 MQTT Topics

### Published Topics (Device → Server)

| Topic | Update Rate | Description |
|-------|-------------|-------------|
| `energy_meter/data` | ~7 seconds | Complete sensor data packet with all channels |
| `energy_meter/realtime` | 1 second | Real-time voltage, current, power readings |
| `energy_meter/relay_status` | 1 second | Current state of all 8 relays |
| `energy_meter/system_status` | 30 seconds | System health, memory, uptime, security status |

### Subscribed Topics (Server → Device)

| Topic | Description |
|-------|-------------|
| `energy_meter/relay_control` | Relay control commands (JSON) |
| `energy_meter/commands` | System commands (restart, reset, etc.) |

### Command Examples

#### Relay Control

```json
// Turn on relay 0
{
  "action": "set_relay",
  "relay": 0,
  "state": true
}

// Toggle relay 3
{
  "action": "toggle_relay",
  "relay": 3
}

// Turn on all relays
{
  "action": "all_on"
}

// Turn off all relays
{
  "action": "all_off"
}

// Set specific relay pattern (binary mask)
{
  "action": "set_all_relays",
  "mask": 170  // 0b10101010 - alternating pattern
}

// Request current status
{
  "action": "get_status"
}
```

#### System Commands

Send as plain text to `energy_meter/commands`:

- `restart` - Restart the device
- `reset_wifi` - Clear WiFi credentials and restart
- `get_relay_status` - Request relay status update
- `get_system_status` - Request system status update

---

## 🔐 Security Features

### SSL/TLS Mutual Authentication

The system implements **mutual TLS authentication** with custom certificates:

1. **Server Authentication**: ESP32 verifies MQTT broker using CA certificate
2. **Client Authentication**: MQTT broker verifies ESP32 using client certificate
3. **Encryption**: All data encrypted with TLS 1.2

### Certificate Validation

During build, the system validates:

- Certificate file existence
- Minimum file size (>100 bytes)
- Valid PEM format markers
- BEGIN/END certificate boundaries

### NVS Encryption

Sensitive data stored in Non-Volatile Storage (NVS) with encryption enabled:

- WiFi credentials
- Relay states
- Certificate information
- System configuration

---

## ⚡ System Architecture

### Multi-Core Task Distribution

| Task | Core | Priority | Stack | Function |
|------|------|----------|-------|----------|
| **Button Task** | Core 0 | 24 (MAX) | 4096 | Emergency controls, unblockable |
| **Sampling Task** | Core 1 | 5 | 4096 | High-speed ADC sampling (10kHz) |
| **Processing Task** | Core 0 | 4 | 8192 | RMS calculation, power metrics |
| **Communication Task** | Core 0 | 1 | 8192 | MQTT, UART, network I/O |

### Task Priorities

The button task has **maximum priority (24)** and **cannot be blocked** by any other task, including:

- MQTT connection issues
- Network timeouts
- Memory allocation failures
- System freezes

This ensures **emergency controls always work**, even during system malfunction.

### Data Flow

```
ADC Sampling (10kHz)
    ↓
Cycle Data Buffer (200 samples)
    ↓
Data Queue (10 cycles)
    ↓
Processing Task (RMS, Power calculations)
    ↓
MQTT Publishing (encrypted)
```

---

## 🛠️ Troubleshooting

### Common Issues

#### 1. Certificate Build Errors

**Error**: `❌ CA certificate not found!`

**Solution**: 
- Ensure all three certificate files exist in `certs/` directory
- Check file names match exactly: `ca_cert.crt`, `esp32_cert.crt`, `esp32_key.key`
- Verify files are in PEM format with proper BEGIN/END markers

#### 2. MQTT Connection Failed

**Error**: `MQTT SSL Error occurred`

**Solutions**:
- Verify broker address and port in `common.h`
- Check certificate validity dates: `openssl x509 -in certs/ca_cert.crt -text -noout`
- Ensure broker is configured to accept your client certificate
- Test broker connectivity: `openssl s_client -connect your-broker:8883 -CAfile certs/ca_cert.crt`

#### 3. WiFi Connection Issues

**Symptoms**: Device stuck in provisioning mode

**Solutions**:
- Press reset button for 3 seconds to clear WiFi credentials
- Check WiFi SSID and password for typos
- Ensure WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
- Verify WiFi router is not blocking new device connections

#### 4. Button Not Responding

**Symptoms**: Button press doesn't trigger actions

**Solutions**:
- Check GPIO 4 physical connection
- Verify pull-up resistor (internal pull-up should be enabled)
- Monitor serial output for button press logs
- Button task priority should always be 24 (check with `emergency_system_status()`)

#### 5. Relay Control Issues

**Symptoms**: Relays not switching

**Solutions**:
- Verify shift register connections (GPIO 5, 18, 19)
- Check 74HC595 power supply (VCC and GND)
- Test shift register with `set_all_relays(0xFF)` command
- Measure voltage on relay coils when activated

#### 6. ADC Reading Errors

**Symptoms**: Incorrect voltage/current readings

**Solutions**:
- Calibrate ADC using `esp_adc_cal_characterize()`
- Check sensor connections to GPIO 34 and 35
- Verify sensor voltage divider ratios
- Adjust `convert_adc_to_measurement()` scaling factors

---

## 📈 Performance Metrics

- **Sampling Rate**: 10 kHz (200 samples per AC cycle at 50Hz)
- **Update Rate**: Real-time data every 1 second
- **Accuracy**: ±1% (with proper calibration)
- **Response Time**: Relay control < 50ms
- **Button Response**: < 50ms (ultra-high priority)
- **MQTT Latency**: < 100ms (network dependent)

---

## 🔍 Debugging

### Enable Debug Logs

Edit `sdkconfig.defaults`:

```ini
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
```

### Monitor Serial Output

```bash
idf.py monitor
```

### Task Stack Analysis

The system includes built-in stack monitoring:

```c
// Automatic emergency status check every 60 seconds
emergency_system_status();

// Manual debug output
debug_task_stacks();
```

## 🔧 Configuration Options

### Kconfig Menu

Access via `idf.py menuconfig`:

```
ESP32 Energy Meter Configuration
├── MQTT Broker URL
├── MQTT Username
├── MQTT Password
├── Default WiFi SSID
└── Default WiFi Password
```

### Build-Time Configuration

Edit `main/common.h` for advanced settings:

```c
// System Configuration
#define NUM_CHANNELS 8               // Number of measurement channels
#define NUM_RELAYS 8                 // Number of relay outputs
#define SAMPLES_PER_CYCLE 200        // ADC samples per AC cycle
#define SYSTEM_FREQUENCY 50.0        // AC frequency (50Hz or 60Hz)

// Network Configuration
#define MQTT_BROKER_PORT 8883        // MQTT SSL/TLS port
#define HOTSPOT_SSID "BitMinds_Energy_Meter"
#define HOTSPOT_PASS "energy123"

// Button Timing
#define WIFI_RESET_HOLD_TIME_MS 3000
#define HOTSPOT_HOLD_TIME_MS 7000
#define FACTORY_RESET_HOLD_TIME_MS 10000
```

---

## 🧪 Testing

### Unit Tests

Test individual relay control:

```bash
# Via MQTT
mosquitto_pub -h your-broker -p 8883 --cafile certs/ca_cert.crt --cert certs/esp32_cert.crt --key certs/esp32_key.key -t energy_meter/relay_control -m '{"action":"set_relay","relay":0,"state":true}'
```

### System Tests

1. **WiFi Provisioning**: Clear credentials and test web interface
2. **Button Emergency**: Test all button hold durations
3. **MQTT Security**: Verify SSL/TLS connection with Wireshark
4. **Relay Control**: Test all 256 relay combinations (0x00 to 0xFF)
5. **Power Failure**: Test relay state restoration after power cycle

---

## 📚 API Reference

### Relay Control Functions

```c
// Initialize relay system
esp_err_t relay_control_init(void);

// Set individual relay
void set_relay_state(int relay_num, bool state);

// Set all relays with bitmask
void set_all_relays(uint8_t relay_mask);

// Toggle relay
void toggle_relay(int relay_num);

// Get relay state
bool get_relay_state(int relay_num);
uint8_t get_relay_mask(void);
```

### MQTT Functions

```c
// Initialize MQTT client
esp_err_t mqtt_client_init(void);

// Start/stop MQTT
void mqtt_client_start(void);
void mqtt_client_stop(void);

// Publish data
void publish_sensor_data(complete_data_packet_t *packet);
void publish_relay_status(void);
void publish_realtime_data(void);
```

### WiFi Functions

```c
// Initialize WiFi manager
esp_err_t wifi_manager_init(void);

// Connect to network
esp_err_t wifi_connect(const char* ssid, const char* password);

// Provisioning mode
void start_provisioning_mode(void);
void stop_provisioning_mode(void);
```

---

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style

- Follow ESP-IDF coding standards
- Use meaningful variable names
- Add comments for complex logic
- Update documentation for new features

---

## 📄 License

This project is licensed under the MIT License. See `LICENSE` file for details.

---

## 🎯 Roadmap

### Future Enhancements

- [ ] Web-based dashboard for real-time monitoring
- [ ] Cloud data logging and analytics
- [ ] Mobile app for remote control
- [ ] Energy consumption reports
- [ ] Power quality analysis (harmonics, distortion)
- [ ] OTA (Over-The-Air) firmware updates
- [ ] Multiple user authentication
- [ ] Historical data storage with SQLite

---

## 📊 Version History

### v2.1.0 (Current)
- ✅ Ultra-high priority button system (unblockable)
- ✅ Custom SSL/TLS certificates support
- ✅ Multi-core task optimization
- ✅ Real-time data (1-second updates)
- ✅ Web-based WiFi provisioning
- ✅ 8-channel energy monitoring
- ✅ 8-relay control via shift register

### v2.0.0
- ✅ ESP-IDF v4.4+ compatibility
- ✅ MQTT with SSL/TLS
- ✅ NVS encryption
- ✅ Solar integration

### v1.0.0
- ✅ Basic energy monitoring
- ✅ WiFi connectivity
- ✅ MQTT communication

---

## 🙏 Acknowledgments

- **Espressif Systems** - ESP-IDF framework
- **Eclipse Paho** - MQTT library
- **mbedTLS** - SSL/TLS implementation
- **Community Contributors** - Bug reports and feature suggestions

---

## 📞 Contact

**Project Maintainer**: Haneen
**Email**: kormathhaneen@gmail.com  

---

## 📖 Additional Resources

### Hardware Design
- ESP32 Datasheet: [Espressif](https://www.espressif.com/en/products/socs/esp32)
- 74HC595 Datasheet: [Texas Instruments](https://www.ti.com/product/SN74HC595)
- ZMPT101B Voltage Sensor: [Datasheet](https://components101.com/sensors/zmpt101b-voltage-sensor-module)
- SCT-013 Current Sensor: [Datasheet](https://openenergymonitor.org/forum-archive/node/156.html)

### Software Development
- ESP-IDF Programming Guide: [Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- FreeRTOS Documentation: [Website](https://www.freertos.org/)
- MQTT Protocol: [MQTT.org](https://mqtt.org/)

---

**Last Updated**: January 2025  
**Project Version**: 2.1.0  
**ESP-IDF Version**: 4.4.x / 5.x

---

*This README is a living document. Please report any inaccuracies or suggestions for improvement.*