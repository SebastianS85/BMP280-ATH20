# AHT20-BMP280 Sensor Project for ESP-IDF v5.4

A comprehensive ESP32 sensor monitoring system using AHT20 (temperature/humidity) and BMP280 (pressure/temperature) sensors with ESP-IDF v5.4.

## 📋 Project Overview

This project demonstrates how to interface with multiple I2C sensors on an ESP32-S3, providing environmental monitoring capabilities including:
- Temperature readings from two sensors
- Humidity measurements
- Barometric pressure readings
- Altitude calculations
- Advanced environmental calculations (heat index, dew point)

## 🔧 Hardware Requirements

- **ESP32-S3** development board
- **AHT20** Temperature & Humidity Sensor
- **BMP280** Pressure & Temperature Sensor
- Breadboard and jumper wires
- 3.3V power supply

## 📐 Wiring Diagram

| Pin Function | ESP32-S3 GPIO | AHT20 | BMP280 |
|--------------|---------------|-------|---------|
| **SCL (Clock)** | GPIO 20 | SCL | SCL |
| **SDA (Data)** | GPIO 21 | SDA | SDA |
| **VCC** | 3.3V | VDD | VCC |
| **GND** | GND | GND | GND |

### I2C Addresses
- **AHT20**: 0x38
- **BMP280**: 0x77 (auto-detects 0x76 as fallback)

## 🚀 Getting Started

### Prerequisites
- ESP-IDF v5.4 installed and configured
- Python 3.x with ESP-IDF tools
- USB cable for ESP32-S3

### Installation & Build

```bash
# Clone the repository
git clone https://github.com/SebastianS85/BMP280-ATH20.git
cd BMP280-ATH20

# Set ESP-IDF environment (Windows)
$env:IDF_PATH = 'c:\esp\v5.4\esp-idf'

# Configure the project for ESP32-S3
idf.py set-target esp32s3

# Build the project
idf.py build

# Flash to device (replace COM15 with your port)
idf.py flash -p COM15

# Monitor output
idf.py monitor -p COM15
```

## 📚 Component Structure

### 📁 Project Organization

```
├── components/
│   ├── AHT20/                 # AHT20 sensor driver
│   │   ├── include/AHT20.h    # Header file
│   │   ├── AHT20.c            # Implementation
│   │   └── CMakeLists.txt     # Build configuration
│   ├── BMP280/                # BMP280 sensor driver
│   │   ├── include/BMP280.h   # Header file
│   │   ├── BMP280.c           # Implementation
│   │   └── CMakeLists.txt     # Build configuration
│   ├── i2c/                   # I2C bus management
│   │   ├── include/i2c_bus.h  # Header file
│   │   ├── i2c_bus.c          # Implementation
│   │   └── CMakeLists.txt     # Build configuration
│   └── sensors_combined/      # Advanced sensor functions
│       ├── include/sensors_combined.h
│       ├── sensors_combined.c
│       └── CMakeLists.txt
├── main/
│   ├── main.c                 # Main application
│   └── CMakeLists.txt
└── README.md                  # This file
```

## 🔍 Available Functions

### AHT20 Functions

| Function | Description | Return Type |
|----------|-------------|-------------|
| `aht20_init()` | Initialize AHT20 sensor with reset and calibration | `esp_err_t` |
| `aht20_read_temperature(float *temp)` | Read temperature only | `esp_err_t` |
| `aht20_read_humidity(float *humidity)` | Read humidity only | `esp_err_t` |
| `aht20_read_temp_hum(float *temp, float *hum)` | Read both values in one call | `esp_err_t` |
| `aht20_get_status(uint8_t *status)` | Get sensor status (busy/calibrated) | `esp_err_t` |
| `aht20_read_data()` | Legacy function with logging | `void` |

### BMP280 Functions

| Function | Description | Return Type |
|----------|-------------|-------------|
| `bmp280_init()` | Initialize BMP280 with auto-address detection | `esp_err_t` |
| `bmp280_configure()` | Configure sensor registers | `esp_err_t` |
| `bmp280_read_calibration()` | Read and validate calibration data | `esp_err_t` |
| `bmp280_read_temperature(float *temp)` | Read calibrated temperature | `esp_err_t` |
| `bmp280_read_pressure(float *pressure)` | Read calibrated pressure (hPa) | `esp_err_t` |
| `bmp280_calculate_altitude(float pressure, float *alt)` | Calculate altitude from pressure | `esp_err_t` |
| `bmp280_read_all(float *temp, float *press, float *alt)` | Read all values | `esp_err_t` |
| `bmp280_get_id()` | Read and log chip ID | `void` |
| `bmp280_get_status()` | Read sensor status | `void` |

### I2C Bus Functions

| Function | Description | Return Type |
|----------|-------------|-------------|
| `i2c_bus_init()` | Initialize I2C bus with pullups | `esp_err_t` |
| `i2c_scanner()` | Scan for I2C devices (debugging) | `void` |

### Advanced Functions (sensors_combined)

| Function | Description | Return Type |
|----------|-------------|-------------|
| `read_all_sensors(sensor_data_t *data)` | Read all sensors and calculate derived values | `esp_err_t` |
| `calculate_derived_values(sensor_data_t *data)` | Calculate heat index, dew point, etc. | `esp_err_t` |
| `print_all_sensor_data(const sensor_data_t *data)` | Formatted output of all values | `void` |
| `calculate_heat_index(float temp, float humidity)` | Calculate apparent temperature | `float` |
| `calculate_dew_point(float temp, float humidity)` | Calculate condensation temperature | `float` |

## 📊 Sensor Specifications

### AHT20 (Temperature & Humidity)
- **Temperature Range**: -40°C to +85°C
- **Temperature Accuracy**: ±0.3°C
- **Humidity Range**: 0-100% RH
- **Humidity Accuracy**: ±2% RH
- **Resolution**: 0.01°C, 0.024% RH

### BMP280 (Pressure & Temperature)
- **Pressure Range**: 300-1100 hPa
- **Pressure Accuracy**: ±1 hPa
- **Temperature Range**: -40°C to +85°C
- **Temperature Accuracy**: ±1°C
- **Resolution**: 0.16 Pa, 0.01°C

## 🖥️ Sample Output

```
I (2798) i2c: I2C bus initialized successfully
I (2798) AHT20: AHT20 initialized successfully
I (2808) BMP280: BMP280 found at address 0x77
I (2808) BMP280: BMP280 Chip ID: 0x58
I (2818) BMP280: BMP280 reset successfully
I (2928) BMP280: Calibration data read successfully
I (2928) BMP280: BMP280 configured successfully
I (2928) BMP280: BMP280 initialization complete

=== SENSOR READING CYCLE ===

I (3938) AHT20: AHT20 Temperature: 22.67 °C
I (3938) i2c: ✓ AHT20 Temperature: 22.67 °C
I (4018) AHT20: AHT20 Humidity: 61.85 %RH
I (4018) i2c: ✓ AHT20 Humidity: 61.85 %RH
I (5028) BMP280: Temperature: 22.45 °C
I (5028) i2c: ✓ BMP280 Temperature: 22.45 °C
I (5038) BMP280: Pressure: 1013.21 hPa
I (5038) i2c: ✓ BMP280 Pressure: 1013.21 hPa
I (5038) BMP280: Calculated Altitude: 8.45 meters
```

## ⚙️ Configuration

### I2C Configuration
- **Frequency**: 400 kHz (fast mode)
- **Internal Pull-ups**: Enabled
- **Glitch Filter**: 7 counts
- **Timeout**: 1000ms for critical operations

### Timing Parameters
- **AHT20 Measurement Time**: 80ms
- **BMP280 Reset Time**: 100ms
- **Reading Cycle**: 10 seconds

## 🛠️ Troubleshooting

### Common Issues

1. **"I2C transaction unexpected nack detected"**
   - Check wiring connections
   - Verify 3.3V power supply
   - Ensure sensors are properly connected to GPIO 20/21

2. **"Invalid calibration data"**
   - BMP280 may need reset (handled automatically)
   - Check I2C communication
   - Verify sensor is genuine BMP280

3. **Temperature readings are 0.00°C**
   - Calibration data not loaded
   - Sensor not configured properly
   - Check initialization sequence

### Debug Features
- Enable I2C scanner: Call `i2c_scanner()` in main.c
- Raw calibration data logging
- Detailed error messages with `esp_err_to_name()`

## 📖 API Documentation

### Error Handling
All functions return `esp_err_t` values:
- `ESP_OK`: Success
- `ESP_FAIL`: General failure
- `ESP_ERR_INVALID_STATE`: Sensor not initialized
- `ESP_ERR_INVALID_RESPONSE`: Invalid sensor data

### Data Structures

```c
typedef struct {
    // AHT20 data
    float aht20_temperature;
    float aht20_humidity;
    uint8_t aht20_status;
    
    // BMP280 data
    float bmp280_temperature;
    float bmp280_pressure_hpa;
    float bmp280_altitude_m;
    
    // Calculated values
    float temperature_diff;
    float heat_index;
    float dew_point;
} sensor_data_t;
```

## 🔄 Versions

- **ESP-IDF**: v5.4
- **Target**: ESP32-S3
- **Language**: C
- **Build System**: CMake

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is open source and available under the MIT License.

## 🙏 Acknowledgments

- ESP-IDF framework by Espressif Systems
- AHT20 and BMP280 sensor datasheets
- ESP32 community for libraries and examples