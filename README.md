
# Intelligent Real-Time Speed Adaptation and Crash Detection System for EVs

## Overview
This project implements a smart speed adaptation system for electric bikes that automatically adjusts the maximum speed based on location-specific speed limits, along with crash detection capabilities. The system fetches real-time speed limits from a mobile app and communicates with an ESP32 board to control the vehicle speed, while also providing crash detection and emergency alerts.

## Features
- Real-time speed limit adaptation based on location
- Automatic speed control via PWM
- Crash detection using MPU6050 sensor
- Emergency alert system
- Bluetooth communication between mobile app and ESP32
- Location-based speed limit fetching using Google APIs

## Hardware Requirements
- ESP32 Microcontroller
- MPU6050 Sensor
- DC Motor (for testing)
- Sliding Potentiometer
- STM32 Blue Pill
- LED indicators

## Software Requirements
- Arduino IDE
- Android Studio (for mobile app)
- Required Libraries:
  - BluetoothSerial
  - Adafruit_MPU6050
  - Adafruit_Sensor
  - Wire

## Pin Connections

### ESP32 Connections
| ESP32 Pin | Component      | Connection Description |
|-----------|---------------|----------------------|
| GPIO 2    | MPU Trigger   | Crash detection input |
| GPIO 4    | Potentiometer | Throttle input (0-4095) |
| GPIO 5    | DC Motor      | PWM output for speed control |
| VCC (3.3V)| Components    | Power supply for sensors |
| GND       | Components    | Common ground |

### MPU6050 Connections (to STM32 Blue Pill)
| STM32 Pin | MPU6050 Pin | Connection Description |
|-----------|-------------|----------------------|
| PB6 (SCL) | SCL        | I2C Clock |
| PB7 (SDA) | SDA        | I2C Data |
| 3.3V      | VCC        | Power supply |
| GND       | GND        | Ground |
| PB15      | -          | Crash signal to ESP32 (GPIO 2) |
| PC13      | -          | Onboard LED |
| PC15      | -          | External LED |

### Additional Connections
- Connect the DC motor through an appropriate motor driver
- The sliding potentiometer should have its outer pins connected to 3.3V and GND, with the wiper connected to GPIO 4
- External LED should have a current-limiting resistor (220-330Ω) in series

### Wiring Notes
1. All GND pins should be connected together to create a common ground
2. Use appropriate voltage levels (3.3V) for all components
3. Include pull-up resistors for I2C lines if not internally provided
4. Ensure proper power supply filtering with capacitors 

## Android Application

### Features
- Real-time GPS location tracking
- Speed limit fetching using Google Maps API
- Bluetooth connection to ESP32
- Emergency contact management
- Nearby hospital detection
- Crash alert system

### App Components
1. **Location Services**
   - Uses Google Maps API for real-time location
   - Fetches speed limits using Geo API Fi
   - Updates speed limits every 2 seconds

2. **Bluetooth Communication**
   - Connects to ESP32 module
   - Sends speed limit data
   - Receives crash detection alerts
   - Provides connection status feedback

3. **Emergency System**
   - Stores emergency contact information
   - Uses Geo API Fi to locate nearby hospitals
   - Retrieves hospital information using Go Map Pro API
   - Sends automated SMS alerts during crashes

### APK Installation
1. **Direct Installation**
   - Download `SpeedAdapter.apk` from the releases section
   - Enable "Install from Unknown Sources" in Android settings
   - Install the APK file

2. **Required Permissions**
   The app will request the following permissions:
   - Location access (Fine location)
   - Bluetooth access
   - SMS sending capability
   - Internet access

3. **First-Time Setup**
   - Grant all requested permissions
   - Add emergency contacts
   - Connect to ESP32 device via Bluetooth
   - Test the connection using test mode

4. **Minimum Requirements**
   - Android 6.0 (API level 23) or higher
   - Bluetooth 4.0 capability
   - GPS enabled device
   - Internet connection

### Troubleshooting
- Ensure Bluetooth is enabled
- Check if location services are active
- Verify internet connectivity
- Make sure all permissions are granted
- Restart the app if connection issues persist
  
### Testing Mode
- Includes a "Fake Location" mode for testing
- Simulates different speed limits
- Tests crash detection alerts
- Verifies emergency contact system

## Installation & Setup
1. Clone this repository
2. Upload the ESP32 code to your ESP32 board
3. Upload the MPU code to your STM32 board
4. Install and run the Android application
5. Connect the hardware components according to the wiring diagram

## Usage
1. Power on the system
2. Connect to the ESP32 via Bluetooth from the mobile app
3. The system will automatically:
   - Fetch speed limits based on location
   - Control motor speed to stay within limits
   - Detect crashes and send alerts when necessary

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)

## Team
- Karthik S
- Shiva Kumar S
- Zabiulla
- Mounith H
- Rohith D

## Acknowledgments
- RV College of Engineering, Bangalore
- Dr. Somesh Nandi, RV College of Engineering, Bangalore
- Dr. Rohini S Halikar, RV College of Engineering, Bangalore
