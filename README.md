# ACTUATOR25 REVISED SYLLABUS

## Project Overview
This repository contains Arduino and ESP8266 robotics projects organized by day. Each project includes complete source code and configuration files for building various types of robots.

## Project Structure

### DAY 1 - Basic Arduino Robotics
- **LINE_FOLLOWER_BOT** - Arduino Uno line following robot
- **OBSTACLE_AVOIDER_BOT** - Arduino Uno obstacle avoidance robot

### DAY 2 - Advanced Robotics with ESP8266
- **ESP_OBSTACLE_AVOIDER** - ESP8266 NodeMCU obstacle avoidance robot
- **LIGHT_FOLLOWER_BOT** - Arduino Uno light following robot
- **WEB_CONTROLLED_BOT** - ESP8266 NodeMCU web-controlled robot

## How to View and Program the Code

### Method 1: Using Arduino IDE (Recommended for Beginners)

#### Step 1: Locate the Project Code
1. Navigate to the desired project folder:
   - For Arduino Uno projects: `CODE/DAY1/` or `CODE/DAY2/LIGHT_FOLLOWER_BOT/`
   - For ESP8266 projects: `CODE/DAY2/ESP_OBSTACLE_AVOIDER/` or `CODE/DAY2/WEB_CONTROLLED_BOT/`

2. Open the `src/main.cpp` file in any text editor to view the code

#### Step 2: Copy Code to Arduino IDE
1. Open Arduino IDE
2. Create a new sketch (File → New)
3. Copy the entire contents of `src/main.cpp` from your chosen project
4. Paste it into the Arduino IDE sketch window

#### Step 3: Configure Board Settings
**For Arduino Uno Projects:**
- Tools → Board → Arduino AVR Boards → Arduino Uno
- Tools → Port → Select your Arduino port (e.g., COM3, COM4, etc.)

**For ESP8266 Projects:**
- Tools → Board → ESP8266 Boards → NodeMCU 1.0 (ESP-12E Module)
- Tools → Port → Select your ESP8266 port
- Tools → Upload Speed → 115200

#### Step 4: Upload the Code
1. Click the "Upload" button (→) in Arduino IDE
2. Wait for the upload to complete
3. Open Serial Monitor (Tools → Serial Monitor) to see debug output

### Method 2: Using PlatformIO (Advanced Users)

#### Step 1: Install PlatformIO
1. Install PlatformIO IDE extension in VS Code
2. Or install PlatformIO Core via command line

#### Step 2: Open Project
1. Open the specific project folder (e.g., `CODE/DAY1/LINE_FOLLOWER_BOT/`)
2. PlatformIO will automatically detect the `platformio.ini` configuration

#### Step 3: Build and Upload
1. Click the PlatformIO icon in VS Code
2. Click "Build" to compile the project
3. Click "Upload" to program the device
4. Click "Monitor" to view serial output

## Hardware Requirements

### Arduino Uno Projects
- Arduino Uno board
- Motor driver module (L298N or similar)
- DC motors (2x)
- IR sensors (2x for line follower, 1x for obstacle avoider)
- Wheels and chassis
- Battery pack (6-12V)

### ESP8266 Projects
- NodeMCU ESP8266 board
- Motor driver module
- DC motors (2x)
- Ultrasonic sensor (for obstacle avoidance)
- Wheels and chassis
- Battery pack (6-12V)

## Pin Connections

### Line Follower Bot (Arduino Uno)
- Left IR Sensor: Pin 4
- Right IR Sensor: Pin 5
- Left Motor 1: Pin 6
- Left Motor 2: Pin 7
- Right Motor 1: Pin 8
- Right Motor 2: Pin 9
- Left Motor PWM: Pin 10
- Right Motor PWM: Pin 11

### Obstacle Avoider Bot (Arduino Uno)
- Ultrasonic Trig: Pin 12
- Ultrasonic Echo: Pin 13
- Left Motor 1: Pin 6
- Left Motor 2: Pin 7
- Right Motor 1: Pin 8
- Right Motor 2: Pin 9
- Left Motor PWM: Pin 10
- Right Motor PWM: Pin 11

### ESP8266 Projects
Check the individual `src/main.cpp` files for specific pin assignments as they may vary.

## Troubleshooting

### Common Issues:
1. **Port not found**: Make sure the board is properly connected via USB
2. **Upload failed**: Check if the correct board is selected
3. **Serial monitor not working**: Verify the correct baud rate (9600 for Arduino, 115200 for ESP8266)
4. **Motors not moving**: Check motor driver connections and power supply

### For ESP8266 Projects:
- Make sure you have the ESP8266 board package installed in Arduino IDE
- Board URL: `http://arduino.esp8266.com/stable/package_esp8266com_index.json`

## Additional Resources
- Arduino IDE: https://www.arduino.cc/en/software
- PlatformIO: https://platformio.org/
- ESP8266 Documentation: https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/

## Support
For technical support or questions about the projects, please refer to the individual project folders and their respective documentation.
