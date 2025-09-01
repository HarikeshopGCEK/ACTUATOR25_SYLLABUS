# ACTUATOR25 - ROBOTICS TEXTBOOK (PART 2)

**Advanced Robotics with ESP8266 and WiFi**

---

## CHAPTER 7: ESP8266 AND WIFI ROBOTICS

### 7.1 ESP8266 vs Arduino

**ESP8266 Advantages:**
- Built-in WiFi capability
- Higher clock speed (80MHz)
- More GPIO pins
- Lower cost
- Smaller size

**ESP8266 Specifications:**
- **Microcontroller**: Tensilica L106 32-bit
- **Operating Voltage**: 3.3V
- **Digital I/O Pins**: 11
- **Analog Input Pins**: 1
- **Flash Memory**: 4MB
- **SRAM**: 80KB
- **Clock Speed**: 80MHz

### 7.2 WiFi Capabilities

**Modes of Operation:**
1. **Station Mode**: Connect to existing WiFi network
2. **Access Point Mode**: Create WiFi network
3. **Station + AP Mode**: Both simultaneously

**Key Libraries:**
```cpp
#include <ESP8266WiFi.h>        // WiFi functionality
#include <ESP8266WebServer.h>   // Web server
#include <ESP8266HTTPClient.h>  // HTTP client
```

### 7.3 Pin Mapping

**NodeMCU Pin Mapping:**
```
D0  → GPIO16
D1  → GPIO5
D2  → GPIO4
D3  → GPIO0
D4  → GPIO2
D5  → GPIO14
D6  → GPIO12
D7  → GPIO13
D8  → GPIO15
```

**Important Notes:**
- D0, D3, D8 have special functions
- Use D1-D7 for general I/O
- Analog input only on A0

---

## CHAPTER 8: WEB CONTROLLED ROBOT

### 8.1 Concept and Working Principle

**Objective:** Control robot via web browser
**Method:** ESP8266 creates WiFi access point
**Interface:** HTML web page with control buttons

**Features:**
- WiFi access point creation
- Web server hosting
- Real-time motor control
- Mobile-friendly interface

### 8.2 Code Analysis

**WiFi Setup:**
```cpp
const char* ssid = "MY_BOT_AP";
const char* password = "12345678";

void setup() {
    // Create Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP()); // Usually 192.168.4.1
}
```

**Web Server Setup:**
```cpp
ESP8266WebServer server(80);

void setup() {
    // Define web routes
    server.on("/", handleRoot);
    server.on("/forward", moveForward);
    server.on("/backward", moveBackward);
    server.on("/left", moveLeft);
    server.on("/right", moveRight);
    server.on("/stop", stop);
    
    server.begin();
}
```

**HTML Interface:**
```cpp
void handleRoot() {
    String html = "<html><body style='text-align:center;'>";
    html += "<h1 style='color:white;'>Web Controlled Car</h1>";
    html += "<button onclick=\"fetch('/forward')\">Forward</button>";
    html += "<button onclick=\"fetch('/backward')\">Backward</button>";
    html += "<button onclick=\"fetch('/left')\">Left</button>";
    html += "<button onclick=\"fetch('/right')\">Right</button>";
    html += "<button onclick=\"fetch('/stop')\">Stop</button>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}
```

**Main Loop:**
```cpp
void loop() {
    server.handleClient(); // Handle web requests
}
```

### 8.3 Hardware Setup

**Components Required:**
- NodeMCU ESP8266
- L298N motor driver
- 2x DC motors with wheels
- Robot chassis
- Battery pack (3.3V-5V)

**Wiring Diagram:**
```
Motor Driver:
ENA → D1 (PWM)
IN1 → D6
IN2 → D7
IN3 → D8
IN4 → D0
ENB → D1 (PWM)
```

### 8.4 Usage Instructions

**Connection Steps:**
1. Upload code to NodeMCU
2. Connect to "MY_BOT_AP" WiFi network
3. Open browser and go to `http://192.168.4.1`
4. Use buttons to control robot

**Security Considerations:**
- Change default password
- Implement authentication if needed
- Consider encryption for sensitive applications

---

## CHAPTER 9: LIGHT FOLLOWER ROBOT

### 9.1 Concept and Working Principle

**Objective:** Follow light sources automatically
**Sensors:** Three light sensors (left, middle, right)
**Logic:** Move toward brightest light source

**Algorithm:**
```
Middle sensor > threshold → Move forward
Left sensor > right sensor → Turn left
Right sensor > left sensor → Turn right
All sensors < threshold → Stop
```

### 9.2 Code Analysis

**Sensor Setup:**
```cpp
#define LEFT_LIGHT_SENSOR D4
#define RIGHT_LIGHT_SENSOR D5
#define MIDDLE_LIGHT_SENSOR D6

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

**Light Following Algorithm:**
```cpp
void lightFollowingAlgorithm() {
    // Read light sensor values
    leftLightValue = analogRead(LEFT_LIGHT_SENSOR);
    rightLightValue = analogRead(RIGHT_LIGHT_SENSOR);
    middleLightValue = analogRead(MIDDLE_LIGHT_SENSOR);
    
    // Decision making
    if (middleLightValue > 500) {
        moveForward();  // Strong light ahead
    }
    else if (leftLightValue > rightLightValue && leftLightValue > 300) {
        turnLeft();     // More light on left
    }
    else if (rightLightValue > leftLightValue && rightLightValue > 300) {
        turnRight();    // More light on right
    }
    else {
        stop();         // No significant light
    }
}
```

**Web Monitoring Interface:**
```cpp
void handleRoot() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>Light Follower Bot Status</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;text-align:center;background:#f0f0f0;margin:20px;}";
    html += ".container{max-width:600px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}";
    html += "h1{color:#333;}";
    html += ".status{background:#e8f5e8;padding:15px;border-radius:5px;margin:20px 0;font-size:18px;font-weight:bold;}";
    html += ".sensor{background:#f8f8f8;padding:10px;margin:10px 0;border-radius:5px;}";
    html += ".auto{background:#fff3cd;padding:10px;margin:10px 0;border-radius:5px;}";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>🤖 Light Follower Bot</h1>";
    html += "<div class='status'>Status: " + botStatus + "</div>";
    html += "<div class='sensor'>Left Light Sensor: " + String(leftLightValue) + "</div>";
    html += "<div class='sensor'>Middle Light Sensor: " + String(middleLightValue) + "</div>";
    html += "<div class='sensor'>Right Light Sensor: " + String(rightLightValue) + "</div>";
    html += "<div class='auto'>Auto Mode: Active - Following Light</div>";
    html += "<p>Page refreshes every 2 seconds</p>";
    html += "</div>";
    html += "<script>setTimeout(function(){location.reload();}, 2000);</script>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}
```

### 9.3 Hardware Setup

**Components Required:**
- NodeMCU ESP8266
- 3x Light sensors (LDR)
- L298N motor driver
- 2x DC motors with wheels
- Robot chassis
- Battery pack (3.3V-5V)

**Light Sensor Circuit:**
```
LDR + 10kΩ Resistor Voltage Divider:
VCC → LDR → Analog Pin
     ↓
   10kΩ Resistor
     ↓
   GND
```

### 9.4 Calibration and Optimization

**Sensor Calibration:**
1. Test in different lighting conditions
2. Adjust threshold values
3. Consider ambient light compensation
4. Implement dynamic threshold adjustment

**Performance Optimization:**
- Filter sensor noise
- Implement hysteresis
- Add speed control based on light intensity
- Consider multiple light source scenarios

---

## CHAPTER 10: ADVANCED CONCEPTS AND FUTURE

### 10.1 PID Control

**Proportional-Integral-Derivative Control:**
- **Proportional**: Response proportional to error
- **Integral**: Accumulated error correction
- **Derivative**: Rate of change prediction

**Application in Robotics:**
- Line following with smooth movement
- Distance maintenance
- Speed control
- Position control

**PID Implementation Example:**
```cpp
float Kp = 2.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.5;  // Derivative gain

float error, lastError, integral;
float setpoint = 0;  // Target value

float calculatePID(float input) {
    error = setpoint - input;
    integral += error;
    float derivative = error - lastError;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    
    return output;
}
```

### 10.2 Sensor Fusion

**Combining Multiple Sensors:**
- **Redundancy**: Multiple sensors for same measurement
- **Complementary**: Different sensors for different aspects
- **Kalman Filter**: Optimal estimation from noisy data

**Examples:**
- IMU + GPS for navigation
- Multiple distance sensors for mapping
- Camera + ultrasonic for obstacle detection

**Simple Sensor Fusion Example:**
```cpp
float fuseSensors(float sensor1, float sensor2, float weight1, float weight2) {
    return (sensor1 * weight1 + sensor2 * weight2) / (weight1 + weight2);
}
```

### 10.3 Machine Learning in Robotics

**Applications:**
- **Computer Vision**: Object recognition
- **Path Planning**: Optimal route finding
- **Behavior Learning**: Adaptive responses
- **Predictive Maintenance**: Fault detection

**Implementation:**
- TensorFlow Lite for microcontrollers
- Edge AI processing
- Cloud-based learning

### 10.4 IoT and Cloud Robotics

**Cloud Integration:**
- **Data Logging**: Sensor data storage
- **Remote Monitoring**: Real-time status
- **Fleet Management**: Multiple robot coordination
- **Over-the-Air Updates**: Remote firmware updates

**Technologies:**
- MQTT protocol
- REST APIs
- WebSocket connections
- Cloud platforms (AWS, Google Cloud, Azure)

**MQTT Example:**
```cpp
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    client.setServer("mqtt.example.com", 1883);
    client.setCallback(callback);
}

void publishData() {
    String data = "{\"temperature\":" + String(temp) + ",\"humidity\":" + String(hum) + "}";
    client.publish("robot/sensors", data.c_str());
}
```

### 10.5 Advanced Algorithms

**Path Planning:**
- **A* Algorithm**: Optimal path finding
- **RRT (Rapidly-exploring Random Trees)**: Dynamic planning
- **SLAM (Simultaneous Localization and Mapping)**: Environment mapping

**Control Systems:**
- **Fuzzy Logic**: Human-like decision making
- **Neural Networks**: Pattern recognition
- **Genetic Algorithms**: Optimization

**Simple A* Implementation:**
```cpp
struct Node {
    int x, y;
    float g, h, f;
    Node* parent;
};

float heuristic(int x1, int y1, int x2, int y2) {
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}
```

### 10.6 Future Trends

**Emerging Technologies:**
1. **5G Robotics**: Low-latency remote control
2. **Edge Computing**: Local AI processing
3. **Swarm Robotics**: Multi-robot coordination
4. **Soft Robotics**: Flexible and adaptive robots
5. **Bio-inspired Robotics**: Nature-inspired designs

**Industry Applications:**
- **Agriculture**: Autonomous farming
- **Healthcare**: Medical assistance robots
- **Logistics**: Warehouse automation
- **Space Exploration**: Planetary rovers
- **Underwater**: Ocean exploration

### 10.7 Learning Resources

**Online Platforms:**
- Arduino Official Documentation
- ESP8266 Community Forums
- ROS (Robot Operating System) Tutorials
- Coursera Robotics Courses
- MIT OpenCourseWare

**Books:**
- "Arduino Cookbook" by Michael Margolis
- "Making Embedded Systems" by Elecia White
- "Probabilistic Robotics" by Sebastian Thrun
- "Introduction to Autonomous Mobile Robots" by Roland Siegwart

**Projects to Try:**
1. **Maze Solver**: Advanced path finding
2. **Gesture Control**: Hand gesture recognition
3. **Voice Control**: Speech recognition
4. **Autonomous Navigation**: GPS + compass
5. **Multi-Robot Communication**: Robot-to-robot interaction

---

## APPENDIX

### A.1 Common Error Codes and Solutions

**Arduino Errors:**
- **"Board not found"**: Check USB connection and drivers
- **"Upload failed"**: Verify board selection and port
- **"Compilation error"**: Check syntax and libraries

**Hardware Issues:**
- **Motors not moving**: Check power supply and connections
- **Sensors not responding**: Verify wiring and voltage
- **WiFi not connecting**: Check credentials and signal strength

### A.2 Pin Reference Tables

**Arduino Uno Pin Mapping:**
| Pin | Function | Notes |
|-----|----------|-------|
| 0-1 | Serial | TX/RX |
| 2-13 | Digital I/O | 3,5,6,9,10,11 are PWM |
| A0-A5 | Analog Input | 10-bit resolution |
| 5V | Power | 5V output |
| 3.3V | Power | 3.3V output |
| GND | Ground | Common ground |

**NodeMCU Pin Mapping:**
| Pin | Function | Notes |
|-----|----------|-------|
| D0 | GPIO16 | Boot mode |
| D1 | GPIO5 | I2C SCL |
| D2 | GPIO4 | I2C SDA |
| D3 | GPIO0 | Boot mode |
| D4 | GPIO2 | Built-in LED |
| D5 | GPIO14 | SPI SCK |
| D6 | GPIO12 | SPI MISO |
| D7 | GPIO13 | SPI MOSI |
| D8 | GPIO15 | SPI SS |
| A0 | Analog | 10-bit ADC |

### A.3 Component Specifications

**Common Sensors:**
| Sensor | Voltage | Current | Output | Range |
|--------|---------|---------|--------|-------|
| IR Sensor | 3.3-5V | 20mA | Digital/Analog | 2-30cm |
| Ultrasonic | 5V | 15mA | Digital | 2-400cm |
| LDR | 3.3-5V | 1mA | Analog | Variable |
| Temperature | 3.3-5V | 1mA | Digital/Analog | -40°C to +125°C |

**Motor Specifications:**
| Type | Voltage | Current | Speed | Torque |
|------|---------|---------|-------|--------|
| DC Motor | 6-12V | 100-500mA | 100-300 RPM | Variable |
| Servo Motor | 4.8-6V | 100-500mA | 60°/sec | 1-25 kg-cm |
| Stepper Motor | 5-12V | 200-800mA | Variable | High |

### A.4 Mathematical Formulas

**Distance Calculation (Ultrasonic):**
```
Distance = (Time × Speed of Sound) / 2
Speed of Sound = 340 m/s = 0.034 cm/μs
```

**PWM Duty Cycle:**
```
Duty Cycle = (Pulse Width / Period) × 100%
Average Voltage = (Duty Cycle / 100) × Supply Voltage
```

**Voltage Divider:**
```
Vout = Vin × (R2 / (R1 + R2))
```

**Ohm's Law:**
```
V = I × R
P = V × I
```

### A.5 WiFi Network Analysis

**Network Performance:**
- **Signal Strength**: -30dBm (excellent) to -90dBm (poor)
- **Channel Congestion**: Use WiFi analyzer apps
- **Interference**: Avoid 2.4GHz interference sources

**Security Best Practices:**
- Use WPA2/WPA3 encryption
- Change default passwords
- Implement MAC address filtering
- Regular firmware updates

---

## CONCLUSION

This textbook has covered the fundamental concepts of Arduino robotics, from basic electronics to advanced WiFi-controlled robots. The progression from simple line following to complex light following with web interfaces demonstrates the evolution of robotics technology.

**Key Takeaways:**
1. **Fundamentals First**: Understanding basic electronics and programming is crucial
2. **Hands-on Learning**: Building and testing robots is the best way to learn
3. **Iterative Development**: Start simple and add complexity gradually
4. **Problem Solving**: Robotics involves troubleshooting and optimization
5. **Continuous Learning**: Technology evolves rapidly; stay updated

**Next Steps:**
- Experiment with the provided code
- Modify and improve the robots
- Explore advanced concepts
- Join robotics communities
- Contribute to open-source projects

Remember: The best way to learn robotics is by doing. Start building, keep experimenting, and never stop learning!

---

**Happy Robotics! 🤖**

*This textbook is part of the ACTUATOR25 workshop organized by ROBOCEK, GCEK.*
