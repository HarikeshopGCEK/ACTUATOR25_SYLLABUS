# ACTUATOR25 - COMPLETE ROBOTICS TEXTBOOK

**A Comprehensive Guide to Arduino, ESP8266, and Autonomous Robotics**

---

## TABLE OF CONTENTS

1. [Chapter 1: Arduino Fundamentals](#chapter-1-arduino-fundamentals)
2. [Chapter 2: Electronics and Components](#chapter-2-electronics-and-components)
3. [Chapter 3: Motor Control and Movement](#chapter-3-motor-control-and-movement)
4. [Chapter 4: Sensors and Sensing](#chapter-4-sensors-and-sensing)
5. [Chapter 5: Line Follower Robot](#chapter-5-line-follower-robot)
6. [Chapter 6: Obstacle Avoider Robot](#chapter-6-obstacle-avoider-robot)
7. [Chapter 7: ESP8266 and WiFi Robotics](#chapter-7-esp8266-and-wifi-robotics)
8. [Chapter 8: Web Controlled Robot](#chapter-8-web-controlled-robot)
9. [Chapter 9: Light Follower Robot](#chapter-9-light-follower-robot)
10. [Chapter 10: Advanced Concepts and Future](#chapter-10-advanced-concepts-and-future)

---

## CHAPTER 1: ARDUINO FUNDAMENTALS

### 1.1 What is Arduino?

Arduino is an open-source electronics platform based on easy-to-use hardware and software. It consists of a programmable circuit board (microcontroller) and a development environment for writing software.

**Key Features:**
- **Microcontroller**: Atmel AVR chip (ATmega328P in Arduino Uno)
- **Operating Voltage**: 5V
- **Digital I/O Pins**: 14 (6 can be used as PWM outputs)
- **Analog Input Pins**: 6
- **Flash Memory**: 32KB (0.5KB used by bootloader)
- **SRAM**: 2KB
- **EEPROM**: 1KB
- **Clock Speed**: 16MHz

### 1.2 Arduino IDE and Programming

**Arduino IDE Structure:**
```cpp
void setup() {
    // Code that runs once at startup
    // Initialize pins, variables, libraries
}

void loop() {
    // Code that runs continuously
    // Main program logic
}
```

**Key Functions:**
- `pinMode(pin, mode)`: Sets pin as INPUT or OUTPUT
- `digitalWrite(pin, value)`: Writes HIGH or LOW to digital pin
- `digitalRead(pin)`: Reads HIGH or LOW from digital pin
- `analogWrite(pin, value)`: Writes analog value (0-255) to PWM pin
- `analogRead(pin)`: Reads analog value (0-1023) from analog pin
- `Serial.begin(baud)`: Initializes serial communication
- `Serial.print()`: Prints data to serial monitor

### 1.3 Digital vs Analog

**Digital Signals:**
- Only two states: HIGH (5V) or LOW (0V)
- Used for: switches, LEDs, digital sensors
- Functions: `digitalWrite()`, `digitalRead()`

**Analog Signals:**
- Continuous range of values
- PWM (Pulse Width Modulation): Simulates analog output
- Analog input: 0-1023 range (10-bit resolution)
- Functions: `analogWrite()`, `analogRead()`

### 1.4 Basic Circuit Concepts

**Voltage (V):** Electrical pressure
**Current (A):** Flow of electrons
**Resistance (Ω):** Opposition to current flow
**Power (W):** V × A

**Ohm's Law:** V = I × R

---

## CHAPTER 2: ELECTRONICS AND COMPONENTS

### 2.1 Essential Components

**Arduino Uno:**
- Main microcontroller board
- USB connection for programming
- Power regulation and protection

**Breadboard:**
- Prototyping platform
- Connected rows and columns
- No soldering required

**Jumper Wires:**
- Male-to-male, male-to-female, female-to-female
- Connect components to Arduino

**Resistors:**
- Limit current flow
- Color-coded values
- Common values: 220Ω, 1kΩ, 10kΩ

### 2.2 Motor Driver (L298N/L293D)

**Purpose:** Control DC motors with Arduino
**Why needed:** Arduino can't provide enough current for motors

**Pin Functions:**
- **ENA/ENB**: Enable pins (PWM for speed control)
- **IN1/IN2**: Control left motor direction
- **IN3/IN4**: Control right motor direction
- **VCC**: Logic power (5V)
- **VM**: Motor power (6-12V)
- **GND**: Ground connection

**Motor Control Logic:**
```
Left Motor:
IN1=HIGH, IN2=LOW  → Forward
IN1=LOW,  IN2=HIGH → Backward
IN1=LOW,  IN2=LOW  → Stop

Right Motor:
IN3=HIGH, IN4=LOW  → Forward
IN3=LOW,  IN4=HIGH → Backward
IN3=LOW,  IN4=LOW  → Stop
```

### 2.3 DC Motors

**Working Principle:**
- Electromagnetic induction
- Commutator switches current direction
- Brushes maintain electrical contact

**Specifications:**
- **Voltage**: 6-12V typical
- **Current**: 100-500mA per motor
- **Speed**: 100-300 RPM
- **Torque**: Varies by motor type

**Speed Control:**
- PWM (Pulse Width Modulation)
- Duty cycle determines average voltage
- Higher duty cycle = faster speed

---

## CHAPTER 3: MOTOR CONTROL AND MOVEMENT

### 3.1 Basic Movement Functions

**Forward Movement:**
```cpp
void moveForward() {
    digitalWrite(LEFT_MOTOR_1, HIGH);
    digitalWrite(LEFT_MOTOR_2, LOW);
    digitalWrite(RIGHT_MOTOR_1, HIGH);
    digitalWrite(RIGHT_MOTOR_2, LOW);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
}
```

**Backward Movement:**
```cpp
void moveBackward() {
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, HIGH);
    digitalWrite(RIGHT_MOTOR_1, LOW);
    digitalWrite(RIGHT_MOTOR_2, HIGH);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
}
```

**Turning Logic:**
- **Left Turn**: Left motor backward, right motor forward
- **Right Turn**: Left motor forward, right motor backward
- **Stop**: Both motors off

### 3.2 Differential Drive

**Principle:** Two independently controlled wheels
**Advantages:**
- Simple construction
- Good maneuverability
- Can turn in place

**Movement Types:**
1. **Forward/Backward**: Both wheels same direction
2. **Turning**: Wheels opposite directions
3. **Curved Path**: Different wheel speeds

---

## CHAPTER 4: SENSORS AND SENSING

### 4.1 IR Sensors (Infrared)

**Working Principle:**
- IR LED emits infrared light
- Photodiode detects reflected light
- Black surfaces absorb IR, white surfaces reflect
- Output: Digital (HIGH/LOW) or Analog

**Line Detection:**
```
Black Line: IR absorbed → LOW output
White Surface: IR reflected → HIGH output
```

**Calibration:**
- Test on black and white surfaces
- Find threshold value
- Adjust sensitivity if needed

### 4.2 Ultrasonic Sensors (HC-SR04)

**Working Principle:**
- Trig pin sends ultrasonic pulse
- Echo pin receives reflected pulse
- Time difference = distance calculation
- Formula: Distance = (Time × Speed of Sound) / 2

**Pin Functions:**
- **VCC**: 5V power
- **Trig**: Trigger pin (send pulse)
- **Echo**: Echo pin (receive pulse)
- **GND**: Ground

**Distance Calculation:**
```cpp
float measure_distance(int trig, int echo) {
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH);
    float distance = duration * 0.034 / 2;
    return distance;
}
```

### 4.3 Light Sensors (LDR - Light Dependent Resistor)

**Working Principle:**
- Resistance changes with light intensity
- More light = lower resistance
- Voltage divider circuit
- Analog output (0-1023)

**Applications:**
- Light following robots
- Automatic lighting systems
- Solar tracking

---

## CHAPTER 5: LINE FOLLOWER ROBOT

### 5.1 Concept and Working Principle

**Objective:** Follow a black line on white surface
**Sensors:** Two IR sensors (left and right)
**Logic:** Keep line between sensors

**Algorithm:**
```
Both sensors on white → Move forward
Left sensor on black → Turn left
Right sensor on black → Turn right
Both sensors on black → Stop (end of line)
```

### 5.2 Code Analysis

**Pin Definitions:**
```cpp
#define LEFT_IR 4      // Left IR sensor
#define RIGHT_IR 5     // Right IR sensor
#define LEFT_MOTOR_1 6 // Left motor control 1
#define LEFT_MOTOR_2 7 // Left motor control 2
#define RIGHT_MOTOR_1 8 // Right motor control 1
#define RIGHT_MOTOR_2 9 // Right motor control 2
#define LMOTOR_PWM 10  // Left motor speed control
#define RMOTOR_PWM 11  // Right motor speed control
```

**Setup Function:**
```cpp
void setup() {
    Serial.begin(9600);  // Initialize serial communication
    
    // Configure IR sensors as inputs
    pinMode(LEFT_IR, INPUT);
    pinMode(RIGHT_IR, INPUT);
    
    // Configure motor pins as outputs
    pinMode(LEFT_MOTOR_1, OUTPUT);
    pinMode(LEFT_MOTOR_2, OUTPUT);
    pinMode(RIGHT_MOTOR_1, OUTPUT);
    pinMode(RIGHT_MOTOR_2, OUTPUT);
    pinMode(LMOTOR_PWM, OUTPUT);
    pinMode(RMOTOR_PWM, OUTPUT);
}
```

**Main Loop Logic:**
```cpp
void loop() {
    // Read sensor values
    int LDATA = digitalRead(LEFT_IR);
    int RDATA = digitalRead(RIGHT_IR);
    
    // Decision making
    if (LDATA == 0 && RDATA == 0) {
        moveForward();  // Both sensors on white
    }
    else if (LDATA == 0 && RDATA == 1) {
        moveLeft();     // Right sensor on black
    }
    else if (LDATA == 1 && RDATA == 0) {
        moveRight();    // Left sensor on black
    }
    else if (LDATA == 1 && RDATA == 1) {
        stop();         // Both sensors on black
    }
}
```

### 5.3 Hardware Setup

**Components Required:**
- Arduino Uno
- 2x IR sensors
- L298N motor driver
- 2x DC motors with wheels
- Robot chassis
- Battery pack (6-12V)

**Wiring Diagram:**
```
IR Sensors:
Left IR → Pin 4
Right IR → Pin 5

Motor Driver:
ENA → Pin 10 (PWM)
IN1 → Pin 6
IN2 → Pin 7
IN3 → Pin 8
IN4 → Pin 9
ENB → Pin 11 (PWM)
```

### 5.4 Troubleshooting

**Common Issues:**
1. **Robot not following line**: Check sensor calibration
2. **Erratic movement**: Adjust motor speeds
3. **Sensors not responding**: Check wiring and power
4. **Line detection issues**: Clean sensors, adjust height

---

## CHAPTER 6: OBSTACLE AVOIDER ROBOT

### 6.1 Concept and Working Principle

**Objective:** Navigate while avoiding obstacles
**Sensors:** Two ultrasonic sensors (left and right)
**Logic:** Measure distances and turn away from obstacles

**Algorithm:**
```
Both distances > threshold → Move forward
Left distance < threshold → Turn right
Right distance < threshold → Turn left
Both distances < threshold → Stop
```

### 6.2 Code Analysis

**Pin Definitions:**
```cpp
#define LEFT_US_TRIG 4   // Left ultrasonic trigger
#define LEFT_US_ECHO 5   // Left ultrasonic echo
#define RIGHT_US_TRIG 6  // Right ultrasonic trigger
#define RIGHT_US_ECHO 7  // Right ultrasonic echo
#define LEFT_MOTOR_1 8   // Left motor control 1
#define LEFT_MOTOR_2 9   // Left motor control 2
#define RIGHT_MOTOR_1 10 // Right motor control 1
#define RIGHT_MOTOR_2 11 // Right motor control 2
#define LMOTOR_PWM 12    // Left motor speed
#define RMOTOR_PWM 3     // Right motor speed
```

**Distance Measurement Function:**
```cpp
float measure_distance(int trig, int echo) {
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH);
    float distance = duration * 0.034 / 2;
    return distance;
}
```

**Main Loop Logic:**
```cpp
void loop() {
    // Measure distances
    left_distance = measure_distance(LEFT_US_TRIG, LEFT_US_ECHO);
    right_distance = measure_distance(RIGHT_US_TRIG, RIGHT_US_ECHO);
    
    // Decision making
    if (left_distance > MAX_DISTANCE && right_distance > MAX_DISTANCE) {
        moveForward();  // Clear path ahead
    }
    else if (left_distance < MAX_DISTANCE && right_distance > MAX_DISTANCE) {
        moveRight();    // Obstacle on left
    }
    else if (left_distance > MAX_DISTANCE && right_distance < MAX_DISTANCE) {
        moveLeft();     // Obstacle on right
    }
    else if (left_distance < MAX_DISTANCE && right_distance < MAX_DISTANCE) {
        stop();         // Obstacles on both sides
    }
}
```

### 6.3 Hardware Setup

**Components Required:**
- Arduino Uno
- 2x Ultrasonic sensors (HC-SR04)
- L298N motor driver
- 2x DC motors with wheels
- Robot chassis
- Battery pack (6-12V)

**Wiring Diagram:**
```
Left Ultrasonic:
Trig → Pin 4
Echo → Pin 5

Right Ultrasonic:
Trig → Pin 6
Echo → Pin 7

Motor Driver:
ENA → Pin 12 (PWM)
IN1 → Pin 8
IN2 → Pin 9
IN3 → Pin 10
IN4 → Pin 11
ENB → Pin 3 (PWM)
```

### 6.4 Optimization Techniques

**Distance Threshold:**
- Adjust `MAX_DISTANCE` based on robot speed
- Typical values: 15-25 cm
- Consider sensor mounting height

**Turning Strategy:**
- Sharp turns for immediate obstacles
- Gradual turns for distant obstacles
- Implement different turn speeds

---

*This is Part 1 of the ACTUATOR25 Robotics Textbook. Continue to Part 2 for ESP8266 and advanced robotics concepts.*
