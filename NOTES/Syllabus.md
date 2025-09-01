# ACTUATOR25 - BEGINNER ROBOTICS WORKSHOP SYLLABUS

**Organized by ROBOCEK, GCEK**

## Workshop Overview
The workshop spans two days, with progressive sessions that start from Arduino basics and extend to advanced robotics using NodeMCU (ESP8266). Participants will build and program various autonomous robots while learning fundamental robotics concepts.

---

## DAY 1 – ARDUINO ROBOTICS FOUNDATIONS
**Theme**: Learn Arduino basics and build your first autonomous robots.

### Session 1: Introduction to Arduino (1.5 hours)
**Learning Objectives:**
- Understand the basics of Arduino and microcontrollers
- Learn Arduino IDE overview: writing, uploading, and testing code
- Introduction to hardware components: pins, sensors, and motor drivers

**Topics Covered:**
- What is Arduino and how it works
- Arduino IDE installation and setup
- Basic programming concepts (setup, loop, digital I/O)
- Hardware components introduction:
  - Arduino Uno board
  - Motor driver (L293D/L298N)
  - IR sensors
  - Ultrasonic sensors
  - DC motors and wheels

**Hands-on Activity:**
- First Arduino program: LED Blink
- Basic pin configuration and testing

### Session 2: Hardware + Programming Together (2 hours)
**Learning Objectives:**
- Master digital I/O operations with LEDs and buzzers
- Learn motor driver control with Arduino
- Understand IR sensor basics for line detection
- Learn ultrasonic sensor basics for distance measurement

**Topics Covered:**
- Digital I/O examples with LED and buzzer
- Motor driver (L293D/L298N) control with Arduino
- IR sensor basics and line detection principles
- Ultrasonic sensor basics and distance measurement
- PWM (Pulse Width Modulation) for motor speed control

**Hands-on Activities:**
- LED blinking and buzzer control
- Motor control with different speeds
- IR sensor testing on black/white surfaces
- Ultrasonic sensor distance measurement

### Session 3: Line Follower Bot (2.5 hours)
**Learning Objectives:**
- Understand the concept of line detection using IR sensors
- Learn programming logic for line following
- Build and test a complete line following robot

**Topics Covered:**
- Line following algorithm and logic
- IR sensor calibration and threshold setting
- Motor control for precise movement
- Error handling and debugging

**Project: LINE_FOLLOWER_BOT**
- **Location**: `CODE/DAY1/LINE_FOLLOWER_BOT/src/main.cpp`
- **Hardware**: Arduino Uno, 2x IR sensors, 2x DC motors, motor driver
- **Pin Connections**:
  - Left IR Sensor: Pin 4
  - Right IR Sensor: Pin 5
  - Left Motor 1: Pin 6, Left Motor 2: Pin 7
  - Right Motor 1: Pin 8, Right Motor 2: Pin 9
  - Left Motor PWM: Pin 10, Right Motor PWM: Pin 11

**Hands-on Activity:**
- Assemble the line follower robot
- Upload and test the code
- Calibrate sensors for optimal performance
- Test on different line patterns

### Session 4: Obstacle Avoider Bot (2.5 hours)
**Learning Objectives:**
- Use ultrasonic sensor for obstacle detection
- Implement logic: detect object → stop/turn
- Build and test an autonomous obstacle avoidance robot

**Topics Covered:**
- Ultrasonic sensor working principle
- Distance calculation and threshold setting
- Obstacle avoidance algorithms
- Motor control for turning and stopping

**Project: OBSTACLE_AVOIDER_BOT**
- **Location**: `CODE/DAY1/OBSTACLE_AVOIDER_BOT/src/main.cpp`
- **Hardware**: Arduino Uno, ultrasonic sensor, 2x DC motors, motor driver
- **Pin Connections**:
  - Ultrasonic Trig: Pin 12, Echo: Pin 13
  - Left Motor 1: Pin 6, Left Motor 2: Pin 7
  - Right Motor 1: Pin 8, Right Motor 2: Pin 9
  - Left Motor PWM: Pin 10, Right Motor PWM: Pin 11

**Hands-on Activity:**
- Assemble the obstacle avoider robot
- Upload and test the code
- Mini challenge: tune detection distance or speed
- Test in obstacle course

---

## DAY 2 – NODEMCU ROBOTICS ADVANCED
**Theme**: Transition from Arduino to NodeMCU and build smarter robots.

### Session 1: Introduction to NodeMCU (1.5 hours)
**Learning Objectives:**
- Understand NodeMCU (ESP8266) vs Arduino differences and advantages
- Set up NodeMCU in Arduino IDE
- Write and upload first NodeMCU program

**Topics Covered:**
- ESP8266 architecture and capabilities
- WiFi functionality introduction
- NodeMCU pin mapping vs Arduino
- Setting up ESP8266 board package in Arduino IDE
- First program: LED Blink on NodeMCU

**Hands-on Activity:**
- Install ESP8266 board package
- Configure Arduino IDE for NodeMCU
- Upload LED blink program
- Test basic functionality

### Session 2: Obstacle Avoider Bot with NodeMCU (2 hours)
**Learning Objectives:**
- Recreate Arduino obstacle avoider using NodeMCU
- Focus on pin mapping and adapting Arduino logic
- Understand ESP8266-specific programming considerations

**Topics Covered:**
- ESP8266 pin mapping and GPIO functions
- Adapting Arduino code for NodeMCU
- Serial communication and debugging
- Power management considerations

**Project: ESP_OBSTACLE_AVOIDER**
- **Location**: `CODE/DAY2/ESP_OBSTACLE_AVOIDER/src/main.cpp`
- **Hardware**: NodeMCU ESP8266, ultrasonic sensor, 2x DC motors, motor driver
- **Key Differences**: Uses ESP8266 GPIO pins (D1, D2, etc.)

**Hands-on Activity:**
- Group activity: build and test ESP8266 obstacle avoider
- Compare performance with Arduino version
- Debug and optimize the code

### Session 3: Web Controlled Bot (2.5 hours)
**Learning Objectives:**
- Understand browser-based robot control concepts
- Host a simple web server on NodeMCU
- Create web interface to control motor driver

**Topics Covered:**
- ESP8266 WiFi capabilities
- Web server hosting on microcontroller
- HTML/CSS for web interface
- HTTP requests and responses
- Motor control via web interface

**Project: WEB_CONTROLLED_BOT**
- **Location**: `CODE/DAY2/WEB_CONTROLLED_BOT/src/main.cpp`
- **Hardware**: NodeMCU ESP8266, 2x DC motors, motor driver
- **Features**:
  - Creates WiFi access point
  - Web interface with control buttons
  - Real-time motor control via browser

**Hands-on Activity:**
- Upload web controlled bot code
- Connect to robot's WiFi network
- Control robot using smartphone/computer browser
- Test all movement directions

### Session 4: Light Follower Bot (2.5 hours)
**Learning Objectives:**
- Build a robot that follows light sources
- Use multiple light sensors for direction detection
- Implement advanced sensor fusion algorithms

**Topics Covered:**
- Light sensor principles and calibration
- Multi-sensor data fusion
- Advanced motor control algorithms
- Web-based monitoring and control

**Project: LIGHT_FOLLOWER_BOT**
- **Location**: `CODE/DAY2/LIGHT_FOLLOWER_BOT/src/main.cpp`
- **Hardware**: NodeMCU ESP8266, 3x light sensors, 2x DC motors, motor driver
- **Features**:
  - Three light sensors for precise direction detection
  - WiFi connectivity for remote monitoring
  - Web interface showing sensor values and bot status
  - Automatic light following with manual override

**Hands-on Activity:**
- Assemble light follower robot
- Calibrate light sensors
- Test automatic light following behavior
- Use web interface for monitoring and manual control
- Demonstration with flashlight as "light target"

---

## WRAP-UP & FUTURE SCOPE (1 hour)

### Session 5: Recap and Advanced Concepts
**Learning Objectives:**
- Recap Arduino vs NodeMCU robotics differences
- Explore advanced robotics concepts
- Encourage continued learning and projects

**Topics Covered:**
- **Recap**: Arduino vs NodeMCU robotics capabilities
- **Extensions and Advanced Projects**:
  - Gesture-controlled bot using accelerometers
  - Maze solver with advanced algorithms
  - Bluetooth-controlled robots
  - IoT integration and cloud connectivity
  - Computer vision integration
  - Machine learning for robotics

**Future Learning Paths:**
- Advanced sensor integration (IMU, GPS, cameras)
- ROS (Robot Operating System) introduction
- AI and machine learning in robotics
- IoT and cloud robotics
- Advanced control algorithms (PID, path planning)

**ROBOCEK Lab Opportunities:**
- Continued project development
- Advanced robotics workshops
- Competition preparation
- Research projects
- Mentorship programs

---

## TECHNICAL REQUIREMENTS

### Software Requirements
- Arduino IDE 1.8.x or later
- ESP8266 board package for Arduino IDE
- PlatformIO (optional, for advanced users)

### Hardware Requirements per Participant
**Day 1 (Arduino Projects):**
- Arduino Uno board
- L298N or L293D motor driver module
- 2x DC motors with wheels
- 2x IR sensors (for line follower)
- 1x Ultrasonic sensor (for obstacle avoider)
- Robot chassis and battery pack
- USB cable and jumper wires

**Day 2 (NodeMCU Projects):**
- NodeMCU ESP8266 board
- L298N or L293D motor driver module
- 2x DC motors with wheels
- 1x Ultrasonic sensor (for ESP obstacle avoider)
- 3x Light sensors (for light follower)
- Robot chassis and battery pack
- USB cable and jumper wires

### Workshop Setup
- Computer lab with Arduino IDE installed
- WiFi network for NodeMCU projects
- Testing areas for robot demonstrations
- Basic tools (screwdrivers, wire strippers)
- Spare components for troubleshooting

---

## ASSESSMENT & EVALUATION

### Day 1 Assessment
- **Practical**: Successful line follower robot operation
- **Practical**: Successful obstacle avoider robot operation
- **Theory**: Understanding of Arduino programming concepts

### Day 2 Assessment
- **Practical**: Web-controlled bot functionality
- **Practical**: Light follower bot performance
- **Theory**: Understanding of ESP8266 and WiFi concepts

### Final Project
- Participants can choose to enhance any of the built robots
- Optional: Integration of multiple sensors and features
- Presentation of final project to the group

---

## SUPPORT & RESOURCES

### Documentation
- Complete code available in `CODE/` directory
- Detailed README.md with programming instructions
- Pin connection diagrams for each project

### Troubleshooting Guide
- Common issues and solutions
- Hardware troubleshooting tips
- Software debugging techniques

### Additional Resources
- Arduino official documentation
- ESP8266 community resources
- Robotics tutorials and guides
- ROBOCEK lab access for continued learning

---

**Workshop Duration**: 2 Days (8-10 hours per day)  
**Target Audience**: Beginners with basic programming knowledge  
**Prerequisites**: Basic understanding of electronics and programming concepts  
**Certificate**: Provided upon successful completion of all projects
