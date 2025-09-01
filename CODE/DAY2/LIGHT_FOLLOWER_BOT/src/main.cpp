#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Light sensor pins
#define LEFT_LIGHT_SENSOR D4
#define RIGHT_LIGHT_SENSOR D5
#define MIDDLE_LIGHT_SENSOR D6

// Motor pins
#define LEFT_MOTOR_1 D1
#define LEFT_MOTOR_2 D2
#define RIGHT_MOTOR_1 D8
#define RIGHT_MOTOR_2 D0
#define LMOTOR_PWM D3
#define RMOTOR_PWM D7

#define LEFT_SPEED 100
#define RIGHT_SPEED 100

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

ESP8266WebServer server(80);

// Global variables for status
String botStatus = "Stopped";
int leftLightValue = 0;
int rightLightValue = 0;
int middleLightValue = 0;

void moveForward()
{
    digitalWrite(LEFT_MOTOR_1, HIGH);
    digitalWrite(LEFT_MOTOR_2, LOW);
    digitalWrite(RIGHT_MOTOR_1, HIGH);
    digitalWrite(RIGHT_MOTOR_2, LOW);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
    botStatus = "Moving Forward";
}

void moveBackward()
{
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, HIGH);
    digitalWrite(RIGHT_MOTOR_1, LOW);
    digitalWrite(RIGHT_MOTOR_2, HIGH);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
    botStatus = "Moving Backward";
}

void turnLeft()
{
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, HIGH);
    digitalWrite(RIGHT_MOTOR_1, HIGH);
    digitalWrite(RIGHT_MOTOR_2, LOW);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
    botStatus = "Turning Left";
}

void turnRight()
{
    digitalWrite(LEFT_MOTOR_1, HIGH);
    digitalWrite(LEFT_MOTOR_2, LOW);
    digitalWrite(RIGHT_MOTOR_1, LOW);
    digitalWrite(RIGHT_MOTOR_2, HIGH);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
    botStatus = "Turning Right";
}

void stop()
{
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, LOW);
    digitalWrite(RIGHT_MOTOR_1, LOW);
    digitalWrite(RIGHT_MOTOR_2, LOW);
    analogWrite(LMOTOR_PWM, 0);
    analogWrite(RMOTOR_PWM, 0);
    botStatus = "Stopped";
}

void handleRoot()
{
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

void lightFollowingAlgorithm()
{
    // Read light sensor values
    leftLightValue = analogRead(LEFT_LIGHT_SENSOR);
    rightLightValue = analogRead(RIGHT_LIGHT_SENSOR);
    middleLightValue = analogRead(MIDDLE_LIGHT_SENSOR);
    
    Serial.print("Left: "); Serial.print(leftLightValue);
    Serial.print(" Middle: "); Serial.print(middleLightValue);
    Serial.print(" Right: "); Serial.println(rightLightValue);
    
    // Light following logic
    if (middleLightValue > 500) {
        // Strong light in middle - go forward
        moveForward();
    }
    else if (leftLightValue > rightLightValue && leftLightValue > 300) {
        // More light on left - turn left
        turnLeft();
    }
    else if (rightLightValue > leftLightValue && rightLightValue > 300) {
        // More light on right - turn right
        turnRight();
    }
    else {
        // No significant light - stop
        stop();
    }
}

void setup()
{
    Serial.begin(115200);
    
    // Setup motor pins
    pinMode(LEFT_MOTOR_1, OUTPUT);
    pinMode(LEFT_MOTOR_2, OUTPUT);
    pinMode(RIGHT_MOTOR_1, OUTPUT);
    pinMode(RIGHT_MOTOR_2, OUTPUT);
    pinMode(LMOTOR_PWM, OUTPUT);
    pinMode(RMOTOR_PWM, OUTPUT);
    
    // Setup light sensor pins
    pinMode(LEFT_LIGHT_SENSOR, INPUT);
    pinMode(RIGHT_LIGHT_SENSOR, INPUT);
    pinMode(MIDDLE_LIGHT_SENSOR, INPUT);
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Setup web server
    server.on("/", handleRoot);
    server.begin();
    Serial.println("Web server started");
    
    // Initial stop
    stop();
}

void loop()
{
    // Handle web server requests
    server.handleClient();
    
    // Run light following algorithm
    lightFollowingAlgorithm();
    
    delay(100); // Small delay for stability
}