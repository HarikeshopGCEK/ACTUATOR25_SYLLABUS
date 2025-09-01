#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define LEFT_MOTOR_1 D6
#define LEFT_MOTOR_2 D7
#define RIGHT_MOTOR_1 D8
#define RIGHT_MOTOR_2 D0
#define LMOTOR_PWM D1
#define RMOTOR_PWM D1

#define LEFT_SPEED 100
#define RIGHT_SPEED 100

ESP8266WebServer server(80);
const char* ssid = "MY_BOT_AP";
const char* password = "12345678";

void moveForward()
{
    digitalWrite(LEFT_MOTOR_1, HIGH);
    digitalWrite(LEFT_MOTOR_2, LOW);
    digitalWrite(RIGHT_MOTOR_1, HIGH);
    digitalWrite(RIGHT_MOTOR_2, LOW);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
}

void moveBackward()
{
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, HIGH);
    digitalWrite(RIGHT_MOTOR_1, LOW);
    digitalWrite(RIGHT_MOTOR_2, HIGH);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
}

void moveLeft()
{
    digitalWrite(LEFT_MOTOR_1, HIGH);
    digitalWrite(LEFT_MOTOR_2, LOW);
    digitalWrite(RIGHT_MOTOR_1, LOW);
    digitalWrite(RIGHT_MOTOR_2, HIGH);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
}

void moveRight()
{
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, HIGH);
    digitalWrite(RIGHT_MOTOR_1, HIGH);
    digitalWrite(RIGHT_MOTOR_2, LOW);
    analogWrite(LMOTOR_PWM, LEFT_SPEED);
    analogWrite(RMOTOR_PWM, RIGHT_SPEED);
}

void stop()
{
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, LOW);
    digitalWrite(RIGHT_MOTOR_1, LOW);
    digitalWrite(RIGHT_MOTOR_2, LOW);
    analogWrite(LMOTOR_PWM, 0);
    analogWrite(RMOTOR_PWM, 0);
}

void handleRoot()
{
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


void setup()
{
    Serial.begin(115200);
    
    // Create Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    
    Serial.println("Access Point Created!");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    
    pinMode(LEFT_MOTOR_1, OUTPUT);
    pinMode(LEFT_MOTOR_2, OUTPUT);
    pinMode(RIGHT_MOTOR_1, OUTPUT);
    pinMode(RIGHT_MOTOR_2, OUTPUT);
    pinMode(LMOTOR_PWM, OUTPUT);
    pinMode(RMOTOR_PWM, OUTPUT);

    server.on("/", handleRoot);
    server.on("/forward", moveForward);
    server.on("/backward", moveBackward);
    server.on("/left", moveLeft);
    server.on("/right", moveRight);
    server.on("/stop", stop);

    server.begin();
    Serial.println("Server started");
    Serial.println("Connect to MY_BOT_AP WiFi network and visit: http://192.168.4.1");
}
void loop()
{
    server.handleClient();
}
