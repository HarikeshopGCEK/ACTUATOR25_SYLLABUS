#include <Arduino.h>

// Define the pins for the IR sensors and motors
#define LEFT_US_TRIG 4
#define LEFT_US_ECHO 5
#define RIGHT_US_TRIG 6
#define RIGHT_US_ECHO 7

// Define the pins for the motors
#define LEFT_MOTOR_1 8
#define LEFT_MOTOR_2 9
#define RIGHT_MOTOR_1 10
#define RIGHT_MOTOR_2 11
#define LMOTOR_PWM 12
#define RMOTOR_PWM 3
// Define the speed for the motors
#define LEFT_SPEED 100
#define RIGHT_SPEED 100 

#define MAX_DISTANCE 20
float left_distance, right_distance;
 float measure_distance(int trig, int echo)
 {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  temp = pulseIn(echo, HIGH);
  distance = temp / 58.0;
  return distance;
 }
void setup() {
  Serial.begin(9600);
  pinMode(LEFT_US_TRIG, OUTPUT);
  pinMode(LEFT_US_ECHO, INPUT);
  pinMode(RIGHT_US_TRIG, OUTPUT);
  pinMode(RIGHT_US_ECHO, INPUT);
  pinMode(LEFT_MOTOR_1, OUTPUT);
  pinMode(LEFT_MOTOR_2, OUTPUT);
  pinMode(RIGHT_MOTOR_1, OUTPUT);
  pinMode(RIGHT_MOTOR_2, OUTPUT);
  pinMode(LMOTOR_PWM, OUTPUT);
  pinMode(RMOTOR_PWM, OUTPUT);
  Serial.println("Obstacle avoider bot initialized");
  delay(1000);
}

void loop() {
  Serial.println("Hello, World!");
}