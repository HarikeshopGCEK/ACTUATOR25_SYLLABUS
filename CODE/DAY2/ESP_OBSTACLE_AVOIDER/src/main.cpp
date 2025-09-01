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
  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}
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
void setup() {
  Serial.begin(115200);
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
  digitalWrite(LEFT_US_TRIG, LOW);
  digitalWrite(RIGHT_US_TRIG, LOW);
  delay(100);
}

void loop() {
  left_distance = measure_distance(LEFT_US_TRIG, LEFT_US_ECHO);
  right_distance = measure_distance(RIGHT_US_TRIG, RIGHT_US_ECHO);
  Serial.println("Left distance: " + String(left_distance));
  Serial.println("Right distance: " + String(right_distance));
  delay(100);
  if (left_distance > MAX_DISTANCE && right_distance > MAX_DISTANCE) {
    moveForward();
  }
  else if (left_distance < MAX_DISTANCE && right_distance > MAX_DISTANCE) {
    moveRight();
  }
  else if (left_distance > MAX_DISTANCE && right_distance < MAX_DISTANCE) {
    moveLeft();
  }
  else if (left_distance < MAX_DISTANCE && right_distance < MAX_DISTANCE) {
    stop();
  }
}