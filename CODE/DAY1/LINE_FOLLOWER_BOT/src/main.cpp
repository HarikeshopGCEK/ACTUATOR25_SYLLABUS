#include <Arduino.h>

// Define the pins for the IR sensors and motors
#define LEFT_IR 4
#define RIGHT_IR 5
#define LEFT_MOTOR_1 6
#define LEFT_MOTOR_2 7
#define RIGHT_MOTOR_1 8 
#define RIGHT_MOTOR_2 9
#define LMOTOR_PWM 10
#define RMOTOR_PWM 11

// Define the speed for the motors
#define LEFT_SPEED 100
#define RIGHT_SPEED 100

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
  Serial.begin(9600);
  // Initialize the pins for the IR sensors and motors
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
  pinMode(LEFT_MOTOR_1, OUTPUT);
  pinMode(LEFT_MOTOR_2, OUTPUT);
  pinMode(RIGHT_MOTOR_1, OUTPUT);
  pinMode(RIGHT_MOTOR_2, OUTPUT);
  pinMode(LMOTOR_PWM, OUTPUT);
  pinMode(RMOTOR_PWM, OUTPUT);\
  Serial.println("Pins initialized");
  Serial.println("Line follower bot initialized");
  delay(1000);
}

void loop() {
  int LDATA = digitalRead(LEFT_IR);
  int RDATA = digitalRead(RIGHT_IR);
  Serial.println("LDATA: " + String(LDATA));
  Serial.println("RDATA: " + String(RDATA));
  delay(100);
  if (LDATA == 0 && RDATA == 0) {
    moveForward();
    Serial.println("Forward");
  }
  else if (LDATA == 0 && RDATA == 1) {
    moveRight();
    Serial.println("Right");
  }
  else if (LDATA == 1 && RDATA == 0) {
    moveLeft();
    Serial.println("Right");
  }
  else if (LDATA == 1 && RDATA == 1) {
    stop();
  }
  delay(500);
  }