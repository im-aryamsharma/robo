#ifndef SETUP_HPP
#define SETUP_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

extern Servo scanServo;

// Gyro state variables
extern float currentAngle;
extern float gyroZOffset;
extern float gyroZ;
extern unsigned long lastTime;

// Servos
void setServoAngle(int angle, int rest = -1);
void centerServo(int d);

// Gyro sensor
bool setupGyro();
void calibrateGyro();
int16_t readGyroZ();
void updateGyroAngle();
void resetAngle();
float getAngle();

// Ultrasonic sensors
int getDistance();

#endif