#ifndef UTILS_HPP
#define UTILS_HPP

#include <Arduino.h>
#include <FastLED.h>

// Starter
int getDistance();
void ledOn(CRGB color);
void ledOff();
void setServoAngle(int angle, int rest);
void centerServo(int d);
bool setupGyro();
void calibrateGyro();
int16_t readGyroZ();
void updateGyroAngle();
void resetAngle();
float getAngle();

// Custom functions
struct SensorData {
  int avg_distance;
  int avg_angle;
};

// Setup functions
void setupArduino();

// helper functions
void print(String system, String msg, int d);
SensorData update_sensors();
void move_motors(int lspeed, int rspeed);

void update_array(int* array, int length, uint8_t i, int value);
int get_array_average(int* array, int length);
void reset_array(int* array, int length, int val);

CRGB get_roaming_colour(int d);
CRGB get_following_colour(int d);
void shake_head();
void freak_out();
int get_speed(int d);
void gyro_turn(int target_heading);

#endif