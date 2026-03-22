#include "searching.hpp"
#include "utils.hpp"
#include "config.hpp"

extern SensorData sensor_data;

void searching()
{
  ledOn(CRGB(48, 25, 52));

  setServoAngle(0, 1000);
  setServoAngle(180, 1000);

  move_motors(-200, 200);
  delay(300);
  move_motors(0, 0);

  setServoAngle(0, 1000);
  setServoAngle(180, 1000);

  move_motors(200, -200);
  delay(600);
  move_motors(0, 0);

  setServoAngle(0, 1000);
  setServoAngle(180, 1000);

  centerServo(2000);

  freak_out();

  state = 0;
}