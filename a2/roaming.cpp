#include "roaming.hpp"
#include "utils.hpp"
#include "pins.hpp"
#include "config.hpp"

extern SensorData sensor_data;

void roaming()
{
  int center = analogRead(LINE_C);
  bool on_center = center < LINE_THRESHOLD;

  if (on_center)
  {
    state = 3;
    return;
  }

  ledOn(get_roaming_colour(sensor_data.avg_angle));

  move_motors(NORMAL_SPEED - sensor_data.avg_angle, NORMAL_SPEED + sensor_data.avg_angle);

  if (sensor_data.avg_angle < MIN_DISTANCE) {
    state = 1;
    move_motors(0, 0);
    return;
  }

  if (sensor_data.avg_angle < MIN_DISTANCE + 5) {
    move_motors(-NORMAL_SPEED, -NORMAL_SPEED);
    shake_head();
    move_motors(-200, 200);
    delay(random(500, 2500));
  }
}