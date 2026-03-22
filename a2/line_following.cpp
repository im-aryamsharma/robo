#include "line_following.hpp"
#include "utils.hpp"
#include "pins.hpp"
#include "config.hpp"

extern SensorData sensor_data;


void follow_line()
{
  int left = analogRead(LINE_L);
  int center = analogRead(LINE_C);
  int right = analogRead(LINE_R);

  bool on_center = center < LINE_THRESHOLD;
  bool on_left = left < LINE_THRESHOLD;
  bool on_right = right < LINE_THRESHOLD;

  move_motors(0, 0);

  if (on_center && on_left && on_right) {
    move_motors(NORMAL_SPEED, NORMAL_SPEED);
    print("motor", "path", 0);
    // move_motors(SPEED_NORMAL, SPEED_NORMAL);
  }

  else if (on_left && !on_right) {
    move_motors(NORMAL_SPEED, -NORMAL_SPEED);
    print("motor", "path", -1);
    // move_motors(-SPEED_TURN, SPEED_TURN);
  }

  else if (on_right && !on_left) {
    move_motors(-NORMAL_SPEED, NORMAL_SPEED);
    print("motor", "path", 1);
    // move_motors(-SPEED_TURN, SPEED_TURN);
  }

  if (!on_center && !on_left && !on_right) {
    state = 0;
  }
}