#include "sticky_following.hpp"
#include "utils.hpp"
#include "config.hpp"

extern int angles[GYRO_LOOKBACK];
extern int distances[DISTANCE_LOOKBACK];

extern SensorData sensor_data;

int flip = 1;

int get_distance_at_angle(int angle, int delay)
{
  setServoAngle(angle, delay);
  return getDistance();
}

int survey(int min_, int max_, int seperation, int delay, int flip)
{
  int best_distance = 10000;
  int best_angle = 90;

  switch (flip)
  {
    case 1:
      for (int i = min_; i <= max_; i += seperation)
      {
        int distance = get_distance_at_angle(i, delay);
        print("sur", "distance", distance);
        if (distance < best_distance && distance < MAX_DISTANCE && distance > 0)
        {
          // return i;
          best_distance = distance;
          best_angle = i;
        }
      }
      break;
    
    case -1:
      for (int i = max_; i >= min_; i -= seperation)
      {
        int distance = get_distance_at_angle(i, delay);
        print("sur", "distance", distance);
        if (distance < best_distance && distance < MAX_DISTANCE && distance > 0)
        {
          // return i;
          best_distance = distance;
          best_angle = i;
        }
      }
      break;
  }

  return best_angle;
}

int convert_servo_to_gyro_deg(int degrees)
{
  return (-degrees + 90) / 2;
}

void sticky_following()
{
  int FOV = 60;
  int cone_max = 90 + FOV;
  int cone_min = 90 - FOV;

  const int steps = 10;
  int seperation = (cone_max - cone_min) / steps;

  int best_angle = survey(cone_min, cone_max, seperation, 150, flip);
  flip *= -1;

  int distance = get_distance_at_angle(best_angle, 200);

  // gyro_turn(convert_servo_to_gyro_deg(best_angle));

  int offset = best_angle - 90;
  int adjust = constrain(abs(offset) * 8, 0, NORMAL_SPEED);

  int left_speed = NORMAL_SPEED;
  int right_speed = NORMAL_SPEED;

  if (offset > 0)
  {
    left_speed = NORMAL_SPEED - adjust / 2;
    right_speed = NORMAL_SPEED + adjust;
  }
  if (offset < 0)
  {
    left_speed = NORMAL_SPEED + adjust;
    right_speed = NORMAL_SPEED - adjust / 2;
  }

  left_speed = constrain(left_speed, -NORMAL_SPEED, NORMAL_SPEED);
  right_speed = constrain(right_speed, -NORMAL_SPEED, NORMAL_SPEED);

  move_motors(left_speed, right_speed);
}