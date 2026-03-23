#include "sticky_following.hpp"
#include "utils.hpp"
#include "config.hpp"

extern int angles[GYRO_LOOKBACK];
extern int distances[DISTANCE_LOOKBACK];

extern SensorData sensor_data;

int FOV = 40;
const int steps = 10;

int cone_max = 90 + FOV;
int cone_min = 90 - FOV;
int seperation = (cone_max - cone_min) / steps;

int error_max = 90 + (seperation / 2);
int error_min = 90 - (seperation / 2);

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

  int measurement_error = int(seperation * 0.3);
  int counting_errors = 0;

  switch (flip)
  {
    case 1:
      for (int i = min_; i <= max_; i += seperation)
      {
        int distance = get_distance_at_angle(i, delay);
        // print("sur", "distance", distance);
        ledOn(get_roaming_colour(distance));
        if (distance < MIN_DISTANCE) { --measurement_error; }
        if (distance < MIN_DISTANCE && measurement_error == 0)
        {
          state = 1;
          return 90;
        }
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
        // print("sur", "distance", distance);
        ledOn(get_roaming_colour(distance));
        if (distance < MIN_DISTANCE) { --measurement_error; }
        if (distance < MIN_DISTANCE && measurement_error == 0)
        {
          state = 1;
          return 90;
        }
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
  int best_angle = survey(cone_min, cone_max, seperation, 175, flip);
  flip *= -1;

  // int distance = get_distance_at_angle(best_angle, 200);

  int offset = best_angle - 90;
  int adjust = constrain(abs(offset) * 4, 0, MIN_SPEED);

  int left_speed = NORMAL_SPEED;
  int right_speed = NORMAL_SPEED;

  if (offset > 0)
  {
    left_speed -= adjust;
    right_speed += adjust;
  }
  else if (offset < 0)
  {
    left_speed += adjust;
    right_speed -= adjust;
  }
  else if (error_min <= offset && offset <= error_max)
  {
    left_speed = left_speed;
    right_speed = right_speed;
  }


  left_speed = constrain(left_speed, 0, NORMAL_SPEED);
  right_speed = constrain(right_speed, 0, NORMAL_SPEED);

  move_motors(left_speed, right_speed);

  if (getDistance() < MIN_DISTANCE || state != 0) {
    state = 1;
    move_motors(0, 0);
    return;
  }

  if (getDistance() < MIN_DISTANCE + 5) {
    move_motors(-NORMAL_SPEED, -NORMAL_SPEED);
    shake_head();
    move_motors(-200, 200);
    delay(random(500, 1000));
  }
}