#include "following.hpp"
#include "utils.hpp"
#include "config.hpp"

extern int angles[GYRO_LOOKBACK];
extern int distances[DISTANCE_LOOKBACK];

extern SensorData sensor_data;

void following()
{
  ledOn(get_following_colour(sensor_data.avg_distance));

  if (sensor_data.avg_distance >= MIN_DISTANCE && sensor_data.avg_distance < MAX_DISTANCE)
  {
    int speed = get_speed(sensor_data.avg_distance);
    move_motors(speed, speed);
  }

  if (sensor_data.avg_distance > MAX_DISTANCE)
  {
    move_motors(0, 0);
    state = 2;
  }
}