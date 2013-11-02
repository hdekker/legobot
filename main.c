#include <unistd.h>  // sleep
#include <time.h>    // nanosleep
#include <stdio.h>   // printf
#include "lms2012.h"
#include "motors.h"
#include "sensors.h"
#include "robot.h"
#include "navi.h"

const int MAX_SAMPLES = 2;


void sleep_ms(unsigned milliseconds)
{
  struct timespec t;
  t.tv_sec = milliseconds / 1000u;
  t.tv_nsec = (milliseconds % 1000u) * 1e6;
  nanosleep(&t, NULL);
}


int main(int argc, const char* argv[])
{
  int i;
  int j;
  sensors_initialize();
  motors_initialize();
  sleep(1);

  sensors_set_color_mode(2, SENSORS_NXT_COL_COL);
  
  int left = 0;
  int right = 0;
  int angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT);
  int dist = sensors_get_ul_distance(3);
  printf("initial: angle=%d, dist=%u\n", angle, dist);
  motors_step_speed(ROBOT_RADAR_MOTOR_PORT, -ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, 0, ROBOT_RADAR_SWING/4);
  sleep(1);
  angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT);
  dist = sensors_get_ul_distance(3);
  printf("prepare: angle=%d, dist=%u\n", angle, dist);
  sleep(1);
  for(i = 0;i<MAX_SAMPLES;i++)
  {
    motors_step_speed(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, ROBOT_RADAR_SWING/2, ROBOT_RADAR_SWING/4);
    while (angle < (ROBOT_RADAR_SWING/2 - 1))
    {
      angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT);
      dist = sensors_get_ul_distance(3);
      left = motors_get_angle(ROBOT_WHEEL_LEFT_PORT);
      right = motors_get_angle(ROBOT_WHEEL_RIGHT_PORT);
      printf("angle=%d, dist=%u\n", angle, dist);
      navi_update_map(angle, dist, left, right);
      sleep_ms(150);
    }
    sleep_ms(200);
    motors_step_speed(ROBOT_RADAR_MOTOR_PORT, -ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, ROBOT_RADAR_SWING/2, ROBOT_RADAR_SWING/4);
    while (angle > -(ROBOT_RADAR_SWING/2 - 1))
    {
      angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT);
      dist = sensors_get_ul_distance(3);
      left = motors_get_angle(ROBOT_WHEEL_LEFT_PORT);
      right = motors_get_angle(ROBOT_WHEEL_RIGHT_PORT);
      printf("angle=%d, dist=%u\n", angle, dist);
      navi_update_map(angle, dist, left, right);
    }
    sleep_ms(200);
  }
  motors_step_speed(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, 0, ROBOT_RADAR_SWING/4);
  
  for(i = 0;i<MAX_SAMPLES;i++)
  {
    // The ports are designated as PORT_NUMBER-1
    printf("\n");
    printf("Touched = %u\n", sensors_get_touched(0));
    printf("Infrared = %u\n", sensors_get_ir_distance(1));
    printf("Color = %u\n", sensors_get_color(2));
    printf("Ultrasonic = %u\n", sensors_get_ul_distance(3));
    printf("Speed = %d\n", motors_get_motor_speed(0));
    printf("Tacho = %d\n", motors_get_angle(1));
    printf("Buttons "); for (j=0; j<BUTTONS; j++) printf("%d ", sensors_is_button_pressed(j)); printf("\n");
    sleep(1);
  }
  
  motors_terminate();
  sensors_terminate();
  return 0;
}
