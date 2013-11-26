#include <stdio.h>
#include <stdlib.h>
#include "robot.h"
#include "motors.h"
#include "sensors.h"
#include "common.h"
#include "command.h"

void command_initialize()
{
  return;
}


void command_terminate()
{
  return;
}


int command_get_forward_distance_mm()
{
  int distance = sensors_get_ir_distance(ROBOT_INFRARED_SENSOR_PORT);
  distance *= 10; // From pct to mm 
  printf("command_get_forward_distance_mm: distance=%d\n", distance);
  return distance;
}


int command_get_right_distance_mm()
{
  int distance = sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT);
  printf("command_get_right_distance_mm: distance=%d\n", distance);
  return distance;
}


void command_move_forward()
{
  printf("command_move_forward\n");
  motors_set_speed(ROBOT_WHEEL_LEFT_PORT, ROBOT_SPEED);
  motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, ROBOT_SPEED);
  sleep_ms(100);
}

void command_move_stop()
{
  printf("command_move_stop\n");
  motors_stop(ROBOT_WHEEL_LEFT_PORT);
  motors_stop(ROBOT_WHEEL_RIGHT_PORT);
  sleep_ms(100);
}

void command_move_distance(int distance_mm)
{
  int left_angle = motors_get_angle(ROBOT_WHEEL_LEFT_PORT);
  int right_angle = motors_get_angle(ROBOT_WHEEL_RIGHT_PORT);

  int wheel_rotation = 360 * ((float)distance_mm/1000.0) / ROBOT_WHEEL_CIRCUMFERENCE;

  printf("command_move_distance: distance_mm=%d, wheel_rotation=%d\n", distance_mm, wheel_rotation);
  
  left_angle += wheel_rotation;
  right_angle += wheel_rotation;  
  
  motors_start_move_to_angle(ROBOT_WHEEL_LEFT_PORT, ROBOT_SPEED, left_angle, 0);
  motors_start_move_to_angle(ROBOT_WHEEL_RIGHT_PORT, ROBOT_SPEED, right_angle, 0);
  motors_wait_move_to_angle(ROBOT_WHEEL_LEFT_PORT);
  motors_wait_move_to_angle(ROBOT_WHEEL_RIGHT_PORT);
  sleep_ms(100);
}

void command_turn_angle(int angle_deg)
{
  int left_angle = motors_get_angle(ROBOT_WHEEL_LEFT_PORT);
  int right_angle = motors_get_angle(ROBOT_WHEEL_RIGHT_PORT);

  // Both wheels are responsible of half of the rotation
  float wheel_displacement = ROBOT_WHEEL_DISTANCE_X * angle_deg * PI; 
  int wheel_rotation = abs(wheel_displacement / ROBOT_WHEEL_CIRCUMFERENCE);

  printf("command_turn: left=%d, right=%d, rotation=%d, angle=%d\n", left_angle, right_angle, wheel_rotation, angle_deg);
  
  if (angle_deg<0) 
  {
    left_angle += wheel_rotation;
    right_angle -= wheel_rotation;  
  }
  else
  {
    left_angle -= wheel_rotation;
    right_angle += wheel_rotation;  
  }
  
  motors_start_move_to_angle(ROBOT_WHEEL_LEFT_PORT, ROBOT_SPEED, left_angle, 0);
  motors_start_move_to_angle(ROBOT_WHEEL_RIGHT_PORT, ROBOT_SPEED, right_angle, 0);
  motors_wait_move_to_angle(ROBOT_WHEEL_LEFT_PORT);
  motors_wait_move_to_angle(ROBOT_WHEEL_RIGHT_PORT);
  sleep_ms(100);
}

void command_turn_left()
{
  command_turn_angle(90);
}


void command_turn_right()
{
  command_turn_angle(-90);
}
