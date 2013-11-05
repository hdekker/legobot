#include "robot.h"
#include "motors.h"
#include "sensors.h"
#include "common.h"

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
  motors_move_to_angle(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, 0);
  sleep_ms(100);
  int distance = sensors_get_us_distance_mm(ROBOT_DISTANCE_SENSOR_PORT);
  printf("command_get_forward_distance_mm: distance=%d\n", distance);
  return distance;
}


int command_get_right_distance_mm()
{
  motors_move_to_angle(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, 90);
  sleep_ms(100);
  int distance = sensors_get_us_distance_mm(ROBOT_DISTANCE_SENSOR_PORT);
  printf("command_get_right_distance_mm: distance=%d\n", distance);
  return distance;
}


void command_move_forward()
{
  printf("command_move_forward\n");
  motors_set_speed(ROBOT_WHEEL_LEFT_PORT, ROBOT_SPEED);
  motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, ROBOT_SPEED);
}

void command_move_stop()
{
  printf("command_move_stop\n");
  motors_stop(ROBOT_WHEEL_LEFT_PORT);
  motors_stop(ROBOT_WHEEL_RIGHT_PORT);
}


void command_turn(int clockwise)
{
  command_move_stop();
  sleep_ms(100);
  
  int left_angle = motors_get_angle(ROBOT_WHEEL_LEFT_PORT);
  int right_angle = motors_get_angle(ROBOT_WHEEL_RIGHT_PORT);

  float wheel_displacement = ROBOT_WHEEL_DISTANCE_X * PI / 4.0;
  int wheel_rotation = (wheel_displacement / ROBOT_WHEEL_CIRCUMFERENCE) * 360;

  printf("command_turn: left=%d, right=%d, rotation=%d, clockwise=%d\n", left_angle, right_angle, wheel_rotation, clockwise);
  
  if (clockwise) 
  {
    left_angle += wheel_rotation;
    right_angle -= wheel_rotation;  
  }
  else
  {
    left_angle -= wheel_rotation;
    right_angle += wheel_rotation;  
  }
  
  motors_start_move_to_angle(ROBOT_WHEEL_LEFT_PORT, ROBOT_SPEED, left_angle);
  motors_start_move_to_angle(ROBOT_WHEEL_RIGHT_PORT, ROBOT_SPEED, right_angle);
  motors_wait_move_to_angle(ROBOT_WHEEL_LEFT_PORT, left_angle);
  motors_wait_move_to_angle(ROBOT_WHEEL_RIGHT_PORT, right_angle);
  sleep_ms(100);
}

void command_turn_left()
{
  command_turn(0);
}


void command_turn_right()
{
  command_turn(1);
}
