#include <stdio.h>
#include <stdlib.h>
#include "common.h"
#include "command.h"
#include "sensors.h"
#include "motors.h"
#include "lms2012.h"

#include "robot.h"

//User defined inputs
int Desired_dist_right  = 50;
int Desired_dist_front  = 100;
int Speed_turn_left     = 40;
int Speed_turn_right    = 60;
int Motor_speed_forward = 40; 

//All other inputs
int error;
int Turn;
int Motor_speed_left;
int Motor_speed_right;
int lasterror = 0;
int derivative;
bool must_turn_left = false;    //0 = no turn right, 1 = turn right


void move_forward(int duration_ms, int speed)
{
     motors_set_speed(ROBOT_WHEEL_LEFT_PORT, speed);
     motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, speed);
     sleep_ms(duration_ms);
}

void move_away_from_wall()
{
     motors_set_speed(ROBOT_WHEEL_LEFT_PORT, 0);
     motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, 50);
     sleep_ms(400);
     motors_set_speed(ROBOT_WHEEL_LEFT_PORT, 50);
     motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, 0);
     sleep_ms(200);  
}


void turn_right(int speed)
{
  int portA = ROBOT_WHEEL_LEFT_PORT;
  int portB = ROBOT_WHEEL_RIGHT_PORT;
  int right_distance2 = command_get_right_distance_mm();
  
  if (right_distance2 > 150)
  {
  motors_set_speed(portA, speed+15);
  motors_set_speed(portB, 15);
  sleep_ms(50);
  }
}


bool turn_left(int speed)
{
  int portA = ROBOT_WHEEL_LEFT_PORT;
  int portB = ROBOT_WHEEL_RIGHT_PORT;
  bool keep_turning2; 
  int forward_distance = command_get_forward_distance_mm();
  
  motors_set_speed(portA, -30);
  motors_set_speed(portB, 30+speed);
  sleep_ms(500);

    forward_distance = command_get_forward_distance_mm();

    if ( forward_distance < 200)  //Tune this parameter!!!!
    { 
      keep_turning2 = true;
    }
    else
    {
      sleep_ms(800); // TURN BIT MORE TO MAKE SURE RIGHT SENSOR MEASURES WALL CORRECTLY
      keep_turning2 = false;
    }
  
  return keep_turning2;
}


void maze_execute(bool* keep_running)
{
  // 5000ms and speed 50km/h
  move_forward(5000, 50);
  
  while (*keep_running)
  {    
    int forward_distance = command_get_forward_distance_mm();
    int right_distance = command_get_right_distance_mm();
  
    
    // Do we need to START turning left?
    if (not must_turn_left)
    {
      if ((forward_distance < Desired_dist_front) && (right_distance < 3*Desired_dist_right))
      {
         must_turn_left = true;
      }
    }

    // Turn left
    if (must_turn_left)
    {
      bool Var_turning = turn_left(Speed_turn_left);
      must_turn_left = Var_turning;
      lasterror = 0;
    }
    
    // Turn right
    if ((right_distance > 3*Desired_dist_right) && (right_distance < 2200) && (not must_turn_left))
    {
      turn_right(Speed_turn_right);
      lasterror = 0;
    }
    
    // Move away from wall
    if ((right_distance >= 2200) && (not must_turn_left))
    {
     move_away_from_wall();
     lasterror = 0;
    }
    
    // Follow wall
    if ((not must_turn_left) && (forward_distance >= Desired_dist_front) && (right_distance < 3*Desired_dist_right)) // Follow wall
    {
      // Control action
      error = right_distance - Desired_dist_right;
      derivative = error - lasterror;
      Turn  = 2*error + 2*derivative;
      
      Motor_speed_left = Motor_speed_forward + Turn;
      Motor_speed_right = Motor_speed_forward - Turn;
      
      // Saturation motor speeds
      if ( Motor_speed_left > 100) { Motor_speed_left = 100; }
      if ( Motor_speed_left < -100) { Motor_speed_left = -100; }
      if ( Motor_speed_right > 100) { Motor_speed_right = 100; }
      if ( Motor_speed_right < -100) { Motor_speed_right = -100; }
      
      // Set motor
      motors_set_speed(ROBOT_WHEEL_LEFT_PORT, Motor_speed_left);
      motors_set_speed(ROBOT_WHEEL_RIGHT_PORT, Motor_speed_right);
      
      lasterror = error;
      sleep_ms(25); // Prevent busy loop
    }
  }
 
  printf("maze_execute stopping...\n");
  command_move_stop();
}
