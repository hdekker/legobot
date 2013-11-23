#include <stdio.h>
#include "common.h"
#include "command.h"
#include "sensors.h"
#include "motors.h"
#include "lms2012.h"

void maze_initialize()
{
  ; // To be implemented
}

void maze_terminate()
{
  ; // To be implemented
}

void maze_execute(bool* keep_running)
{
  printf("maze skipped\n"); return;
  
  command_move_forward();
  
  do
  {
    sleep_ms(100); // Prevent busy loop
    
    int forward_distance = command_get_forward_distance_mm();
    if (forward_distance < 200) 
    {
      int right_distance = command_get_right_distance_mm();
      if (right_distance > 200)
      {
        command_move_stop();
        command_turn_right();
        command_move_forward();
      }
      else
      {
        command_move_stop();
        command_turn_left();
        command_move_forward();
      }
    }
    
  } while (*keep_running);
  
  printf("A button was pressed, stopping...\n");
  command_move_stop();
  //motors_move_to_angle(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, 0);
}
