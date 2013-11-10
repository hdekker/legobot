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

void maze_execute(int* keep_running)
{
  int buttons_pressed = 0;
  
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
    
    // Check how many buttons were pressed. Just one is enough to stop program
    buttons_pressed = 0;
    for (int j=0; j<BUTTONS; j++) buttons_pressed += sensors_is_button_pressed(j);
    
  } while ((*keep_running) && (buttons_pressed == 0));
  
  printf("A button was pressed, stopping...\n");
  command_move_stop();
  //motors_move_to_angle(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, 0);
}
