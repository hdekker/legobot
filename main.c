#include <signal.h>
#include <unistd.h>  // sleep
#include <stdio.h>   // printf
#include <stdlib.h>  // malloc
#include <pthread.h>
#include "lms2012.h"

#include "screen.h"
#include "motors.h"
#include "sensors.h"
#include "robot.h"
#include "navi.h"
#include "common.h"
#include "command.h"
#include "maze.h"
#include "ball.h"

static bool keep_running = true;

void intHandler(int dummy)
{
  keep_running = false;
}

void* busyloop(void* ptr)
{
  int i = 0;
  while (1) i++;
  return NULL;
}

typedef void(*game_func)(bool*);

typedef struct function_args
{
  game_func function;
  bool * ptr_keep_running;
} function_args;

  
void* run(void* args)
{
  function_args* functionargs = (function_args*) args;
  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
  pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
  functionargs->function(functionargs->ptr_keep_running);
  free(functionargs);
  return NULL;
}

pthread_t start(game_func gamefunc, bool* ptr_keep_running)
{
  pthread_t thread = 0UL;
  function_args* args = (function_args*) malloc(sizeof(function_args));
  args->function = gamefunc;
  args->ptr_keep_running = ptr_keep_running;
  pthread_create(&thread, NULL, run, (void*)args);
  return thread;
}

void stop(pthread_t thread)
{
  pthread_cancel(thread);
  pthread_join(thread, NULL);
}


int main(int argc, const char* argv[])
{
  pthread_t thread = 0UL;
  
  signal(SIGINT, intHandler);
  
  screen_initialize();
  screen_clear();
  sensors_initialize(&keep_running);
  motors_initialize();
  command_initialize();
  maze_initialize();
  ball_initialize();

  if (keep_running)
  {
    thread = start(maze_execute, &keep_running);
    while (keep_running and (pthread_kill(thread, 0)==0)) sleep(1);
    stop(thread);
  }
  
  if (keep_running)
  {
    thread = start(ball_execute, &keep_running);
    while (keep_running and (pthread_kill(thread, 0)==0)) sleep(1);
    stop(thread);
  }
  
  ball_terminate();
  maze_terminate();
  command_terminate();
  motors_terminate();
  sensors_terminate(&keep_running);
  screen_terminate();
  
  return 0;
}



void main_test()
{
  /*
  int i;
  int j;
  
  //sensors_set_color_mode(2, SENSORS_NXT_COL_COL);
  screen_draw_circle(SCREEN_WIDTH/2, SCREEN_HEIGTH/2, 8);  
  
  //for(i = 0;i<60;i++)
  //{
  //  // The ports are designated as PORT_NUMBER-1
  //  printf("Ultrasonic = %u [mm]\n", sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT));
  //  sleep(1);
  //}
  
  int left = 0;
  int right = 0;
  int motor_angle = 0;
  int angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT) * ROBOT_RADAR_DIRECTION + ROBOT_RADAR_ROTATION;
  int dist = sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT);
  printf("initial: angle=%d, dist=%u\n", angle, dist);
  motors_step_speed(ROBOT_RADAR_MOTOR_PORT, -ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, 0, ROBOT_RADAR_SWING/4);
  sleep(3);
  motor_angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT);
  for(i = 0;i<2;i++)
  {
    motors_step_speed(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, ROBOT_RADAR_SWING/2, ROBOT_RADAR_SWING/4);
    while (motor_angle < (ROBOT_RADAR_SWING/2 - 2))
    {
      motor_angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT);
      angle = motor_angle * ROBOT_RADAR_DIRECTION + ROBOT_RADAR_ROTATION;
      dist = sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT);
      left = motors_get_angle(ROBOT_WHEEL_LEFT_PORT);
      right = motors_get_angle(ROBOT_WHEEL_RIGHT_PORT);
      //printf("clockwise: angle=%d, dist=%u\n", angle, dist);
      navi_update_map(angle, dist, left, right);
      sleep_ms(10);
    }
    sleep_ms(200);
    motors_step_speed(ROBOT_RADAR_MOTOR_PORT, -ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, ROBOT_RADAR_SWING/2, ROBOT_RADAR_SWING/4);
    while (motor_angle > -(ROBOT_RADAR_SWING/2 - 2))
    {
      motor_angle = motors_get_angle(ROBOT_RADAR_MOTOR_PORT);
      angle = motor_angle * ROBOT_RADAR_DIRECTION + ROBOT_RADAR_ROTATION;
      dist = sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT);
      left = motors_get_angle(ROBOT_WHEEL_LEFT_PORT);
      right = motors_get_angle(ROBOT_WHEEL_RIGHT_PORT);
      //printf("anticlock: angle=%d, dist=%u\n", angle, dist);
      navi_update_map(angle, dist, left, right);
      sleep_ms(10);
    }
    sleep_ms(200);
    //screen_clear();
    
    if (i == 0)
    {
      printf("\n\nmoving forward...\n\n");
      motors_step_speed(ROBOT_WHEEL_LEFT_PORT, ROBOT_SPEED, 90, 540, 90);
      motors_step_speed(ROBOT_WHEEL_RIGHT_PORT, ROBOT_SPEED, 90, 540, 90);
    }

  }
  sleep_ms(3000);
  motors_step_speed(ROBOT_RADAR_MOTOR_PORT, ROBOT_RADAR_SPEED, ROBOT_RADAR_SWING/4, 0, ROBOT_RADAR_SWING/4);
  
  for(i = 0;i<0;i++)
  {
    // The ports are designated as PORT_NUMBER-1
    printf("\n");
    printf("Touched = %u\n", sensors_get_touched(0));
    printf("Infrared = %u\n", sensors_get_ir_distance(1));
    printf("Color = %u\n", sensors_get_color(2));
    printf("Ultrasonic = %u\n", sensors_get_us_distance_mm(ROBOT_ULTRASONIC_SENSOR_PORT));
    printf("Speed = %d\n", motors_get_motor_speed(0));
    printf("Tacho = %d\n", motors_get_angle(1));
    printf("Buttons "); for (j=0; j<BUTTONS; j++) printf("%d ", sensors_is_button_pressed(j)); printf("\n");
    sleep(1);
  }
  */
}