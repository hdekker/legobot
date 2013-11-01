#include <unistd.h>  // sleep
#include <stdio.h>   // printf
#include "lms2012.h"
#include "motors.h"
#include "sensors.h"

const int MAX_SAMPLES = 100;



int main(int argc, const char* argv[])
{
  int i;
  int j;
  sensors_initialize();
  motors_initialize();

  for(i = 0;i<MAX_SAMPLES;i++)
  {
    // The ports are designated as PORT_NUMBER-1
    printf("Touched = %u\n", sensors_get_touched(0));
    printf("Infrared = %u\n", sensors_get_ir_distance(1));
    printf("Color = %u\n", sensors_get_color(2));
    printf("Ultrasonic = %u\n", sensors_get_ul_distance(3));
    printf("Speed = %d\n", motors_get_motor_speed(0));
    printf("Tacho = %d\n", motors_get_motor_tacho(1));
    printf("Buttons "); for (j=0; j<BUTTONS; j++) printf("%d ", sensors_is_button_pressed(j)); printf("\n");
    sleep(1);
  }
  
  motors_terminate();
  sensors_terminate();
  return 0;
}
