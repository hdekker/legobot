#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "lms2012.h"

#include "motors.h"

static int pwm_file;
static int motor_file;
static MOTORDATA *pMotorData;
static int angles[INPUTS];

int motors_initialize()
{
  // Open the device file associated to the motor controllers
  if((pwm_file = open(PWM_DEVICE_NAME, O_WRONLY)) == -1)
  {
    printf("Failed to open the pwm device\n");
    return -1;
  }

  // Open the device file associated to the motor encoders
  if((motor_file = open(MOTOR_DEVICE_NAME, O_RDWR | O_SYNC)) == -1)
  {
    printf("Failed to open the encoder device\n");
    return -1;
  }
  
  pMotorData = (MOTORDATA*)mmap(0, sizeof(MOTORDATA)*vmOUTPUTS, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, motor_file, 0);
  if (pMotorData == MAP_FAILED)
  {
    printf("Failed to map the encoder file\n");
    return -1;
  } 
  
  motors_reset_all();
  
  return 0;
}


int motors_terminate()
{
  motors_stop_all();
  printf("Closing encoder device\n");
  close(motor_file);
  printf("Closing pwm device\n");
  close(pwm_file);
  
  return 0;
}

void motors_reset_all()
{
  char motor_command[2];
  
  motor_command[0] = opOUTPUT_RESET;
  motor_command[1] = 0x15;
  write(pwm_file, motor_command, 2);
  
  motor_command[0] = opOUTPUT_CLR_COUNT;
  motor_command[1] = 0x15;
  write(pwm_file, motor_command, 2);
  
  int port;
  for (port=0; port<INPUTS; port++)
  {
    angles[port] = motors_get_angle(port);
  }
}

void motors_stop_all()
{
  char motor_command[3];
  
  // Switch to open loop
  motor_command[0] = opOUTPUT_POWER;
  motor_command[1] = 0x15;
  motor_command[2] = 0;
  write(pwm_file, motor_command, 3);

  // Stop all motors
  motor_command[0] = opOUTPUT_STOP;
  motor_command[1] = 0x15;
  motor_command[2] = 1;
  write(pwm_file, motor_command, 3);
  
  // Reset port
  //motor_command[0] = opOUTPUT_RESET;
  //motor_command[1] = 0x15;
  //write(pwm_file, motor_command, 2);
}

SBYTE motors_get_motor_speed(int port)
{
  return pMotorData[port].Speed;
}

SLONG motors_get_angle(int port)
{
  return pMotorData[port].TachoSensor - angles[port];
}


//opOUTPUT_STEP_SPEED   LAYER   NOS      SPEED   STEP1   STEP2   STEP3   BRAKE
int motors_step_speed(int port, int speed, int step1, int step2, int step3)
{
  port = 1 << port;
  //printf("motors_step_speed\n");
 
   STEPSPEED step_speed;
   step_speed.Cmd = opOUTPUT_STEP_SPEED;
   step_speed.Nos = port;
   step_speed.Speed = speed;
   step_speed.Step1 = step1;
   step_speed.Step2 = step2;
   step_speed.Step3 = step3;
   step_speed.Brake = 1;
   write(pwm_file, &step_speed, sizeof(STEPSPEED));
  
  char motor_command[3];
  //motor_command[0] = opOUTPUT_SPEED;
  //motor_command[1] = port;
  //motor_command[2] = speed;
  //write(pwm_file, motor_command, 3);
  
  motor_command[0] = opOUTPUT_START;
  motor_command[1] = port;
  write(pwm_file, motor_command, 2);
  
  return 0;
}
