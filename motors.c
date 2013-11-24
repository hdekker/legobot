#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "lms2012.h"

#include "motors.h"
#include "common.h"

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
  sleep_ms(100);
  
  printf("Closing encoder device\n");
  if (pMotorData && pMotorData != MAP_FAILED)
    munmap(pMotorData, sizeof(MOTORDATA)*vmOUTPUTS);
  close(motor_file);
  
  printf("Closing pwm device\n");
  close(pwm_file);
  
  return 0;
}

void motors_reset_angle(int port)
{
  angles[port] = pMotorData[port].TachoSensor;
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
    motors_reset_angle(port);
  }
}

void motors_stop(int port)
{
  char motor_command[3];
  port = 1 << port;
  motor_command[0] = opOUTPUT_STOP;
  motor_command[1] = port;
  motor_command[2] = 1;
  write(pwm_file, motor_command, 3);
  fsync(pwm_file);
}

void motors_stop_all()
{
  char motor_command[3];
  
  // Switch to open loop
  //motor_command[0] = opOUTPUT_POWER;
  //motor_command[1] = 0x15;
  //motor_command[2] = 0;
  //write(pwm_file, motor_command, 3);

  // Stop all motors
  motor_command[0] = opOUTPUT_STOP;
  motor_command[1] = 0x15;
  motor_command[2] = 1;
  write(pwm_file, motor_command, 3);
  fsync(pwm_file);
  
  // Reset port
  //motor_command[0] = opOUTPUT_RESET;
  //motor_command[1] = 0x15;
  //write(pwm_file, motor_command, 2);
}

void motors_set_speed(int port, int speed)
{
  char motor_command[3];
  port = 1 << port;
  
  // Set speed
  motor_command[0] = opOUTPUT_POWER;
  motor_command[1] = port;
  motor_command[2] = speed;
  write(pwm_file, motor_command, 3);
  
  // Start the motor
  motor_command[0] = opOUTPUT_START;
  write(pwm_file, motor_command, 2);
  fsync(pwm_file);
}

SBYTE motors_get_motor_speed(int port)
{
  return pMotorData[port].Speed;
}

SLONG motors_get_angle(int port)
{
  SLONG angle = pMotorData[port].TachoSensor - angles[port];
  printf("motors_get_angle: port=%d, angle=%d\n", port, angle);
  return angle;
}

//opOUTPUT_STEP_SPEED   LAYER   NOS      SPEED   STEP1   STEP2   STEP3   BRAKE
void motors_step_speed(int port, int speed, int step1, int step2, int step3)
{
  printf("motors_step_speed: port=%d, speed=%d, step1=%d, step2=%d, step3=%d\n", port, speed, step1, step2, step3);
  port = 1 << port;
 
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
  fsync(pwm_file);
}


void motors_start_move_to_angle(int port, int speed, int angle_deg, int maxturns)
{
  // angle (-90 = left, 0 = forward, +90 = right)
  SLONG motor_angle = motors_get_angle(port);
  //int swing_edge;
  if (motor_angle < angle_deg)
  {
    int swing = angle_deg - motor_angle;
    if (maxturns)
    {
      //printf("swing_pos_0 = %d\n", swing);
      swing %= ((maxturns * 360) + 1);
      //printf("swing_pos_1 = %d\n", swing);
    }
    int swing_edge = swing / 5;
    motors_step_speed(port, speed, swing_edge, swing - (swing_edge * 2), swing_edge);
  }
  else if (motor_angle > angle_deg)
  {
    int swing = motor_angle - angle_deg;
    if (maxturns)
    {
      //printf("swing_neg_0 = %d\n", swing);
      swing %= ((maxturns * 360) + 1);
      //printf("swing_neg_1 = %d\n", swing);
    }
    int swing_edge = swing / 5;
    motors_step_speed(port, -speed, swing_edge, swing - (swing_edge * 2), swing_edge);
  }
}


void motors_wait_move_to_angle(int port)
{
  // Wait until motors movement is finished. This is detected when motor speed is 0.
  // To prevent errors, the last 4 motor speed measurements have to be 0.
  
  SBYTE motor_speeds[4] = { -1, -1, -1, -1 };
  int i = 0;
  do
  {
    sleep_ms(100);
    motor_speeds[i%4] = motors_get_motor_speed(port);
    //SLONG motor_angle = motors_get_angle(port);
    //if ((i%10) > 0) printf("motors_wait_move_to_angle: port=%d, cur_angle=%d, speed=%d\n", port, motor_angle, motor_speeds[i%4]);
    // Speed of last 4 measurements
    i++;
  } while ((abs(motor_speeds[0]) + abs(motor_speeds[1]) + abs(motor_speeds[2]) + abs(motor_speeds[3])) > 0);
}


void motors_move_to_angle(int port, int speed, int angle_deg, int maxturns)
{
  motors_start_move_to_angle(port, speed, angle_deg, maxturns);
  motors_wait_move_to_angle(port);
}
