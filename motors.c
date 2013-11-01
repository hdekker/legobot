#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "lms2012.h"

static int encoder_file;
static MOTORDATA *pMotorData;

int motors_initialize()
{
  //Open the device file associated to the motor encoders
  if((encoder_file = open(MOTOR_DEVICE_NAME, O_RDWR | O_SYNC)) == -1)
  {
    printf("Failed to open the encoder device\n");
    return -1;
  }
  pMotorData = (MOTORDATA*)mmap(0, sizeof(MOTORDATA)*vmOUTPUTS, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, encoder_file, 0);
  if (pMotorData == MAP_FAILED)
  {
    printf("Failed to map the encoder file\n");
    return -1;
  } 
  
  return 0;
}

int motors_terminate()
{
  printf("Closing encoder device\n");
  close(encoder_file);
  
  return 0;
}

SBYTE motors_get_motor_speed(int port)
{
  return pMotorData[port].Speed;
}

SLONG motors_get_motor_tacho(int port)
{
  return pMotorData[port].TachoSensor;
}

