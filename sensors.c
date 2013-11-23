#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "lms2012.h"
#include "sensors.h"
#include "robot.h"
#include "common.h"

static int ad_file;
static ANALOG *pAnalog;

static int uart_file;
static UART *pUart;

static int ui_file;
static UI *pUI;

static int iic_file;
static IIC *pIic;
static IICDAT IicDat;
  
static pthread_t thread;
static int button_pressed_count[BUTTONS];

void* check_button_thread_func(void* ptr)
{
  bool* keep_running;
  keep_running = (bool*) ptr;
  bool button_states[BUTTONS];
  for (int i=0; i<BUTTONS; i++)
  {
    button_states[i] = false;
    button_pressed_count[i] = 0;
  }
    
  printf("Check button thread running\n");
  while (*keep_running)
  {
    sleep_ms(10);
    for (int i=0; i<BUTTONS; i++)
    {
      int pressed = pUI->Pressed[i];
      if (pressed)
      {
        if (!button_states[i])
        {
          button_states[i] = true;
          button_pressed_count[i] += 1;
          printf("pressed button %d\n", i);
        }
      }
      else // pressed == false
      {
        if (button_states[i])
        {
          button_states[i] = false;
          printf("pressed released %d\n", i);
        }
      }
    }
    if (button_pressed_count[BUTTON_CODE_ESC])
    {
      *keep_running = 0;
    }
  }
  printf("Check button thread stopping\n");
  return NULL;
}


int sensors_initialize(bool* keep_running)
{
  //Open the device file
  if((ad_file = open(ANALOG_DEVICE_NAME, O_RDWR | O_SYNC)) == -1)
  {
    printf("Failed to open analog device\n");
    return -1;
  }
  pAnalog = (ANALOG*)mmap(0, sizeof(ANALOG), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, ad_file, 0);
  if (pAnalog == MAP_FAILED)
  {
    printf("Failed to map analog device\n");
    return -1;
  }
  printf("AD device ready\n");

  //Open the uart device file
  if((uart_file = open(UART_DEVICE_NAME, O_RDWR | O_SYNC)) == -1)
  {
    printf("Failed to open uart device\n");
    return -1;
  }
  pUart = (UART*)mmap(0, sizeof(UART), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, uart_file, 0);
  if (pUart == MAP_FAILED)
  {
    printf("Failed to map uart device\n");
    return -1;
  }
  printf("UART device is ready\n");
  
  //Open the device UI button file
  if((ui_file = open(UI_DEVICE_NAME, O_RDWR | O_SYNC)) == -1)
  {
    printf("Failed to open the ui device\n");
    return -1;
  }
  pUI = (UI*)mmap(0, sizeof(UI)*vmOUTPUTS, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, ui_file, 0);
  if (pUI == MAP_FAILED)
  {
    printf("Failed to map the ui file\n");
    return -1;
  } 

  //Open the device file
  if((iic_file = open(IIC_DEVICE_NAME, O_RDWR | O_SYNC)) == -1)
  {
    printf("Failed to open iic device\n");
    return -1;
  }
  pIic = (IIC*)mmap(0, sizeof(IIC), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, iic_file, 0);
  if (pIic == MAP_FAILED)
  {
    printf("Failed to map iic device\n");
    return -1;
  }
  printf("IIC device is ready\n");

  //Setup IIC to read 2 packets
  IicDat.Port = IIC_PORT;
  IicDat.Time = 0;
  IicDat.Repeat = 0;
  IicDat.RdLng = 2;
  IicDat.WrLng = 2;
  // Set the device I2C address
  IicDat.WrData[0] = 0x01;
  // Specify the register that will be read (0x42 = angle/distance)
  IicDat.WrData[1] = 0x42;
  // Setup I2C communication
  ioctl(iic_file, IIC_SETUP, &IicDat);
  
  // from http://python-ev3.org/types.html
  //  Type  Mode  Name      DataSets  Format  Figures  Decimals  Views  Conn. Pins  RawMin   RawMax   PctMin  PctMax  SiMin    SiMax    Time  IdValue  Symbol  
  //  4     0     NXT-COL-REF   1       2     5        0         5      119   0x0E    200.0   1500.0       0     100      0.0    100.0   300        0  pct
  //  4     1     NXT-COL-AMB   1       2     5        0         5      119   0x11    200.0   2900.0       0     100      0.0    100.0   300        0  pct
  //  4     2     NXT-COL-COL   1       0     2        0         5      119   0x0D      0.0      8.0       0     100      0.0      8.0   300        0  col
  //  29    0     COL-REFLECT   1       0     3        0         3      122   0x2D      0.0    100.0       0     100      0.0    100.0    10        0  pct
  //  29    1     COL-AMBIENT   1       0     3        0         3      122   0x2D      0.0    100.0       0     100      0.0    100.0    10        0  pct
  //  29    2     COL-COLOR     1       0     2        0         3      122   0x2D      0.0      8.0       0     100      0.0      8.0    10        0  col
  
  //  Type  Mode  Name      DataSets  Format  Figures  Decimals  Views  Conn. Pins  RawMin   RawMax   PctMin  PctMax  SiMin    SiMax    Time  IdValue  Symbol  
  //  30    0     US-DIST-CM    1       1     5        1         3      122   0x2D      0.0   2550.0       0     100      0.0    255.0    10        0  cm
  //  30    1     US-DIST-IN    1       1     5        1         3      122   0x2D      0.0   1000.0       0     100      0.0    100.0    10        0  inch
  //  30    2     US-LISTEN     1       0     1        0         3      122   0x2D      0.0      1.0       0     100      0.0      1.0    10        0  _
  DEVCON DevCon;
  memset(&DevCon, 0, sizeof(DEVCON));
  
  DevCon.Type[ROBOT_COLOR_SENSOR_PORT] = 29; // TYPE_NXT_COLOR, but ignored on EV3;
  DevCon.Mode[ROBOT_COLOR_SENSOR_PORT] = ROBOT_COLOR_SENSOR_MODE;
  DevCon.Connection[ROBOT_COLOR_SENSOR_PORT] = CONN_INPUT_UART;
  
  DevCon.Type[ROBOT_INFRARED_SENSOR_PORT] = 33;
  DevCon.Mode[ROBOT_INFRARED_SENSOR_PORT] = ROBOT_INFRARED_SENSOR_MODE;
  DevCon.Connection[ROBOT_INFRARED_SENSOR_PORT] = CONN_INPUT_UART;
  
  DevCon.Type[ROBOT_ULTRASONIC_SENSOR_PORT] = 30;
  DevCon.Mode[ROBOT_ULTRASONIC_SENSOR_PORT] = ROBOT_ULTRASONIC_SENSOR_MODE;
  DevCon.Connection[ROBOT_ULTRASONIC_SENSOR_PORT] = CONN_INPUT_UART;
  
  ioctl(uart_file, UART_SET_CONN, &DevCon);
  
  pthread_create(&thread, NULL, check_button_thread_func, (void*) keep_running);
  
  return 0;
  
}

int sensors_terminate(bool* keep_running)
{
  // Make sure thread stopped before closing io mapped memory
  *keep_running = 0;
  pthread_join(thread, NULL);
  
  // Close the device files
  printf("Closing analog device\n");
  if (pAnalog && pAnalog != MAP_FAILED)
    munmap(pAnalog, sizeof(ANALOG));
  close(ad_file);
  
  printf("Closing uart device\n");
  if (pUart && pUart != MAP_FAILED)
    munmap(pUart, sizeof(UART));
  close(uart_file);
  
  printf("Closing ui device\n");
  if (pUI && pUI != MAP_FAILED)
    munmap(pUI, sizeof(UI)*vmOUTPUTS);
  close(ui_file);
  
  printf("Closing iic device\n");
  if (pIic && pIic != MAP_FAILED)
    munmap(pIic, sizeof(IIC));
  close(iic_file);
  
  return 0;
}

UWORD sensors_get_touched(int port)
{
  // The ports are designated as PORT_NUMBER-1
  return (UWORD) (pAnalog->Pin6[port][pAnalog->Actual[port]] > 256);
}

UWORD sensors_get_ir_distance(int port)
{
  // The ports are designated as PORT_NUMBER-1
  return pUart->Raw[port][pUart->Actual[port]][0];
}

UWORD sensors_get_us_distance_mm(int port)
{
  // Works for new EV3 ultrasone sensor.
  
  // The ports are designated as PORT_NUMBER-1
  // Value in tenth of millimeter
  return (unsigned char) pUart->Raw[port][pUart->Actual[port]][1]*256 + pUart->Raw[port][pUart->Actual[port]][0];
}

UWORD sensors_get_ul_distance(int port)
{
  // Works for old NXT ultrasonic sensor.
  
  // The ports are designated as PORT_NUMBER-1
  return (unsigned char) pIic->Raw[port][pIic->Actual[port]][0]; //*256 + pIic->Raw[port][pIic->Actual[port]][1];
}

int sensors_is_button_pressed(int button)
{
  return button_pressed_count[button] > 0;
}

void sensors_clear_buttons_pressed()
{
  for (int i=0; i<BUTTONS; i++) button_pressed_count[i] = 0;
}

UWORD sensors_get_color(int port)
{
  return pUart->Raw[port][pUart->Actual[port]][0];
}

