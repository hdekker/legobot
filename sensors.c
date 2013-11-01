#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "lms2012.h"
#include "sensors.h"

static int ad_file;
static ANALOG *pAnalog;

static int uart_file;
static UART *pUart;

static int ui_file;
static UI *pUI;

static int iic_file;
static IIC *pIic;
static IICDAT IicDat;
  
int sensors_initialize()
{
  //Open the device file
  if((ad_file = open(ANALOG_DEVICE_NAME, O_RDWR | O_SYNC)) == -1)
  {
    printf("Failed to open analogue device\n");
    return -1;
  }
  pAnalog = (ANALOG*)mmap(0, sizeof(ANALOG), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, ad_file, 0);
  if (pAnalog == MAP_FAILED)
  {
    printf("Failed to map analogue device\n");
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
  
  return 0;
}

int sensors_terminate()
{
  // Close the device files
  printf("Closing ad device\n");
  close(ad_file);
  printf("Closing uart device\n");
  close(uart_file);
  printf("Closing ui device\n");
  close(ui_file);
  printf("Closing iic device\n");
  close(iic_file);
  return 0;
}

int sensors_set_color_mode(int port, SENSORS_NXT_COL mode)
{
  // from http://python-ev3.org/types.html
  //  Type  Mode  Name      DataSets  Format  Figures  Decimals  Views  Conn. Pins  RawMin   RawMax   PctMin  PctMax  SiMin    SiMax    Time  IdValue  Symbol  
  //  4     0     NXT-COL-REF   1       2     5        0         5      119   0x0E    200.0   1500.0       0     100      0.0    100.0   300        0  pct
  //  4     1     NXT-COL-AMB   1       2     5        0         5      119   0x11    200.0   2900.0       0     100      0.0    100.0   300        0  pct
  //  4     2     NXT-COL-COL   1       0     2        0         5      119   0x0D      0.0      8.0       0     100      0.0      8.0   300        0  col
  //  4     3     NXT-COL-GRN   1       2     5        0         5      119   0x0F    200.0   1500.0       0     100      0.0    100.0   300        0  pct
  //  4     4     NXT-COL-BLU   1       2     5        0         5      119   0x10    200.0   1500.0       0     100      0.0    100.0   300        0  pct
  //  4     5     NXT-COL-RAW 
  DEVCON DevCon;
  DevCon.Type[port] = TYPE_NXT_COLOR;
  DevCon.Mode[port] = mode;
  DevCon.Connection[port] = CONN_INPUT_UART;
  ioctl(uart_file, UART_SET_CONN, &DevCon);
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

UWORD sensors_get_ul_distance(int port)
{
  // The ports are designated as PORT_NUMBER-1
  return (unsigned char) pIic->Raw[port][pIic->Actual[port]][0]; //*256 + pIic->Raw[port][pIic->Actual[port]][1];
}

DATA8 sensors_is_button_pressed(int button)
{
  return pUI->Pressed[button];
}

UWORD sensors_get_color(int port)
{
  return pUart->Raw[port][pUart->Actual[port]][0];
}

