
#ifndef SENSORS_H_
#define SENSORS_H_

#include <lmstypes.h>

#define IIC_PORT 0x2;

//typedef enum 
//{
 // SENSORS_NXT_COL_REF = 0,
 // SENSORS_NXT_COL_AMB = 1,
 // SENSORS_NXT_COL_COL = 2,
  //SENSORS_NXT_COL_GRN = 3,
  //SENSORS_NXT_COL_BLU = 4,
  //SENSORS_NXT_COL_RAW = 5,
//} SENSORS_NXT_COL;

int sensors_initialize();
int sensors_terminate();

//int sensors_set_color_mode(int port, SENSORS_NXT_COL mode);

UWORD sensors_get_touched(int port);
UWORD sensors_get_ir_distance(int port);
UWORD sensors_get_ul_distance(int port); // NXT ultrasonic sensor
UWORD sensors_get_us_distance_mm(int port); // EV3 ultrasonic sensor
UWORD sensors_get_color(int port);
DATA8 sensors_is_button_pressed(int button);

#endif /* SENSORS_H_ */
