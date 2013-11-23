
#ifndef SENSORS_H_
#define SENSORS_H_

#include <lmstypes.h>

#define IIC_PORT 0x2;

typedef enum 
{
 BUTTON_CODE_UP = 0,
 BUTTON_CODE_OK = 1,
 BUTTON_CODE_DOWN = 2,
 BUTTON_CODE_RIGHT = 3,
 BUTTON_CODE_LEFT = 4,
 BUTTON_CODE_ESC = 5,
} BUTTON_CODE;

int sensors_initialize(bool* keep_running);
int sensors_terminate(bool* keep_running);

//int sensors_set_color_mode(int port, SENSORS_NXT_COL mode);

UWORD sensors_get_touched(int port);
UWORD sensors_get_ir_distance(int port);
UWORD sensors_get_ul_distance(int port); // NXT ultrasonic sensor
UWORD sensors_get_us_distance_mm(int port); // EV3 ultrasonic sensor
UWORD sensors_get_color(int port);


int sensors_is_button_pressed(int button);
void sensors_clear_buttons_pressed();

#endif /* SENSORS_H_ */
