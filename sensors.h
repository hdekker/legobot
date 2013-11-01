
#ifndef SENSORS_H_
#define SENSORS_H_

#include <lmstypes.h>

#define IIC_PORT 0x2;

int sensors_initialize();
int sensors_terminate();

UWORD sensors_get_touched(int port);
UWORD sensors_get_ir_distance(int port);
UWORD sensors_get_ul_distance(int port);
UWORD sensors_get_color(int port);
DATA8 sensors_is_button_pressed(int button);

#endif /* SENSORS_H_ */
