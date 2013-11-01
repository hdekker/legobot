
#ifndef MOTORS_H_
#define MOTORS_H_

#include <lmstypes.h>

int motors_initialize();
int motors_terminate();

SBYTE motors_get_motor_speed(int port);
SLONG motors_get_motor_tacho(int port);

#endif /* MOTORS_H_ */
