
#ifndef MOTORS_H_
#define MOTORS_H_

#include <lmstypes.h>

int motors_initialize();
int motors_terminate();

SBYTE motors_get_motor_speed(int port);
SLONG motors_get_angle(int port);

void motors_reset_all();
void motors_stop_all();
int motors_step_speed(int port, int speed, int step1, int step2, int step3);

#endif /* MOTORS_H_ */
