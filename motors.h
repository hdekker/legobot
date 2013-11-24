
#ifndef MOTORS_H_
#define MOTORS_H_

#include <lmstypes.h>

int motors_initialize();
int motors_terminate();

SBYTE motors_get_motor_speed(int port);
SLONG motors_get_angle(int port);

void motors_reset_all();
void motors_reset_angle(int port);
void motors_stop_all();
void motors_stop(int port);
void motors_set_speed(int port, int speed);
void motors_step_speed(int port, int speed, int step1, int step2, int step3);

void motors_move_to_angle(int port, int speed, int angle_deg, int maxturns);
void motors_start_move_to_angle(int port, int speed, int angle_deg, int maxturns);
void motors_wait_move_to_angle(int port);

#endif /* MOTORS_H_ */
