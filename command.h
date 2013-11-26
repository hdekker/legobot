#ifndef COMMAND_H_
#define COMMAND_H_

void command_initialize();
void command_terminate();

int command_get_forward_distance_mm();
int command_get_right_distance_mm();

void command_move_forward();
void command_move_stop();
void command_move_distance(int distance_mm);
void command_turn_angle(int angle_deg);
void command_turn_left();
void command_turn_right();



#endif /* COMMAND_H_ */
