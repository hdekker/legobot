#ifndef NAVI_H_
#define NAVI_H_

typedef struct
{
  float x;
  float y;
  float a;
} NAVI_pos;

void navi_initialize();
void navi_terminate();


NAVI_pos navi_get_pos();
void navi_update_map(int radar_angle, int distance, int left_angle, int right_angle);

#endif /* NAVI_H_ */
