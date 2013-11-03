#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "robot.h"
#include "navi.h"

static int previous_left_angle = 0;
static int previous_right_angle = 0;


static NAVI_pos pos;
unsigned short* p_map;

void navi_initialize()
{
  pos.x = 0.0;
  pos.y = 0.0;
  pos.a = 0.0;
  
  // unsigned short map, 4x4 pixel per allocated word
  int words = (ROBOT_MAP_DIMENSION * ROBOT_MAP_DIMENSION) / 16;
  p_map = malloc(words);
  if (p_map == NULL)
  {
    printf("Failed to allocate map\n");
    exit(-1);
  }
}

void navi_terminate()
{
  free(p_map);
}

NAVI_pos navi_get_pos()
{
  return pos;
}

void navi_update_pos(int left_angle, int right_angle)
{
  // Source: http://rossum.sourceforge.net/papers/DiffSteer/
  // The forumulas in [6] are simple and convenient. They work well for algorithms as 
  // long as you remember that they are approximations... a fact that is sometimes
  // overlooked. Essentially, [6] computes the robot's total rotation and distance 
  // traveled, then treats it as if it had completed the full rotation as a pivot the very
  // beginning of the maneuver and then traveled in a straight line. Of course, this kind
  // of "point-and-shoot" description is an incomplete representation of the reality. But 
  // so long as the robot's actual path doesn't involve too much of a turn, everything is fine.

  float delta_l = left_angle - previous_left_angle;
  float delta_r = right_angle - previous_right_angle;
  previous_left_angle = left_angle;
  previous_right_angle = right_angle;
  
  float sl = (delta_l * ROBOT_WHEEL_CIRCUMFERENCE) / 360.0f;
  float sr = (delta_r * ROBOT_WHEEL_CIRCUMFERENCE) / 360.0f;
  float sm = (sr + sl) / 2.0f;
  pos.a = ((sr - sl) / (2.0f * ROBOT_WHEEL_DISTANCE_X)) + pos.a;
  pos.x = (sm * cos(pos.a)) + pos.x;
  pos.y = (sm * sin(pos.a)) + pos.y;
  
  //printf("navi_update_pos l=%d, r=%d, x=%.3f, y=%.3f, a=%.3f\n", left_angle, right_angle, pos.x, pos.y, pos.a);
}


void navi_update_map(int radar_angle_deg, int distance_cm, int left_angle, int right_angle)
{
  navi_update_pos(left_angle, right_angle);
  // Don't look further than 50 cm at the moment, because of ultrasonic wide beam
  if ((distance_cm > 5) && (distance_cm < 60))
  {
    float a = pos.a - DEG_TO_RAD(radar_angle_deg);
    float dx = cos(a) * (float) distance_cm / 100.0f;
    float dy = sin(a) * (float) distance_cm / 100.0f;
    float x = dx + pos.x;  
    float y = dy + pos.y;
    printf("pos l=%d, r=%d, x=%.2f, y=%.2f, a=%.2f\n", left_angle, right_angle, pos.x, pos.y, pos.a);
    printf("map a=%d, d=%.2f, dx=%.2f, dy=%.2f, x=%.2f, y=%.2f\n\n", radar_angle_deg, ((float)distance_cm/100.0f), dx, dy, x, y);
  }
}

