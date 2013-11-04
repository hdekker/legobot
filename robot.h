#ifndef ROBOT_H_
#define ROBOT_H_

#define PI 3.1415
#define DEG_TO_RAD(x)    ((float)x * (PI / 180.0))

#define ROBOT_SPEED 20

// Sensors
#define ROBOT_DISTANCE_SENSOR_PORT 3

// Actuators
#define ROBOT_WHEEL_LEFT_PORT   0 // A
#define ROBOT_WHEEL_RIGHT_PORT  3 // D
//#define ROBOT_BALL_THROW_MOTOR_PORT 2 // C
#define ROBOT_RADAR_MOTOR_PORT  1 // B

#define ROBOT_RADAR_SWING  200
#define ROBOT_RADAR_ROTATION 90      // mounted radar angle 90 degrees
#define ROBOT_RADAR_DIRECTION (-1)   // clockwise is negative angle
#define ROBOT_RADAR_SPEED  5

#define ROBOT_WHEEL_DISTANCE_X   0.18   // SI meter
#define ROBOT_WHEEL_DIAMETER     0.042  // SI meter
#define ROBOT_WHEEL_CIRCUMFERENCE  (ROBOT_WHEEL_DIAMETER * PI)

#define ROBOT_MAP_DIMENSION 2000
#define ROBOT_MAP_OFFSET    (ROBOT_MAP_DIMENSION / 2)

#endif /* ROBOT_H_ */
