#ifndef ROBOT_H_
#define ROBOT_H_

#define PI 3.1415
#define DEG_TO_RAD(x)    ((float)x * (PI / 180.0))

#define ROBOT_SPEED 30

// Sensors
#define ROBOT_COLOR_SENSOR_PORT 3
#define ROBOT_COLOR_SENSOR_MODE 2

#define ROBOT_INFRARED_SENSOR_PORT 2
#define ROBOT_INFRARED_SENSOR_MODE 0

#define ROBOT_ULTRASONIC_SENSOR_PORT 1
#define ROBOT_ULTRASONIC_SENSOR_MODE 0

// Actuators
#define ROBOT_WHEEL_RIGHT_PORT 0 // A
#define ROBOT_BALL_CATCH_PORT  1 // B
#define ROBOT_BALL_THROW_PORT  2 // C
#define ROBOT_WHEEL_LEFT_PORT  3 // D

#define ROBOT_RADAR_SWING  200
#define ROBOT_RADAR_ROTATION 90      // mounted radar angle 90 degrees
#define ROBOT_RADAR_DIRECTION (-1)   // clockwise is negative angle
#define ROBOT_RADAR_SPEED  5

#define ROBOT_WHEEL_DISTANCE_X   0.24   // SI meter
#define ROBOT_WHEEL_DIAMETER     0.036  // SI meter
#define ROBOT_WHEEL_CIRCUMFERENCE  (ROBOT_WHEEL_DIAMETER * PI)

#define ROBOT_MAP_DIMENSION 2000
#define ROBOT_MAP_OFFSET    (ROBOT_MAP_DIMENSION / 2)

#endif /* ROBOT_H_ */
