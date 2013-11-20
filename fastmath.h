#ifndef FASTMATH_H_
#define FASTMATH_H_

#include <climits>

#define TRI_SHIFT         (12)                     // Shift (multiplier) used in trigonometric functions
#define MAX_TAN_VALUE     (2048<<TRI_SHIFT)        // Maximum value that tangent function will return
#define UINT16_2_PI       (USHRT_MAX)              // Number of degrees in circle    (360 degrees on 360 degree scale)
#define UINT16_PI         (UINT16_2_PI/2)          // Number of degrees in PI        (180 degrees on 360 degree scale)
#define UINT16_H_PI       (UINT16_2_PI/4)          // Number of degrees in half PI   (90 degrees on 360 degree scale)
#define UINT16_Q_PI       (UINT16_2_PI/8)          // Number of degrees in fourth PI (45 degrees on 360 degree scale)
#define UINT16_S_PI       (UINT16_2_PI/12)         // Number of degrees in sixth PI  (30 degrees on 360 degree scale)
#define DEG2UINT16(a)     (a * UINT16_2_PI / 360)  // Conversion from degrees on 360 scale to degrees on 65536 scale
#define UINT162DEG(a)     (a * 360 / UINT16_2_PI)  // Conversion from degrees on 65536 scale to degrees on 360 scale
#define POS_DEGREES(a)    ((a + UINT16_2_PI) % UINT16_2_PI)

void fastmath_init();
int fastmath_sin(int angle);
int fastmath_cos(int angle);
int fastmath_tan(int angle); // Returns divider times UINT16_PI
int fastmath_atan2(int y, int x);
uint32_t fastmath_sqrt(uint32_t x);
int fastmath_log2(uint32_t x);
int fastmath_gaussrandom(int mean, int dev);

#endif /* FASTMATH_H_ */