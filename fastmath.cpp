#include <cmath>
#include <climits>
#include <iostream>
#include "fastmath.h"

#define MINMAX(n,limit) (((n)>(limit)) ? (limit) : ((n)<(-limit) ? -limit : n))

int16_t sin_lut[UINT16_H_PI];
int gauss_lut[5] = { 1000, 11750, 39347, 67535, 86466 };


int fastmath_gaussrandom(int mean, int dev)
{
  int rnd = rand();
  int base = gauss_lut[rnd % (sizeof(gauss_lut)/sizeof(gauss_lut[0]))];
  int var = dev * ((rnd%2)?(rnd%base):-(rnd%base)) / 31415;
  return mean + var;
}

// Fast functions
int fastmath_sin(int angle)
{
  return sin_lut[static_cast<uint16_t>(angle) >> 2];
}

int fastmath_cos(int angle)
{
  return sin_lut[static_cast<uint16_t>(angle + UINT16_H_PI) >> 2];
}

int fastmath_tan(int angle)
{
  int s = fastmath_sin(angle);
  int c = fastmath_cos(angle);
  return (c) ? MINMAX((s << TRI_SHIFT)/c, MAX_TAN_VALUE) : ((s>0) ? MAX_TAN_VALUE : -MAX_TAN_VALUE);
}

uint32_t fastmath_sqrt(uint32_t x)  
{  
  uint32_t c = 0x8000;  
  uint32_t g = 0x8000;  

  for(;;) {  
    if(g*g > x) g ^= c;  
    c >>= 1;  
    if(c == 0) return g;  
    g |= c;  
  }  
}  

/*    
uint32_t fastmath_sqrt(uint32_t x)
{
  uint32_t res = 0;
  uint32_t add = 0x8000;   

  for(int i=0; i<16; i++)
  {
    uint32_t temp = res | add;
    uint32_t g2 = temp * temp;      
    if (x >= g2)
    {
      res = temp;           
    }
    add >>= 1;
  }
  return res;
}
*/

// Returns angle 0 .. 65535
int fastmath_atan2(int y, int x)
{
  int result = 0;
  int y2;

  if ((x | y) == 0) return 0;
  if (y < 0) { result += -32768; y = -y, x = -x; }
  if (x < 0) { result += 16384; y2 = y; y = -x; x = y2; }
  if (y > x) { result += 8192; y2 = y; y -= x; x += y2; }
  if (2 * y > x)
  {
    result += 4836; /* atan(1/2) * 32768/M_PI */
    y2 = y;
    y = 2 * y - x;
    x = 2 * x + y2;
  }
#if 1
  if (4 * y > x)
  {
    result += 2555; /* atan(1/4) * 32768/M_PI */
    y2 = y;
    y = 4 * y - x;
    x = 4 * x + y2;
  }
#if 1
  if (8 * y > x)
  {
    result += 1297; /* atan(1/8) * 32768/M_PI */
    y2 = y;
    y = 8 * y - x;
    x = 8 * x + y2;
  }
#if 0
  if (16 * y > x)
  {
    result += 651;  /* atan(1/16) * 32768/M_PI */
    y2 = y;
    y = 16 * y - x;
    x = 16 * x + y2;
  }
#if 0
  if (32 * y > x)
  {
    result += 326;  /* atan(1/32) * 32768/M_PI */
    y2 = y;
    y = 32 * y - x;
    x = 32 * x + y2;
  }
  return POS_DEGREES(result + 10427 * y / x);  /* up to atan(1/32) */
#else
  return POS_DEGREES(result + 10417 * y / x);  /* up to atan(1/16) */
#endif
#else
  return POS_DEGREES(result + 10377 * y / x);  /* up to atan(1/8) */
#endif
#else
  return POS_DEGREES(result + 10221 * y / x);  /* up to atan(1/4) */
#endif
#else
  return POS_DEGREES(result + 9672 * y / x);   /* up to atan(1/2) */
#endif
}
    
int fastmath_log2(uint32_t x)
{
  int ret = 0;
  while (x != 0) {
    x >>= 1;
    ret++;
  }
  return ret;
}

void fastmath_init()
{
  for (int i=0; i<UINT16_H_PI; i++)
  {
    sin_lut[i] = static_cast<int16_t>(sin((i * 2 * 3.14159265358979) / UINT16_H_PI) * (1<<TRI_SHIFT));
  }
  
  /*

  // Test fast functions
  std::cout << "fastmath_sin[0]" << fastmath_sin(0) << "\n";
  std::cout << "fastmath_sin[16384]" << fastmath_sin(16384) << "\n";
  std::cout << "fastmath_sin[32768]" << fastmath_sin(32768) << "\n";
  std::cout << "fastmath_sin[49152]" << fastmath_sin(49152) << "\n";
  std::cout << "fastmath_sin[65536]" << fastmath_sin(65536) << "\n";
  
  std::cout << "fastmath_cos[0]" << fastmath_cos(0) << "\n";
  std::cout << "fastmath_cos[16384]" << fastmath_cos(16384) << "\n";
  std::cout << "fastmath_cos[32768]" << fastmath_cos(32768) << "\n";
  std::cout << "fastmath_cos[49152]" << fastmath_cos(49152) << "\n";
  std::cout << "fastmath_cos[65536]" << fastmath_cos(65536) << "\n";
  
  std::cout << "fastmath_sqrt[0]" << fastmath_sqrt(0) << "\n";
  std::cout << "fastmath_sqrt[1]" << fastmath_sqrt(1) << "\n";
  std::cout << "fastmath_sqrt[4]" << fastmath_sqrt(4) << "\n";
  std::cout << "fastmath_sqrt[9]" << fastmath_sqrt(9) << "\n";
  std::cout << "fastmath_sqrt[16]" << fastmath_sqrt(16) << "\n";
  
  for (int i=-USHRT_MAX; i<USHRT_MAX*2; i+=1000)
  {
    std::cout << "fastmath_sin(" << i << ")" << fastmath_sin(i) << "\n";
  }
  
  for (int i=-USHRT_MAX; i<USHRT_MAX*2; i+=1000)
  {
    std::cout << "fastmath_cos(" << i << ")" << fastmath_cos(i) << "\n";
  }

  for (int i=-USHRT_MAX; i<USHRT_MAX*2; i+=1000)
  {
    std::cout << "fastmath_sqrt(" << i << ")" << fastmath_sqrt(i) << "\n";
  }
  
  std::cout << "\n\n\n";
  for (int y=-120; y<=120; y+=30)
    for (int x=-120; x<=120; x+=30)
      std::cout << "fastmath_atan2(" << y << ", " << x << ")" << fastmath_atan2(y, x) << "\n";
  for (int i=0; i<100; i++)
  {
    std::cout << "fastmath_gaussrandom(100,10)=" << fastmath_gaussrandom(100, 10) << "\n";
  }
  for (int i=0; i<100; i++)
  {
    std::cout << "fastmath_gaussrandom(500,100)=" << fastmath_gaussrandom(500, 100) << "\n";
  }
  for (int i=0; i<100; i++)
  {
    std::cout << "fastmath_gaussrandom(0,1000)=" << fastmath_gaussrandom(0, 1000) << "\n";
  }
  exit(0);
  */
}
