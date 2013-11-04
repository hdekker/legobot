#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include "lms2012.h"
#include "screen.h"

static int line_length;
static int width = 0;
static int height = 0;
static int bpp = 0;
static char *buffer = NULL;
static size_t buflen;
static int fd = -1;
   
void screen_initialize()
{
  struct fb_var_screeninfo screen_info;
  struct fb_fix_screeninfo fixed_info;
  
  fd = open(LCD_DEVICE_NAME, O_RDWR);
  if (fd >= 0)
  {
    if (!ioctl(fd, FBIOGET_VSCREENINFO, &screen_info) &&
        !ioctl(fd, FBIOGET_FSCREENINFO, &fixed_info))
    {
      height = screen_info.yres_virtual;
      bpp = screen_info.bits_per_pixel;
      line_length = fixed_info.line_length;
      width = fixed_info.line_length * 8 / bpp;
      printf("resolution: w=%d, h=%d, bpp=%d\n", width, height, bpp);
      buflen = height * line_length;
      buffer = mmap(NULL, buflen, PROT_READ|PROT_WRITE,
                    MAP_SHARED, fd, 0);
      if (buffer == MAP_FAILED)
      {
        perror("mmap");
      }
    }
    else
    {
      perror("ioctl");
    }
  }
  else
  {
    perror("open");
  } 
}

void screen_terminate()
{
  if (buffer && buffer != MAP_FAILED)
    munmap(buffer, buflen);
  if (fd >= 0)
    close(fd);
}

void screen_clear()
{
  memset(buffer, 0, buflen);
  msync(buffer, buflen, MS_ASYNC);
}

void screen_draw_circle(int x, int y, int r)
{
  //printf("draw x=%d, y=%d\n", x, y);
  if ((x >=r) && (x < width-r) && (y >= r) && (y < height-r))
  {
    int xx, yy;
    
    for (xx=x-r; xx<x+r+1; xx++)
    {
      for (yy=y-r; yy<y+r+1; yy++)
      {
        screen_draw_pixel(xx, yy);
      }
    }
    msync(buffer, buflen, MS_ASYNC);
  }
  else
  {
    printf("Failed to write circle x=%d, y=%d, r=%d\n", x, y, r);
  }
}

void screen_draw_pixel(int x, int y)
{
  //printf("draw x=%d, y=%d\n", x, y);
  if ((x >=0) && (x < width) && (y >= 0) && (y < height))
  {
    char *p_old = buffer + x * bpp / 8 + y * line_length;
    int move = (x % 4) * 2;
    //printf("move=%d old=%02x OR char=%02x\n", move, *p_old, (char)0xC0>>move);
    *p_old |= (char)0xC0>>move;
    //msync(buffer, buflen, MS_ASYNC);
  }
  else
  {
    printf("Failed to write pixel to x=%d, y=%d\n", x, y);
  }
}

void screen_clear_pixel(int x, int y)
{
  //printf("clear x=%d, y=%d\n", x, y);
  if ((x >=0) && (x < width*8) && (y >= 0) && (y < height))
  {
    char *p_old = buffer + x * bpp/8 + y * line_length;
    int move = (x % 4) * 2;
    //printf("move=%d old=%02x AND char=%02x\n", move, *p_old, (char)((unsigned)0xFF3F>>move));
    *p_old &= (char)((unsigned)0xFF3F>>move);
    //msync(buffer, buflen, MS_ASYNC);
  }
  else
  {
    printf("Failed to write pixel to x=%d, y=%d\n", x, y);
  }
}

void test()
{
  int i, j;
  
  for (j=0; j<1; j++)
  {
    for (i=0; i<4; i++)
    {
      screen_draw_pixel(120+i, 60);
      sleep(1);
    }
    for (i=0; i<4; i++)
    {
      screen_clear_pixel(120+i, 60);
      sleep(1);
    }
  }
  for(i = 0;i<120;i++) screen_draw_pixel(i, i);
  for(i = 120;i<240;i++) screen_draw_pixel(i, 240-i);
  sleep(5);
}