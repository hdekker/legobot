
#ifndef SCREEN_H_
#define SCREEN_H_

#define SCREEN_WIDTH 256
#define SCREEN_HEIGTH 128

void screen_initialize();
void screen_terminate();

void screen_clear();
void screen_draw_pixel(int x, int y);
void screen_clear_pixel(int x, int y);
void screen_draw_circle(int x, int y, int r);

#endif /* SCREEN_H_ */
