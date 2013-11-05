#include <time.h>
#include <stdio.h>

void sleep_ms(unsigned milliseconds)
{
  struct timespec t;
  struct timespec tr;
  t.tv_sec = milliseconds / 1000u;
  t.tv_nsec = (milliseconds % 1000u) * 1e6;
  tr.tv_sec = 0;
  tr.tv_nsec = 0;
  do
  {
    t.tv_sec -= tr.tv_sec;
    t.tv_nsec -= tr.tv_nsec;
    if ((tr.tv_sec != 0) || (tr.tv_nsec != 0))
    {
      ;//printf("yet another nap... sec=%d, nsec=%d\n", (int) t.tv_sec, (int) t.tv_nsec);
    }
    nanosleep(&t, &tr);
  } while ((tr.tv_sec != 0) && (tr.tv_nsec != 0));
}
