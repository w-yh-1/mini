#ifndef SELECT_DELAY
#define SELECT_DELAY
#include <stdio.h>    // 引入 NULL
#include <sys/time.h> // 引入 struct timeval

// 获取系统tick可以作为us时长参考
static long int getTickUs(void)
{
    struct timeval tv = {0};
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000u + tv.tv_usec;
}

static void delayUs(long int us)
{
    struct timeval tv;
    tv.tv_sec = us / 1000000;
    tv.tv_usec = us % 1000000;
    select(0, NULL, NULL, NULL, &tv);
}

// 延时初始化
#define DELAY_INIT \
    static long int _tick = 0, _tickErr;
// 每次延时一半时差,直至延时满us或差值小于err,注意要放在循环开始
#define DELAY_US(us, err)              \
    __next_turn__:                     \
    _tickErr = getTickUs() - _tick;    \
    if (_tickErr > 0 && us > _tickErr) \
    {                                  \
        _tickErr = us - _tickErr;      \
        if (_tickErr > err)            \
        {                              \
            delayUs(_tickErr / 2);     \
            goto __next_turn__;        \
        }                              \
    }                                  \
    _tick = getTickUs();
#endif