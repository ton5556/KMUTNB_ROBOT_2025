#ifndef _MULTITIMER_H_
#define _MULTITIMER_H_

#include "Arduino.h"

typedef uint32_t (*PlatformTicksFunction_t)(void);

typedef struct TimersHandle Timers;

typedef void (*TimersCallback_t)(Timers *ptimer, const void *userdata);

struct TimersHandle {
    TimersHandle *next;
    uint32_t deadline;
    TimersCallback_t timersCallBack;
    const void* userdata;
};

/**
 * @brief 设置基准定时(set the base timer)
 * @param ticksFunc 设置定时器(set the timer)
 * @return uint8_t 0: fail, 1: success.
 */
int ticksFuncSet(PlatformTicksFunction_t ticksFunc);

/**
 * @brief 定时器初始化(initialize timer)
 * @param timer 设置目标定时器(set target timer)
 * @param time 设置开始的时间(set time for starting)
 * @param timersCallBack 设置定时器回调函数(set timer callback function)
 * @param 
 * @return None
 */
// void timerInit(void);

/**
 * @brief 定时器开始工作(timer starts to run)
 * @param timer 设置目标定时器(set target timer)
 * @param time 设置开始的时间(set time to start)
 * @param timersCallBack 设置定时器回调函数(set timer callback function)
 * @param 
 * @return uint8_t 0: fail, 1: success.
 */
int timerStart(Timers *ptimer, uint32_t timing, TimersCallback_t timersCallBack, const void *userdata);

/**
 * @brief 定时器停止工作(timer stops running)
 * @param timer 设置目标定时器(set target timer)
 * @return uint8_t 0: fail, 1: success.
 */
int timerStop(Timers *ptimer);

/**
 * @brief 定时器任务运行(timer task running)
 * @param None
 * @return None
 */
int timersTaskRunning(void);

#endif
