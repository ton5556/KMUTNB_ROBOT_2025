#include "MultiTimer.h"
#include "stdio.h"


static Timers *pTimerList = NULL;
static PlatformTicksFunction_t platFormTicksFunction = NULL;


int ticksFuncSet(PlatformTicksFunction_t ticksFunc)
{
  platFormTicksFunction = ticksFunc;
  if(platFormTicksFunction == NULL)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}

int timerStart(Timers *ptimer, uint32_t timing, TimersCallback_t timersCallBack, const void *userdata)
{
  if (!ptimer || !timersCallBack)
  {
    // _LOG("Timer initalize error.");
    return -1;    
  }
  Timers **pNextTimer = &pTimerList;
  for (; *pNextTimer; pNextTimer = &(*pNextTimer)->next)
  {
    if(ptimer == *pNextTimer)
    {
      *pNextTimer = ptimer->next;
      break;
    }
  }

  ptimer->deadline = platFormTicksFunction() + timing;
  ptimer->timersCallBack = timersCallBack;
  ptimer->userdata = userdata;
  for (pNextTimer = &pTimerList;; pNextTimer = &(*pNextTimer)->next)
  {
    if (!*pNextTimer)
    {
      ptimer->next = NULL;
      *pNextTimer = ptimer;
      break;
    }
    if (ptimer->deadline < (*pNextTimer)->deadline)
    {
      ptimer->next = *pNextTimer;
      *pNextTimer = ptimer;
      break;
    }
  }
  return 1;
}

int timerStop(Timers *ptimer)
{
  Timers **pNextTimer = &pTimerList;
  for (; *pNextTimer; pNextTimer = &(*pNextTimer)->next)
  {
    Timers *entry = pTimerList;
    if(entry == ptimer)
    {
      *pNextTimer = ptimer->next;
      break;
    }
  }
  return 1;
}

int timersTaskRunning(void)
{
  Timers *entry = pTimerList;
  for (; entry; entry = entry->next)
  {
    if (platFormTicksFunction() < entry->deadline)
    {
      return (int)(entry->deadline - platFormTicksFunction());
    }
    pTimerList = entry->next;
    if(entry->timersCallBack)
    {
      entry->timersCallBack(entry, entry->userdata);
    }
  }
  return 1;
}