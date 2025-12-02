#include "MultiTimer.h"
#include "stdio.h"


static Timers *pTimerList = NULL;
//记录各个定时器的定时器列表(Record the timer list of each timer)
//各个定时器的排布/存放原则：按各个定时器下一次被触发的系统时间排布（越早被触发的越靠前，使得可以被系统及时遍历到，处理其触发事件）(Principle of arrangement/storage of each timer: arranged according to the next system time each timer will be triggered (the earlier the trigger, the closer to the front, so that it can be traversed by the system in time and its trigger event can be processed))
static PlatformTicksFunction_t platFormTicksFunction = NULL;

//将用户传入的函数ticksFunc与我们自定义的platFormTicksFunction函数绑定(Bind the user-provided function ticksFunc with our custom platFormTicksFunction function)
//每次执行platFormTicksFunction，等效于执行ticksFunc(Each time platFormTicksFunction is executed, it is equivalent to executing ticksFunc)
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
  //判断定时器与回调函数是否有效（已被预先定义）(Check if the timer and callback function are valid (pre-defined))
  {
    // _LOG("Timer initalize error.");
    return -1;    
  }
  //将定时器列表pTimerList的地址赋给pNextTimer(assign the address of pTimerList to pNextTimer)
  Timers **pNextTimer = &pTimerList;

  //遍历定时器列表的上记录的各个定时器，直到最后一个(Traverse the timer list to the last one)
  for (; *pNextTimer; pNextTimer = &(*pNextTimer)->next)
  {
    //如果在定时器列表中找到了参数传入的定时器（该定时器已经预先记录到定时器列表中），则先将当前定时器移除(If the timer passed in as a parameter (the timer has been pre-recorded in the timer list) is found in the timer list, remove the current timer first)
    if(ptimer == *pNextTimer)
    {
      /*原先逻辑：a->next= ptimer(Original logic: a->next= ptimer)
            ptimer->next=b   (定时器先后顺序：a-ptimer-b)(timer order: a-ptimer-b)
        操作逻辑：a->next=ptimer->next（也就是b）(Operation logic: a->next=ptimer->next (which is b))
        操作后逻辑：a->next=b  （定时器先后顺序：a-b）(ptimer从定时器列表中移除)(Logical result after operation: a->next=b (timer order: a-b) (ptimer is removed from the timer list))
      */
      *pNextTimer = ptimer->next;  
      break;
    }
  }

  ptimer->deadline = platFormTicksFunction() + timing;
  //platFormTicksFunction()得到当前系统定时  timing记录定时持续时间    deadline为下一次定时器到点的时候的系统时间(Get the current system time with platFormTicksFunction(), record the timing duration with timing, and record the system time when the next timer is due in deadline)
  ptimer->timersCallBack = timersCallBack;  //绑定定时器触发的回调函数)(bind callback function triggered by the timer)
  ptimer->userdata = userdata;  //绑定用户自定义数据(bine user-defined data)
  //遍历定时器列表的上记录的各个定时器，直到最后一个(Traverse the timers recorded in the timer list until the last one)
  //遍历(Traverse)
  for (pNextTimer = &pTimerList;; pNextTimer = &(*pNextTimer)->next)
  {
    //如果遍历到了列表的结尾（说明该定时器的下一次触发时间是最晚的），则将定时器追加记录到定时器列表的最后(If it reaches the end of the list (the next time the timer is triggered is the latest), append the timer to the end of the timer list)
    {
      ptimer->next = NULL;
      *pNextTimer = ptimer;
      break;
    }
    //按各个定时器下一次被触发的系统时间插入当前传入的定时器，按下一次触发时间由早到晚的顺序排布定时器(Insert the current timer according to the next time each timer will be triggered, arranged in order from early to late according to the next trigger time)
    if (ptimer->deadline < (*pNextTimer)->deadline)
    {
      ptimer->next = *pNextTimer;
      *pNextTimer = ptimer;
      break;
    }
  }
  return 1;
}

//关闭当前定时器（将当前定时器从定时器列表中移除出去）(Stop the current timer (remove the current timer from the timer list))
int timerStop(Timers *ptimer)
{
  Timers **pNextTimer = &pTimerList;
  for (; *pNextTimer; pNextTimer = &(*pNextTimer)->next)
  {
    Timers *entry = pTimerList;
    if(entry == ptimer)
    {
        /*原先逻辑：a->next= ptimer(Original logic: a->next= ptimer)
        ptimer->next=b   (定时器先后顺序：a-ptimer-b)(timer order: a-ptimer-b)
        操作逻辑：a->next=ptimer->next（也就是b）(Operation logic: a->next=ptimer->next (which is b))
        操作后逻辑：a->next=b  （定时器先后顺序：a-b）(ptimer从定时器列表中移除)(Logical result after operation: a->next=b (timer order: a-b) (ptimer is removed from the timer list))
      */
      *pNextTimer = ptimer->next;
      break;
    }
  }
  return 1;
}

int timersTaskRunning(void)
{
  //将定时器列表pTimerList（注：定时器列表内各个定时器按下一次触发时间的早晚顺序排布）的地址赋给entry (Assign the address of the timer list pTimerList (note: each timer in the timer list is arranged in order of the next trigger time) to entry)  
  Timers *entry = pTimerList;
  //遍历访问定时器列表的每一个定时器(Traverse and access each timer in the timer list)
  for (; entry; entry = entry->next)
  {
    /*如果当前定时器的下一次触发时间晚于当前系统时间（排在后面的定时器
      自然也还没到下一次被触发的时间），则退出函数，结束遍历(If the next trigger time of the current timer is later than the current system time (the timers at the back have not yet reached the next trigger time), exit the function and end the traversal)*/
    if (platFormTicksFunction() < entry->deadline)
    {
      return (int)(entry->deadline - platFormTicksFunction());
    }

    //如果当前定时器到点被触发，则先将该定时器从定时器列表中移除，再执行当前定时器的触发回调函数(If the current timer is triggered at the due time, first remove the timer from the timer list, and then execute the trigger callback function of the current timer)
    pTimerList = entry->next;
    if(entry->timersCallBack)
    {
      entry->timersCallBack(entry, entry->userdata);
    }
  }
  return 1;
}