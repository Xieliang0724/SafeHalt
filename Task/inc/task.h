#ifndef __TASK_H__
#define __TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"


typedef enum {
    TASK_STATE_ESTOP_PRESSED   = 0, /* 急停按下               */
    TASK_STATE_ESTOP_RELEASED  = 1, /* 急停松开，等待复位      */
    TASK_STATE_NORMAL          = 2, /* 已复位，正常运行        */
} TaskState_t;

void        Task_Init(void);        /* 上电初始化，在 GPIO Init 之后调用 */
void        Task_EStopISR(void);    /* 在 EXTI3 / EXTI4 中断中调用      */
void        Task_Update(void);      /* 在主循环中周期调用                */

TaskState_t Task_GetState(void);    /* 获取当前状态                      */

#ifdef __cplusplus
}
#endif

#endif /* __TASK_H__ */
