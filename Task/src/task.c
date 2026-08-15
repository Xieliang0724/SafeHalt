#include "task.h"
#include "io_logic.h"

static volatile TaskState_t task_state = TASK_STATE_ESTOP_RELEASED;
static          uint8_t     prev_reset = 0; /* g_io.input.reset 上次值，用于检测上升沿 */

/* 任意一路急停激活即返回真 */
static uint8_t estop_active(void)
{
    return (g_io.input.estop1 || g_io.input.estop2);
}

/**
 * Task_Init
 * 上电时根据急停引脚的实际电平设置初始状态
 * 注意：调用前须先执行一次 IO_Update()
 */
void Task_Init(void)
{
    if (estop_active()) {
        task_state = TASK_STATE_ESTOP_PRESSED;
    } else {
        task_state = TASK_STATE_ESTOP_RELEASED;
    }
    prev_reset = g_io.input.reset;
}

/**
 * Task_EStopISR
 * 在 EXTI3 / EXTI4 中断里调用，任意一路急停上升沿即触发
 */
void Task_EStopISR(void)
{
    task_state = TASK_STATE_ESTOP_PRESSED;
}

/**
 * Task_Update
 * 在主循环中周期调用（IO_Update 之后）
 * 0. 兜底：引脚实际激活但状态不符时强制置为 ESTOP_PRESSED
 * 1. ESTOP_PRESSED  → ESTOP_RELEASED（两路信号均撤除时）
 * 2. ESTOP_RELEASED → NORMAL（检测到复位按键上升沿时）
 */
void Task_Update(void)
{
    /* 兜底：弥补 EXTI 可能的漏检 */
    if (estop_active() && task_state != TASK_STATE_ESTOP_PRESSED) {
        task_state = TASK_STATE_ESTOP_PRESSED;
    }

    switch (task_state)
    {
        case TASK_STATE_ESTOP_PRESSED:
            if (!estop_active()) {
                task_state = TASK_STATE_ESTOP_RELEASED;
            }
            break;

        case TASK_STATE_ESTOP_RELEASED:
            /* 检测 reset 上升沿 */
            if (g_io.input.reset && !prev_reset) {
                task_state = TASK_STATE_NORMAL;
            }
            break;

        case TASK_STATE_NORMAL:
            break;
    }

    prev_reset = g_io.input.reset;
}

TaskState_t Task_GetState(void)
{
    return task_state;
}
