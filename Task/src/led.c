#include "led.h"
#include "io_logic.h"
#include "task.h"

/**
 * LED_Init
 * 上电初始化，全部熄灭
 */
void LED_Init(void)
{
    g_io.output.led.working = 0;
    g_io.output.led.network = 0;
    g_io.output.led.system  = 0;
}

/**
 * LED_Update
 * LED判断逻辑
 */
void LED_Update(void)
{
    static uint32_t working_last_tick = 0;

    /* system 灯：正常运行时常亮，其他状态熄灭 */
    g_io.output.led.system = (Task_GetState() == TASK_STATE_NORMAL) ? 1u : 0u;

    /* working 灯：正常运行时 1s 间隔闪烁（500ms 亮 / 500ms 灭），其他状态熄灭 */
	if ((HAL_GetTick() - working_last_tick) >= 500u) {
		working_last_tick = HAL_GetTick();
		g_io.output.led.working ^= 1u;
	}
	
	/* 状态给到PNP1和PNP2 */
	g_io.output.PNP_out[0]=g_io.output.led.system;
	g_io.output.PNP_out[1]=g_io.output.PNP_out[0] ^ 1u;
}