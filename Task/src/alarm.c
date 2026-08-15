#include "alarm.h"
#include "io_logic.h"
#include "bsp_W25Q128.h"
#include <string.h>

Alarm_t g_alarm;

static uint32_t inconsist_start_tick = 0;
static uint8_t  inconsist_timing     = 0;


void Alarm_Init(void)
{
    memset(&g_alarm, 0, sizeof(g_alarm));
    inconsist_timing     = 0;
    inconsist_start_tick = 0;
	
}

/**
 * Alarm_Update
 * 主循环中周期调用（IO_Update 之后）
 *
 * 急停双路不一致检测：
 *   estop1 == estop2 → 一致，清除报警和计时
 *   estop1 != estop2 → 开始计时；持续超过 ALARM_ESTOP_INCONSIST_TIMEOUT_MS → 激活报警
 */
void Alarm_Update(void)
{
    if (g_io.input.estop1 != g_io.input.estop2) {
        if (!inconsist_timing) {
            inconsist_timing     = 1;
            inconsist_start_tick = HAL_GetTick();
        } else if ((HAL_GetTick() - inconsist_start_tick) >= ALARM_ESTOP_INCONSIST_TIMEOUT_MS) {
            g_alarm.flags.estop_inconsistent = 1;
        }
    } else {
        inconsist_timing                 = 0;
        inconsist_start_tick             = 0;
    }
	
	//复位清除故障
	if(g_io.input.reset)
	{
		memset(&g_alarm,0,sizeof(g_alarm));
	}
}
