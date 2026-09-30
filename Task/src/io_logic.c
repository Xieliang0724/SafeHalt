#include "io_logic.h"
#include "task.h"
#include "bsp_modbus.h"

IO_t g_io;

/**
 * IO_InputUpdate
 * 读取所有输入引脚并刷新 g_io.input
 */
static void IO_InputUpdate(void)
{
    g_io.input.estop1 		= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) ? 1u : 0u;
    g_io.input.estop2 		= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1u : 0u;
	
    g_io.input.reset  		= (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[0]  	= (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[1]  	= (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[2]  	= (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[3]  	= (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[4]  	= (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[5]  	= (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[6]  	= (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[7]  	= (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET) ? 0u : 1u;
	
}

/**
 * IO_OutputUpdate
 * 将 g_io.output 的逻辑值写入实际硬件引脚
 * 在主循环最后面调用一次，集中管理所有输出映射
 *
 * LED 引脚映射：
 *   led.working  — PE5（高=亮，500ms 闪烁）
 *   led.network  — PE3（低=亮，无人赋值→恒灭）
 *   led.system   — PE2=stop_NO 镜像(高=亮)，PE4=stop_OFF 反相(高=亮)
 */
static void IO_OutputUpdate(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,
        g_io.output.led.system ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,
        g_io.output.led.network ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,
        (g_io.output.led.system ^ 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,
        g_io.output.led.working ? GPIO_PIN_SET : GPIO_PIN_RESET);
	
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,
        g_io.output.relay[0]  ? GPIO_PIN_SET : GPIO_PIN_RESET);
	
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,
        g_io.output.relay[1]  ? GPIO_PIN_SET : GPIO_PIN_RESET);
	
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,
        g_io.output.relay[2]  ? GPIO_PIN_SET : GPIO_PIN_RESET);
	
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,
        g_io.output.relay[3]  ? GPIO_PIN_SET : GPIO_PIN_RESET);
	
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,
        g_io.output.PNP_out[0] ? GPIO_PIN_RESET : GPIO_PIN_SET);
		
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,
        g_io.output.PNP_out[1] ? GPIO_PIN_RESET : GPIO_PIN_SET);
		
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,
        g_io.output.PNP_out[2] ? GPIO_PIN_RESET : GPIO_PIN_SET);
		
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10,
        g_io.output.PNP_out[3] ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * modbus_data_mapping
 * modbus地址信息映射（正向：状态 → 寄存器）
 * 只对"只读区"做正向映射：
 *   reg 0        任务状态
 *   reg 20~30    输入状态
 *   reg 35~39    LED / PNP_out[0..1]（由 LED 逻辑主导，回读实际值）
 * 可写输出区 reg 31~34（relay）与 40~41（PNP_out[2..3]）不做正向覆盖，
 * 由 modbus_write_mapping() 反向控制（见下）。
 */
static void modbus_data_mapping(void)
{
	//逻辑变量映射
	mb_holding_regs[0]=(Task_GetState()==TASK_STATE_NORMAL)? 1 : 0;
	
	//IO状态映射
	mb_holding_regs[20]=g_io.input.estop1;
	mb_holding_regs[21]=g_io.input.estop2;
	mb_holding_regs[22]=g_io.input.reset;
	mb_holding_regs[23]=g_io.input.PNP_in[0];
	mb_holding_regs[24]=g_io.input.PNP_in[1];
	mb_holding_regs[25]=g_io.input.PNP_in[2];
	mb_holding_regs[26]=g_io.input.PNP_in[3];
	mb_holding_regs[27]=g_io.input.PNP_in[4];
	mb_holding_regs[28]=g_io.input.PNP_in[5];
	mb_holding_regs[29]=g_io.input.PNP_in[6];
	mb_holding_regs[30]=g_io.input.PNP_in[7];

	//LED / PNP_out[0..1]：LED 逻辑为唯一驱动源，仅回读实际值
	mb_holding_regs[35]=g_io.output.led.working;
	mb_holding_regs[36]=g_io.output.led.network;
	mb_holding_regs[37]=g_io.output.led.system;
	mb_holding_regs[38]=g_io.output.PNP_out[0];
	mb_holding_regs[39]=g_io.output.PNP_out[1];
}

/**
 * modbus_write_mapping
 * modbus地址信息映射（反向：寄存器 → 输出）
 * 上位机可写输出寄存器（值钳位为 0/1）：
 *   reg 31~34 → relay[0..3]
 *   reg 40~41 → PNP_out[2..3]
 * 必须在 IO_OutputUpdate() 之前调用，保证当周期生效。
 */
static void modbus_write_mapping(void)
{
	g_io.output.relay[0]   = (mb_holding_regs[31] != 0u) ? 1u : 0u;
	g_io.output.relay[1]   = (mb_holding_regs[32] != 0u) ? 1u : 0u;
	g_io.output.relay[2]   = (mb_holding_regs[33] != 0u) ? 1u : 0u;
	g_io.output.relay[3]   = (mb_holding_regs[34] != 0u) ? 1u : 0u;
	g_io.output.PNP_out[2] = (mb_holding_regs[40] != 0u) ? 1u : 0u;
	g_io.output.PNP_out[3] = (mb_holding_regs[41] != 0u) ? 1u : 0u;
}

/**
 * IO_Update
 * 刷新所有数据
 * 顺序：读输入 → 寄存器映射（正向+反向）→ 写输出引脚
 */
void IO_Update(void)           
{
	IO_InputUpdate();
	modbus_data_mapping();
	modbus_write_mapping();
	IO_OutputUpdate();
}
