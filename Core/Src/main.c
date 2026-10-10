/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_modbus.h"
#include "io_logic.h"
#include "led.h"
#include "task.h"
#include "alarm.h"
#include "bsp_W25Q128.h"
#include "params.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static char    status_buf[160];
static uint32_t status_last_tick = 0;

/* ---- 软件串口 TX：TIM5 按 115200 位周期产生中断，逐位输出到 PA0 ---- */
#define SWUART_TX_PIN    GPIO_PIN_0
static TIM_HandleTypeDef htim_swuart;
static volatile uint8_t  sw_active   = 0;
static volatile uint8_t  sw_bits_left = 0;
static volatile uint32_t sw_shift    = 0;

static void SWUART_Timer_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin   = GPIO_PIN_0;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

    __HAL_RCC_TIM5_CLK_ENABLE();
    htim_swuart.Instance = TIM5;
    htim_swuart.Init.Prescaler = 0;
    htim_swuart.Init.Period = 728;
    htim_swuart.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_swuart.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim_swuart);
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

static void SWUART_SendChar(uint8_t b)
{
    while (sw_active) { }
    sw_active     = 1;
    sw_shift      = b;
    sw_bits_left  = 10;
    __HAL_TIM_SET_COUNTER(&htim_swuart, 0);
    HAL_TIM_Base_Start_IT(&htim_swuart);
}

static void SWUART_Send(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        SWUART_SendChar(buf[i]);
    }
}

void TIM5_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim_swuart, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim_swuart, TIM_FLAG_UPDATE);
        if (sw_bits_left == 10) {
            HAL_GPIO_WritePin(GPIOA, SWUART_TX_PIN, GPIO_PIN_RESET);
            sw_bits_left--;
        } else if (sw_bits_left > 1) {
            if (sw_shift & 0x01u) HAL_GPIO_WritePin(GPIOA, SWUART_TX_PIN, GPIO_PIN_SET);
            else                  HAL_GPIO_WritePin(GPIOA, SWUART_TX_PIN, GPIO_PIN_RESET);
            sw_shift >>= 1u;
            sw_bits_left--;
        } else if (sw_bits_left == 1) {
            HAL_GPIO_WritePin(GPIOA, SWUART_TX_PIN, GPIO_PIN_SET);
            sw_bits_left--;
        } else {
            HAL_TIM_Base_Stop_IT(&htim_swuart);
            sw_active = 0;
        }
    }
}

/* ---- 软件串口 RX（中断驱动）：PA1 起始位下降沿 + TIM4 逐位采样 ---- */
#define SWUART_RX_PIN   GPIO_PIN_1
static TIM_HandleTypeDef htim_swuart_rx;
static volatile uint8_t  rx_active = 0;
static volatile uint8_t  rx_count = 0;
static volatile uint8_t  rx_shift = 0;
static volatile uint8_t  rx_buf[128];
static volatile uint16_t rx_cnt   = 0;

static void SWUART_Rx_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin   = GPIO_PIN_1;
    gpio.Mode  = GPIO_MODE_IT_FALLING;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    __HAL_RCC_TIM4_CLK_ENABLE();
    htim_swuart_rx.Instance = TIM4;
    htim_swuart_rx.Init.Prescaler = 0;
    htim_swuart_rx.Init.Period = 728;
    htim_swuart_rx.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_swuart_rx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim_swuart_rx);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void EXTI1_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1)) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
        if (!rx_active) {
            rx_active = 1;
            rx_count = 0;
            rx_shift = 0;
            __HAL_TIM_SET_COUNTER(&htim_swuart_rx, 364);
            HAL_TIM_Base_Start_IT(&htim_swuart_rx);
        }
    }
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim_swuart_rx, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim_swuart_rx, TIM_FLAG_UPDATE);
        if (!rx_active) return;
        if (rx_count == 0) {
            rx_count = 1;
        } else if (rx_count <= 8) {
            if (HAL_GPIO_ReadPin(GPIOA, SWUART_RX_PIN) == GPIO_PIN_SET) {
                rx_shift |= (uint8_t)(1u << (rx_count - 1));
            }
            rx_count++;
        } else {
            HAL_TIM_Base_Stop_IT(&htim_swuart_rx);
            if (rx_cnt < sizeof(rx_buf)) {
                rx_buf[rx_cnt++] = rx_shift;
            }
            rx_active = 0;
        }
    }
}

/* ESP32 网络状态（由 [NET] 行解析得到） */
static char    net_sta[12]  = "UNKNOWN";
static char    net_ssid[32] = "";
static char    net_ip[16]   = "";
static char    net_gw[16]   = "";
static char    net_ap[8]    = "UNKNOWN";
static uint32_t net_port    = 0;
static int      net_rssi    = 0;

/* 每秒发送一次 GD 状态日志（急停/按钮/指示灯/网络信息） */
static void Send_Status(void)
{
    if ((HAL_GetTick() - status_last_tick) >= 1000u) {
        status_last_tick = HAL_GetTick();
        const char *st;
        switch (Task_GetState()) {
            case TASK_STATE_ESTOP_PRESSED:  st = "ESTOP"; break;
            case TASK_STATE_ESTOP_RELEASED: st = "FREE";  break;
            default: st = "NORMAL"; break;
        }
        uint8_t pnp = 0;
        for (uint8_t i = 0; i < 8; i++) pnp |= (uint8_t)(g_io.input.PNP_in[i] << i);
        uint8_t po = (uint8_t)(g_io.output.PNP_out[0] | (g_io.output.PNP_out[1]<<1) |
                               (g_io.output.PNP_out[2]<<2) | (g_io.output.PNP_out[3]<<3));
        snprintf(status_buf, sizeof(status_buf),
"[GD] st=%s e1=%d e2=%d rst=%d btn=%02X po=%02X led=%d%d%d sta=%s ssid=%s ip=%s gw=%s rssi=%d ap=%s port=%lu\r\n",
            st, g_io.input.estop1, g_io.input.estop2, g_io.input.reset, pnp, po,
            g_io.output.led.system, g_io.output.led.working, g_io.output.led.network,
            net_sta, net_ssid, net_ip, net_gw, net_rssi, net_ap, (unsigned long)net_port);
        SWUART_Send((const uint8_t*)status_buf, strlen(status_buf));
    }
}

/* 日志缓存：USART1 收到的 ESP32 日志，由主循环发到 PA0 */
static volatile uint8_t  esp_log_buf[256];
static volatile uint16_t esp_log_len = 0;
static volatile uint8_t  esp_log_pending = 0;

void Modem_QueueLog(const uint8_t *buf, uint16_t len)
{
    if (len > sizeof(esp_log_buf)) len = sizeof(esp_log_buf);
    for (uint16_t i = 0; i < len; i++) esp_log_buf[i] = buf[i];
    esp_log_len = len;
    esp_log_pending = 1;
}

static void Parse_Net(const char *line)
{
    if (strncmp(line, "[NET]", 5) != 0) return;
    char sta[12]="", ssid[32]="", ip[16]="", gw[16]="", ap[8]="";
    int rssi = 0;
    unsigned port = 0;
    if (sscanf(line, "[NET] sta=%11s ssid=%31s ip=%15s gw=%15s rssi=%d ap=%7s port=%u",
               sta, ssid, ip, gw, &rssi, ap, &port) == 7) {
        strncpy(net_sta,  sta,  sizeof(net_sta)-1);
        strncpy(net_ssid, ssid, sizeof(net_ssid)-1);
        strncpy(net_ip,   ip,   sizeof(net_ip)-1);
        strncpy(net_gw,   gw,   sizeof(net_gw)-1);
        strncpy(net_ap,   ap,   sizeof(net_ap)-1);
        net_port = port;
        net_rssi = rssi;
    }
}

static void Handle_Log(void)
{
    if (esp_log_pending) {
        esp_log_pending = 0;
        if (strncmp((const char*)esp_log_buf, "[NET]", 5) == 0) {
            Parse_Net((const char*)esp_log_buf);
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  Alarm_Init();
  Modbus_Init();
  SWUART_Timer_Init();
  SWUART_Rx_Init();
  /* 启动 USART1 接收 ESP32 日志（转发到 USART3 以太网日志口）*/
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, mb_uart1_rx_buf, MB_BUF_SIZE);
  LED_Init();
  IO_Update();   /* 先刷新一次输入，Task_Init 才能读到真实急停电平 */
  Task_Init();
  W25Q128_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IO_Update();
    Task_Update();
    Alarm_Update();
    LED_Update();
    Params_Update();
    Send_Status();
    Handle_Log();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
