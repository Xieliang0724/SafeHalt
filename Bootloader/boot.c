/* GD32 OTA bootloader
 * 位于 0x08000000（32KB）。收到 app 的升级请求后，通过 USART2(PA2/PA3)
 * 从 ESP32 接收固件，缓存到 RAM，校验后写入 app 区并跳转。 */
#include "stm32f4xx_hal.h"
#include "ota_proto.h"
#include <string.h>

#define OTA_RX_BUFSZ   (64u*1024u)      /* 固件缓存（RAM，64KB） */
#define OTA_UART_RX_SIZE 600u
#define OTA_FIRST_BYTE_TIMEOUT_MS 20000u

#define OTA_FAIL_RX_TIMEOUT 0xE001u
#define OTA_FAIL_RX_FRAME   0xE002u
#define OTA_FAIL_RX_CRC     0xE003u
#define OTA_FAIL_RX_REARM   0xE004u
#define OTA_FAIL_SIZE       0xE005u
#define OTA_FAIL_SEQUENCE   0xE006u
#define OTA_FAIL_FLASH      0xE007u
#define OTA_FAIL_APP_CRC    0xE008u
#define OTA_FAIL_UNKNOWN    0xE0FFu

static uint8_t  ota_buf[OTA_RX_BUFSZ];  /* 收到的新固件暂存 */
static UART_HandleTypeDef huart_ota;
static DMA_HandleTypeDef hdma_usart2_rx;
static DMA_HandleTypeDef hdma_usart2_tx;
static uint8_t  ota_resp[6];
static uint8_t  ota_uart_rx_buf[OTA_UART_RX_SIZE];
static volatile uint16_t ota_uart_rx_len;
static volatile uint8_t ota_uart_rx_ready;
static volatile uint32_t ota_failure_reason;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_Init(void);
static void Error_Handler(void);
static uint16_t OTA_CRC16(const uint8_t *d, uint32_t len);
static void OTA_SendResp(uint8_t status, uint8_t seq);
static uint32_t OTA_ReadWord(uint32_t addr);
static void OTA_WriteStatus(const OtaStatus_t *st);
static int  OTA_WriteApp(const uint8_t *buf, uint32_t len);
static int  OTA_StartRx(void);
static int  OTA_ReceiveFirmware(OtaStatus_t *st);
static void JumpToApp(uint32_t addr);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_Init();

    OtaStatus_t st;
    memcpy(&st, (const void *)OTA_STATUS_ADDR, sizeof(st));

    if (st.magic == OTA_STATE_MAGIC && st.state == OTA_STATE_UPDATE) {
        /* 不主动回 READY：等 ESP32 发 SYNC 后再应答，避免和触发回显串帧 */
        if (OTA_ReceiveFirmware(&st)) {
            JumpToApp(OTA_APP_ADDR);                   /* 新固件已写好，跳转 */
        } else {
            st.state = OTA_STATE_NONE;                 /* 失败，清标志 */
            st.app_size = ota_failure_reason ? ota_failure_reason : OTA_FAIL_UNKNOWN;
            st.app_crc = 0;
            OTA_WriteStatus(&st);
            JumpToApp(OTA_APP_ADDR);                   /* 尝试跳现有 app */
        }
    }

    JumpToApp(OTA_APP_ADDR);
    while (1) { }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInit = {0};
    RCC_ClkInitTypeDef RCC_ClkInit = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInit.HSEState = RCC_HSE_ON;
    RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInit.PLL.PLLM = 4;
    RCC_OscInit.PLL.PLLN = 168;
    RCC_OscInit.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInit.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInit) != HAL_OK) Error_Handler();
    RCC_ClkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                            RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void MX_USART2_Init(void)
{
    huart_ota.Instance = USART2;
    huart_ota.Init.BaudRate = 115200;
    huart_ota.Init.WordLength = UART_WORDLENGTH_8B;
    huart_ota.Init.StopBits = UART_STOPBITS_1;
    huart_ota.Init.Parity = UART_PARITY_NONE;
    huart_ota.Init.Mode = UART_MODE_TX_RX;
    huart_ota.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_ota.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart_ota) != HAL_OK) Error_Handler();

    /* Use the same USART2 DMA streams as the working App Modbus path. */
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart_ota, hdmarx, hdma_usart2_rx);

    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart_ota, hdmatx, hdma_usart2_tx);

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __HAL_UART_ENABLE_IT(&huart_ota, UART_IT_IDLE);
    if (!OTA_StartRx()) Error_Handler();
}

static int OTA_StartRx(void)
{
    ota_uart_rx_len = 0;
    ota_uart_rx_ready = 0;
    __HAL_UART_CLEAR_IDLEFLAG(&huart_ota);
    return HAL_UART_Receive_DMA(&huart_ota, ota_uart_rx_buf,
                                sizeof(ota_uart_rx_buf)) == HAL_OK;
}

void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart_ota, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(&huart_ota, UART_IT_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart_ota);
        if (!ota_uart_rx_ready) {
            uint16_t len = OTA_UART_RX_SIZE -
                           __HAL_DMA_GET_COUNTER(huart_ota.hdmarx);
            HAL_UART_DMAStop(&huart_ota);
            if (len > 0) {
                ota_uart_rx_len = len;
                ota_uart_rx_ready = 1;
            } else if (!OTA_StartRx()) {
                ota_failure_reason = OTA_FAIL_RX_REARM;
                ota_uart_rx_ready = 1;
            }
        }
    }
    HAL_UART_IRQHandler(&huart_ota);
}

static uint16_t OTA_CRC16(const uint8_t *d, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static uint32_t OTA_ReadWord(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

static void OTA_SendResp(uint8_t status, uint8_t seq)
{
    ota_resp[0] = OTA_MAGIC; ota_resp[1] = 0x05; ota_resp[2] = status; ota_resp[3] = seq;
    uint16_t c = OTA_CRC16(ota_resp, 4);
    ota_resp[4] = (uint8_t)(c & 0xFF); ota_resp[5] = (uint8_t)(c >> 8);
    HAL_UART_Transmit_DMA(&huart_ota, ota_resp, 6);
    while (huart_ota.gState != HAL_UART_STATE_READY) { }   /* 等发送完成 */
}

/* 读一帧：返回类型；返回 -1 失败 */
static int OTA_RxFrame(uint8_t *frame, int maxlen)
{
    uint32_t start = HAL_GetTick();
    while (!ota_uart_rx_ready) {
        if ((HAL_GetTick() - start) >= OTA_FIRST_BYTE_TIMEOUT_MS) {
            ota_failure_reason = OTA_FAIL_RX_TIMEOUT;
            return -1;
        }
        HAL_Delay(1);
    }

    __disable_irq();
    uint16_t rx_len = ota_uart_rx_len;
    ota_uart_rx_ready = 0;
    __enable_irq();

    if (rx_len == 0 || rx_len > (uint16_t)maxlen) {
        ota_failure_reason = OTA_FAIL_RX_FRAME;
        OTA_StartRx();
        return -1;
    }
    memcpy(frame, ota_uart_rx_buf, rx_len);
    if (!OTA_StartRx()) {
        ota_failure_reason = OTA_FAIL_RX_REARM;
        return -1;
    }

    int offset = 0;
    while (offset < rx_len && frame[offset] != OTA_MAGIC) offset++;
    if (offset >= rx_len || (rx_len - offset) < 7) {
        ota_failure_reason = OTA_FAIL_RX_FRAME;
        return -1;
    }
    if (offset > 0) {
        memmove(frame, frame + offset, rx_len - offset);
        rx_len -= offset;
    }

    uint16_t payload_len = (uint16_t)frame[3] | ((uint16_t)frame[4] << 8);
    if (payload_len > 512 || (uint32_t)(5u + payload_len + 2u) > rx_len) {
        ota_failure_reason = OTA_FAIL_RX_FRAME;
        return -1;
    }
    uint16_t frame_crc = OTA_CRC16(frame, 5u + payload_len);
    uint16_t rx_crc = (uint16_t)frame[5u + payload_len] |
                      ((uint16_t)frame[6u + payload_len] << 8);
    if (frame_crc != rx_crc) {
        ota_failure_reason = OTA_FAIL_RX_CRC;
        return -2;
    }
    return frame[1];
}

static void OTA_WriteStatus(const OtaStatus_t *st)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef e; uint32_t err = 0;
    e.TypeErase = FLASH_TYPEERASE_SECTORS; e.Sector = FLASH_SECTOR_11;
    e.NbSectors = 1; e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&e, &err);
    for (uint32_t i = 0; i < sizeof(OtaStatus_t); i += 4) {
        uint32_t w; memcpy(&w, (const uint8_t *)st + i, 4);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, OTA_STATUS_ADDR + i, w);
    }
    HAL_FLASH_Lock();
}

/* The 64KB OTA image spans at most GD32F407 sectors 2, 3 and 4. */
static int OTA_WriteApp(const uint8_t *buf, uint32_t len)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef e; uint32_t err = 0;
    e.TypeErase = FLASH_TYPEERASE_SECTORS;
    e.Sector = FLASH_SECTOR_2;          /* 0x08008000 */
    e.NbSectors = (len <= 0x4000u) ? 1u : ((len <= 0x8000u) ? 2u : 3u);
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&e, &err) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }
    for (uint32_t off = 0; off < len; off += 4) {
        uint32_t w = 0xFFFFFFFF; memcpy(&w, &buf[off], 4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, OTA_APP_ADDR + off, w) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
    }
    HAL_FLASH_Lock();
    return 1;
}

static int OTA_ReceiveFirmware(OtaStatus_t *st)
{
    int      started = 0, done = 0;
    uint32_t total_size = 0, got = 0;
    uint16_t total_crc = 0, calc = 0;
    uint16_t seq = 0;
    uint8_t  frame[600];
    ota_failure_reason = 0;

    while (!done) {
        int type = OTA_RxFrame(frame, sizeof(frame));
        if (type == OTA_FRAME_ABORT) {
            ota_failure_reason = OTA_FAIL_RX_FRAME;
            OTA_SendResp(OTA_ST_ABORT, (uint8_t)seq);
            return 0;
        }
        if (type == -1) {
            if (!ota_failure_reason) ota_failure_reason = OTA_FAIL_RX_TIMEOUT;
            OTA_SendResp(OTA_ST_ABORT, (uint8_t)seq);
            return 0;
        }
        if (type == -2) {
            if (!ota_failure_reason) ota_failure_reason = OTA_FAIL_RX_CRC;
            OTA_SendResp(OTA_ST_CRC, (uint8_t)seq);
            return 0;
        }
        if (type == OTA_FRAME_SYNC) { OTA_SendResp(OTA_ST_READY, 0); continue; }

        if (type == OTA_FRAME_START) {
            if (started) {
                ota_failure_reason = OTA_FAIL_SEQUENCE;
                OTA_SendResp(OTA_ST_BUSY, (uint8_t)seq);
                return 0;
            }
            /* payload: size(4 LE) + total_crc(2 LE)，位于 frame[5..10] */
            total_size = (uint32_t)frame[5] | ((uint32_t)frame[6] << 8) |
                         ((uint32_t)frame[7] << 16) | ((uint32_t)frame[8] << 24);
            total_crc  = (uint16_t)frame[9] | ((uint16_t)frame[10] << 8);
            if (total_size == 0 || total_size > OTA_RX_BUFSZ) {
                ota_failure_reason = OTA_FAIL_SIZE;
                OTA_SendResp(OTA_ST_CRC, 0);
                return 0;
            }
            st->app_size = total_size; st->app_crc = total_crc;
            started = 1; seq = 0; got = 0;
            OTA_SendResp(OTA_ST_OK, 0);
        } else if (type == OTA_FRAME_DATA) {
            if (!started || frame[2] != (seq & 0xFF)) {
                ota_failure_reason = OTA_FAIL_SEQUENCE;
                OTA_SendResp(OTA_ST_SEQ, (uint8_t)seq);
                return 0;
            }
            int len = frame[3] | (frame[4] << 8);
            if (got + len > total_size || len < 1) {
                ota_failure_reason = OTA_FAIL_SEQUENCE;
                OTA_SendResp(OTA_ST_SEQ, (uint8_t)seq);
                return 0;
            }
            memcpy(&ota_buf[got], &frame[5], len);
            got += len; seq++;
            OTA_SendResp(OTA_ST_OK, (uint8_t)((seq - 1) & 0xFF));
        } else if (type == OTA_FRAME_END) {
            if (!started || got != total_size) {
                ota_failure_reason = OTA_FAIL_SEQUENCE;
                OTA_SendResp(OTA_ST_CRC, (uint8_t)seq);
                return 0;
            }
            calc = OTA_CRC16(ota_buf, total_size);
            if (calc != total_crc) {
                ota_failure_reason = OTA_FAIL_APP_CRC;
                OTA_SendResp(OTA_ST_CRC, (uint8_t)seq);
                return 0;
            }
            done = 1;
        } else {
            ota_failure_reason = OTA_FAIL_RX_FRAME;
            OTA_SendResp(OTA_ST_ABORT, (uint8_t)seq);
        }
    }

    if (!OTA_WriteApp(ota_buf, total_size)) {
        ota_failure_reason = OTA_FAIL_FLASH;
        OTA_SendResp(OTA_ST_ABORT, (uint8_t)seq);
        return 0;
    }
    /* 校验写入结果 */
    if (OTA_CRC16((const uint8_t *)OTA_APP_ADDR, total_size) != total_crc) {
        ota_failure_reason = OTA_FAIL_APP_CRC;
        OTA_SendResp(OTA_ST_CRC, (uint8_t)seq);
        return 0;
    }
    st->state = OTA_STATE_READY;
    OTA_WriteStatus(st);
    OTA_SendResp(OTA_ST_READY, (uint8_t)seq);
    return 1;
}

static void JumpToApp(uint32_t addr)
{
    uint32_t sp = OTA_ReadWord(addr);
    uint32_t pc = OTA_ReadWord(addr + 4);
    HAL_UART_DMAStop(&huart_ota);
    __HAL_UART_DISABLE_IT(&huart_ota, UART_IT_IDLE);
    HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    /* 关闭 SysTick 中断，让 app 用 HAL_Init 重新初始化，避免定时器状态残留 */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    __set_MSP(sp);
    SCB->VTOR = addr;
    __enable_irq();
    typedef void (*JumpFn)(void);
    JumpFn j = (JumpFn)pc;
    j();
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

/* HAL_Init() 需要 SysTick 滴答来驱动 HAL_IncTick / HAL_GetTick */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
