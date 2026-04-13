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
#include "main.h" //page switch, button, data value sending with CRC is OK

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include "dwin.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
#define DEBUG_QUEUE_SIZE 8
#define DEBUG_MSG_LEN    128

static char debug_queue[DEBUG_QUEUE_SIZE][DEBUG_MSG_LEN];
static volatile uint8_t dbg_head = 0;
static volatile uint8_t dbg_tail = 0;
static volatile uint8_t dbg_busy = 0;

uint8_t dma_rx_buffer[64];


#define DWIN_UART_HANDLE 		huart1

#define DWIN_FRAME_HEADER_0  	0x5A
#define DWIN_FRAME_HEADER_1  	0xA5

#define DWIN_CMD_WRITE       	0x82
#define DWIN_CMD_READ        	0x83

#define DWIN_VP_TOUCH_CONTROL  	0x0000
#define DWIN_VP_PAGE_CHANGE    	0x0084
#define DWIN_VP_SYSTEM_RESET   	0x0004

/* ---------------- CRC option ----------------
 * NOTE:
 *   The standard DGUS UART protocol (5A A5 ...) typically does NOT use CRC.
 *   Enable this only if your project explicitly adds CRC at the end of each
 *   frame and both sides (MCU + HMI) follow the same rule.
 *
 * CRC implementation used here:
 *   - CRC-16/IBM (Modbus) poly 0xA001, init 0xFFFF
 *   - returns CRC with a final byte-swap: (crc>>8)|(crc<<8)
 *
 * Assumed framing when CRC enabled:
 *   [0]=0x5A [1]=0xA5 [2]=LEN [3..]=PAYLOAD ... [CRC_HI][CRC_LO]
 *   Where LEN includes the 2 CRC bytes.
 */
#define DWIN_USE_CRC           1

/* User-provided (working) CRC function */
static uint16_t dwin_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;

    while (length--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    /* Byte-swap so returned value is big-endian (HI:LO) */
    return (uint16_t)((crc >> 8) | (crc << 8));
}


uint8_t dwin_rx_buf[32];
uint8_t dwin_rx_idx = 0;
uint8_t dwin_rx_byte;

uint8_t page;



#define DWIN_RX_BUF_SIZE 64

typedef enum
{
    DWIN_RX_IDLE,
    DWIN_RX_COLLECTING,
    DWIN_RX_READY,
    DWIN_RX_ERROR
} dwin_rx_state_t;

typedef struct
{
    uint8_t  buf[DWIN_RX_BUF_SIZE];
    uint16_t len;
    dwin_rx_state_t state;
} dwin_rx_t;

static dwin_rx_t dwin_rx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void dwin_switch_page(uint16_t page_id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t queue_next(uint8_t idx)
{
    return (idx + 1) % DEBUG_QUEUE_SIZE;
}

void DebugPrint(const char *format, ...)
{
    uint8_t next = queue_next(dbg_head);

    // Queue full → drop message (safe behavior)
    if (next == dbg_tail)
        return;

    va_list args;
    va_start(args, format);
    vsnprintf(debug_queue[dbg_head], DEBUG_MSG_LEN, format, args);
    va_end(args);

    dbg_head = next;

    // Start TX if idle
    if (!dbg_busy)
    {
        dbg_busy = 1;
        HAL_UART_Transmit_DMA(
            &huart3,
            (uint8_t *)debug_queue[dbg_tail],
            strlen(debug_queue[dbg_tail])
        );
    }
}




//void check_dma_data(void)
//{
//    static uint16_t old_pos = 0;
//    static uint32_t total_pos = 0;
//
//    uint16_t pos =
//        sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
//
//    if (pos == old_pos)
//        return;
//
//    char msg[128];
//    int len = 0;
//
//    if (pos > old_pos)
//    {
//        for (uint16_t i = old_pos; i < pos; i++)
//        {
//            len += snprintf(&msg[len], sizeof(msg) - len,
//                            "[%lu] %02X '%c'\r\n",
//                            total_pos++, dma_rx_buffer[i],
//                            dma_rx_buffer[i]);
//        }
//    }
//    else
//    {
//        for (uint16_t i = old_pos; i < sizeof(dma_rx_buffer); i++)
//        {
//            len += snprintf(&msg[len], sizeof(msg) - len,
//                            "[%lu] %02X '%c'\r\n",
//                            total_pos++, dma_rx_buffer[i],
//                            dma_rx_buffer[i]);
//        }
//
//        for (uint16_t i = 0; i < pos; i++)
//        {
//            len += snprintf(&msg[len], sizeof(msg) - len,
//                            "[%lu] %02X '%c'\r\n",
//                            total_pos++, dma_rx_buffer[i],
//                            dma_rx_buffer[i]);
//        }
//    }
//
//    DebugPrint("%s", msg);
//    old_pos = pos;
//}


void check_dma_data(void)
{
    static uint16_t old_pos = 0;
    static uint32_t total_pos = 0;

    uint16_t pos =
        sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

    if (pos == old_pos)
        return;

    char msg[128];
    int len = 0;

    if (pos > old_pos)
    {
        for (uint16_t i = old_pos; i < pos; i++)
        {

			page = dma_rx_buffer[i];

			/* Quick page-switch test from UART3 terminal:
			 *   send '0' -> page 0x0000
			 *   send '1' -> page 0x0001
			 */
			if (page == '0') { dwin_switch_page(0x0000); }
			if (page == '1') { dwin_switch_page(0x0001); }
			if (page == '2') { dwin_switch_page(0x0002); }
			if (page == '6') { dwin_switch_page(0x0006); }

            len += snprintf(&msg[len], sizeof(msg) - len,
                            "[%lu] %02X '%c'\r\n",
                            total_pos++, page,
							page);
        }
    }
    else
    {
        for (uint16_t i = old_pos; i < sizeof(dma_rx_buffer); i++)
        {
            len += snprintf(&msg[len], sizeof(msg) - len,
                            "[%lu] %02X '%c'\r\n",
                            total_pos++, page,
							page);
        }

        for (uint16_t i = 0; i < pos; i++)
        {
            len += snprintf(&msg[len], sizeof(msg) - len,
                            "[%lu] %02X '%c'\r\n",
                            total_pos++, page,
							page);
        }
    }

    DebugPrint("%s", msg);
    old_pos = pos;
}



void dwin_write_u16(uint16_t vp, uint16_t val)
{
    uint8_t data[2];
    data[0] = (val >> 8) & 0xFF;
    data[1] = val & 0xFF;

    dwin_cmd_write(vp, data, 2);

    DebugPrint("WRITE VP=0x%04X VAL=%d\r\n", vp, val);
}




void dwin_process_rx(uint8_t *buf, uint8_t len)
{
    if (len < 9)
        return;

    if (buf[0] != 0x5A || buf[1] != 0xA5)
        return;

    if (buf[3] != 0x83)   // Return Key Code
        return;

    uint8_t page_id    = buf[4];
    uint8_t control_id = buf[5];

    DebugPrint("DWIN BTN: page=%02X ctrl=%02X\r\n",page_id, control_id);

    uint16_t value = 0;

    if (control_id == 0x01)
        value = 100;
    else if (control_id == 0x02)
        value = 255;

    dwin_write_u16(0x2004, value);
}


void dwin_uart4_test_tx(void)
{
    /* Example WRITE VP 0x2004 = 0x002F */
#if DWIN_USE_CRC
    /* LEN includes CRC bytes when CRC is enabled */
    uint8_t tx_buf[10] = {
        0x5A, 0xA5,
        0x07,             /* 0x05 payload + 0x02 CRC */
        0x82,
        0x20, 0x04,
        0x00, 0x4F,
        0x00, 0x00        /* CRC placeholder (HI, LO) */
    };


    uint8_t len = tx_buf[2];
    /* CRC over everything except CRC itself (matches RX check below) */
    uint16_t crc = dwin_crc16(&tx_buf[3], len - 2);
    tx_buf[8] = (uint8_t)((crc >> 8) & 0xFFu); /* CRC_HI */
    tx_buf[9] = (uint8_t)(crc & 0xFFu);        /* CRC_LO */
    HAL_UART_Transmit(&huart4, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
#else
    uint8_t tx_buf[8] = {
        0x5A, 0xA5,
        0x05,
        0x82,
        0x20, 0x04,
        0x00, 0x2F
    };
    HAL_UART_Transmit(&huart4, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
#endif
    for(int i=0; i<sizeof(tx_buf);i++)
    	DebugPrint("%02X ",tx_buf[i]);
    DebugPrint("\r\n");
}


void dwin_switch_page(uint16_t page_id)
{
#if DWIN_USE_CRC
    uint8_t tx_buf[12] = {
        0x5A, 0xA5,
        0x09,             /* 1(CMD)+2(VP)+4(data)+2(CRC) = 9 */
        0x82,
        0x00, 0x84,       /* VP = 0x0084 (page switch) */
        0x5A, 0x01,       /* switch-page command */
        (uint8_t)(page_id >> 8),
        (uint8_t)(page_id & 0xFF),
        0x00, 0x00        /* CRC placeholder (HI, LO) */
    };

    /* CRC over everything except CRC itself */
    uint8_t len = tx_buf[2];
    /* CRC over everything except CRC itself (matches RX check below) */
    uint16_t crc = dwin_crc16(&tx_buf[3], len - 2);
    tx_buf[10] = (uint8_t)((crc >> 8) & 0xFFu); /* CRC_HI */
    tx_buf[11] = (uint8_t)(crc & 0xFFu);        /* CRC_LO */
    HAL_UART_Transmit(&huart4, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
    DebugPrint("[DWIN] SwitchPage -> 0x%04X\r\n", page_id);
#else
    uint8_t tx_buf[10] = {
        0x5A, 0xA5,
        0x07,             /* 1(CMD)+2(VP)+4(data) = 7 */
        0x82,
        0x00, 0x84,
        0x5A, 0x01,
        (uint8_t)(page_id >> 8),
        (uint8_t)(page_id & 0xFF)
    };
    HAL_UART_Transmit(&huart4, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
    DebugPrint("[DWIN] SwitchPage(noCRC) -> 0x%04X\r\n", page_id);
#endif
}



void dwin_rx_start(void)
{
    dwin_rx.len = 0;
    dwin_rx.state = DWIN_RX_IDLE;

    HAL_UARTEx_ReceiveToIdle_IT(
        &huart4,
        dwin_rx.buf,
        DWIN_RX_BUF_SIZE
    );
}


void dwin_handle_touch(uint8_t *p, uint16_t len)
{
    uint16_t vp =
        ((uint16_t)p[4] << 8) | p[5];

    uint8_t page_id    = p[6];
    uint8_t control_id = p[8];

    DebugPrint(
        "BTN: VP=0x%04X Page=%d Ctrl=%d\r\n",
        vp, page_id, control_id
    );

}




void dwin_process_frame(uint8_t *buf, uint16_t len)
{
    uint8_t cmd = buf[3];

    if (cmd == 0x83)   // Return Key Code
    {
        uint16_t vp = (buf[4] << 8) | buf[5];
        uint8_t key_event = buf[6];
        uint8_t key_id = buf[8];

        DebugPrint(
            "[DWIN] Key VP=0x%04X Event=%d ID=%d\r\n",
            vp, key_event, key_id
        );

        /* Example mapping */
        if (vp == 0x2001 && key_id == 0x01)
        {
            DebugPrint("Button 1 pressed\r\n");
        }
        else if (vp == 0x2002 && key_id == 0x02)
        {
            DebugPrint("Button 2 pressed\r\n");
        }
    }
}


void dwin_rx_fsm(void)
{
    if (dwin_rx.state != DWIN_RX_READY)
        return;

    /* Basic validation */
    if (dwin_rx.len < 6)
    {
        dwin_rx.state = DWIN_RX_ERROR;
        return;
    }

    if (dwin_rx.buf[0] != 0x5A || dwin_rx.buf[1] != 0xA5)
    {
    	DebugPrint("B\r\n");
        dwin_rx.state = DWIN_RX_ERROR;
        return;
    }

    uint8_t payload_len = dwin_rx.buf[2];

    if ((uint16_t)payload_len + 3u != dwin_rx.len)
    {
    	DebugPrint("C\r\n");
        dwin_rx.state = DWIN_RX_ERROR;
        return;
    }

#if DWIN_USE_CRC
    /* Need at least CRC16 */
    if (payload_len < 2u)
    {
        dwin_rx.state = DWIN_RX_ERROR;
        return;
    }

    for(int i=0; i<dwin_rx.len;i++)
    	DebugPrint("%02X ",dwin_rx.buf[i]);
    DebugPrint("\r\n");

    /* Verify CRC (CRC is last 2 bytes: HI then LO) */
    uint16_t rx_crc = (	dwin_rx.buf[payload_len + 1] << 8 |
    					dwin_rx.buf[payload_len + 2]);
    uint16_t calc_crc = dwin_crc16(&dwin_rx.buf[3], payload_len - 2);


    if (rx_crc != calc_crc)
    {
        DebugPrint(
            "[DWIN] CRC FAIL: rx=0x%04X calc=0x%04X len=%u\r\n",
            rx_crc, calc_crc, dwin_rx.len
        );
        dwin_rx.state = DWIN_RX_ERROR;
        return;
    }


#endif

    DebugPrint(
        "[DWIN] CRC SUCCESS: rx=0x%04X calc=0x%04X len=%u\r\n",
        rx_crc, calc_crc, dwin_rx.len
    );
    /* Valid frame → decode */
    /* Pass length without CRC to the decoder (safer if you later parse tail) */
#if DWIN_USE_CRC
    dwin_process_frame(dwin_rx.buf, (uint16_t)(dwin_rx.len - 2u));
#else
    dwin_process_frame(dwin_rx.buf, dwin_rx.len);
#endif

    dwin_rx.state = DWIN_RX_IDLE;
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  DebugPrint("\r\n=== UART3 DMA RX Circular + TX Normal ===\r\n");

  DebugPrint("\r\n=== DWIN LCD Test ===\r\n");

  HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, sizeof(dma_rx_buffer));

  dwin_rx_start();

  dwin_uart4_test_tx();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  check_dma_data();
	  dwin_rx_fsm();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart == &huart3)
    {
        dbg_tail = queue_next(dbg_tail);

        if (dbg_tail != dbg_head)
        {
            // More messages pending
            HAL_UART_Transmit_DMA(
                &huart3,
                (uint8_t *)debug_queue[dbg_tail],
                strlen(debug_queue[dbg_tail])
            );
        }
        else
        {
            dbg_busy = 0;
        }
    }
}



void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size
)
{
    if (huart == &huart4)
    {
        dwin_rx.len = Size;
        dwin_rx.state = DWIN_RX_READY;

        /* Restart reception immediately */
        HAL_UARTEx_ReceiveToIdle_IT(
            &huart4,
            dwin_rx.buf,
            DWIN_RX_BUF_SIZE
        );
    }
}

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
