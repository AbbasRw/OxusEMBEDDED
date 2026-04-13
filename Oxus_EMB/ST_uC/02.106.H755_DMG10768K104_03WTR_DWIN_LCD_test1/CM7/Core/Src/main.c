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

UART_HandleTypeDef huart1;
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


uint8_t dwin_rx_buf[32];
uint8_t dwin_rx_idx = 0;
uint8_t dwin_rx_byte;




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
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
            len += snprintf(&msg[len], sizeof(msg) - len,
                            "[%lu] %02X '%c'\r\n",
                            total_pos++, dma_rx_buffer[i],
                            dma_rx_buffer[i]);
        }
    }
    else
    {
        for (uint16_t i = old_pos; i < sizeof(dma_rx_buffer); i++)
        {
            len += snprintf(&msg[len], sizeof(msg) - len,
                            "[%lu] %02X '%c'\r\n",
                            total_pos++, dma_rx_buffer[i],
                            dma_rx_buffer[i]);
        }

        for (uint16_t i = 0; i < pos; i++)
        {
            len += snprintf(&msg[len], sizeof(msg) - len,
                            "[%lu] %02X '%c'\r\n",
                            total_pos++, dma_rx_buffer[i],
                            dma_rx_buffer[i]);
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


void dwin_uart2_test_tx(void)
{
    uint8_t tx_buf[8] = {
        0x5A,
        0xA5,
        0x05,
        0x82,
        0x20,
        0x04,
        0x00,
        0xFF
    };

    HAL_UART_Transmit(&huart4, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
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

//    // Example mapping
//    if (page_id == 1 && control_id == 1)
//    {
//        sys.sys_command.pause_toggle = true;
//    }
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
        dwin_rx.state = DWIN_RX_ERROR;
        return;
    }

    uint8_t payload_len = dwin_rx.buf[2];

    if (payload_len + 3 != dwin_rx.len)
    {
        dwin_rx.state = DWIN_RX_ERROR;
        return;
    }

    /* Valid frame → decode */
    dwin_process_frame(dwin_rx.buf, dwin_rx.len);

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  DebugPrint("\r\n=== UART3 DMA RX Circular + TX Normal ===\r\n");

  DebugPrint("\r\n=== DWIN LCD Test ===\r\n");

//  HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, sizeof(dma_rx_buffer));

  HAL_UART_Receive_IT(&huart1, &dwin_rx_byte, 1);

//  HAL_StatusTypeDef ret;
//
//  const char msg[] = "UART2 TX OK\r\n";

//  ret = HAL_UART_Transmit(&huart2,(uint8_t *)msg, sizeof(msg) - 1,HAL_MAX_DELAY);
//
//  if(ret == HAL_OK)
//	  DebugPrint("\r\n=== UART2 TX is working");
//  else
//	  DebugPrint("\r\n=== UART2 error");

  dwin_rx_start();

  dwin_uart2_test_tx();
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart4.Instance = USART2;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart1)
//    {
//        dwin_rx_buf[dwin_rx_idx++] = dwin_rx_byte;
//
//        if (dwin_rx_idx >= 9)
//        {
//            dwin_process_rx(dwin_rx_buf, dwin_rx_idx);
//            dwin_rx_idx = 0;
//        }
//
//        // Re-arm RX
//        HAL_UART_Receive_IT(&huart1, &dwin_rx_byte, 1);
//    }
//}


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
