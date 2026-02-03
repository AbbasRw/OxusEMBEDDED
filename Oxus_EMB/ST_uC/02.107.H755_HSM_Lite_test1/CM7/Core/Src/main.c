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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 * SYSTEM
	 ├── IDLE
	 ├── RUNNING
	 │     ├── INFUSING
	 │     └── PAUSED
	 └── ALARM

	 typedef enum {
		EVT_NONE,
		EVT_START,
		EVT_PAUSE,
		EVT_RESUME,
		EVT_STOP,
		EVT_ALARM
	} Event_t;

 * */
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

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

typedef enum
{
    EVT_NONE = 0,
    EVT_START,
    EVT_PAUSE,
    EVT_RESUME,
    EVT_STOP,
    EVT_ALARM
} Event_t;


#define DEBUG_QUEUE_SIZE 8
#define DEBUG_MSG_LEN    128

static char debug_queue[DEBUG_QUEUE_SIZE][DEBUG_MSG_LEN];
static volatile uint8_t dbg_head = 0;
static volatile uint8_t dbg_tail = 0;
static volatile uint8_t dbg_busy = 0;

uint8_t dma_rx_buffer[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

typedef void (*StateHandler_t)(Event_t evt);


void State_IDLE(Event_t evt);
void State_RUNNING(Event_t evt);
void State_INFUSING(Event_t evt);
void State_PAUSED(Event_t evt);
void State_ALARM(Event_t evt);



static StateHandler_t currentState;

//static void (*currentState)(Event_t evt);


const char* state_name(StateHandler_t st)
{
    if (st == State_IDLE) return "IDLE";
    if (st == State_INFUSING) return "INFUSING";
    if (st == State_PAUSED) return "PAUSED";
    if (st == State_ALARM) return "ALARM";
    return "UNKNOWN";
}







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


#define TRANSITION(new_state)                            \
    do {                                                 \
        DebugPrint("STATE: %s -> %s\r\n",                \
                   state_name(currentState),             \
                   state_name(new_state));               \
        currentState = (new_state);                      \
    } while (0)



//IDLE
void State_IDLE(Event_t evt)
{
    if (evt == EVT_START)
    {
        TRANSITION(State_INFUSING);
    }
}

//RUNNING (PARENT)
void State_RUNNING(Event_t evt)
{
    switch (evt)
    {
        case EVT_STOP:
            TRANSITION(State_IDLE);
            break;

        case EVT_ALARM:
            TRANSITION(State_ALARM);
            break;

        default:
            break;
    }
}


//INFUSING (child)
void State_INFUSING(Event_t evt)
{
    if (evt == EVT_PAUSE)
    {
        TRANSITION(State_PAUSED);
    }
    else
    {
        State_RUNNING(evt);   // fallback
    }
}



//PAUSED (child)
void State_PAUSED(Event_t evt)
{
    if (evt == EVT_RESUME)
    {
        TRANSITION(State_INFUSING);
    }
    else
    {
        State_RUNNING(evt);   // fallback
    }
}


//ALARM (preemptive)
void State_ALARM(Event_t evt)
{
    if (evt == EVT_STOP)
    {
        TRANSITION(State_IDLE);
    }
}


void HSM_Dispatch(Event_t evt)
{
    if (currentState && evt != EVT_NONE)
    {
        currentState(evt);
    }
}


Event_t parse_char(uint8_t c)
{
    switch (c)
    {
        case 's': return EVT_START;
        case 'p': return EVT_PAUSE;
        case 'r': return EVT_RESUME;
        case 'x': return EVT_STOP;
        case 'a': return EVT_ALARM;
        default:  return EVT_NONE;
    }
}
//static void handle_command(const char *cmd)
//{
//    if (strcmp(cmd, "inject start") == 0)
//    {
//        DebugPrint("CMD OK: Injection started\r\n");
//        // 👉 Call your function here
//        // inject_start();
//    }
//    else if (strcmp(cmd, "inject stop") == 0)
//    {
//        DebugPrint("CMD OK: Injection stopped\r\n");
//        // inject_stop();
//    }
//    else
//    {
//        DebugPrint("Unknown command: %s\r\n", cmd);
//    }
//}



//void check_dma_data(void)
//{
//    static uint16_t old_pos = 0;
//    uint16_t pos =
//        sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
//
//    static char cmd_buf[64];
//    static uint8_t cmd_len = 0;
//
//    if (pos == old_pos)
//        return;
//
//    while (old_pos != pos)
//    {
//        char c = dma_rx_buffer[old_pos++];
//        if (old_pos >= sizeof(dma_rx_buffer))
//            old_pos = 0;
//
//        // Line termination
//        if (c == '\r' || c == '\n')
//        {
//            if (cmd_len > 0)
//            {
//                cmd_buf[cmd_len] = '\0';
//                handle_command(cmd_buf);
//                cmd_len = 0;
//            }
//        }
//        else
//        {
//            if (cmd_len < sizeof(cmd_buf) - 1)
//            {
//                cmd_buf[cmd_len++] = c;
//            }
//        }
//    }
//}



void check_dma_data(void)
{
    static uint16_t old_pos = 0;
    static uint32_t total_pos = 0;

    uint16_t pos = sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

    if (pos == old_pos)
        return;

    /* ---------- No wrap ---------- */
    if (pos > old_pos)
    {
        for (uint16_t i = old_pos; i < pos; i++)
        {
            uint8_t c = dma_rx_buffer[i];
            Event_t evt = parse_char(c);

//            DebugPrint("[%lu] RX: 0x%02X '%c'\r\n",
//                       total_pos++, c, c);

            if (evt != EVT_NONE)
            {
                DebugPrint("EVENT detected: '%c'\r\n\r\n", c);
                HSM_Dispatch(evt);
            }
        }
    }
    /* ---------- Wrap ---------- */
    else
    {
        for (uint16_t i = old_pos; i < sizeof(dma_rx_buffer); i++)
        {
            uint8_t c = dma_rx_buffer[i];
            Event_t evt = parse_char(c);

//            DebugPrint("[%lu] RX: 0x%02X '%c'\r\n",
//                       total_pos++, c, c);

            if (evt != EVT_NONE)
            {
                DebugPrint("EVENT detected: '%c'\r\n\r\n", c);
                HSM_Dispatch(evt);
            }
        }

        for (uint16_t i = 0; i < pos; i++)
        {
            uint8_t c = dma_rx_buffer[i];
            Event_t evt = parse_char(c);

//            DebugPrint("[%lu] RX: 0x%02X '%c'\r\n",
//                       total_pos++, c, c);

            if (evt != EVT_NONE)
            {
                DebugPrint("EVENT detected: '%c'\r\n\r\n", c);
                HSM_Dispatch(evt);
            }
        }
    }

    old_pos = pos;
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
  /* USER CODE BEGIN 2 */




  HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, sizeof(dma_rx_buffer));

  currentState = State_IDLE;
  DebugPrint("-----------HSM Test-----------\n\r");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  check_dma_data();
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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
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
