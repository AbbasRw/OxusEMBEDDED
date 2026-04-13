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
#include "AS5600.h"
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

I2C_HandleTypeDef hi2c2;

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



as5600_t encoder;

void AS5600_Init(as5600_t *enc);
void AS5600_Update(as5600_t *enc, I2C_HandleTypeDef *hi2c);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
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



void AS5600_Init(as5600_t *enc)
{
    enc->raw_angle = 0;
    enc->angle_deg = 0.0f;
    enc->comm_ok = 0;
    enc->magnet_detected  = 0;
    enc->magnet_too_weak  = 0;
    enc->magnet_too_strong= 0;

//    AS5600_SetZeroPosition(&hi2c2, 0x000);  // 0 deg
//    AS5600_SetMaxPosition (&hi2c2, 0xFFF);  // full range
//    AS5600_SetMaxAngle    (&hi2c2, 0xFFF);  // 360 deg range
}

void AS5600_Update(as5600_t *enc, I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2];
    uint8_t val;
    uint16_t val_mag;

    if (HAL_I2C_Mem_Read(hi2c,
                         AS5600_I2C_ADDR,
						 AS5600_REG_RAW_ANGLE_H,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         2,
                         10) == HAL_OK)
    {
        uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
        raw &= 0x0FFF;  // 12-bit mask

        enc->raw_angle = raw;
        enc->angle_deg = (raw * 360.0f) / 4096.0f;

//        if (AS5600_GetStatus(hi2c, &val) == HAL_OK)
//        {
//            enc->magnet_detected  = (val >> 5) & 0x01;
//            enc->magnet_too_weak  = (val >> 4) & 0x01;
//            enc->magnet_too_strong= (val >> 3) & 0x01;
//
//            DebugPrint("MD:%d ML:%d MH:%d\r\n",
//                       enc->magnet_detected,
//                       enc->magnet_too_weak,
//                       enc->magnet_too_strong);
//        }
//
        if (AS5600_GetAGC(hi2c, &val) == HAL_OK)
        {
            enc->agc_value_read  = val;
        }


        if (AS5600_GetMagnitude12(hi2c, &val_mag) == HAL_OK)
        {
        	enc->mag_value_read = val_mag;
            DebugPrint("AGC:%d MAG:%d \r\n",
            			enc->agc_value_read,
						enc->mag_value_read);
        }
//
//        if (AS5600_GetCONF16(hi2c, &val_mag) == HAL_OK)
//        {
//
//            enc->conf_PM  	= (val 		) & 0x03;
//            enc->conf_HYST 	= (val >> 2	) & 0x03;
//            enc->conf_OUTS 	= (val >> 4	) & 0x03;
//            enc->conf_PWMF 	= (val >> 6	) & 0x03;
//            enc->conf_SF 	= (val >> 8	) & 0x03;
//            enc->conf_FTH 	= (val >> 10) & 0x07;
//            enc->conf_WD 	= (val >> 13) & 0x01;
//
//            DebugPrint("PM:%d HYST:%d OUTS:%d PWMF:%d SF:%d FTH:%d WD:%d \r\n",
//            			enc->conf_PM,
//						enc->conf_HYST,
//						enc->conf_OUTS,
//						enc->conf_PWMF,
//						enc->conf_SF,
//						enc->conf_FTH,
//						enc->conf_WD);
//        }
        DebugPrint("\r\n");


        enc->comm_ok = 1;
    }
    else
    {
        enc->comm_ok = 0;
    }
}


//void AS5600_ConfigExample(void)
//{
//    uint16_t conf = 0;
//
//    // 1) Set a software-defined zero window (example values)
//    AS5600_SetZeroPosition(&hi2c4, 0x000);  // 0 deg
//    AS5600_SetMaxPosition (&hi2c4, 0xFFF);  // full range
//    AS5600_SetMaxAngle    (&hi2c4, 0xFFF);  // 360 deg range
//
//    // 2) Read current CONF
//    if (AS5600_ReadCONF(&hi2c4, &conf) == HAL_OK)
//    {
//        DebugPrint("AS5600 CONF = 0x%04X\r\n", conf);
//    }
//    else
//    {
//        DebugPrint("AS5600 CONF read error\r\n");
//    }
//
//    // 3) Optionally modify CONF bits (example: leave unchanged)
//    // conf = conf | SOME_BITS;
//    // AS5600_WriteCONF(&hi2c4, conf);
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, sizeof(dma_rx_buffer));


  DebugPrint("-----------AS5600 Magnetic Encoder I2C Test-----------\n\r");

  AS5600_Init(&encoder);


  static uint32_t last = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (HAL_GetTick() - last >= 100)   // 100 ms period
	  {
		  last = HAL_GetTick();

		  AS5600_Update(&encoder, &hi2c2);

		  if (encoder.comm_ok)
		  {
			  DebugPrint("Angle: %4u  deg: %6.3f \r\n",encoder.raw_angle,encoder.angle_deg);
		  }
		  else
		  {
			  DebugPrint("AS5600 I2C error\r\n");
		  }

	  }

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00401242;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Enable Fast Mode Plus
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
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
