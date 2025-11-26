/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/*
 * Project is DMA ADC multichannel read(IN8,IN9)
 * ADC1 AN8(PB0) and AN9(PB1) activated and read separately. Rank:1 for IN8, and
 * Rank:2 for IN9 adjustments were important(Number of Conversion: 2 in Mx and
 * Scan Conversion Mode: Enabled, Continous Conversion Mode: Enabled, DMA Conversion Request:
 * Enabled). HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_data,2); is outside the while loop.
 * adc_data[2] values are seen during Debug -> Live Expression .
 *  * */
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t dma_rx_buffer[64];
uint16_t old_pos = 0;
uint8_t encoder_push_button = 0;
uint16_t adc_data[2];


#define THRESHOLD       2047        // Dijital esik degeri (örnegin 1.65V esik = 4095 * 1.65 / 3.3 ? 2047)
// Encoder pozisyonu ve yönü

int encoder_position = 0;
char encoder_direction = 'S'; // S: Sabit, CW: Saat yönü, CCW: Saat tersi yönü

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DebugPrint(const char *format, ...)
{
    static char uartBuffer[250];  // <--- persists after function returns
    va_list args;
    va_start(args, format);
    vsnprintf(uartBuffer, sizeof(uartBuffer), format, args);
    va_end(args);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer));
}

void check_dma_data(void) {
    uint16_t pos = sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    if (pos != old_pos) {
        if (pos > old_pos) {
            // Print new data
            for (uint16_t i = old_pos; i < pos; i++) {
                DebugPrint("\r\nGot: '%c' (0x%02X) at pos %d\r\n\r\n",
                       dma_rx_buffer[i], dma_rx_buffer[i], i);

            }
        }
        old_pos = pos;
    }
}


/**
 * @brief Convert ADC raw value to voltage
 * @param adc_raw: Raw ADC value (0-4095)
 * @retval float: Voltage (0.0-3.3V)
 */
float adc_to_voltage(uint16_t adc_raw)
{
    return ((float)adc_raw * 3.3f) / 4095.0f;
}


//void Encoder_Read(void)
//{
//    static uint8_t prev_state = 0;
//
//    uint8_t A = (adc_data[0] > THRESHOLD) ? 1 : 0;
//    uint8_t B = (adc_data[1] > THRESHOLD) ? 1 : 0;
//
//    uint8_t current_state = (A << 1) | B;
//
//    if ((prev_state == 0 && current_state == 1) ||
//        (prev_state == 1 && current_state == 3) ||
//        (prev_state == 3 && current_state == 2) ||
//        (prev_state == 2 && current_state == 0)) {
//        encoder_position--;
//        encoder_direction = 'L'; // CW
//    }
//    else if ((prev_state == 0 && current_state == 2) ||
//             (prev_state == 2 && current_state == 3) ||
//             (prev_state == 3 && current_state == 1) ||
//             (prev_state == 1 && current_state == 0)) {
//        encoder_position++;
//        encoder_direction = 'R'; // CCW
//    } else {
//        encoder_direction = 'S'; // Sabit / Bos geçis
//    }
//
//    prev_state = current_state;
//
//}

void KY040_Simple_Read(void)
{
    static uint8_t last_clk = 0;

    // Read current pin states
    uint8_t clk = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);  // CLK pin
    uint8_t dt = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);   // DT pin

    // Detect falling edge on CLK
    if (last_clk == 1 && clk == 0) {
        if (dt == 0) {
            // CLK and DT both low = Clockwise
        	if(encoder_position < 50)
        	{
        		encoder_position++;
        		encoder_direction = 'R';
        	}

        	else if(encoder_position >= 50)
        	{
        		encoder_position = 50;
        		encoder_direction = 'S';
        	}


        } else {
            // CLK low, DT high = Counter-clockwise
        		if(encoder_position > 0)
        		{
        			encoder_position--;
        			encoder_direction = 'L';

        		}else if(encoder_position <= 0)

        		{
        			encoder_position = 0;
        	        encoder_direction = 'S';
        	    }

        }
    } else {
        encoder_direction = 'S';  // No movement
    }


    last_clk = clk;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, dma_rx_buffer, sizeof(dma_rx_buffer));
  DebugPrint("DMA started! Type something...\r\n\r\n");

//  HAL_ADC_Start_IT(&hadc1);
//  HAL_ADCEX_CALIBRATION_START(&HADC1,ADC_SINGLE_ENDED);	//The function is NOT available on STM32F4 series! This function only exists on newer STM32 families like F3, L4, G4, H7, etc.
//  HAL_ADC_START_DMA(&HADC1,(UINT32_T *)ADC_DATA,2);

//  DebugPrint("Raw ADC: adc_data[0]=%d, adc_data[1]=%d\r\n", adc_data[0], adc_data[1]);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  DebugPrint("Raw ADC: adc_data[0]=%d, adc_data[1]=%d\r\n", adc_data[0], adc_data[1]);
	  KY040_Simple_Read();
	  DebugPrint("POS: %d, DIR: %c, BUT: %d\r\n", encoder_position,encoder_direction,encoder_push_button);
	  if(encoder_push_button)
	  {
		  encoder_push_button = 0;
		  HAL_Delay(1000);
	  }

	  HAL_Delay(10);

//	  check_dma_data();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 3);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 3);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 4);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	encoder_push_button = 1;
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
