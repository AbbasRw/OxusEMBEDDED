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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;


uint8_t dma_rx_buffer[64];
uint8_t dma_tx_buffer[64];
uint16_t old_pos = 0;

static volatile bool uart_tx_busy = false;
static char uartBuffer[250];


volatile uint32_t step_target = 0;
volatile uint32_t step_counter = 0;
volatile uint8_t is_moving = 0;

typedef enum
{
    STEP_FULL  = 1,
    STEP_HALF  = 2,
    STEP_QUART = 4,
    STEP_EIGHTH= 8,
    STEP_SIXTEEN = 16
} MicrostepMode_t;


#define MOTOR_FULL_STEPS_PER_REV   200

typedef struct
{
    MicrostepMode_t microstep;
    uint32_t rpm;
    uint32_t frequency;  // derived frequency for TIM
} StepperMotor_t;


StepperMotor_t motor = {
    .microstep = STEP_FULL,
    .rpm = 0,
    .frequency = 0
};


uint32_t Stepper_CalcFrequency(uint32_t rpm, MicrostepMode_t micro)
{
    uint32_t effective_steps = MOTOR_FULL_STEPS_PER_REV * micro;

    // F = RPM × StepsPerRev / 60
    uint32_t freq = (rpm * effective_steps) / 60;

    // No zero frequency allowed
    return (freq < 1) ? 1 : freq;
}


void Stepper_SetFrequency(uint32_t freq)
{
	if (freq == 0) return;

    uint32_t timer_clk = 200000000;    //system clock
    uint32_t period = (timer_clk / freq) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim5, period);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, period / 2);
}


void Stepper_SetRPM(uint32_t rpm)
{
    if (rpm > 120) rpm = 120; // safety cap

    motor.rpm = rpm;
    motor.frequency = Stepper_CalcFrequency(rpm, motor.microstep);

    Stepper_SetFrequency(motor.frequency);   // updates TIM5 PWM
}


uint32_t Stepper_DegreesToSteps(float degrees)
{
    float steps_per_rev = 200.0f * motor.microstep;
    return (uint32_t)((degrees / 360.0f) * steps_per_rev);
}



void Stepper_SetMicrostep(MicrostepMode_t micro)
{
    motor.microstep = micro;

    // Set GPIO pins (MS1 PF7, MS2 PF8, MS3 PF9)
    switch(micro)
    {
        case STEP_FULL:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
            break;

        case STEP_HALF:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
            break;

        case STEP_QUART:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);
            break;

        case STEP_EIGHTH:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
            break;

        case STEP_SIXTEEN:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
            break;
    }

//    // Recalculate the final PWM frequency for TIM5
//    motor.frequency = Stepper_CalcFrequency(motor.rpm, micro);
//
//    Stepper_SetFrequency(motor.frequency);
}



void DebugPrint(const char *format, ...)
{
	// Wait for previous transmission to complete
	    while (uart_tx_busy);

	    va_list args;
	    va_start(args, format);
	    vsnprintf(uartBuffer, sizeof(uartBuffer), format, args);
	    va_end(args);

	    uart_tx_busy = true;

	    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)uartBuffer, strlen(uartBuffer));
}

void check_dma_data(void)
{
    static uint32_t total_pos = 0;  // new: global linear counter

    uint16_t pos = sizeof(dma_rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

    if (pos != old_pos)
    {
        if (pos > old_pos)
        {
            // Normal forward move (no wrap)
            for (uint16_t i = old_pos; i < pos; i++)
            {
                DebugPrint("\r\nGot: '%c' (0x%02X) at pos %lu\r\n\r\n",
                           dma_rx_buffer[i], dma_rx_buffer[i], total_pos++);
            }
        }
        else
        {
            // Wrapped around — handle end part and restart from 0
            for (uint16_t i = old_pos; i < sizeof(dma_rx_buffer); i++)
            {
                DebugPrint("\r\nGot: '%c' (0x%02X) at pos %lu\r\n\r\n",
                           dma_rx_buffer[i], dma_rx_buffer[i], total_pos++);
            }
            for (uint16_t i = 0; i < pos; i++)
            {
                DebugPrint("\r\nGot: '%c' (0x%02X) at pos %lu\r\n\r\n",
                           dma_rx_buffer[i], dma_rx_buffer[i], total_pos++);
            }
        }

        old_pos = pos;
    }
}


void stepperMotor_EN_ENABLE(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

void stepperMotor_EN_DISABLE(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void stepperMotor_SLP_ENABLE(void)
{

}

void stepperMotor_SLP_DISABLE(void)
{

}

void stepperMotor_RST_ENABLE(void)
{

}


void stepperMotor_RST_DISABLE(void)
{

}


void stepperMotor_DIR_CCW(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}

void stepperMotor_DIR_CW(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}


void A4988_SetMicrostepping_Default(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);  // MS1 → PF7
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);  // MS2 → PF8
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);  // MS3 → PF9
}


void Stepper_MoveDegrees(float degrees, uint32_t rpm)
{
    step_target = Stepper_DegreesToSteps(degrees);
    step_counter = 0;
    is_moving = 1;

    Stepper_SetRPM(rpm);               // frequency
    HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_4);
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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  DebugPrint("\r\n=== UART DMA RX Circular + TX Normal ===\r\n");
  DebugPrint("DMA started! Type something...\r\n");
  DebugPrint("1DMA started! Type something...\r\n");
  DebugPrint("2DMA started! Type something...\r\n");
  DebugPrint("3DMA started! Type something...\r\n");
  DebugPrint("4DMA started! Type something...\r\n");
  DebugPrint("5DMA started! Type something...\r\n");

  stepperMotor_EN_DISABLE();
  A4988_SetMicrostepping_Default();
  stepperMotor_EN_ENABLE();
  stepperMotor_DIR_CW();

  HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, sizeof(dma_rx_buffer));

  // Set microstep
  Stepper_SetMicrostep(STEP_FULL);

  Stepper_MoveDegrees(30.0f, 20);


  /*
   * PWM Frequency (Hz) = (RPM × StepsPerRev) / 60
   * Freq = RPM × 3.3333
   * because StepsPerRev = 200 per rev. 1.8 perstep
   * */
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

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        uart_tx_busy = false;  // Transmission complete
    }
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        step_counter++;

        if (step_counter >= step_target)
        {
            HAL_TIM_PWM_Stop_IT(&htim5, TIM_CHANNEL_4);
            is_moving = 0;
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
