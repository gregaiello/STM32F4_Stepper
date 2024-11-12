/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
#define ADC_WINDOW_SAMPLES 10
#define ADC_WINDOW_CALIBRATION 100
#define ADC_TIMEOUT 100
#define ADC_MAX 4095
#define ADC_MIN 0

#define POT_HIGH_THRESHOLD 2500
#define POT_LOW_THRESHOLD 1500

#define PWM_MAX_DUTY 100
#define PWM_50_DUTY PWM_MAX_DUTY>>1
#define PWM_MIN_DUTY 0

typedef enum {
	STATE_INIT,
	STATE_ADC,
	STATE_PWM,
	STATE_ERROR,
	STATE_MAX
} stateMachine_t;

typedef enum {
	MOTOR_1,
	MOTOR_2,
	MOTOR_MAX
} motor_t;

typedef struct
{
	motor_t motor;
    uint16_t dir;
    uint16_t dc;
    uint16_t pot;
    uint16_t baseline;
} motorControl_t;

stateMachine_t internalState = STATE_INIT;
motorControl_t motor1;
motorControl_t motor2;

TIM_OC_InitTypeDef sConfigOC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void motor_calibration(motorControl_t *motor1, motorControl_t *motor2) {
	  uint32_t sum1 = 0, sum2 = 0;
	  static uint32_t cnt_error1 = 0, cnt_error2 = 0 ;

	  for(int i = ADC_WINDOW_CALIBRATION; i > 0; i--) {
		  HAL_ADC_Start(&hadc1);
		  if(HAL_OK != HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT)) {
			  cnt_error1++;
		  }
		  sum1 += HAL_ADC_GetValue(&hadc1);
		  if(HAL_OK != HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT)) {
			  cnt_error2++;
		  }
		  sum2 += HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);
	  }

	  motor1->baseline = (uint16_t)(sum1 / ADC_WINDOW_CALIBRATION);
	  motor2->baseline = (uint16_t)(sum2 / ADC_WINDOW_CALIBRATION);
}

void motor_init(motorControl_t *motor1, motorControl_t *motor2) {
	  motor1->dc = PWM_MIN_DUTY;
	  motor2->dc = PWM_MIN_DUTY;
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1->dc);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor2->dc);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}


void motor_getValue(motorControl_t *motor1, motorControl_t *motor2) {
	  uint32_t sum1 = 0, sum2 = 0;
	  static uint32_t cnt_error1 = 0, cnt_error2 = 0 ;

	  for(int i = ADC_WINDOW_SAMPLES; i > 0; i--) {
		  HAL_ADC_Start(&hadc1);
		  if(HAL_OK != HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT)) {
			  cnt_error1++;
		  }
		  sum1 += HAL_ADC_GetValue(&hadc1);
		  if(HAL_OK != HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT)) {
			  cnt_error2++;
		  }
		  sum2 += HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);
	  }

	  motor1->pot = (uint16_t)(sum1 / ADC_WINDOW_SAMPLES);
	  motor2->pot = (uint16_t)(sum2 / ADC_WINDOW_SAMPLES);
}

void motor_pwm_setvalue(motorControl_t *motor1, motorControl_t *motor2) {

	if(motor1->pot > POT_HIGH_THRESHOLD) {
		motor1->dir = 1;
		motor1->dc = PWM_50_DUTY;
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);

		// DIR pin
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, GPIO_PIN_SET);
	}
	else if(motor1->pot < POT_LOW_THRESHOLD) {
		motor1->dir = 0;
		motor1->dc = PWM_50_DUTY;
		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
		// DIR pin
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, GPIO_PIN_RESET);
	}
	else {
		motor1->dir = 0;
		motor1->dc = PWM_MIN_DUTY;
		HAL_GPIO_WritePin(GPIOD, LD3_Pin|LD6_Pin, GPIO_PIN_RESET);
		// DIR pin
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, GPIO_PIN_RESET);
	}

	if(motor2->pot > POT_HIGH_THRESHOLD) {
		motor2->dir = 1;
		motor2->dc = PWM_50_DUTY;
		HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
		// DIR pin
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, GPIO_PIN_SET);
	}
	else if(motor2->pot < POT_LOW_THRESHOLD) {
		motor2->dir = 0;
		motor2->dc = PWM_50_DUTY;
		HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
		// DIR pin
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, GPIO_PIN_RESET);
	}
	else {
		motor2->dir = 0;
		motor2->dc = PWM_MIN_DUTY;
		HAL_GPIO_WritePin(GPIOD, LD5_Pin|LD4_Pin, GPIO_PIN_RESET);
		// DIR pin
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, GPIO_PIN_RESET);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = motor1->dc;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1->dc);
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = motor2->dc;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor2->dc);
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(internalState) {
	  case STATE_INIT:
		  memset(&motor1, 0, sizeof(motorControl_t));
		  memset(&motor2, 0, sizeof(motorControl_t));

		  motor1.motor = MOTOR_1;
		  motor2.motor = MOTOR_2;

		  motor_calibration(&motor1, &motor2);

		  motor_init(&motor1, &motor2);

		  internalState = STATE_ADC;
		  break;
	  case STATE_ADC:
		  motor_getValue(&motor1, &motor2);
		  internalState = STATE_PWM;
		  break;
	  case STATE_PWM:

		  motor_pwm_setvalue(&motor1, &motor2);

//		  HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, GPIO_PIN_RESET);
//		  for(int i = 0; i<100000;i++) {
//			  HAL_GPIO_WritePin(GPIOE, PUL_M1_Pin, GPIO_PIN_SET);
//			  HAL_GPIO_WritePin(GPIOE, PUL_M2_Pin, GPIO_PIN_SET);
//			  HAL_Delay(2);
//			  HAL_GPIO_WritePin(GPIOE, PUL_M1_Pin, GPIO_PIN_RESET);
//			  HAL_GPIO_WritePin(GPIOE, PUL_M2_Pin, GPIO_PIN_RESET);
//			  HAL_Delay(1);
//		  }
//
//		  HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, GPIO_PIN_SET);
//		  for(int i = 0; i<100000;i++) {
//			  HAL_GPIO_WritePin(GPIOE, PUL_M1_Pin, GPIO_PIN_SET);
//			  HAL_GPIO_WritePin(GPIOE, PUL_M2_Pin, GPIO_PIN_SET);
//			  HAL_Delay(2);
//			  HAL_GPIO_WritePin(GPIOE, PUL_M1_Pin, GPIO_PIN_RESET);
//			  HAL_GPIO_WritePin(GPIOE, PUL_M2_Pin, GPIO_PIN_RESET);
//			  HAL_Delay(1);
//		  }

		  internalState = STATE_ADC;
		  break;
	  case STATE_ERROR:
		  break;
	  default:
		  break;
	  }
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
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
