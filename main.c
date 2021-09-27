/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIGHT_GREEN 1
#define LIGHT_YELLOW 4
#define LIGHT_RED 2
#define BUTTON_INTERVAL 10
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
int ButtonGetState(void);
void LightToggle(int flags);
void LightSet(int flags, int state);

uint32_t MeasureButtonPressTime(void);
int IsLong(uint32_t time);
void AnimateOverflow(int n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int ButtonGetState(void) {
	switch (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) {
	case GPIO_PIN_SET:
		return 0;
	default:
		return 1;
	}
}

int ButtonGetStateSmooth(void) {
	int sum = 0;
	for (int i = 0; i < BUTTON_INTERVAL; ++i) {
		sum += ButtonGetState();
		HAL_Delay(1);
	}
	return 2 * sum > BUTTON_INTERVAL ? 1 : 0;
}

void LightToggle(int flags) {
	if (flags & LIGHT_GREEN) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	}
	if (flags & LIGHT_YELLOW) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	}
	if (flags & LIGHT_RED) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	}
}

void LightSet(int flags, int state) {
	if (flags & LIGHT_GREEN) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, state);
	}
	if (flags & LIGHT_YELLOW) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, state);
	}
	if (flags & LIGHT_RED) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, state);
	}
}

uint32_t MeasureButtonPressTime(void) {
	while (!ButtonGetStateSmooth()) {
		/* Waiting while button is not pressed */
	}
	uint32_t start = HAL_GetTick();
	while (ButtonGetStateSmooth()) {
		/* Wait while button is pressed */
	}
	uint32_t end = HAL_GetTick();
	return end - start;
}

int IsLong(uint32_t time) {
	return time > 500;
}

void AnimateOverflow(int n) {
	LightSet(LIGHT_GREEN | LIGHT_RED, 0);
	HAL_Delay(250);
	for (int i = 0; i < 2; ++i) {
		LightToggle(LIGHT_GREEN | LIGHT_RED);
		HAL_Delay(500);
	}

	for (int i = 0; i < 2 * n; ++i) {
		LightToggle(LIGHT_GREEN);
		HAL_Delay(250);
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */
	LightSet(LIGHT_GREEN | LIGHT_YELLOW | LIGHT_RED, 0);
	int counter = 0;
	int totalOverflows = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		LightSet(LIGHT_GREEN, counter & 1);
		LightSet(LIGHT_RED, (counter >> 1) & 1);

		if (IsLong(MeasureButtonPressTime()))
			--counter;
		else
			++counter;

		if (counter >= 4) {
			counter -= 4;
			++totalOverflows;
			AnimateOverflow(totalOverflows);
		} else if (counter < 0) {
			counter += 4;
			--totalOverflows;
			if (totalOverflows < 0)
				totalOverflows = 0;
			AnimateOverflow(totalOverflows);
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
