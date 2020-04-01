/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// defines for setting and clearing register bits

typedef unsigned char byte;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int VPA0, VPA1, VPA4, VPA6, VPA7, VPA8, VPA9, VPA10;
uint32_t count;
uint32_t tim, lasttim=15;
uint32_t tBa, tFl1, tFl2, tSN, tTom, tHi, tCC, tRC;

/* SET Note */
// send ( note velocity note velocity )
// MIDI_TX(0000, [ON-OFF], Note, Velocity)

uint8_t noteOff[4] = {0x08, 0x80, 0x47, 0x47};
uint8_t BASS[4] = {0x08, 0x90, 0x24, 120};
uint8_t FloorTom1[4] = {0x08, 0x90, 0x41, 120};
uint8_t FloorTom2[4] = {0x08, 0x90, 0x43, 120};
uint8_t SNARE[4] = {0x08, 0x90, 0x26, 120};
uint8_t TOMS[4] = {0x08, 0x90, 0x45, 120};
uint8_t Hi_Hat[4] = {0x08, 0x90, 0x32, 120};
uint8_t Crash_CymBal[4] = {0x08, 0x90, 0x4D, 120};
uint8_t Ride_CymBal[4] = {0x08, 0x90, 0x3C, 120};
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Read_piezo_sensor();
void Map_Note();
void send_data_tx(uint8_t *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc[2], buffer[2];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc[0] = buffer[0];
	adc[1] = buffer[1];
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	/* Start timer interrupt */
	HAL_TIM_Base_Start_IT(&htim1);
	
	HAL_ADC_Start_DMA(&hadc1, buffer, 2);
	// HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/* Toggle LED PA5 */
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		
		/* Read status Piezo (push, release) */
		Read_piezo_sensor();
	
		
		/* check index sensor and send value to midi */
		Map_Note();
		
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Read_piezo_sensor() {
	VPA0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	VPA1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	VPA4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	VPA6 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	VPA7 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	VPA8 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	VPA9 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	VPA10 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
}

void map_time(uint32_t value) {
	tim = value / 10;		// ms
}

void Map_Note() {
	if(VPA0 && tim > (tBa+lasttim)) {
		tBa = tim;
		send_data_tx(BASS);
		// HAL_Delay(100);
	}
	if(VPA1 && tim > (tFl1+lasttim)) {
		tFl1 = tim;
		send_data_tx(FloorTom1);
		// HAL_Delay(100);
	}
	if(VPA4 && tim > (tFl2+lasttim)) {
		tFl2 = tim;
		send_data_tx(FloorTom2);
		// HAL_Delay(100);
	}
	/*
	if(VPA6 && tim > (tSN+lasttim)) {
		tSN = tim;
		send_data_tx(SNARE);
		// HAL_Delay(100);
	}
	*/
	/* ADC1 Snare */
	if(adc[0] > 50 && tim > (tSN+lasttim)) {
		if(adc[0] > 120) { tSN = tim;  SNARE[3]=110; send_data_tx( SNARE ); }
		else if(adc[0] > 110) { tSN = tim;  SNARE[3]=110; send_data_tx( SNARE ); }
		else if(adc[0] > 100) { tSN = tim;  SNARE[3]=100; send_data_tx( SNARE ); }
		else if(adc[0] > 90) { tSN = tim;  SNARE[3]=90; send_data_tx( SNARE ); }
		else if(adc[0] > 80) { tSN = tim;  SNARE[3]=80; send_data_tx( SNARE ); }
		//else if(adc[0] > 70) { tSN = tim;  SNARE[3]=80; send_data_tx( SNARE ); }
		//else if(adc[0] > 60) { tSN = tim;  SNARE[3]=80; send_data_tx( SNARE ); }
	}
	
	/* ADC2 Crash_CymBal */ 
	if(adc[1] > 50 && tim > (tCC+lasttim)) {
		if(adc[1] > 120) { tCC = tim;  Crash_CymBal[3]=100; send_data_tx( Crash_CymBal ); }
		else if(adc[1] > 110) { tCC = tim;  Crash_CymBal[3]=100; send_data_tx( Crash_CymBal ); }
		else if(adc[1] > 100) { tCC = tim;  Crash_CymBal[3]=100; send_data_tx( Crash_CymBal ); }
		else if(adc[1] > 90) { tCC = tim;  Crash_CymBal[3]=90; send_data_tx( Crash_CymBal ); }
		// else if(adc[1] > 80) { tCC = tim;  Crash_CymBal[3]=80; send_data_tx( Crash_CymBal ); }
		// else if(adc[1] > 70) { tCC = tim;  Crash_CymBal[3]=80; send_data_tx( Crash_CymBal ); }
		// else if(adc[1] > 60) { tCC = tim;  Crash_CymBal[3]=80; send_data_tx( Crash_CymBal ); }
	}
	
	if(VPA7 && tim > (tTom+lasttim)) {
		tTom = tim;
		send_data_tx(TOMS);
		// HAL_Delay(100);
	}
	if(VPA8 && tim > (tHi+lasttim)) {
		tHi = tim;
		send_data_tx(Hi_Hat);
		// HAL_Delay(100);
	}
	/*
	if(VPA9 && tim > (tCC+lasttim)) {
		tCC = tim;
		send_data_tx(Crash_CymBal);
		// HAL_Delay(100);
	}
	*/
	if(VPA10 && tim > (tRC+lasttim)) {
		tRC = tim;
		send_data_tx(Ride_CymBal);
		// HAL_Delay(100);
	}
	
}

void send_data_tx(uint8_t *data) {
	HAL_UART_Transmit_IT(&huart2, data, 4);
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
