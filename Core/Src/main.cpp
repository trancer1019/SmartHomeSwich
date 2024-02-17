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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "../SimpleFIFO.h"
#include "../Debouncer.h"
#include "../SHUartController.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_DATA_ADDRESS 0x08003800
#define UART_BAUD_RATE_ARR (uint32_t[]){300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t uartin; //переменная для хранения байта принятых данных по UART

uint8_t DeviceAdress; //адрес устройства (сохраняется во FLASH)
uint32_t DeviceBaudRate;  //битрейт устройства (сохраняется во FLASH)
uint8_t DeviceConfig; //конфигурация устройства (сохраняется во FLASH)

uint8_t swcount; //определяем количество включенных клавиш
Debouncer **swArr; //массив клавиш
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(uint32_t BaudRate);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
uint64_t readDataFromFlash(void);
void writeDataToFlash(uint64_t data);
void boardLedblink();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SimpleFIFO uartfifo;
SH_UartController rs485Controller(&DeviceAdress, &huart1); // Создание объекта класса для обмена по rs485
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
//	////удалить
//	uint8_t _deviceAdress = 0x02; //адрес устройства
//	uint32_t _deviceBaudRate = 115200; //битрейт устройства
//	uint8_t _deviceConfig = 3; //конфигурация устройства
//	writeDataToFlash((uint64_t) _deviceConfig << 40 | (uint64_t) _deviceBaudRate << 8 | (uint64_t) _deviceAdress);
//	////удалить

	uint64_t flashconfig = readDataFromFlash(); //считываем адрес устройства

	DeviceAdress = flashconfig & 0xFF; //адрес устройства
	DeviceBaudRate = (flashconfig >> 8) & 0xFFFFFFFF;  //битрейт устройства
	DeviceConfig = (flashconfig >> 40) & 0xFF; //конфигурация устройства

	swcount = (DeviceConfig & 0x03) + 1; //определяем количество включенных клавиш

	/*Debouncer ***/swArr = new Debouncer*[swcount]; //массив клавиш
	if (swcount >= 1)
		swArr[0] = new Debouncer(GPIOA, SWICH_1_Pin);
	if (swcount >= 2)
		swArr[1] = new Debouncer(GPIOA, SWICH_2_Pin);
	if (swcount >= 3)
		swArr[2] = new Debouncer(GPIOA, SWICH_3_Pin);
	if (swcount >= 4)
		swArr[3] = new Debouncer(SWICH_4_GPIO_Port, SWICH_4_Pin);
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
	MX_USART1_UART_Init(DeviceBaudRate);
	MX_TIM17_Init();
	MX_TIM16_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &uartin, 1); //запуск приема данных по UART
	HAL_TIM_Base_Start_IT(&htim16); // включаем прерывание
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (!uartfifo.isEmpty()) {
			uint8_t getd = uartfifo.dequeue();
			rs485Controller.update(getd); //выполнение функции обновления соостояни
		}

		HAL_IWDG_Refresh(&hiwdg); // сброс WatchDog'а
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 48000;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 10;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 48000;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 10;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim17, TIM_OPMODE_SINGLE) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(uint32_t BaudRate) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = BaudRate;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : UART_RX_CONTROL_Pin */
	GPIO_InitStruct.Pin = UART_RX_CONTROL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(UART_RX_CONTROL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SWICH_1_Pin SWICH_2_Pin SWICH_3_Pin */
	GPIO_InitStruct.Pin = SWICH_1_Pin | SWICH_2_Pin | SWICH_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SWICH_4_Pin */
	GPIO_InitStruct.Pin = SWICH_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SWICH_4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//-ТАЙМЕРЫ
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16) {
		for (uint8_t i = 0; i < swcount; i++)
			swArr[i]->updateState();
	}
	if (htim->Instance == TIM17) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
}
//-UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uartfifo.enqueue(uartin);
		HAL_UART_Receive_IT(&huart1, (uint8_t*) &uartin, 1);
	}
}

///-----------------------------------------------------
//функция для запуска вспышки светодиода
void boardLedblink() {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Stop_IT(&htim17); // включаем прерывание
	HAL_TIM_Base_Start_IT(&htim17); // включаем прерывание
}

///-----------------------------------------------------
//функция для записи регистра
void deviceWriteRegister(uint8_t registerNumber, uint8_t registerValue) {
	boardLedblink(); //вспышка светодиода

	//-обновление адреса устройства
	if (registerNumber >= 0xF0 && registerNumber <= 0xF2) {
		uint64_t flashconfig = readDataFromFlash(); //считываем адрес устройства

		uint8_t _deviceAdress = flashconfig & 0xFF; //адрес устройства
		uint32_t _deviceBaudRate = (flashconfig >> 8) & 0xFFFFFFFF; //битрейт устройства
		uint8_t _deviceConfig = (flashconfig >> 40) & 0xFF; //конфигурация устройства

		if (registerNumber == 0xF0) {
			_deviceAdress = registerValue;
		} else if (registerNumber == 0xF1) {
			_deviceBaudRate = UART_BAUD_RATE_ARR [registerValue];
		} else if (registerNumber == 0xF2) {
			_deviceConfig = registerValue;
		}

		writeDataToFlash( (uint64_t) _deviceConfig << 40 | (uint64_t) _deviceBaudRate << 8 | (uint64_t) _deviceAdress);
	}
}

//функция для чтения регистра
uint8_t deviceReadRegister(uint8_t registerNumber, bool repeat=false) {
	boardLedblink(); //вспышка светодиода

	uint8_t registerValue = 0;

	if (registerNumber > 0 && registerNumber <= swcount) {
		registerValue = swArr[registerNumber-1]->getState(repeat);
	}
	else if (registerNumber >= 0xF0 && registerNumber <= 0xF2) {
		uint64_t flashconfig = readDataFromFlash(); //считываем адрес устройства

		uint8_t _deviceAdress = flashconfig & 0xFF; //адрес устройства
		uint32_t _deviceBaudRate = (flashconfig >> 8) & 0xFFFFFFFF; //битрейт устройства
		uint8_t _deviceConfig = (flashconfig >> 40) & 0xFF; //конфигурация устройства

		if (registerNumber == 0xF0) //адрес устройства
		{
			registerValue = _deviceAdress;
		}
		else if (registerNumber == 0xF1) //битрейт устройства
		{
			for (uint8_t i = 0; i < sizeof(UART_BAUD_RATE_ARR ) / sizeof(UART_BAUD_RATE_ARR [0]); i++) {
				if (UART_BAUD_RATE_ARR [i] == _deviceBaudRate) {
					registerValue = i;
					break;
				}

			}
		}
		else if (registerNumber == 0xF2) //конфигурация устройства
		{
			registerValue = _deviceConfig;
		}
	}
	else {
		registerValue = 0x00;
	}

	return registerValue;
}

//функция для чтения всех регистров
uint8_t* deviceRead4Register(bool repeat=false) {
	boardLedblink(); //вспышка светодиода

	uint8_t *registerNumbersAndValues = new uint8_t[swcount * 2];

	for (uint8_t i = 0; i < swcount; i++) {
		registerNumbersAndValues[i*2] = i + 1;
		registerNumbersAndValues[i*2 + 1] = swArr[i]->getState(repeat);
	}

	return registerNumbersAndValues;
}

///-------------------------------------------
// Функция для чтения переменной из Flash
uint64_t readDataFromFlash(void) {
	return *((uint64_t*) FLASH_DATA_ADDRESS);
}

// Функция для записи переменной во Flash
void writeDataToFlash(uint64_t data) {
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_DATA_ADDRESS;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError;

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_DATA_ADDRESS, data);

	HAL_FLASH_Lock();  // Заблокировать Flash
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
