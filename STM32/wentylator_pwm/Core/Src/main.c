/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//SCB->CCR |= 0x10;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;		//analog to digital converter
DMA_HandleTypeDef hdma_adc1;	//direct memory access

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim9;		//timer and timer interrupt
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

volatile uint16_t adc_value[2];
float adc_voltage[2];	//czujniki tlenu i cisnienia
float cisnienie;	//wyliczone z adc
float tlen;
volatile uint8_t i2c_rx[2];
float przeplyw;
float objetosc;	//objetosc wtlaczana do płuc
float objetosc_chw;	//objętość chwilowa (jednostkowa) z czujnika przeplywu (i2c)
uint8_t spi_rx[10];	//komunikacja z raspberry pi
uint8_t spi_tx[10];
//volatile uint8_t t_wdech = 1; //[s] interwał czasowy, czas jednostkowy, [nieaktualne, nie korzystamy]
uint16_t objetosc_zad;	//wartosci zadane z aplikacji RPI
uint16_t cisnienie_zad;
uint16_t przeplyw_zad;
uint16_t czas_wdechu;
uint16_t czas_wydechu;	//przelicza sie na ilosc wdechow na minutę
uint16_t PEEP;	//minimalne cisnienie pluc
pid_t pid1;	//proportional integral differential, wygładzanie sterowania turbiną, potem zmienione na PI
//int pid_pwm;
int pwm = 200;
int cis_sredni;	//cisnienie srednie, całka z pola pod wykresem cisnienia
float cisnienie_u;	//wartosci unormowane (po kalibracji)
float przeplyw_u;
uint8_t blad;	//obsluga alarmów, autodianostyka, przekazywane do rpi

//int e;	//do sprawdzenia, nie uzywamy
int przeplyw_sredni;
uint8_t faza;	//wdech, wydech

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int check(int j, int wd_ch, int wy_ch) {
	static int b, z;
	if (j == 1) {
		b = 0;
		z = 0;
	} else if (j < wd_ch) {
		if (40 < cisnienie) {
			z++;
		}
	}

	else if (j > wd_ch && j < wd_ch + wy_ch) {

		if ((0.5 * przeplyw_sredni) < przeplyw) {
			b++;

		}
	} else if (j == wd_ch + wy_ch) {
		if (z > (0.1 * wd_ch)) {
			return 2;
		}
		if (b > ((wd_ch + wy_ch) * 0.1)) {
			if (z > (0.1 * wd_ch)) {
				return 2;
			} else {
				return 1;
			}

		} else
			return 0;
	}
	return 0;
}

void tryb_objetosc(int obj, int c_wdechu, int c_wydechu, int peep) {
	static uint16_t t;
	static int went;

	t++;

	uint16_t t_wd = c_wdechu / 10;
	uint16_t t_wy = c_wydechu / 10;

	if (c_wdechu == 0)
		return;
	blad = check(t, t_wd, t_wy);
	went = (obj * 1000) / (c_wdechu);

	if (t < t_wd) {
		faza = 1;
		if (pwm > 1000)
			pwm = 1000;
		if (pwm < 0)
			pwm = 0;
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (uint16_t )pwm);
		if (przeplyw_sredni > (1.05 * went)) { //&& t % 2 == 1) {

			pwm--;

		}
		if (przeplyw_sredni < (0.95 * went)) { //&& t % 2 == 1) {
			pwm++;
		}
		if (objetosc > obj) {
			HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);

		} else {
			HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 1);
			if (t > 10) {
				przeplyw_sredni *= 19;
				przeplyw_sredni += (int) przeplyw;
				przeplyw_sredni /= 20;
			}
		}

//		if (cisnienie > 25) {
//			HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);
////			return;
//		}

	} else if (t == t_wd) {
		faza = 0;
		HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);

	} else if (t >= t_wd && t < (t_wd + t_wy)) {

		if (cisnienie < peep) {
			HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 1);

		} else
			HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);

	} else {
		t = 0;

	}

}



void tryb_cisnienie(int cis, int c_wdechu, int c_wydechu, int peep) {
	static uint16_t t;

	t++;
	uint16_t t_wd = c_wdechu / 10;
	uint16_t t_wy = c_wydechu / 10;

	if (t < t_wd) {
		faza = 1;
		if (pwm > 1000)
			pwm = 1000;
		if (pwm < 0)
			pwm = 0;
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (uint16_t )pwm);

		if (cis_sredni > cis) {
			pwm--;
			HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);

		}

		else if (cis_sredni < (0.8 * cis)) {
			pwm++;
		}
		HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 1);

		if (t > 20) {
			cis_sredni *= 19;
			cis_sredni += (int) cisnienie;
			cis_sredni /= 20;
		}

	} else if (t == t_wd) {
		faza = 0;
		HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);

	} else if (t >= t_wd && t < (t_wd + t_wy)) {

		HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);
		if (cisnienie < peep) {
			HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 1);

		}

	} else {
		t = 0;

	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {		//oczytywanie danych z czujnika cisnienia
	static uint16_t adc_srednia[2] = { 0, 0 };
	uint8_t ilosc_probek[2] = { 30, 20 };

//cisnienie
	adc_srednia[0] *= (ilosc_probek[0] - 1);
	adc_srednia[0] += adc_value[0];
	adc_srednia[0] /= ilosc_probek[0];
	adc_voltage[0] = (3.3 * adc_srednia[0]) / 1024.0;		//3.3V - maksymalne napięcie czujnika, 1024 - 10 bitowy czujnik

	cisnienie = ((12.5 * adc_voltage[0]) / 3.0 * (1000.0 / 98.1)) - 13; //[cmH2O] - 3V (wzmacniacz) to 10mV i 12,5 kPa (czujnik)
	cisnienie_u *= 999;
	cisnienie_u += cisnienie;
	cisnienie_u /= 1000;
//	if (cisnienie>35){
//		HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, 0);
//	}

//tlen
	adc_srednia[1] *= (ilosc_probek[1] - 1);
	adc_srednia[1] += adc_value[1];
	adc_srednia[1] /= ilosc_probek[1];
	adc_voltage[1] = (3.3 * adc_srednia[1]) / 1024.0;

	tlen = (21.0 * adc_voltage[1]) / 0.5; //[%] - 0.5V (wzmacniacz) to 10mV i 21% (czujnik)
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	static uint16_t i = 0;
//	static float objetosc_chw = 0;
//static uint8_t stan[2] = { 0, 0 };

	uint16_t i2c_value = (i2c_rx[0] << 8) | i2c_rx[1];
	przeplyw = ((i2c_value - 32768.0) / 120.0) * (1000 / 60.0); //[ml/s]
	przeplyw_u *= 99;
	przeplyw_u += przeplyw;
	przeplyw_u /= 100;

//	stan[0] = stan[1];
//	stan[1] = faza;

	if (faza == 1) {	//faza = czy turbina pracuje czy nie
//		i++;
//		sr_przeplyw += przeplyw;

		objetosc_chw = przeplyw * 0.001;
		objetosc += objetosc_chw;
	} else
		objetosc = 0;

//	if (stan[0] > stan[1]) {
////		sr_przeplyw /= i;
////		objetosc = sr_przeplyw * t_wdech;
//		objetosc = 0;
//		//i = 0;
//	}

//		else{
//		sr_przeplyw /= i;
//		objetosc = sr_przeplyw * t_wdech;
//		sr_przeplyw = 0;
//		i=0;
//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim9) {

		if (spi_rx[1] == 1) {
			tryb_objetosc(objetosc_zad, czas_wdechu, czas_wydechu, PEEP);
		} else if (spi_rx[1] == 2) {
			tryb_cisnienie(cisnienie_zad, czas_wdechu, czas_wydechu, PEEP);
		} else if (spi_rx[1] == 0) {
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
		}

	} else if (htim == &htim11) {
		HAL_I2C_Master_Receive_IT(&hi2c3, (64 << 1), (uint8_t*) i2c_rx, 2);

	}

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	spi_tx[0] = 1;
	spi_tx[1] = ((uint16_t) objetosc) >> 8;
	spi_tx[2] = (uint16_t) objetosc & 0xFF;
	spi_tx[3] = (uint8_t) cisnienie_u;
	spi_tx[4] = (uint8_t) tlen;
	spi_tx[5] = ((uint16_t) przeplyw_u) >> 8;
	spi_tx[6] = (uint16_t) przeplyw_u & 0xFF;
	spi_tx[7] = spi_rx[0] + spi_rx[9];
	spi_tx[8] = blad;
	spi_tx[9] = 2;

	if (spi_rx[0] == 1 && spi_rx[9] == 2) {
		if (spi_rx[1] == 1) {
			objetosc_zad = spi_rx[2] << 8 | spi_rx[3];
		} else if (spi_rx[1] == 2) {
			cisnienie_zad = spi_rx[2] << 8 | spi_rx[3];
		}
		czas_wdechu = (int) (spi_rx[4] << 8 | spi_rx[5]);
		czas_wydechu = (int) (spi_rx[6] << 8 | spi_rx[7]);
		PEEP = (int) spi_rx[8];
	}

//	pid_d = ((float) spi_rx[5]) / 100;
//
//	pid_init(&pid1, pid_p, pid_i, pid_d, 1000);

	HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spi_tx, (uint8_t*) spi_rx,
			10);
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
	MX_DMA_Init();
	MX_TIM10_Init();
	MX_SPI1_Init();
	MX_I2C3_Init();
	MX_ADC1_Init();
	MX_TIM11_Init();
	MX_TIM9_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*) spi_tx, (uint8_t*) spi_rx,
			10);

	uint8_t i2c_tx[2];
	i2c_tx[0] = 0x10;
	i2c_tx[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c3, (64 << 1), i2c_tx, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive_IT(&hi2c3, (64 << 1), (uint8_t*) i2c_rx, 2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, 2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_Base_Start_IT(&htim9);

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 9999;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 99;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 99;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 1000;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */
	HAL_TIM_MspPostInit(&htim10);

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 9999;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 9;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 7, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ZAWOR_GPIO_Port, ZAWOR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : ZAWOR_Pin */
	GPIO_InitStruct.Pin = ZAWOR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ZAWOR_GPIO_Port, &GPIO_InitStruct);

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
