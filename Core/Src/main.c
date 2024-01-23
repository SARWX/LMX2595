/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc.h"
#include "string.h" // это для функции strlen()
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint32_t Core_F[2] = {
		0x240190,	//R36 3GHz
		0x24014E};	//R36 2,5Ghz



uint32_t TX_Data[113] = {		// Input freq = 30 MHz, Output = 1 GHz
		0x700000, //R112
		0x6F0000, //R111
		0x6E0000, //R110
		0x6D0000, //R109
		0x6C0000, //R108
		0x6B0000, //R107
		0x6A0000, //R106
		0x690021, //R105
		0x680000, //R104
		0x670000, //R103
		0x663F80, //R102
		0x650011, //R101
		0x640000, //R100
		0x630000, //R99
		0x620200, //R98
		0x610888, //R97
		0x600000, //R96
		0x5F0000, //R95
		0x5E0000, //R94
		0x5D0000, //R93
		0x5C0000, //R92
		0x5B0000, //R91
		0x5A0000, //R90
		0x590000, //R89
		0x580000, //R88
		0x570000, //R87
		0x560000, //R86
		0x55D300, //R85
		0x540001, //R84
		0x530000, //R83
		0x521E00, //R82
		0x510000, //R81
		0x506666, //R80
		0x4F0026, //R79
		0x4E0203, //R78
		0x4D0000, //R77
		0x4C000C, //R76
		0x4B0C80, //R75
		0x4A0000, //R74
		0x49003F, //R73
		0x480000, //R72
		0x470021, //R71
		0x46C350, //R70
		0x450000, //R69
		0x4403E8, //R68
		0x430000, //R67
		0x4201F4, //R66
		0x410000, //R65
		0x401388, //R64
		0x3F0000, //R63
		0x3E0322, //R62
		0x3D00A8, //R61
		0x3C0000, //R60
		0x3B0001, //R59
		0x3A8001, //R58
		0x390020, //R57
		0x380000, //R56
		0x370000, //R55
		0x360000, //R54
		0x350000, //R53
		0x340820, //R52
		0x330080, //R51
		0x320000, //R50
		0x314180, //R49
		0x300300, //R48
		0x2F0300, //R47
		0x2E07FC, //R46
		0x2DC8DE, //R45
		0x2C1EA3, //R44
		0x2B0000, //R43
		0x2A0000, //R42
		0x290000, //R41
		0x280000, //R40
		0x2703E8, //R39
		0x260000, //R38
		0x250404, //R37
		0x24006E, //R36
		0x230004, //R35
		0x220000, //R34
		0x211E21, //R33
		0x200393, //R32
		0x1F43EC, //R31
		0x1E318C, //R30
		0x1D318C, //R29
		0x1C0488, //R28
		0x1B0002, //R27
		0x1A0DB0, //R26
		0x190624, //R25
		0x18071A, //R24
		0x17007C, //R23
		0x160001, //R22
		0x150401, //R21
		0x14E048, //R20
		0x1327B7, //R19
		0x120064, //R18
		0x11012C, //R17
		0x100080, //R16
		0x0F064F, //R15
		0x0E1E70, //R14
		0x0D4000, //R13
		0x0C5001, //R12
		0x0B0018, //R11
		0x0A10D8, //R10
		0x090604, //R9
		0x082000, //R8
		0x0740B2, //R7
		0x06C802, //R6
		0x0500C8, //R5
		0x040A43, //R4
		0x030642, //R3
		0x020500, //R2
		0x010808, //R1
		0x00251C}; //R0




int16_t frec[20][3] = //лимиты делителя
{{-2,10000,20000},
{1,7500,15000},
{2,3570,7500},
{4,1875,3750},
{6,1250,2500},
{8,938,1437},
{12,626,958},
{16,469,718},
{24,313,479},
{32,235,359},
{48,157,239},
{64,118,179},
{72,105,159},
{96,79,119},
{128,59,89},
{192,40,59},
{256,30,44},
{384,20,29},
{512,15,22},
{768,10,14}};

uint32_t RX_Data[113] = {0};

uint8_t TX_Data_Temp[3] = {0x00,0x24,0x12}; // R0 - start, enable reset

char trans_str[3] = {0,};
uint16_t adc = 0;
uint8_t RX_Flag = 0;
uint8_t USB_RX[10] = {0,};
uint8_t RX_USB_Flag = 0;
uint8_t coutner[3] = {0,0,0};
uint8_t count = 0;
//extern uint32_ time_RX;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void User_Delay(uint16_t timeout);
uint16_t LMX_read(uint8_t address);
void LMX_wrie(uint8_t address, uint16_t data);
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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);			// Forbid communication; CS = 1
    // User_Delay(10);
    HAL_Delay(10);															// Wait 10 ms



    // Init LMX2595

    // For what????
			HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
			User_Delay(10);
			HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
			User_Delay(10);
    //

    // HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);		// Enable communication; CS = 0
    // HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);						// Transmit
    LMX_wrie(0, 2); 														// Transmit RESET = 1
    HAL_Delay(200);															// Wait 200 ms
    uint16_t check = LMX_read(1);			// Check R0
    LMX_wrie(0, 0);															// Transmit RESET = 0
    check = LMX_read(1);
    check = LMX_read(14);
  //  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
    User_Delay(10);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    User_Delay(10);

    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    User_Delay(1);
    //TX_Data_Temp[3] = {0x00,0x24,0x12};
    TX_Data_Temp[2] = TX_Data_Temp[2] & (uint8_t)0xFD; //disable reset
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    User_Delay(10);


    for(uint8_t i = 34;i<113;i++){
  	  TX_Data_Temp[0] = (uint8_t)(TX_Data[i] >> 16);
  	  TX_Data_Temp[1] = (uint8_t)(TX_Data[i] >> 8);
  	  TX_Data_Temp[2] = (uint8_t)(TX_Data[i]);
  	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  	  HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);
  	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
  	  User_Delay(1);
    }
    User_Delay(10);

    TX_Data_Temp[2] = TX_Data_Temp[2] | (uint8_t)(1<<3); //калибровка
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    // end Init LMX2595
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*if (USB_RX_Flag) {
	  		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  		  HAL_ADC_Start(&hadc1); 					// запускаем преобразование сигнала АЦП
	  		  HAL_ADC_PollForConversion(&hadc1, 100);	// ожидаем окончания преобразования
	  		  adc = HAL_ADC_GetValue(&hadc1);			// читаем полученное значение в переменную adc
	  		  HAL_ADC_Stop(&hadc1);						// останавливаем АЦП (не обязательно)
	  		  sprintf(trans_str, "%4d\n", adc);
	  		  //time_RX = HAL_GetTick() - time_RX;
	  		  //sprintf(trans_str, "%s t:%d\n\r",trans_str, time_RX);
	  		  CDC_Transmit_FS(trans_str, strlen(trans_str));
	  		  USB_RX_Flag = 0;
	  	  }*/


	  	//CDC_Transmit_FS(TX_Data, strlen(TX_Data));
	  	/*coutner[0]++;
	  	coutner[1]++;
	  	coutner[2]++;
	  	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	  	HAL_SPI_Transmit(&hspi1, coutner, 3, 5000);
	  	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	  	CDC_Transmit_FS(coutner, 3);
	  	//User_Delay(1000);*/
	  	if(RX_USB_Flag != 0){
	  		//USB_RX[1]++;
	  		//CDC_Transmit_FS(USB_RX, 7);
	  		//HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	  		//HAL_SPI_Transmit_IT(&hspi1, TX_Data, sizeof(TX_Data));
	  		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	  		RX_USB_Flag = 0;
	  	}

	  	count = (count == 0)?1:0;


	  	  TX_Data_Temp[0] = (uint8_t)(Core_F[count] >> 16);
	  	  TX_Data_Temp[1] = (uint8_t)(Core_F[count] >> 8);
	  	  TX_Data_Temp[2] = (uint8_t)(Core_F[count]);
	  	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	  	  HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);
	  	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	  	  User_Delay(1);

	  	  /*TX_Data_Temp[0] = (uint8_t)(Core_F[count+1] >> 16);
	  	  TX_Data_Temp[1] = (uint8_t)(Core_F[count+1] >> 8);
	  	  TX_Data_Temp[2] = (uint8_t)(Core_F[count+1]);
	  	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	  	  HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);
	  	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	  	  User_Delay(1);*/


	  	TX_Data_Temp[0] = (uint8_t)(TX_Data[112] >> 16);
	  	TX_Data_Temp[1] = (uint8_t)(TX_Data[112] >> 8);
	  	TX_Data_Temp[2] = (uint8_t)(TX_Data[112]);
	  	  TX_Data_Temp[2] = TX_Data_Temp[2] | (uint8_t)(1<<3); //калибровка
	  	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	  	    HAL_SPI_Transmit(&hspi1, TX_Data_Temp, 3, 5000);
	  	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);


	  	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	  	User_Delay(2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void User_Delay(uint16_t timeout){
	for(uint32_t t = 0;t<=timeout*3600;t++);
}

void LMX_wrie(uint8_t address, uint16_t data) {
	uint8_t send_it[3] = {address, (uint8_t)(data >> 8), (uint8_t)(data)};				// send command to write data
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);					// Enable communication; CS = 0
	HAL_SPI_Transmit(&hspi1, send_it, 3, 5000);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);						// Forbid communication; CS = 1
}
uint16_t LMX_read(uint8_t address) {
	uint8_t send_it[3] = {(address | 1 << 7), 0, 0};
	uint8_t data[3] = {0, 0, 0};
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);					// Enable communication; CS = 0
	HAL_SPI_TransmitReceive(&hspi1, send_it, data, 3, 5000);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);						// Forbid communication; CS = 1
	uint16_t returned_num = data[1];													// MSB, so [0] - is the second byte
	returned_num = returned_num << 8;
	returned_num += data[2];
	return(returned_num);
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
