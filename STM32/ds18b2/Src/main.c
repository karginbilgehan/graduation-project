/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
int16_t values[8];								// signed variable to store calibration values for BMP180
uint16_t Val[3];								// unsigned variable to store calibration values for BMP180
#define 	AC1 					values[0]	// label for calibration value
#define 	AC2 					values[1]	// label for calibration value
#define 	AC3 					values[2]	// label for calibration value
#define 	AC4 					Val[0]		// label for calibration value
#define 	AC5 					Val[1]		// label for calibration value
#define 	AC6 					Val[2]		// label for calibration value
#define 	B1 						values[3]	// label for calibration value
#define 	B2 						values[4]	// label for calibration value
#define 	MB 						values[5]	// label for calibration value
#define 	MC 						values[6]	// label for calibration value
#define 	MD 						values[7]	// label for calibration value
#define 	BMP180_RAW_VALUE_ADDR 	0XF6		// address to read raw values from BMP180
#define 	BMP180_CNTL_REG_ADDR 	0xF4		// address to configure BMP180
#define 	ACC_READ_ID_REG_ADDR	0x0F		// address to read ID from accelerometer
#define 	ACC_EN_DEVICE 			0x27		// enable x,y,z axis, enable normal mode, and low sample rate of IIS328DQ
#define 	RECURCISE_READ_EN 		0X80		// command to read succesive data bytes from IIS328DQ
#define 	LOW_POWER 				0x00		// LOW POWER MODE of BMP180 sensor
#define 	STANDARD 				0x01		// STANDARD MODE of BMP180 sensorCommand to read temperature from BMP180
#define 	HIGH_RESOLUTION 		0x02		// HIGH RESOLUTION MODE of BMP180 sensor
#define 	ULTRA_HIGH_RESOLUTION 	0x03		// ULTRA HIGH RESOLUTION MODE of BMP180 sensor
#define 	READ_TEMP_CMD 			0x2E		// Command to read temperature from BMP180
#define 	READ_PRESSURE_CMD 		0x34		// Command to read temperature from BMP180
#define 	PRESSURE 				0x01
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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/*IT IS FOR DS18B TEMP SENSOR */
void delay(uint16_t delay);
void gpio_set_input (void);
void gpio_set_output (void);
uint8_t ds18b20_init (void);
void write (uint8_t data);
uint8_t read (void);

/*IT IS FOR BMP180 PRES SENSOR*/
void WriteRegister(uint8_t DevAddress,char address,char content);
void ReadRegister(uint8_t DevAddress,uint8_t Address,uint8_t * Buffer,uint8_t length);
int32_t ReadSensor(char data,char mode,uint8_t length);
int16_t ReadInt(char address);
uint16_t ReadUInt(char address);
void ReadBMP180();
void ReadCalPara();
void SetupBMP();

/*IT IS FOR DHT22 SENSOR*/
void set_gpio_output_DHT22 (void);
void set_gpio_input_DHT22 (void);
void DHT22_start (void);
void check_response (void);
uint8_t read_data_DHT22 (void);

/*RUN FUNCTIONS*/
void runDS18B();
void runDHT22();
void run_pulseSensor(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*IT IS FOR DS18B TEMP SENSOR */
GPIO_InitTypeDef GPIO_InitStruct;
uint8_t check =2, temp_l, temp_h;
uint16_t temp;
float temperature_for_ds18b;

/*IT IS FOR BMP180 PRES SENSOR*/
uint8_t BMP180_ADDR = 0xEE;		//i2c address for BMP180
float a_x = 0,a_y = 0,a_z = 0;	//Accelerometer data variable
uint64_t timer1=0,timer2=0;		//variable to store time related parameters
int16_t X = 0, Y = 0, Z = 0;	//variables to store raw accelerometer data
uint8_t a_value[6]={0,0,0,0,0,0};	//Buffer to extract values from Accelerometer
int32_t GLOBAL_PRES = 0;

/*IT IS FOR DHT22 TEMP SENSOR*/
GPIO_InitTypeDef GPIO_InitStruct2;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP,TEMP1,TEMP2, RH1,RH2;
uint8_t checkForDHT22 = 0;

/*IT IS FOR PULSE SENSOR*/
uint8_t tempData[3];
char pulseValue[50];
int lenOfPulseValue = 0;
int pulseRes = 0;


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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /*Micro Second Timer Start */
  HAL_TIM_Base_Start(&htim1);

  /*Press Sensor Initialize*/
  SetupBMP();

  // Pulse sensor initialize with interrupt
  HAL_UART_Receive_IT(&huart6,tempData, 3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 //run_pulseSensor();
	 //HAL_Delay(100);
	 /*FOR DS18B SENSOR */

	 runDS18B();
	 HAL_Delay(100);

	 /*FOR PRESS SENSOR */

	 ReadBMP180();//Read the data from BMP180 sensor
	 HAL_Delay(100);

	 /*FOR DHT22 SENSOR */

	 runDHT22();
	 HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x10;
  sTime.Minutes = 0x30;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x4;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void delay(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

void runDS18B(){

		 char ds18bval[50];//it is for temperature val
		 int lenOfds18=0;
		 int i=0;
		/*FOR DS18B TEMP SENSOR */
		 check = ds18b20_init ();
		 HAL_Delay (1);
		 write (0xCC);  // skip ROM
		 write (0x44);  // convert t
		 HAL_Delay (800);
		 ds18b20_init ();
	     HAL_Delay(1);
		 write (0xCC);  // skip ROM
		 write (0xBE);  // Read Scratchpad
		 temp_l = read();
		 temp_h = read();
		 temp = (temp_h<<8)|temp_l;
		 temperature_for_ds18b = (float)temp/16;

		 /* FOR DS18B UART TRANSMIT*/
		 sprintf(ds18bval,"temp.val=%d%c%c%c%c",(int)temperature_for_ds18b , 0xFF, 0xFF, 0xFF,'\n');
		 while(ds18bval[i]!='\n'){
			 lenOfds18++;
			 ++i;
		 }
		 i=0;
		 HAL_UART_Transmit(&huart1,ds18bval,lenOfds18,100);
}

void runDHT22(){
		 int i=0;
		 char dht22Temp[50],dht22humidity[50];//it is for temperature val
		 int lenOfdht22Temp=0;
		 int lenOfdht22humidity=0;

		 /*FOR DHT22 SENSOR */
		 DHT22_start ();
		 check_response ();
		 Rh_byte1 = read_data_DHT22 ();
		 Rh_byte2 = read_data_DHT22 ();
		 Temp_byte1 = read_data_DHT22 ();
		 Temp_byte2 = read_data_DHT22 ();
		 sum = read_data_DHT22();
		 if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
		 {
		 	TEMP = ((Temp_byte1<<8)|Temp_byte2);
		 	RH = ((Rh_byte1<<8)|Rh_byte2);
		 }
		 TEMP1 = (Temp_byte1/10);
		 TEMP2 = (Temp_byte1 %10);
		 sprintf(dht22Temp,"temp1.val=%d%d%c%c%c%c",(int)TEMP1,(int)TEMP2, 0xFF, 0xFF, 0xFF,'\n');
		 while(dht22Temp[i]!='\n'){
			 lenOfdht22Temp++;
			++i;
	     }
		 i=0;
		 HAL_UART_Transmit(&huart1,dht22Temp,lenOfdht22Temp,100);


		 RH1 = (Rh_byte1 / 10);
		 RH2 = (Rh_byte1 % 10);
		 sprintf(dht22humidity,"humidity.val=%d%d%c%c%c%c",(int)RH1,(int)RH2, 0xFF, 0xFF, 0xFF,'\n');
		 while(dht22humidity[i]!='\n'){
			 lenOfdht22humidity++;
			++i;
		}
		i=0;
		HAL_UART_Transmit(&huart1,dht22humidity,lenOfdht22humidity,100);
}

void run_pulseSensor(void)
{
	int i = 0;
	HAL_UART_Receive_IT(&huart6, (uint8_t *)tempData, 3);
    //pulseRes = (int) tempData[0];
	sscanf(tempData,"%d", &pulseRes);
	//sscanf(str, "%d", &x);
	sprintf(pulseValue,"pulse.val=%d%c%c%c%c",pulseRes,0xFF, 0xFF, 0xFF,'\n');
	while(pulseValue[i]!='\n'){
		lenOfPulseValue++;
		++i;
	}
	i=0;
	HAL_UART_Transmit(&huart1,pulseValue,lenOfPulseValue,100);

	lenOfPulseValue=0;
	//pulseRes = 0;

	//HAL_Delay(500);
	tempData[0] = '\000';
	tempData[1] = '\000';
	tempData[2] = '\000';
	 //HAL_Delay(100);
}
void gpio_set_input (void)
{
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void gpio_set_output (void)
{
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t ds18b20_init (void)
{

	gpio_set_output ();   // set the pin as output
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	gpio_set_input ();    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)))    // if the pin is low i.e the presence pulse is there
	{
		return 0;
	}

	else
	{
		delay (400);
		return 1;
	}
}

void write (uint8_t data)
{
	gpio_set_output ();   // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			gpio_set_output ();  // set as output
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the pin LOW
			delay (1);  // wait for  us

			gpio_set_input ();  // set as input
			delay (60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			gpio_set_output ();
			HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the pin LOW
			delay (60);  // wait for 60 us

			gpio_set_input ();
		}
	}
}


uint8_t read (void)
{
	uint8_t value=0;
	gpio_set_input ();

	for (int i=0;i<8;i++)
	{
		gpio_set_output ();   // set as output

		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the data pin LOW
		delay (2);  // wait for 2 us

		gpio_set_input ();  // set as input
		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}


/* IT IS FOR BMP180 PRES SENSOR*/
/*Function to read calibration parameters */
void ReadCalPara(){
    AC1 = ReadInt(0xAA);	//Read value for AC1 register
    AC2 = ReadInt(0xAC);	//Read value for AC2 register
    AC3 = ReadInt(0xAE);	//Read value for AC3 register
    AC4 = ReadUInt(0xB0);	//Read value for AC4 register
    AC5 = ReadUInt(0xB2);	//Read value for AC5 register
    AC6 = ReadUInt(0xB4);	//Read value for AC6 register
    B1 = ReadInt(0xB6);		//Read value for B1 register
    B2 = ReadInt(0xB8);		//Read value for B2 register
    MB = ReadInt(0xBA);		//Read value for MB register
    MC= ReadInt(0xBC);		//Read value for MC register
    MD = ReadInt(0xBE);		//Read value for MD register
}
/*Read signed integer from I2C bus*/
int16_t ReadInt(char address){
	uint8_t temp[2];									// Temporary variable to store raw value
	int16_t result=0;								// variable to be return
	ReadRegister(BMP180_ADDR,address,temp,2);		// Read raw value from I2C bus
	result=((temp[0]<<8)|temp[1]);					// convert the raw value to integer
	return result;									// Return integer
}
/*Read unsigned integer from I2C bus*/
uint16_t ReadUInt(char address){
	uint8_t temp[2];									// Temporary variable to store raw value
	uint16_t result=0;								// variable to be return
	ReadRegister(BMP180_ADDR,address,temp,2);		// Read raw value from I2C bus
	result=((temp[0]<<8)|temp[1]);					// convert the raw value to integer
	return result;									// Return integer
}
/*Read values from BMP180 sensor*/
void ReadBMP180(){
	int32_t X1,X2,B5,ut,p,B6,X3,B3;					// Temporary signed variable to calculate pressure and temperature
	uint32_t B4,B7;									// Temporary unsigned variable to calculate pressure and temperature
	char sampling_mode = LOW_POWER;		// Working mode for BMP180 sensor
	float temperature=0.0;							// variable to carry temperature data
	int32_t UT,UP;									// variable to carry integer value of Temperature and pressure

	char dizi2[50];// it is for press val
	int lenOfDizi2=0;
	int i=0;

	UT=ReadSensor(READ_TEMP_CMD,sampling_mode,2);		// Read Temperature and given mode
	if(UT!=0){										// check for valid reading
		X1=(UT-AC6) * AC5>>15;						// calculation to obtain
		X2=(MC<<11)/(X1+MD);						// Temperature from raw
		B5=X1+X2;									// values for more details
		ut=(B5+8)>>4;								// refer section 3.5 in BMP180
		temperature = ut *0.1;						// sensor datasheet.

		UP=ReadSensor(READ_PRESSURE_CMD,sampling_mode,3);			// read raw values of pressure
		if(UP!=0){									// check for valid reading
			UP = (UP>>(8-sampling_mode));
			B6=B5-4000;								// calculations to extract pressure
			X1=(B2*((B6*B6)>>12))>>11;				// reading from BMP180 sensor
			X2=(AC2*B6)>>11;						// all the calculations shown
			X3=X1+X2;								// in the datasheet are carry
			B3=((((AC1*4)+X3)<<sampling_mode)+2)/4;// forward in this step
			X1=(AC3*B6)>>13;						// for more details on this
			X2=(B1*((B6*B6)>>12))>>16;				// please refer section 3.5 in
			X3 = (X1 + X2 + 2) >> 2;				// BMP180 sensor datasheet.
			B4=(uint32_t)AC4*(uint32_t)(X3+32768)>>15;					//
			B7=(uint32_t)(UP-B3)*(50000>>sampling_mode);		//
			if (B7 < 0x80000000)					//
				p = (B7 * 2) / B4;					//
			else									//
				p = (B7 / B4) * 2;					//
			X1 = (p >> 8) * (p >> 8);				//
			X1 = (X1 * 3038) >> 16;					//
			X2 = (-7357 * p) >> 16;					//
			GLOBAL_PRES = (p + ((X1 + X2 + 3791) >> 4));		//

			sprintf(dizi2,"press.val=%d%c%c%c%c",(int)GLOBAL_PRES/100, 0xFF, 0xFF, 0xFF,'\n');
			while(dizi2[i]!='\n'){
				lenOfDizi2++;
				++i;
			}
			i=0;
			HAL_UART_Transmit(&huart1,dizi2,lenOfDizi2,100);
		}
	}
}
/*Function to Read the raw values from the sensor*/
int32_t ReadSensor(char data,char mode,uint8_t length){
	uint8_t response[3],x=0;							//Local variables declaration
	data = data | (mode<<6);						//append mode in control register
	WriteRegister(BMP180_ADDR,BMP180_CNTL_REG_ADDR,data);//start pressure sensor in given mode
	do{
		x++;										//increment x to each milliseconds
		HAL_Delay(1);									//delay for 1 milliseconds
		ReadRegister(BMP180_ADDR,BMP180_CNTL_REG_ADDR,&response[0],1);//read busy status of sensor
		if(x>=30)									//if no response received with
			return 0;								// 30 milliseconds then return 0.
	}while((response[0]&0x20)!=0x00);				// wait until sensor is busy
	ReadRegister(BMP180_ADDR,BMP180_RAW_VALUE_ADDR,response,length);//read the raw data from sensor
	int32_t result= 0 ;
	if(length==2)
		result = ((response[0]<<8) | response[1]);	// convert bytes into signed integer
	else if(length==3)
		result = ((response[0]<<16) | (response[1]<<8) | response[2]);//convert bytes into signed integer
	return result;									//return the result
}
/*function to setup BMP180 sensor*/
void SetupBMP(){
	uint8_t prxbuf[1];									// local variable
	ReadRegister(BMP180_ADDR,0xD0,prxbuf,1);		// read BMP180 sensor ID
//	Debug.printf("BMP ID : %02x \r\n",prxbuf[0]);	// display ID
	ReadCalPara();									// read calibration data from the sensor
}
/*function to read RAW values from I2C bus*/
void ReadRegister(uint8_t DevAddress,uint8_t Address,uint8_t * Buffer,uint8_t length){
	HAL_I2C_Master_Transmit(&hi2c1,DevAddress&0xFE,&Address,1,10);// send device address and register address
	HAL_I2C_Master_Receive(&hi2c1,DevAddress|0x01,Buffer,length,10);// read data from sensor
}

/*function to write RAW values on I2C bus*/
void WriteRegister(uint8_t DevAddress,char address,char content){
	uint8_t data[2]={address,content};					// local variable to store addres and data to be written
	HAL_I2C_Master_Transmit(&hi2c1,DevAddress&0xFE,data,2,10);//	write the address and data to the I2C bus
}

/*IT IS FOR DHT22 SENSOR */
void set_gpio_output_DHT22 (void)
{
	/*Configure GPIO pin output: PA4 */
  GPIO_InitStruct2.Pin = GPIO_PIN_4;
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);
}

void set_gpio_input_DHT22 (void)
{
	/*Configure GPIO pin input: PA4 */
  GPIO_InitStruct2.Pin = GPIO_PIN_4;
  GPIO_InitStruct2.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct2.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);
}
void DHT22_start (void)
{
		set_gpio_output_DHT22();  // set the pin as output
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, 0);   // pull the pin low
		delay (18000);   // wait for 18ms
		set_gpio_input_DHT22();   // set as input
}
void check_response (void)
{
	delay (40);
	if (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4))) checkForDHT22 = 1;
	}
	while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)));   // wait for the pin to go low
}

uint8_t read_data_DHT22 (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)) == 0)   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)));  // wait for the pin to go low
	}
	return i;
}
/*--------------------------------------------------------------------------*/

/*IT IS FOR PULSE SENSOR WITH INTERRUTP*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i = 0;
		HAL_UART_Receive_IT(&huart6, tempData, 3);
	    //pulseRes = (int) tempData[0];
		sscanf(tempData,"%d", &pulseRes);
		//sscanf(str, "%d", &x);
		sprintf(pulseValue,"pulse.val=%d%c%c%c%c",pulseRes,0xFF, 0xFF, 0xFF,'\n');
		while(pulseValue[i]!='\n'){
			lenOfPulseValue++;
			++i;
		}
		i=0;
		HAL_UART_Transmit(&huart1,pulseValue,lenOfPulseValue,100);

		lenOfPulseValue=0;
		//pulseRes = 0;

		//HAL_Delay(500);
		tempData[0] = '\000';
		tempData[1] = '\000';
		tempData[2] = '\000';
		 //HAL_Delay(100);
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
