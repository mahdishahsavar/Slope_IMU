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
#include "fatfs.h"
#include "MPU6050.h"
#include "MS5611.h"
#include "stdio.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
FATFS fs;
FIL fil;
FRESULT res;
UINT byteswritten;
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
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



extern float MPU6050_Roll;
extern float MPU6050_Yaw;
extern float MPU6050_Pitch;

unsigned char Mag_Calib[6] = {0};


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MS5611_t MS5611;
	MPU6050_t MPU6050;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    MPU6050.Gyro_X_Offset = -75;
  	MPU6050.Gyro_Y_Offset = -25;
  	MPU6050.Gyro_Z_Offset = 10;
  /* USER CODE END Init */
  	HAL_Delay(500);
  	f_mount(&fs, "", 0);
  	f_open(&fil, "write.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	MS5611_Reset(&hi2c1, &MS5611);
	MS5611_ReadProm(&hi2c1, &MS5611);

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float mpu_temp_c;
    float gyro_x_rad;
    float gyro_y_rad;
    float gyro_z_rad;
    float magn_x_gs;
    float magn_y_gs;
    float magn_z_gs;
    float pressure_float;
    float baro_temp_float;
    int g_h;
    float g_lf;
    int g_l;
    uint8_t tx;
    char str[10];char str1[10];char str2[10];char str3[10];char str4[10];char str5[10];char str6[10];char str7[10];char str8[10];char str9[10];char str10[10];

	// MPU6050 + HMC5883L
	MPU6050_Init(&hi2c1, 2, 2, 5);
	MPU6050_Bypass(&hi2c1);
	HMC5883L_Setup(&hi2c1);
	MPU6050_Master(&hi2c1);
	MPU6050_Slave_Read(&hi2c1);

	MPU6050_Read_All_DMA(&hi2c1, &MPU6050);
	HAL_Delay(20);
	MPU6050_Parsing_NoOffest(&MPU6050);
	if(MPU6050.Mag_X_RAW > MPU6050.Mag_X_Max) MPU6050.Mag_X_Max = MPU6050.Mag_X_RAW;
	if(MPU6050.Mag_X_RAW < MPU6050.Mag_X_Min) MPU6050.Mag_X_Min = MPU6050.Mag_X_RAW;

	if(MPU6050.Mag_Y_RAW > MPU6050.Mag_Y_Max) MPU6050.Mag_Y_Max = MPU6050.Mag_Y_RAW;
	if(MPU6050.Mag_Y_RAW < MPU6050.Mag_Y_Min) MPU6050.Mag_Y_Min = MPU6050.Mag_Y_RAW;

	if(MPU6050.Mag_Z_RAW > MPU6050.Mag_Z_Max) MPU6050.Mag_Z_Max = MPU6050.Mag_Z_RAW;
	if(MPU6050.Mag_Z_RAW < MPU6050.Mag_Z_Min) MPU6050.Mag_Z_Min = MPU6050.Mag_Z_RAW;


	MPU6050.Mag_X_Offset = (MPU6050.Mag_X_Max + MPU6050.Mag_X_Min) / 2;
	MPU6050.Mag_Y_Offset = (MPU6050.Mag_Y_Max + MPU6050.Mag_Y_Min) / 2;
	MPU6050.Mag_Z_Offset = (MPU6050.Mag_Z_Max + MPU6050.Mag_Z_Min) / 2;

	Mag_Calib[0] = MPU6050.Mag_X_Offset >> 8;
	Mag_Calib[1] = MPU6050.Mag_X_Offset;
	Mag_Calib[2] = MPU6050.Mag_Y_Offset >> 8;
	Mag_Calib[3] = MPU6050.Mag_Y_Offset;
	Mag_Calib[4] = MPU6050.Mag_Z_Offset >> 8;
	Mag_Calib[5] = MPU6050.Mag_Z_Offset;

	MPU6050.Mag_X_Offset = Mag_Calib[0] << 8 | Mag_Calib[1];
	MPU6050.Mag_Y_Offset = Mag_Calib[2] << 8 | Mag_Calib[3];
	MPU6050.Mag_Z_Offset = Mag_Calib[4] << 8 | Mag_Calib[5];

	MPU6050_Read_All_DMA(&hi2c1, &MPU6050);
	MPU6050_Parsing(&MPU6050);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   	int count=0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET);
  while (1)
  {
	  	// extract the raw values
	  	int16_t  accel_x  = MPU6050.Ax;
	  	int16_t  accel_y  = MPU6050.Ay;
	  	int16_t  accel_z  = MPU6050.Az;
//	  	int16_t  mpu_temp = rx_buffer[6]  << 8 | rx_buffer[7];
	  	int16_t  gyro_x   = MPU6050.Gx;
	  	int16_t  gyro_y   = MPU6050.Gy;
	  	int16_t  gyro_z   = MPU6050.Gz;
	  	int16_t  magn_x   = MPU6050.Mag_X_RAW;
	  	int16_t  magn_y   = MPU6050.Mag_Y_RAW;
	  	int16_t  magn_z   = MPU6050.Mag_Z_RAW;

	  // convert accelerometer readings into G's
	  	accel_x_g = accel_x / 8192.0f;
	  	accel_y_g = accel_y / 8192.0f;
	  	accel_z_g = accel_z / 8192.0f;

	  	// convert temperature reading into degrees Celsius
//	  	mpu_temp_c = mpu_temp / 340.0f + 36.53f;

	  	// convert gyro readings into Radians per second
	  	gyro_x_rad = gyro_x / 939.650784f;
	  	gyro_y_rad = gyro_y / 939.650784f;
	  	gyro_z_rad = gyro_z / 939.650784f;

	  	// convert magnetometer readings into Gauss's
	  	magn_x_gs = magn_x / 660.0f;
	  	magn_y_gs = magn_y / 660.0f;
	  	magn_z_gs = magn_z / 660.0f;


	  g_h=accel_x_g;
	  g_lf=accel_x_g-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str, "%d.%d", g_h,g_l);


	  g_h=accel_y_g;
	  g_lf=accel_y_g-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str2,"%d.%d", g_h,g_l);


	  g_h=accel_z_g;
	  g_lf=accel_z_g-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str3, "%d.%d", g_h,g_l);


	  g_h=gyro_x_rad;
	  g_lf=gyro_x_rad-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str4, "%d.%d", g_h,g_l);


	  g_h=gyro_y_rad;
	  g_lf=gyro_y_rad-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str5, "%d.%d", g_h,g_l);


	  g_h=gyro_z_rad;
	  g_lf=gyro_z_rad-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str6, "%d.%d", g_h,g_l);


	  g_h=magn_x;
	  g_lf=magn_x-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str7, "%d.%d", g_h,g_l);


	  g_h=magn_y;
	  g_lf=magn_y-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str8, "%d.%d", g_h,g_l);


	  g_h=magn_z;
	  g_lf=magn_z-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str9, "%d.%d", g_h,g_l);


/*	  g_h=mpu_temp_c;
	  g_lf=mpu_temp_c-g_h;
	  g_l= trunc(g_lf*1000);
	  sprintf(str10, "%d.%d", g_h,g_l);
*/
	  f_puts(str , &fil);
	  f_puts("  ", &fil);
	  f_puts(str2 , &fil);
	  f_puts("  ", &fil);
	  f_puts(str3 , &fil);
	  f_puts( "  ", &fil);
	  f_puts(str4 , &fil);
	  f_puts("  ", &fil);
	  f_puts(str5 , &fil);
	  f_puts("  ", &fil);
	  f_puts(str6 , &fil);
	  f_puts( "  ", &fil);
	  f_puts(str7 , &fil);
	  f_puts("  ", &fil);
	  f_puts(str8 , &fil);
	  f_puts("  ", &fil);
	  f_puts(str9 , &fil);
	  f_puts( "  ", &fil);
	  f_puts(str10 , &fil);
	  f_puts( "  ", &fil);
	  f_puts( "\n", &fil);

	  count++;
	  if (count>5000){
		f_close(&fil);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_RESET);
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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
