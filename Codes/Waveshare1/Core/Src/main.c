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
#include "ssd1306.h"
#include "stdio.h"
#include "string.h"
#include "Waveshare_10Dof-D.h"
#include "mpu925x.h"
#include "mpu9255.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
FATFS fs;
FIL fil;

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

//I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
//static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 uint32_t total_time=-65535;
 MPU9255_t MPU9255;
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
  uint8_t u8Ret[1]={0};
  HAL_I2C_Mem_Read(&hi2c1, DevAddr,RegAddr,I2C_MEMADD_SIZE_8BIT,u8Ret,1,1000);
  return u8Ret[0];
}

void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t value)
{
  uint8_t buf[2]={0};

  buf[0] = RegAddr;
  buf[1] = value;
  HAL_I2C_Master_Transmit(&hi2c1,DevAddr,buf,2,100);


}

uint8_t mpu925x_stm32_i2c_hal_read(mpu925x_t *mpu925x, uint8_t slave_address, uint8_t reg, uint8_t *buffer, uint8_t size)
{
        return HAL_I2C_Mem_Read(mpu925x->master_specific.bus_handle, slave_address << 1, reg, 1, buffer, size, HAL_MAX_DELAY);
}

//mpu925x.master_specific.bus_read = mpu925x_stm32_i2c_hal_read;

uint8_t mpu925x_stm32_i2c_hal_write(mpu925x_t *mpu925x, uint8_t slave_address, uint8_t reg, uint8_t *buffer, uint8_t size)
{
        return HAL_I2C_Mem_Write(mpu925x->master_specific.bus_handle, slave_address << 1, reg, 1, buffer, size, HAL_MAX_DELAY);
}

//mpu925x.master_specific.bus_write = mpu925x_stm32_i2c_hal_write;

void mpu925x_stm32_hal_delay_ms(mpu925x_t *mpu925x, uint32_t delay)
{
        HAL_Delay(delay);
}

//mpu925x.master_specific.delay_ms = mpu925x_stm32_hal_delay_ms;

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  uint16_t timer_val=0;

 // mpu925x.master_specific.bus_handle = &hi2c1;

  HAL_Delay(10);
  f_mount(&fs, "", 0);
  f_open(&fil, "write.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
  HAL_Delay(10);

	IMU_EN_SENSOR_TYPE enMotionSensorType;
	IMU_EN_SENSOR_TYPE enPressureType;
//  	IMU_ST_ANGLES_DATA stAngles;
// 	IMU_ST_SENSOR_DATA stGyroRawData;
// 	IMU_ST_SENSOR_DATA stAccelRawData;
//  	IMU_ST_SENSOR_DATA stMagnRawData;
	int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
// 	icm20948init();
// 	bmp280Init();
	imuInit(&enMotionSensorType, &enPressureType);
/*  	if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
	{
      ssd1306_SetCursor(5,5);
		 char retVal= ssd1306_WriteString("Motion ICM-20948\n", Font_7x10, White);
		      ssd1306_UpdateScreen();
	}
	else
	{
      ssd1306_SetCursor(5,5);
		char retVal= ssd1306_WriteString("Motion NULL\n", Font_7x10, White);
		      ssd1306_UpdateScreen();
	}*/
	if(IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
	{
      ssd1306_SetCursor(5,15);
		char retVal= ssd1306_WriteString("Pressure BMP280\n", Font_7x10, White);
		      ssd1306_UpdateScreen();
	}
	else
	{
      ssd1306_SetCursor(0,13);
		char retVal= ssd1306_WriteString("Pressure NULL\n", Font_7x10, White);
		      ssd1306_UpdateScreen();
	}
	HAL_Delay(20);
      //ssd1306_Init();
      //ssd1306_Fill(White);
      //ssd1306_SetCursor(5,5);
      //char retVal= ssd1306_WriteString("Hello", Font_7x10, White);
      //ssd1306_UpdateScreen();



      // Wait till' initializition is complete. Will be in endless loop if sensor
      // is unreachable (wiring is not correct, sensor is damaged...).
      //while (mpu925x_init(&mpu925x, 0)!=0);
      while (MPU9255_Init(&hi2c1) == 1);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  HAL_Delay(10);
    int g_h;
     float g_lf;
     int g_l;
     uint8_t tx;
     char str[10];char str1[10];char str2[10];char str3[10];char str4[10];char str5[10];char str6[10];char str7[10];char str8[10];char str9[10];char str10[10];char str11[10];char str12[10]; char str13[10];

     int count=0;
         	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET);


            HAL_Delay(10);
      //      HAL_NVIC_EnableIRQ(TIM3_IRQn);
     //       __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE );
            HAL_TIM_Base_Start_IT(&htim10);
            float pitch;
            float yaw;
            float roll;
            uint8_t rawAccelData[6];  // x/y/z accel register data stored here
            uint16_t accelCount[3];
  while (1)
  {




	    /* USER CODE END WHILE */



	      // Use sensor data (e.g. print).
	   /*   printf("Acceleration: %f, %f, %f\n"
	             "Rotation: %f, %f, %f\n"
	             "Magnetic field: %f, %f, %f\n",
	             mpu925x.sensor_data.acceleration[0], mpu925x.sensor_data.acceleration[1], mpu925x.sensor_data.acceleration[2],
	             mpu925x.sensor_data.rotation[0], mpu925x.sensor_data.rotation[1], mpu925x.sensor_data.rotation[2],
	             mpu925x.sensor_data.magnetic_field[0], mpu925x.sensor_data.magnetic_field[1], mpu925x.sensor_data.magnetic_field[2]);

	*/

	//	  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

		  	//	printf("\r\n /-------------------------------------------------------------/ \r\n");
		  	//	printf("\r\n Roll: %.2f     Pitch: %.2f     Yaw: %.2f \r\n",stAngles.fRoll, stAngles.fPitch, stAngles.fYaw);


		 /* 		  	  sprintf(str, "%d", count);
		        ssd1306_SetCursor(5,15);
		  		retVal= ssd1306_WriteString(str, Font_7x10, White);
		  		      ssd1306_UpdateScreen();*/

		  //		printf("\r\n Acceleration: X: %d     Y: %d     Z: %d \r\n",stAccelRawData.s16X, stAccelRawData.s16Y, stAccelRawData.s16Z);
		  	//	printf("\r\n Gyroscope: X: %d     Y: %d     Z: %d \r\n",stGyroRawData.s16X, stGyroRawData.s16Y, stGyroRawData.s16Z);
		  	//	printf("\r\n Magnetic: X: %d     Y: %d     Z: %d \r\n",stMagnRawData.s16X, stMagnRawData.s16Y, stMagnRawData.s16Z);
		  	//	printf("\r\n Pressure: %.2f     Altitude: %.2f \r\n",(float)s32PressureVal/100, (float)s32AltitudeVal/100);
		  //		printf("\r\n Temperature: %.1f \r\n", (float)s32TemperatureVal/100);
		  //		HAL_Delay(100);


	      /*

						mpu925x.sensor_data.acceleration[0], mpu925x.sensor_data.acceleration[1], mpu925x.sensor_data.acceleration[2],
	             mpu925x.sensor_data.rotation[0], mpu925x.sensor_data.rotation[1], mpu925x.sensor_data.rotation[2],
	             mpu925x.sensor_data.magnetic_field[0], mpu925x.sensor_data.magnetic_field[1], mpu925x.sensor_data.magnetic_field[2]

	       */
		 /* timer_val = __HAL_TIM_GET_COUNTER(&htim10);
		  sprintf(str, "%d",(total_time+timer_val)/10000 );
		  	  		  	  ssd1306_SetCursor(0,21);
		  	  		  	  retVal= ssd1306_WriteString(str, Font_7x10, White);
		  	  		  	  ssd1306_UpdateScreen();*/
		  	  		      /* USER CODE BEGIN 3 */
	  // mpu925x_get_all_raw(&mpu925x);

		//readAccelData(&hi2c1,&accelCount);
	    	//  destination[0] = ((int16_t)rawAccelData[0] << 8) | rawAccelData[1];
	    //	pitch = MPU9255.pitch;
	    //	yaw = MPU9255.yaw;
	   // 	roll= MPU9255.roll;
		        // Get sensor data.
	    		HAL_Delay(10);

		    	readAll(&hi2c1, &MPU9255);



		        HAL_Delay(10);
		   /*     g_h=MPU9255.AccelX;
		        		  		  	  g_lf=(MPU9255.AccelX)-g_h;
		        		  		  	  g_l= abs(trunc(g_lf*1000));
		        		  		  	  sprintf(str3, "%d.%d", g_h,g_l);
		        		  	  		  	  ssd1306_SetCursor(0,21);
		        		  	  		  	  retVal= ssd1306_WriteString(str3, Font_7x10, White);
		        		  	  		  	  ssd1306_UpdateScreen();*/

			  	pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

			    timer_val = __HAL_TIM_GET_COUNTER(&htim10);

		  		      g_h=MPU9255.AccelX;
		  		  	  g_lf=(MPU9255.AccelX-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str, "%d.%d", g_h,g_l);


		  		  	  g_h=MPU9255.AccelY;
		  		  	  g_lf=(MPU9255.AccelY-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str2,"%d.%d", g_h,g_l);


		  		  	  g_h=MPU9255.AccelZ;
		  		  	  g_lf=(MPU9255.AccelZ-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str3, "%d.%d", g_h,g_l);


		  		  	  g_h= MPU9255.GyroX;
		  		  	  g_lf=(MPU9255.GyroX-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str4, "%d.%d", g_h,g_l);


		  		  	  g_h=MPU9255.GyroY;
		  		  	  g_lf=(MPU9255.GyroY-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str5, "%d.%d", g_h,g_l);


		  		  	  g_h=MPU9255.GyroZ;
		  		  	  g_lf=(MPU9255.GyroZ-g_h);
		  		  	  g_l=abs(trunc(g_lf*1000));
		  		  	  sprintf(str6, "%d.%d", g_h,g_l);


		  		  	  g_h=MPU9255.MagX;
		  		  	  g_lf=(MPU9255.MagX-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str7, "%d.%d", g_h,g_l);


		  		  	  g_h=MPU9255.MagY;
		  		  	  g_lf=(MPU9255.MagY-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str8, "%d.%d", g_h,g_l);


		  		  	  g_h=MPU9255.MagZ;
		  		  	  g_lf=(MPU9255.MagZ-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str9, "%d.%d", g_h,g_l);


		  		  	  g_h=(float)s32TemperatureVal/100;
		  		  	  g_lf=(((float)s32TemperatureVal/100)-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str10, "%d.%d", g_h,g_l);

		  		  	  g_h=(float)s32PressureVal/100;
		  		  	  g_lf=(((float)s32PressureVal/100)-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str11, "%d.%d", g_h,g_l);

		  		  	  g_h=(float)s32AltitudeVal/100;
		  		  	  g_lf=(((float)s32AltitudeVal/100)-g_h);
		  		  	  g_l= abs(trunc(g_lf*1000));
		  		  	  sprintf(str12, "%d.%d", g_h,g_l);

		  		  	  sprintf(str13, "%d",total_time+timer_val);

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
		  		  	  f_puts(str11 , &fil);
		  		 	  f_puts( "  ", &fil);
		  		 	  f_puts(str12 , &fil);
		  		 	  f_puts( "  ", &fil);
		  		 	  f_puts(str13, &fil);
		  		 	  f_puts( "  ", &fil);
		  		  	  f_puts( "\n", &fil);

		  		  	  count++;
		  		  	  if (count>2500)
		  		  	  {
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
//static void MX_I2C1_Init(void)
//{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 400000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//  {
 //   Error_Handler();
//  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

//}

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 17999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim10 )
  {
    total_time+=65535;
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
