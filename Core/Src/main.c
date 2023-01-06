/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM_MY_LCD16X2.h"
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
#define mpu9265Address 0x68<<1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
void getAccelerationMEM(int16_t* x, int16_t* y, int16_t* z);
void getRotationMEM(int16_t* x, int16_t* y, int16_t* z);
void getCompasMEM(int16_t* x, int16_t* y, int16_t* z);

uint8_t i2cBuf[8];
int16_t	ax,ay,az;
float Xaccel, Yaccel, Zaccel;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LCD1602_Begin8BIT(RS_GPIO_Port, RS_Pin, E_Pin, D0_GPIO_Port, D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
  LCD1602_noBlink();
  LCD1602_noCursor();


  i2cBuf[0] = 0x18;
  //init accelerometer for +-16g
  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1C, 1, i2cBuf, 1, 10);

  i2cBuf[0] = 0x18;
  //init accelerometer for +-2000 dps
  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x1B, 1, i2cBuf, 1, 10);

  LCD1602_print("x:");
  LCD1602_setCursor(1, 10);
  LCD1602_print("z:");
  LCD1602_2ndLine();
  LCD1602_print("y:");



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_I2C_Mem_Read(&hi2c1, mpu9265Address, 0x3B, 2, &i2cBuf[1], 2, 100);

	  /*getAccelerationMEM(&ax, &ay, &az);
	  Xaccel = ax/2048.0;
	  Yaccel = ay/2048.0;
	  Zaccel = az/2048.0;*/

	  /*getRotationMEM(&ax, &ay, &az);
	  Xaccel = ax/16.384;
	  Yaccel = ay/16.384;
	  Zaccel = az/16.384;*/

	  getCompasMEM(&ax, &ay, &az);
	  Xaccel = ax/1.7066666666666666666666666666667;
	  Yaccel = ay/1.7066666666666666666666666666667;
	  Zaccel = az/1.7066666666666666666666666666667;

	  LCD1602_setCursor(1, 3);
	  if(Xaccel > 0) LCD1602_print(" ");
	  LCD1602_PrintFloat(Xaccel, 2);


	  LCD1602_setCursor(1, 12);
	  if(Zaccel > 0) LCD1602_print(" ");
	  LCD1602_PrintFloat(Zaccel, 2);


	  LCD1602_setCursor(2, 3);
	  if(Yaccel > 0) LCD1602_print(" ");
	  LCD1602_PrintFloat(Yaccel, 2);

	  HAL_Delay((int)(1000/10));

	  /*LCD1602_clear();
	  LCD1602_print("Hello World");
	  LCD1602_2ndLine();
	  LCD1602_print("Patrick Here!");
	  HAL_Delay(1000);
	  LCD1602_clear();
	  LCD1602_PrintInt(1234);
	  HAL_Delay(1000);
	  LCD1602_2ndLine();
	  LCD1602_PrintFloat(1234.567, 2);
	  HAL_Delay(1000);*/
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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
	i2cBuf[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1, mpu9265Address, i2cBuf, 1, 10);

	HAL_I2C_Master_Receive(&hi2c1, mpu9265Address, &i2cBuf[1], 6, 10);

	*x = i2cBuf[1]<<8 | i2cBuf[2];
	*y = i2cBuf[3]<<8 | i2cBuf[4];
	*z = i2cBuf[5]<<8 | i2cBuf[6];

	return;
}

void getAccelerationMEM(int16_t* x, int16_t* y, int16_t* z)
{
	HAL_I2C_Mem_Read(&hi2c1, mpu9265Address, 0x3B, 1, &i2cBuf[1], 6, 100);

	*x = (int16_t)(i2cBuf[1]<<8 | i2cBuf[2]);
	*y = (int16_t)(i2cBuf[3]<<8 | i2cBuf[4]);
	*z = (int16_t)(i2cBuf[5]<<8 | i2cBuf[6]);

	return;
}

void getRotationMEM(int16_t* x, int16_t* y, int16_t* z)
{
	HAL_I2C_Mem_Read(&hi2c1, mpu9265Address, 0x3B, 1, &i2cBuf[1], 6, 100);

	*x = (int16_t)(i2cBuf[1]<<8 | i2cBuf[2]);
	*y = (int16_t)(i2cBuf[3]<<8 | i2cBuf[4]);
	*z = (int16_t)(i2cBuf[5]<<8 | i2cBuf[6]);

	return;
}

void getCompasMEM(int16_t* x, int16_t* y, int16_t* z)
{
	i2cBuf[0] = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, mpu9265Address, 0x37, 1, i2cBuf, 1, 100);

	i2cBuf[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, 0x0C<<1, 0x0A, 1, i2cBuf, 1, 100);
	HAL_Delay(100);

	HAL_I2C_Mem_Read(&hi2c1, 0x0C<<1, 0x03, 1, &i2cBuf[1], 6, 100);

	*x = ((int16_t)i2cBuf[2]<<8) | i2cBuf[1];
	*y = ((int16_t)i2cBuf[4]<<8) | i2cBuf[3];
	*z = ((int16_t)i2cBuf[6]<<8) | i2cBuf[5];

	return;
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
