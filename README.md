**WARNING:** This library is written on an STM32F407-DISC1 so its header file is `stm32f4xx_hal.h`. İf you use another board, for example STM32F103C8 BluePill, just change that to `stm32f1xx_hal.h` in our BME280_STM32.h header file.
```c
#include "stm32f1xx_hal.h"
```
**WARNING:** We used the `printf()` function in this library. Therefore, you should activate the printf function. If you don't know how to do that just google it :)

Firstly, you must create a project with CubeMX and activate I2C protocol. Then add the header file `BME280_STM32.h` into the `../Core/Inc` directory and the `BME280.c` source file into the `../Core/Src` directory.

![image](https://github.com/user-attachments/assets/295d89be-4cc2-4bf5-8bb9-74a19ace7034)

**WARNING:** Change this line in the header file for whichever I2C protocol you have opened. For example hi2c2
```c
extern I2C_HandleTypeDef hi2c1;
#define bme_i2c (hi2c1)
```

The first thing you need to do before using the library is to change the I2C address in the `BME280_STM32.h` header file according to the module you are using (it's `0x76` or `0x77`). If you have designed a pcb and are using a BME280 chip, this value changes depending on what you connect the SDO pin of the chip to.

```c
#define BME280_ADDR (0x76 << 1) 
```
Now, lets explain how can use BNO055 library. With this library, you can obtain temperature, humidity and pressure datas. The first thing you need to do is to set the parameters of the BME280. So came to the main.c file of your project and create a function named like Sensor_Init or something.  

```c
void Sensor_Init(void);

void Sensor_Init(void)
{

  //Init structure definition section
	BME280_Init_t BME280_InitStruct = {0};

	//Reset section
	Reset_BME280();

	/*============================ *BME280 Initialization* ============================*/

	BME280_InitStruct.Filter = FILTER_8;     				//FILTER_X
	BME280_InitStruct.Mode = BME280_NORMAL_MODE;		 	//SLEEP, NORMAL or FORCE can be written
	BME280_InitStruct.OverSampling_H = OVERSAMPLING_16;		//OVERSAMPLING_X
	BME280_InitStruct.OverSampling_P = OVERSAMPLING_16;		//OVERSAMPLING_X
	BME280_InitStruct.OverSampling_T = OVERSAMPLING_16;		//OVERSAMPLING_X
	BME280_InitStruct.SPI_EnOrDıs = SPI3_W_DISABLE;			//SPI3_W_DISABLE or SPI3_W_ENABLE can be written
	BME280_InitStruct.T_StandBy = T_SB_250;					//T_SB_X

	BME280Init(BME280_InitStruct);
}
```
- Save the sensor parameter settings in the structure named `BME280_InitStruct` The values ​​that can be written in the comments section for each parameter are shown. For example, if you type `FILTER_` and then press `Ctrl + space`, you can see the current values.

And that's all. To initialize the BME280 sensor, all you need to do is run the `Sensor Init();` function before the while loop. So how can we access the sensor data? To do this, define a variable named `BME280_Data_t BME280;` at the top of the main.c file. The `BME280_Data_t ` structure holds the sensor datas on each axis ​​and its structure is as follows.
```c
typedef struct {
    float Temperature;
    float Pressure;
    float Humidity;
    float AltitudeP;
    float AltitudeTP;
}BME280_Data_t;
```
Now come to the while loop and write this function.
```c
BME280Calculation(&BME280);
```
Thats all, you obtained sensor datas.
### Full example
```c
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
#include "BME280_STM32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

BME280_Data_t BME280; 		/*!BME280 data structure definition which hold temperature, pressure, humidity and altitude parameters*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Sensor_Init(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  Sensor_Init();  				/*!Initialization of sensor groups*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    BME280Calculation(&BME280);
  }
  /* USER CODE END 3 */
}

void Sensor_Init(void)
{

  //Init structure definition section
	BME280_Init_t BME280_InitStruct = {0};

	//Reset section
	Reset_BME280();

	/*============================ *BME280 Initialization* ============================*/

	BME280_InitStruct.Filter = FILTER_8;     				//FILTER_X
	BME280_InitStruct.Mode = BME280_NORMAL_MODE;		 	//SLEEP, NORMAL or FORCE can be written
	BME280_InitStruct.OverSampling_H = OVERSAMPLING_16;		//OVERSAMPLING_X
	BME280_InitStruct.OverSampling_P = OVERSAMPLING_16;		//OVERSAMPLING_X
	BME280_InitStruct.OverSampling_T = OVERSAMPLING_16;		//OVERSAMPLING_X
	BME280_InitStruct.SPI_EnOrDıs = SPI3_W_DISABLE;			//SPI3_W_DISABLE or SPI3_W_ENABLE can be written
	BME280_InitStruct.T_StandBy = T_SB_250;					//T_SB_X

	BME280Init(BME280_InitStruct);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD10 PD12 PD1 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
