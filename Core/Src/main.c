/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	CHANNEL_A,
	CHANNEL_B,
	CHANNEL_C,
	CHANNEL_D,
	CHANNEL_E,
	CHANNEL_F,
} CurrentChannel;

volatile CurrentChannel currentChannel = CHANNEL_A; // Начинаем с канала A
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint16_t adr_choice = 0x73; //Расширитель I2C

uint8_t disk_status[6] = {0}; // 6 байт для хранения состояний 24 дисков

uint8_t I2CInit_1[2] = {0x1, 0x0};
uint8_t I2CInit_2[2] = {0x2, 0x0};
uint8_t I2CTransmit[2] = {0x00, 0x00};


uint8_t isReceiving_A = 0;
uint16_t sgpioBuffer_A = 0;
uint8_t bitCounter_A = 0;

uint8_t isReceiving_B = 0;
uint16_t sgpioBuffer_B = 0;
uint8_t bitCounter_B = 0;

uint8_t isReceiving_C = 0;
uint16_t sgpioBuffer_C = 0;
uint8_t bitCounter_C = 0;

uint8_t isReceiving_D = 0;
uint16_t sgpioBuffer_D = 0;
uint8_t bitCounter_D = 0;

uint8_t isReceiving_E = 0;
uint16_t sgpioBuffer_E = 0;
uint8_t bitCounter_E = 0;

uint8_t isReceiving_F = 0;
uint16_t sgpioBuffer_F = 0;
uint8_t bitCounter_F = 0;

uint8_t sgpio_started = 0;
uint8_t sgpio_detected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */




void Update_Disk_Status(uint8_t disk_id, uint8_t status) {
	if (disk_id >= 24) return; // Проверка на корректность disk_id

	// Вычисляем позицию в массиве disk_status
	uint8_t byte_index = disk_id / 4; // Каждый байт содержит 4 диска
	uint8_t bit_offset = (disk_id % 4) * 2; // Смещение в байте (0, 2, 4, 6)

	// Очищаем старые биты и записываем новые
	disk_status[byte_index] &= ~(0x03 << bit_offset); // Очистка битов
	disk_status[byte_index] |= (status & 0x03) << bit_offset; // Запись новых битов
}

void ProcessSGPIOData(uint16_t sgpioData, uint8_t startIndex)
{

	for (int i = 0; i < 4; ++i) {
		uint8_t diskIndex = startIndex + i; // ндекс диска (0-3 для A, 4- 7 для B)
		uint8_t diskStatus = (sgpioData >> (3 * i)) & 0x07; // �?звлечение 3 бит для диска

		if (((diskStatus >> 0) & 0x01) == 1) Update_Disk_Status(diskIndex, 0x01); // 1-й бит - активность
		if (((diskStatus >> 1) & 0x01) == 1) Update_Disk_Status(diskIndex, 0x02);   // 2-й бит - локация
		if (((diskStatus >> 2) & 0x01) == 1) Update_Disk_Status(diskIndex, 0x03);    // 3-й бит - ошибка

		// Обновление статуса диска

	}
}
void sendI2c(int reg, int value, uint16_t adr_rep)
{
	I2CTransmit[0] = reg;
	I2CTransmit[1] = value;
	HAL_I2C_Master_Transmit(&hi2c2, (adr_rep << 1), I2CTransmit, 2, 100);
	HAL_Delay(10);
}


void redriver_init_1 (uint16_t adr_rep) //RX1
{
	sendI2c(0x06, 0x18, adr_rep); 	//�?нициализация каналов
	sendI2c(0x0F, 0x00, adr_rep);	// db B_0
	sendI2c(0x10, 0xAE, adr_rep);  // VOD/VID

	sendI2c(0x11, 0x00, adr_rep);  // Ослабелние
	sendI2c(0x16, 0x00, adr_rep);  //db B_1
	sendI2c(0x17, 0xAE, adr_rep);

	sendI2c(0x18, 0x00, adr_rep);
	sendI2c(0x1D, 0x00, adr_rep);	//db B_2
	sendI2c(0x1E, 0xAE, adr_rep);

	sendI2c(0x1F, 0x00, adr_rep);
	sendI2c(0x24, 0x00, adr_rep);	//db B_3
	sendI2c(0x25, 0xAE, adr_rep);
	sendI2c(0x26, 0x00, adr_rep);
	sendI2c(0x2C, 0x00, adr_rep);	//db A_0
	sendI2c(0x2D, 0xAE, adr_rep);
	sendI2c(0x2E, 0x00, adr_rep);
	sendI2c(0x33, 0x00, adr_rep);	//db A_1
	sendI2c(0x34, 0xAE, adr_rep);
	sendI2c(0x35, 0x00, adr_rep);
	sendI2c(0x3A, 0x00, adr_rep);	//db A_2
	sendI2c(0x3B, 0xAE, adr_rep);
	sendI2c(0x3C, 0x00, adr_rep);
	sendI2c(0x41, 0x00, adr_rep);	//db A_3
	sendI2c(0x42, 0xAE, adr_rep);
	sendI2c(0x43, 0x00, adr_rep);
}

void redriver_init_2 (uint16_t adr_rep) //RX2
{
	sendI2c(0x06, 0x18, adr_rep); 	//�?нициализация каналов
	sendI2c(0x0F, 0x00, adr_rep);	//db B_0
	sendI2c(0x10, 0xAE, adr_rep);
	sendI2c(0x11, 0x00, adr_rep);
	sendI2c(0x16, 0x00, adr_rep);  //db B_1
	sendI2c(0x17, 0xAE, adr_rep);
	sendI2c(0x18, 0x00, adr_rep);
	sendI2c(0x1D, 0x00, adr_rep);	//db B_2
	sendI2c(0x1E, 0xAE, adr_rep);
	sendI2c(0x1F, 0x00, adr_rep);
	sendI2c(0x24, 0x01, adr_rep);	//db B_3
	sendI2c(0x25, 0xAE, adr_rep);
	sendI2c(0x26, 0x00, adr_rep);
	sendI2c(0x2C, 0x01, adr_rep);	//db A_0
	sendI2c(0x2D, 0xAE, adr_rep);
	sendI2c(0x2E, 0x00, adr_rep);
	sendI2c(0x33, 0x00, adr_rep);	//db A_1
	sendI2c(0x34, 0xAE, adr_rep);
	sendI2c(0x35, 0x00, adr_rep);
	sendI2c(0x3A, 0x00, adr_rep);	//db A_2
	sendI2c(0x3B, 0xAE, adr_rep);
	sendI2c(0x3C, 0x00, adr_rep);
	sendI2c(0x41, 0x00, adr_rep);	//db A_3
	sendI2c(0x42, 0xAE, adr_rep);
	sendI2c(0x43, 0x00, adr_rep);
}

void redriver_init_3 (uint16_t adr_rep) //RX3
{
	sendI2c(0x06, 0x18, adr_rep); 	//�?нициализация каналов
	sendI2c(0x0F, 0x00, adr_rep);	//db B_0
	sendI2c(0x10, 0xAE, adr_rep);
	sendI2c(0x11, 0x00, adr_rep);
	sendI2c(0x16, 0x00, adr_rep);  //db B_1
	sendI2c(0x17, 0xAE, adr_rep);
	sendI2c(0x18, 0x00, adr_rep);
	sendI2c(0x1D, 0x01, adr_rep);	//db B_2
	sendI2c(0x1E, 0xAE, adr_rep);
	sendI2c(0x1F, 0x00, adr_rep);
	sendI2c(0x24, 0x00, adr_rep);	//db B_3
	sendI2c(0x25, 0xAE, adr_rep);
	sendI2c(0x26, 0x00, adr_rep);
	sendI2c(0x2C, 0x00, adr_rep);	//db A_0
	sendI2c(0x2D, 0xAE, adr_rep);
	sendI2c(0x2E, 0x00, adr_rep);
	sendI2c(0x33, 0x00, adr_rep);	//db A_1
	sendI2c(0x34, 0xAE, adr_rep);
	sendI2c(0x35, 0x00, adr_rep);
	sendI2c(0x3A, 0x00, adr_rep);	//db A_2
	sendI2c(0x3B, 0xAE, adr_rep);
	sendI2c(0x3C, 0x00, adr_rep);
	sendI2c(0x41, 0x00, adr_rep);	//db A_3
	sendI2c(0x42, 0xAE, adr_rep);
	sendI2c(0x43, 0x00, adr_rep);
}

void redriver_init_other1 (uint16_t adr_rep) //TX1
{
	sendI2c(0x06, 0x18, adr_rep); 	//�?нициализация каналов
	sendI2c(0x0F, 0x00, adr_rep);	// db B_0
	sendI2c(0x10, 0xAE, adr_rep);  // VOD/VID
	sendI2c(0x11, 0x00, adr_rep);  // Ослабелние
	sendI2c(0x16, 0x00, adr_rep);  //db B_1
	sendI2c(0x17, 0xAE, adr_rep);
	sendI2c(0x18, 0x00, adr_rep);
	sendI2c(0x1D, 0x00, adr_rep);	//db B_2
	sendI2c(0x1E, 0xAE, adr_rep);
	sendI2c(0x1F, 0x00, adr_rep);
	sendI2c(0x24, 0x00, adr_rep);	//db B_3
	sendI2c(0x25, 0xAE, adr_rep);
	sendI2c(0x26, 0x00, adr_rep);
	sendI2c(0x2C, 0x00, adr_rep);	//db A_0
	sendI2c(0x2D, 0xAE, adr_rep);
	sendI2c(0x2E, 0x00, adr_rep);
	sendI2c(0x33, 0x00, adr_rep);	//db A_1
	sendI2c(0x34, 0xAE, adr_rep);
	sendI2c(0x35, 0x00, adr_rep);
	sendI2c(0x3A, 0x00, adr_rep);	//db A_2
	sendI2c(0x3B, 0xAE, adr_rep);
	sendI2c(0x3C, 0x03, adr_rep);
	sendI2c(0x41, 0x00, adr_rep);	//db A_3
	sendI2c(0x42, 0xAE, adr_rep);
	sendI2c(0x43, 0x00, adr_rep);
}

void redriver_init_other2 (uint16_t adr_rep) //TX2
{
	sendI2c(0x06, 0x18, adr_rep); 	//�?нициализация каналов
	sendI2c(0x0F, 0x00, adr_rep);	// db B_0
	sendI2c(0x10, 0xAE, adr_rep);
	sendI2c(0x11, 0x00, adr_rep);
	sendI2c(0x16, 0x00, adr_rep);  //db B_1
	sendI2c(0x17, 0xAE, adr_rep);
	sendI2c(0x18, 0x00, adr_rep);
	sendI2c(0x1D, 0x00, adr_rep);	//db B_2
	sendI2c(0x1E, 0xAE, adr_rep);
	sendI2c(0x1F, 0x00, adr_rep);
	sendI2c(0x24, 0x03, adr_rep);	//db B_3
	sendI2c(0x25, 0xA8, adr_rep);
	sendI2c(0x26, 0x00, adr_rep);
	sendI2c(0x2C, 0x03, adr_rep);	//db A_0
	sendI2c(0x2D, 0xA8, adr_rep);
	sendI2c(0x2E, 0x00, adr_rep);
	sendI2c(0x33, 0x00, adr_rep);	//db A_1
	sendI2c(0x34, 0xAE, adr_rep);
	sendI2c(0x35, 0x00, adr_rep);
	sendI2c(0x3A, 0x00, adr_rep);	//db A_2
	sendI2c(0x3B, 0xAE, adr_rep);
	sendI2c(0x3C, 0x00, adr_rep);
	sendI2c(0x41, 0x00, adr_rep);	//db A_3
	sendI2c(0x42, 0xAE, adr_rep);
	sendI2c(0x43, 0x00, adr_rep);
}

void redriver_init_other3 (uint16_t adr_rep) //TX3
{
	sendI2c(0x06, 0x18, adr_rep); 	//�?нициализация каналов
	sendI2c(0x0F, 0x00, adr_rep);	//db B_0
	sendI2c(0x10, 0xA8, adr_rep);
	sendI2c(0x11, 0x05, adr_rep);
	sendI2c(0x16, 0x00, adr_rep);  //db B_1
	sendI2c(0x17, 0xAE, adr_rep);
	sendI2c(0x18, 0x00, adr_rep);
	sendI2c(0x1D, 0x03, adr_rep);	//db B_2
	sendI2c(0x1E, 0xA8, adr_rep);
	sendI2c(0x1F, 0x00, adr_rep);
	sendI2c(0x24, 0x00, adr_rep);	//db B_3
	sendI2c(0x25, 0xAE, adr_rep);
	sendI2c(0x26, 0x05, adr_rep);
	sendI2c(0x2C, 0x00, adr_rep);	//db A_0
	sendI2c(0x2D, 0xAE, adr_rep);
	sendI2c(0x2E, 0x03, adr_rep);
	sendI2c(0x33, 0x00, adr_rep);	//db A_1
	sendI2c(0x34, 0xAE, adr_rep);
	sendI2c(0x35, 0x00, adr_rep);
	sendI2c(0x3A, 0x00, adr_rep);	//db A_2
	sendI2c(0x3B, 0xAE, adr_rep);
	sendI2c(0x3C, 0x03, adr_rep);
	sendI2c(0x41, 0x00, adr_rep);	//db A_3
	sendI2c(0x42, 0xAE, adr_rep);
	sendI2c(0x43, 0x03, adr_rep);
}
void Redriver_Init(void)
{
	HAL_I2C_Master_Transmit(&hi2c2, (adr_choice << 1), I2CInit_1, 1, 1);  //Выбор канала 0 на расширителе
	//RX
	HAL_Delay(10);
	redriver_init_1 (0x59); //Зашиваем 1 редрайвер на прием
	HAL_Delay(10);
	redriver_init_2 (0x5B); //2
	HAL_Delay(10);
	redriver_init_3 (0x5F); //3
	HAL_Delay(10);
	HAL_I2C_Master_Transmit(&hi2c2, (adr_choice << 1), I2CInit_2, 1, 1);  //Выбор канала 1 на расширителе
	//TX
	HAL_Delay(10);
	redriver_init_other1 (0x59); //Зашиваем 1 редрайвер на передачу
	HAL_Delay(10);
	redriver_init_other2 (0x5B); //2
	HAL_Delay(10);
	redriver_init_other3 (0x5F); //3
}

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(100);
	Redriver_Init();
	HAL_Delay(100);
	// Включаем прерывания для I2C
	HAL_NVIC_EnableIRQ(I2C1_IRQn);

	// Пример обновления состояния дисков

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_I2C_Slave_Transmit(&hi2c1, disk_status, 6, HAL_MAX_DELAY);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hi2c1.Init.Timing = 0x00C12469;
  hi2c1.Init.OwnAddress1 = 72;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hi2c2.Init.Timing = 0x00C12469;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 63999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : C_SGPIO_SClock_Pin E_SGPIO_SLoad_Pin C_SGPIO_SLoad_Pin */
  GPIO_InitStruct.Pin = C_SGPIO_SClock_Pin|E_SGPIO_SLoad_Pin|C_SGPIO_SLoad_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A_SGPIO_SClock_Pin A_SGPIO_SLoad_Pin B_SGPIO_SClock_Pin B_SGPIO_SLoad_Pin
                           D_SGPIO_SClock_Pin */
  GPIO_InitStruct.Pin = A_SGPIO_SClock_Pin|A_SGPIO_SLoad_Pin|B_SGPIO_SClock_Pin|B_SGPIO_SLoad_Pin
                          |D_SGPIO_SClock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A_SGPIO_SData_Pin B_SGPIO_SData_Pin */
  GPIO_InitStruct.Pin = A_SGPIO_SData_Pin|B_SGPIO_SData_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E_SGPIO_SClock_Pin F_SGPIO_SClock_Pin F_SGPIO_SLoad_Pin */
  GPIO_InitStruct.Pin = E_SGPIO_SClock_Pin|F_SGPIO_SClock_Pin|F_SGPIO_SLoad_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C_SGPIO_SData_Pin E_SGPIO_SData_Pin D_SGPIO_SData_Pin */
  GPIO_InitStruct.Pin = C_SGPIO_SData_Pin|E_SGPIO_SData_Pin|D_SGPIO_SData_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : F_SGPIO_SData_Pin */
  GPIO_InitStruct.Pin = F_SGPIO_SData_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(F_SGPIO_SData_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D_SGPIO_SLoad_Pin */
  GPIO_InitStruct.Pin = D_SGPIO_SLoad_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(D_SGPIO_SLoad_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == A_SGPIO_SLoad_Pin && currentChannel == CHANNEL_A)
	{
		// Начало приема данных для канала A
		isReceiving_A = 1;
		sgpioBuffer_A = 0;
		bitCounter_A = 0;
		sgpio_started = 1;
	}
	else if (GPIO_Pin == B_SGPIO_SLoad_Pin && currentChannel == CHANNEL_B)
	{
		// Начало приема данных для канала B
		isReceiving_B = 1;
		sgpioBuffer_B = 0;
		bitCounter_B = 0;
		sgpio_started = 1;
	}
	else if (GPIO_Pin == C_SGPIO_SLoad_Pin && currentChannel == CHANNEL_C)
	{
		// Начало приема данных для канала B
		isReceiving_C = 1;
		sgpioBuffer_C = 0;
		bitCounter_C = 0;
		sgpio_started = 1;
	}
	else if (GPIO_Pin == D_SGPIO_SLoad_Pin && currentChannel == CHANNEL_D)
	{
		// Начало приема данных для канала B
		isReceiving_D = 1;
		sgpioBuffer_D = 0;
		bitCounter_D = 0;
		sgpio_started = 1;
	}
	else if (GPIO_Pin == E_SGPIO_SLoad_Pin && currentChannel == CHANNEL_E)
	{
		// Начало приема данных для канала B
		isReceiving_E = 1;
		sgpioBuffer_E = 0;
		bitCounter_E = 0;
		sgpio_started = 1;
	}
	else if (GPIO_Pin == F_SGPIO_SLoad_Pin && currentChannel == CHANNEL_F)
	{
		// Начало приема данных для канала B
		isReceiving_F = 1;
		sgpioBuffer_F = 0;
		bitCounter_F = 0;
		sgpio_started = 1;
	}

	if (GPIO_Pin == A_SGPIO_SClock_Pin && isReceiving_A && currentChannel == CHANNEL_A)
	{
		// Прием данных для канала A
		uint8_t dataBit = HAL_GPIO_ReadPin(A_SGPIO_SData_GPIO_Port, A_SGPIO_SData_Pin);

		// Сохранение бита в буфер
		sgpioBuffer_A |= (dataBit << bitCounter_A);
		bitCounter_A++;

		// Если принято 12 бит, завершить прием
		if (bitCounter_A >= 12) {
			isReceiving_A = 0;
			ProcessSGPIOData(sgpioBuffer_A, 0); // Обработка данных для дисков 0-3

			if(sgpioBuffer_A !=0)
			{
				sgpio_detected = 1;
				HAL_TIM_Base_Stop_IT(&htim2);
			}

			sgpioBuffer_A = 0; // Сброс буфера

			// Переключаемся на канал B только если прием данных завершен
			if (!isReceiving_B) {
				currentChannel = CHANNEL_B;
			}

		}
	}
	else if (GPIO_Pin == B_SGPIO_SClock_Pin && isReceiving_B && currentChannel == CHANNEL_B)
	{
		// Прием данных для канала B
		uint8_t dataBit = HAL_GPIO_ReadPin(B_SGPIO_SData_GPIO_Port, B_SGPIO_SData_Pin);

		// Сохранение бита в буфер
		sgpioBuffer_B |= (dataBit << bitCounter_B);
		bitCounter_B++;

		// Если принято 12 бит, завершить прием
		if (bitCounter_B >= 12) {
			isReceiving_B = 0;
			ProcessSGPIOData(sgpioBuffer_B, 4); // Обработка данных для дисков 4-7
			if(sgpioBuffer_B != 0)
			{
				sgpio_detected = 1;
				HAL_TIM_Base_Stop_IT(&htim2);
			}

			sgpioBuffer_B = 0; // Сброс буфера

			// Переключаемся на канал A
			if (!isReceiving_A) {
				currentChannel = CHANNEL_C;
			}
		}
	}else if (GPIO_Pin == C_SGPIO_SClock_Pin && isReceiving_C && currentChannel == CHANNEL_C)
	{
		// Прием данных для канала A
		uint8_t dataBit = HAL_GPIO_ReadPin(C_SGPIO_SData_GPIO_Port, C_SGPIO_SData_Pin);

		// Сохранение бита в буфер
		sgpioBuffer_C |= (dataBit << bitCounter_C);
		bitCounter_C++;

		// Если принято 12 бит, завершить прием
		if (bitCounter_C >= 12) {
			isReceiving_C = 0;
			ProcessSGPIOData(sgpioBuffer_C, 0); // Обработка данных для дисков 0-3

			if(sgpioBuffer_C !=0)
			{
				sgpio_detected = 1;
				HAL_TIM_Base_Stop_IT(&htim2);
			}

			sgpioBuffer_C = 0; // Сброс буфера

			// Переключаемся на канал B только если прием данных завершен
			if (!isReceiving_C) {
				currentChannel = CHANNEL_D;
			}

		}
	}
	else if (GPIO_Pin == D_SGPIO_SClock_Pin && isReceiving_D && currentChannel == CHANNEL_D)
	{
		// Прием данных для канала B
		uint8_t dataBit = HAL_GPIO_ReadPin(D_SGPIO_SData_GPIO_Port, D_SGPIO_SData_Pin);

		// Сохранение бита в буфер
		sgpioBuffer_D |= (dataBit << bitCounter_D);
		bitCounter_D++;

		// Если принято 12 бит, завершить прием
		if (bitCounter_D >= 12) {
			isReceiving_D = 0;
			ProcessSGPIOData(sgpioBuffer_D, 4); // Обработка данных для дисков 4-7
			if(sgpioBuffer_D != 0)
			{
				sgpio_detected = 1;
				HAL_TIM_Base_Stop_IT(&htim2);
			}

			sgpioBuffer_D = 0; // Сброс буфера

			// Переключаемся на канал A
			if (!isReceiving_D) {
				currentChannel = CHANNEL_E;
			}
		}
	}else if (GPIO_Pin == E_SGPIO_SClock_Pin && isReceiving_E && currentChannel == CHANNEL_E)
	{
		// Прием данных для канала A
		uint8_t dataBit = HAL_GPIO_ReadPin(E_SGPIO_SData_GPIO_Port, E_SGPIO_SData_Pin);

		// Сохранение бита в буфер
		sgpioBuffer_E |= (dataBit << bitCounter_E);
		bitCounter_E++;

		// Если принято 12 бит, завершить прием
		if (bitCounter_E >= 12) {
			isReceiving_E = 0;
			ProcessSGPIOData(sgpioBuffer_E, 0); // Обработка данных для дисков 0-3

			if(sgpioBuffer_E !=0)
			{
				sgpio_detected = 1;
				HAL_TIM_Base_Stop_IT(&htim2);
			}

			sgpioBuffer_E = 0; // Сброс буфера

			// Переключаемся на канал B только если прием данных завершен
			if (!isReceiving_E) {
				currentChannel = CHANNEL_F;
			}

		}
	}
	else if (GPIO_Pin == F_SGPIO_SClock_Pin && isReceiving_F && currentChannel == CHANNEL_F)
	{
		// Прием данных для канала B
		uint8_t dataBit = HAL_GPIO_ReadPin(F_SGPIO_SData_GPIO_Port, F_SGPIO_SData_Pin);

		// Сохранение бита в буфер
		sgpioBuffer_F |= (dataBit << bitCounter_F);
		bitCounter_F++;

		// Если принято 12 бит, завершить прием
		if (bitCounter_F >= 12) {
			isReceiving_F = 0;
			ProcessSGPIOData(sgpioBuffer_F, 4); // Обработка данных для дисков 4-7
			if(sgpioBuffer_F != 0)
			{
				sgpio_detected = 1;
				HAL_TIM_Base_Stop_IT(&htim2);
			}

			sgpioBuffer_F = 0; // Сброс буфера

			// Переключаемся на канал A
			if (!isReceiving_F) {
				currentChannel = CHANNEL_A;
			}
		}
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
