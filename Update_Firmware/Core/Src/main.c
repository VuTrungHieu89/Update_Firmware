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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
volatile char rx_buff[5820] = {0};
uint8_t update_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USART1_Init(){
  // PB6 TX, PB7 RX
  // Baud rate 9600
  // DMA for USART

   __HAL_RCC_GPIOB_CLK_ENABLE();
   uint32_t* GPIOB_MODER = (uint32_t*)(0x40020400);
   uint32_t* GPIOB_AFRL = (uint32_t*)(0x40020420);
   *GPIOB_MODER &= ~(0b1111 << 12);     *GPIOB_MODER |= (0b1010 << 12);
   *GPIOB_AFRL &= ~(0b11111111 << 24);  *GPIOB_AFRL |= (0b01110111 << 24);

  	__HAL_RCC_USART1_CLK_ENABLE();
  	uint32_t* USART1_BRR = (uint32_t*)(0x40011008);
  	uint32_t* USART1_CR1 = (uint32_t*)(0x4001100c);

  	*USART1_BRR |= (104 << 4) | (3 << 0);
  	*USART1_CR1 |= (0b11 << 2);
  	*USART1_CR1 &= ~(1 << 10) & ~(1 << 12);
  	*USART1_CR1 |= (0b1 << 13);

  	uint32_t* USART1_CR3 = (uint32_t*)(0x40011014);
  	*USART1_CR3 |= (1 << 6);
}

void DMA2_USART1_Init(){
  	// USART1_RX Stream 5 chanel 4
  	__HAL_RCC_DMA2_CLK_ENABLE();
  	uint32_t* DMA2_S5CR = (uint32_t*)(0x40026488);
  	uint32_t* DMA2_S5NDTR = (uint32_t*)(0x4002648c);
  	uint32_t* DMA2_S5M0AR = (uint32_t*)(0x40026494);
  	uint32_t* DMA2_S5PAR = (uint32_t*)(0x40026490);
  	uint32_t* NVIC_ISER2 = (uint32_t*)(0xe000e108);

  	*DMA2_S5CR &= ~(0b111 << 25); *DMA2_S5CR |= (0b100 << 25);
  	*DMA2_S5CR |= (1 << 10);
  	*DMA2_S5CR |= (1 << 8);
  	*DMA2_S5PAR = (uint32_t)(0x40011004);
  	*DMA2_S5M0AR = (uint32_t)(rx_buff);
  	*DMA2_S5NDTR = (sizeof(rx_buff));
  	*DMA2_S5CR |= (1 << 4);
  	*NVIC_ISER2 |= (1 << 4);

  	*DMA2_S5CR |= (1 << 0);
}

void DMA2_Stream5_IRQHandler(){
	uint32_t* DMA2_HIFCR = (uint32_t*)(0x4002640c);
	*DMA2_HIFCR |= (1 << 11);
	update_flag = 1;
}

void USART1_Send_Data(uint8_t data){
  	uint32_t* USART1_SR = (uint32_t*)(0x40011000);
  	uint32_t* USART1_DR = (uint32_t*)(0x40011004);
  	while((*USART1_SR >> 7) == 0 );
  	*USART1_DR = data;
  	while(((*USART1_SR >> 6) & 1) == 0);
}

uint8_t USART1_Revc_Data(){
  	uint32_t* USART1_SR = (uint32_t*)(0x40011000);
  	uint32_t* USART1_DR = (uint32_t*)(0x40011004);
  	uint8_t data = 0;
  	while(((*USART1_SR >> 5) & 1 ) == 0);
  	data = *USART1_DR;
  	return data;
}
__attribute__((section(".FuncInRam"))) int Erase(int Sector){
  	if((Sector < 0) || (Sector > 7)){
  		return -1;
  	}
  	uint32_t* FLASH_SR = (uint32_t*)(0x40023c0c);
  	uint32_t* FLASH_CR = (uint32_t*)(0x40023c10);
  	uint32_t* FLASH_KEYR = (uint32_t*)(0x40023c04);

  	while(((*FLASH_SR >> 16) & 1) == 1);
  	if (((*FLASH_CR >> 31) & 1) == 1){
  		*FLASH_KEYR = 0x45670123;
  		*FLASH_KEYR = 0xCDEF89AB;
  	}
  	*FLASH_CR |= (1 << 1);
  	*FLASH_CR |=(Sector << 3);
  	*FLASH_CR |= (1 << 16);
  	while(((*FLASH_SR >> 16) & 1) == 1);
  	*FLASH_CR &= ~(1 << 1);

  	return 0;
}

__attribute__((section(".FuncInRam"))) void Program(char* address, char* data, uint32_t data_size){
  	uint32_t* FLASH_SR = (uint32_t*)(0x40023c0c);
  	uint32_t* FLASH_CR = (uint32_t*)(0x40023c10);
  	uint32_t* FLASH_KEYR = (uint32_t*)(0x40023c04);
  	while(((*FLASH_SR >> 16) & 1) == 1);
  	if (((*FLASH_CR >> 31) & 1) == 1){
  		*FLASH_KEYR = 0x45670123;
  		*FLASH_KEYR = 0xCDEF89AB;
  	}
  	*FLASH_CR |= (1 << 0);
  	for(int i = 0; i < data_size; i++){
  		address[i] = data[i];
  	}
  	while(((*FLASH_SR >> 16) & 1) == 1);
  	*FLASH_CR &= ~(1 << 0);
}
__attribute__((section(".FuncInRam"))) void Reset_system(){
	uint32_t* Control_AIRCR = (uint32_t*)(0xe000ed0c);
	*Control_AIRCR = (0x5FA << 16);
	*Control_AIRCR |= (1 << 2);
}
__attribute__((section(".FuncInRam"))) void Update_Firmware(){
	Erase(0);
	Program((char*)0x08000000, (char*)rx_buff, sizeof(rx_buff));
	Reset_system();
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
  USART1_Init();
  DMA2_USART1_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  HAL_SuspendTick();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(update_flag == 1){
		  Update_Firmware();
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
