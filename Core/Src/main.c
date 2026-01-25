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
  * Default packet structure:
  * 1st byte - receiver address (1; 2; 3; 4; 5; 6)
  * 2st byte - packet function (AA = CHANGE PARAMS; FF = Keep awake) //ToDo: Set new interval
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> //memcpy
#include <stdbool.h> //bool
#include "si4432.h"
#include "i2c_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//RX defines
//RX packet: 0-DeviceHW_Number (from DIP switch); 1-DeviceSW_Number (Address); 2-MessageType (DATA,MOD,AWAKE); 3-data
//Device HW_NUMBER: 0-not used; 1-DIP SWITCH; 2-DIP SWITCH; 3-DIP SWITCH; ...
//Device SW_NUMBER: assigned by transmitter for more effective transmission: 0-not used; 1;2;3;4;5;...
//MOD: Address,

#define COMMON_ADDRESS 0x00
#define MAX_RX_NUM 6 //maximum number of transmit groups for lights

#define NUM_OF_CHANNELS   6   //num of data bytes -> channels used by one receiver
#define NUM_OF_RECEIVERS  1

#define DATA_PACKET_LENGTH 12 //payload size; 64 = max for SI4432; //ToDo: HEADER_SIZE+NUM_OF_CHANNELS
#define DATA_PACKET_MARK 0xAA
#define DATA_HEADER_SIZE 6    //num of bytes for header
#define DATA_START_POS 3 //Position of packet on which DMX data starts

#define MOD_PACKET_LENGTH 5
#define MOD_ADDRESS_MARK 0xCC

#define AWAKE_PACKET_LENGTH 3
#define AWAKE_PACKET_PERIOD 20 //20ms by default
#define AWAKE_PACKET_MARK 0xFF

//DMX defines
#define DMXPACKET_SIZE 513 //do not change
#define DMX_STARTBYTE 0 //defines what startbyte the receiver listens to; 0 default for light control by standard

I2C_LCD_HandleTypeDef lcd1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//ToDo: Rozmístit do samostatných souborů DMX a RF

static uint8_t dmxRX[514]; 			 	 //DMX receive buffer
static uint8_t dmxPacket[513]; 	 //One received DMX packet
static uint8_t dmxPrevPacket[513]; 	//One previously received DMX packet
static bool dmxPacketRdy=false; 	     //Flag - dmxPacketReady
static bool readyToTransmit=true;
static uint16_t currentDMXAddress=1; 	 //Current address of the DMX receiver
uint8_t toSendPacket[6];


static uint8_t dataPacket[MAX_RX_NUM][DATA_PACKET_LENGTH]; //RF transmit buffer; ToDo: Rename to dataPacket

static uint8_t awakePacket[AWAKE_PACKET_LENGTH]; //RF transmit buffer - awake packet; only one -> changes it's address in a for loop



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void addAdressToPacket(void) //adds address into the header
{
	for(uint8_t i=0; i<MAX_RX_NUM; i++)
	{
		dataPacket[i][0]=i+1;
	}
}

//---DMX receiving Callback---
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	//memcpy(dmxPrevPacket, dmxPacket, DMXPACKET_SIZE);
	//memcpy(&dmxPacket[0], &dmxRX[1], DMXPACKET_SIZE);

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, 514); //DMXPACKET_SIZE
    dmxPacketRdy=true;
    //dmxPacket[3]=255;
    //HAL_Delay(5000);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Prisel BREAK (Framing Error) -> Restartujeme prijem

        HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, 514);
        //dmxPacketRdy=true;
    }
}
//---RF EXTI Callback---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//EXTI for RF module (SI4432)
	if (GPIO_Pin == GPIO_PIN_11)
	{
    	uint8_t b; //jen pro reset 0x03 registru
    	SI44_Read(0x04, &b, 1);
        SI44_Read(0x03, &b, 1); //jinak by uz neaktivoval IRQ
        readyToTransmit=true;

        //What caused the interrupt?
		/*switch(b)
		{
		case(0b00000100): //TX done
				readyToTransmit=true;
				break;
		case(0b00000010): //RX done
				//rxDoneFlag=true; - in case of receiving
				break;
		}*/
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	dmxPacket[0]=0;
	dmxPacket[1]=255;
	dmxPacket[2]=255;
	dmxPacket[3]=0;
	dmxPacket[4]=0;
	dmxPacket[5]=0;
	dmxPacket[6]=0;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, DMXPACKET_SIZE+1);
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, DMXPACKET_SIZE+1);

  //---Initialize:---

  //---SI4432--- //musi byt pred SI, jinak se buguje jako krava
  SI44_Init(&hspi2, GPIOB, GPIO_PIN_12);
  HAL_Delay(500); //ToDo: zkratit
  SI44_PresetConfig();
  SI44_SetAGCMode(0b00100000); //6th bit - sgin
  SI44_SetInterrupts1(0b00000100); //1st bit = CRC error
  SI44_SetInterrupts2(0b00000000);
  SI44_SetTXPower(SI44_TX_POWER_11dBm);    //Set TX power to 11dBm (12.5 mW)

  //---LCD---
  lcd1.hi2c = &hi2c1;
  lcd1.address = (0x27 << 1);
  lcd_init(&lcd1);
  lcd_gotoxy(&lcd1, 0, 0);
  lcd_puts(&lcd1, "LightTube");


  //HAL_Delay(5000);
  //---Variables---
  //awakePacket[0]=COMMON_ADDRESS;
  //awakePacket[2]=AWAKE_PACKET_MARK;
  //HAL_Delay(2000);

  //---DMX receiving---
  //ALERT! musí být offset u dmxRX - a! pravděpodobně mi nultý paket  blbě vysílá vysílač nebo blbě přijímá přijímač - na bakalářce jsem to neměl jak zkusit


  uint8_t testPacket[6];
  testPacket[0]=255;
  testPacket[1]=0;
  testPacket[2]=0;
  testPacket[3]=0;
  testPacket[4]=0;
  testPacket[5]=0;

  //HAL_Delay(2000);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, 514);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, 514);
  //static uint8_t test[515];
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, test, 512);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  //dmxPacketRdy=true;
	  //dmxPacket[1]=255;
	  //dmxPacket[2]=255;

	  //lcd_gotoxy(&lcd1, 0, 1);
	  //lcd_puts(&lcd1, "   ");
	  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, DMXPACKET_SIZE);
	 /* static uint32_t lastTick=0;

	  //---
	    char buffer[16];  // buffer pro převod čísla na text

	    sprintf(buffer, "%d", dmxRX[25+1]);  // převede int -> text //offset
	  lcd_gotoxy(&lcd1, 0, 1);
	  lcd_puts(&lcd1, buffer);
	  //dmxRX[1]=5;
	  //---

	  //Send RF with data --- data are sent with the speed of DMX bus - max. period approx. 22ms?
*/
	  //ToDo: Do solo souboru
	  if(readyToTransmit==true&&dmxPacketRdy==true)
	  {
		  readyToTransmit=false;
		  memcpy(&testPacket[0], &dmxRX[3], 3);
		  SI44_SendPacket(testPacket, sizeof(testPacket));
		  dmxPacketRdy=false;
	  }/*
	  if(dmxPacketRdy==true&&readyToTransmit==true) //if DMX packet ready and RF not currently transmitting
	  {
		  //if(dmxPacket[0]==DMX_STARTBYTE) //don't listen to messages that are not meant for you...
		  //{
			  for(uint16_t i=currentDMXAddress; i<currentDMXAddress+(NUM_OF_CHANNELS*NUM_OF_RECEIVERS); i++)
			  {
				  if(dmxPacket[i]!=dmxPrevPacket[i])//if the packet has changed
				  {
					  memcpy(&toSendPacket[0], &dmxPacket[currentDMXAddress], NUM_OF_CHANNELS*NUM_OF_RECEIVERS);
					  SI44_SendPacket(toSendPacket, sizeof(toSendPacket));
					  break;
				  }
			  }

		  //}
		  readyToTransmit=false;
		  dmxPacketRdy=false;
	  }/*
	  for(uint8_t i=0; i<255;i++)
	  {
		  //testPacket[0]=testPacket[0]+30;
		  SI44_SendPacket(testPacket, sizeof(testPacket));
		  HAL_Delay(1000);

	  }*/


	  //if dmxPacketRdy -> rfPacketRdy
	  //if rfPacketRdy -> odesli
	  //if
//--------------
	  //Send RF packet to keep receivers awake
	  //ToDo: set address function
	  /*if(HAL_GetTick()-lastTick>AWAKE_PACKET_PERIOD)
	  {
		  for(uint8_t i=0; i<activeGroups; i++) //awake packet doesnt isnt multi dimensional array
		  {
			  awakePacket[1]=i+1;
			  nrf24_transmit(awakePacket, AWAKE_PACKET_LENGTH);
		  }
		  lastTick=HAL_GetTick();
	  }*/
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_GPIO_NSS_GPIO_Port, SPI2_GPIO_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI2_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_GPIO_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_GPIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI2_GPIO_NSS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
