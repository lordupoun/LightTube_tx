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
#include "gui.h"
#include "flash_storage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//DATA PACKETS MARKS:
#define BROADCAST_PACKET_MARK 0xA1
#define INDIVIDUAL_PACKET_MARK 0xA2
#define DATA_MAX_PACKET_SIZE 33

#define AWAKE_PACKET_MARK 0xBB
#define AWAKE_PACKET_SIZE 2
//AWAKE_PACKET_PERIOD is defined by TIM3; 750ms by default

//DMX defines
#define DMX_PACKET_SIZE 513 //do not change
#define DMX_STARTBYTE 0 //defines what startbyte the receiver listens to; 0 default for light control by standard

#define BT_RECEIVE_BYTES 31 //NUM of bytes to receive from BT module;

//MOVED TO main.h
//#define NUM_COMMON_CHANNELS 1
//#define NUM_CHANNELS 5



I2C_LCD_HandleTypeDef lcd1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//ToDo: Rozmístit do samostatných souborů DMX a RF
//ToDo: do mainu jako static

//VARIABLES
static uint8_t dataPacketSize = 8;		 //Current size of RF data packet
static uint8_t numOfReceivers = 1;		 //Number of receivers = number of uniquely addressable tubes (if 1, all share the same data)
static uint16_t currentDMXAddress=20; 	 	 //Current address of the DMX receiver
static uint8_t awakePacketID = 0;		 //ID of awake packet = the same as an ID of the last data packet
static uint8_t packetType=BROADCAST_PACKET_MARK; //Currently transmitted packetType
void (*active_mode_func)(void) = NULL;  //pointer to currently active mode startup function

static uint8_t rxBuff[DMX_PACKET_SIZE+1]; 	    //receive buffer for both DMX and bluetooth
static uint8_t dmxPacket[DMX_PACKET_SIZE]; 		//One received DMX packet
static uint8_t dmxPrevPacket[DMX_PACKET_SIZE]; 	//One previously received DMX packet
static uint8_t txBuff[DATA_MAX_PACKET_SIZE];		//Buffer for RF TX
static Mode_t currentMode = DMX512;
static Transmit_t currentTransmit = BROADCAST;
static GPIO_TypeDef* clickedButtonGPIO;
static uint16_t clickedButtonPin;
static uint8_t holdCount=0;

//the rest of variables is in main

//FLAGS:
static volatile bool dmxPacketRdy=false;   //DMX packet prepared for parsing
static volatile bool btPacketRdy=false;    //BT packet prepared for parsing
static volatile bool readyToTransmit=true; //Si4432 has finished previous transmission
static volatile bool dmxCopied=false;	   //DMX / BT packet is parsed
static volatile bool awakePacketRdy=0;	   //Time to send awake packet if 1
static volatile bool signalLostDMX=0;	   //Signal lost  if 1
static volatile bool debounceTimerFinished=false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//Timer for sending awakePackets (to keep devices informed about the presence of RF signal)
    if (htim->Instance == TIM2)
    {
    	awakePacketRdy=1;
    }
    //Checks if DMX signal is present
    if (htim->Instance == TIM3)
    {
    	signalLostDMX=1;
    }
    if (htim->Instance == TIM4)
    {
    	debounceTimerFinished=true;
    }
}


//---DMX receiving Callback---
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if(huart->Instance==USART1)
	{
		dmxPacketRdy=true;
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, rxBuff, DMX_PACKET_SIZE); //Should be outside of NVIC callback
	}
	else if (huart->Instance==USART2)
	{
		btPacketRdy=true;
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, rxBuff, BT_RECEIVE_BYTES);
	}
}
//---DMX receiving failed Callback---
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	dmxPacketRdy=false;
    	HAL_UART_AbortReceive(&huart1);
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, rxBuff, DMX_PACKET_SIZE);
    }
    else if (huart->Instance == USART2)
    {
    	btPacketRdy=false;
    	HAL_UART_AbortReceive(&huart2);
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, rxBuff, BT_RECEIVE_BYTES);
    }
}
//---RF EXTI Callback---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//ToDo: edit and add commented code to enable receiving RF
	//EXTI for RF module (SI4432)
	//if (GPIO_Pin == GPIO_PIN_8)
	//{
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
	//}
}
void broadcast_mode()
{
	//Add STOP TIM?
	currentTransmit=BROADCAST;
	HAL_UART_AbortReceive(&huart1);
	HAL_UART_AbortReceive(&huart2);
	packetType=BROADCAST_PACKET_MARK;
	numOfReceivers=1;
	dataPacketSize=8;  //Should reapply values in app
	active_mode_func(); //Looks like it won't be needed
}

void individual_mode()
{
	currentTransmit=INDIVIDUAL;
	HAL_UART_AbortReceive(&huart1);
	HAL_UART_AbortReceive(&huart2);
	packetType=INDIVIDUAL_PACKET_MARK;
	numOfReceivers=6;
	dataPacketSize=33;
	//AUTOMATICALLY MOVES ADDRESS IF OUT OF BOUNDS FOR INDIVIDUAL MODE
	uint16_t bugfix = 512-(NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS);
	if(currentDMXAddress>bugfix)
	{
		currentDMXAddress=bugfix;
	}
	active_mode_func();
}

void dmx_activate()
{
	currentMode=DMX512;
	active_mode_func = dmx_activate;
	HAL_UART_AbortReceive(&huart2);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, rxBuff, DMX_PACKET_SIZE);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, rxBuff, DMX_PACKET_SIZE);
	HAL_TIM_Base_Start_IT(&htim3);
	//gui_setScreen();
	//gui_drawBase();
	//lcd_gotoxy(&lcd1, 0, 1);
	//lcd_puts(&lcd1, "MODE: DMX512");
}
void bluetooth_activate()
{
	currentMode=BLUETOOTH;
	active_mode_func = bluetooth_activate;
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_UART_AbortReceive(&huart1);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, rxBuff, BT_RECEIVE_BYTES);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, rxBuff, BT_RECEIVE_BYTES);
	//gui_setScreen();
	//gui_drawBase();
	//lcd_gotoxy(&lcd1, 0, 1);
	//lcd_puts(&lcd1, "MODE: BLUETOOTH");
}


void bluetooth_at_commands()
{
	/*HAL_UART_Abort(&huart2);
	HAL_UART_DeInit(&huart2); //REMOVE COMMENTS IF CHANGING BAUD RATE (default AT=38400kbps)
	huart2.Init.BaudRate = 9600;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
	    //inicializace neuspesna -> LCD
	}
	else
	{
		//inicializace uspesna
	}*/
	HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	//HAL_UART_Transmit(&huart2, "AT\r\n", strlen("AT\r\n"), 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2, "AT+NAME=LightTube\r\n", strlen("AT+NAME=LightTube\r\n"), 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2, "AT+CLASS=0x0\r\n", strlen("AT+CLASS=0x0\r\n"), 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2, "AT+UART=38400,0,0\r\n", strlen("AT+UART=38400,0,0\r\n"), 100);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2, "AT+PSWD=8182\r\n", strlen("AT+PSWD=8182\r\n"), 100);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_RESET);
	lcd_puts(&lcd1, "DONE");
	/*HAL_Delay(1000);
	HAL_UART_DeInit(&huart2);
	huart2.Init.BaudRate = 38400;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
	    //inicializace neuspesna -> LCD
	}
	else
	{
		//inicializace uspesna
	}*/

}
void load_settings_from_EEPROM()
{
	uint16_t address;
	Mode_t mode;
	Transmit_t transmit;
	flash_loadSettings(&address, &mode, &transmit);

	currentDMXAddress=address;
	if(mode==DMX512)
		dmx_activate();
	else
		bluetooth_activate();
	if(transmit==BROADCAST)
		broadcast_mode();
	else
		individual_mode();
}

void identify_tube_address() //ToDo: implement as a custom packet
{
	/*for(uint8_t i=4; i<dataPacketSize; i=i+4)
	{
		txBuff[i]=250;
	}*/
	txBuff[6]=250;
	while(readyToTransmit!=true)
	{

	}
	awakePacketID+=1;
	txBuff[0]=BROADCAST_PACKET_MARK;
	txBuff[1]=awakePacketID;
	uint8_t b; //jen pro reset 0x03 registru
	SI44_Read(0x04, &b, 1);
	SI44_Read(0x03, &b, 1);
	SI44_SendPacket(txBuff, dataPacketSize);
	readyToTransmit=false;
}

void reset_timer(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //---LCD---
  HAL_Delay(100);
  lcd1.hi2c = &hi2c1;
  lcd1.address = (0x27 << 1);
  lcd_init(&lcd1);
  HAL_Delay(100);
  lcd_gotoxy(&lcd1,0,0);
  lcd_puts(&lcd1, "LightTube");
  lcd_gotoxy(&lcd1, 3, 1);
  lcd_puts(&lcd1, "-LOADING-");



  //---Initialize:---
  //---SI4432---
  HAL_GPIO_WritePin(SPI2_GPIO_ShutDN_GPIO_Port, SPI2_GPIO_ShutDN_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  SI44_Init(&hspi2, SPI2_GPIO_NSS_GPIO_Port, SPI2_GPIO_NSS_Pin); //beware - changes have been made!
  HAL_Delay(10); //ToDo: zkratit
  SI44_PresetConfig();
  HAL_Delay(50);
  //SI44_SetAGCMode(0b00100000); //6th bit - sgin - for RX side
  SI44_SetInterrupts1(0b00000100); //1st bit = CRC error
  HAL_Delay(5);
  SI44_SetInterrupts2(0b00000000);
  HAL_Delay(5);
  SI44_SetModuleAntenna();
  HAL_Delay(5);
  SI44_SetTXPower(SI44_TX_POWER_20dBm);
  HAL_Delay(100);



  //---DMX receiving---
  //ALERT! musí být offset u rxBuff - a! pravděpodobně mi nultý paket  blbě vysílá vysílač nebo blbě přijímá přijímač - na bakalářce jsem to neměl jak zkusit

  //---Variables---
  //
  load_settings_from_EEPROM();
  gui_init(&currentMode, &currentTransmit, &currentDMXAddress, &lcd1, &numOfReceivers);
  gui_setScreen();
  HAL_TIM_Base_Start_IT(&htim2);
  //individual_mode();

  //dmx_activate();
  //broadcast_mode();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //ToDo: Do solo souboru
	  //ToDo: Prohazovat kam se ukládá

	  //---DMX parsing---
	  if(dmxPacketRdy==true)
	  {
		  dmxPacketRdy=false;
		  reset_timer(&htim3);
		  memcpy(dmxPrevPacket, dmxPacket, DMX_PACKET_SIZE);
		  //__disable_irq(); //The packet shouldn't be rewritten by DMX_IT while copying
		  memcpy(&dmxPacket[0], &rxBuff[0], DMX_PACKET_SIZE);
		  //__enable_irq();
		  for(uint16_t i=currentDMXAddress; i<currentDMXAddress+(NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS); i++)
		  //for(uint16_t i=0; i<6; i++)
		  {
			  if(dmxPacket[i]!=dmxPrevPacket[i])//if the packet has changed
			  {
				  dmxCopied=true;
				  break;
			  }
		  }
		  //char uartBuf[50];
		  //int len = sprintf(uartBuf, "DMX:%d %d %d\r\n", dmxPacket[1], dmxPacket[2], dmxPacket[3]);
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)uartBuf, len);
	  }

	  //---BT parsing---
	  if(btPacketRdy==true)
	  {
		  btPacketRdy=false;
		  memcpy(&dmxPrevPacket[1], &dmxPacket[1], NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS);
		  memcpy(&dmxPacket[1], &rxBuff[0], NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS);
		  for(uint16_t i=1; i<=NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS; i++)
		  //for(uint16_t i=0; i<6; i++)
		  {
			  if(dmxPacket[i]!=dmxPrevPacket[i])//if the packet has changed
			  {
				  dmxCopied=true;
				  break;
			  }
		  }
		  //char uartBuf[50];
		  //int len = sprintf(uartBuf, "DMX:%d %d %d\r\n", dmxPacket[1], dmxPacket[2], dmxPacket[3]);
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)uartBuf, len);
	  }

	  //---DATA PACKET SENDING---
	  if(readyToTransmit==true&&dmxCopied==true)
	  {
		  uint8_t b; //jen pro reset 0x03 registru
		  SI44_Read(0x04, &b, 1);
		  SI44_Read(0x03, &b, 1); //jinak by uz neaktivoval IRQ


		  awakePacketID+=1;
		  txBuff[0]=packetType;
		  txBuff[1]=awakePacketID; //Todo: ZPOŽDĚNÍ NA STRANĚ VYSÍLAČE?

		  dmxCopied=false;

		  if (currentMode == DMX512)
		  {
			  memcpy(&txBuff[2], &dmxPacket[currentDMXAddress], NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS);
		  }
		  else if (currentMode == BLUETOOTH)
		  {
			  memcpy(&txBuff[2], &dmxPacket[1], NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS);
		  }
		  SI44_SendPacket(txBuff, dataPacketSize); //ToDo: dataPacketSize - vyjadrit v definech a numOfReceivers

		  readyToTransmit=false;
		  //char uartBuf[50];
		  //int len = sprintf(uartBuf, "%d %d %d\r\n", txBuff[1], txBuff[2], txBuff[3]);
		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)uartBuf, len);

		  //HAL_UART_Transmit_IT(&huart3, (uint8_t*)uartBuf, len);

	  }
	  //---AWAKE PACKET SEND---
	  if(readyToTransmit==true&&awakePacketRdy==1)
	  {
		  awakePacketRdy=0;
		  uint8_t b; //jen pro reset 0x03 registru
		  SI44_Read(0x04, &b, 1);
		  SI44_Read(0x03, &b, 1); //jinak by uz neaktivoval IRQ

		  txBuff[0]=AWAKE_PACKET_MARK;
		  txBuff[1]=awakePacketID;

		  SI44_SendPacket(txBuff, AWAKE_PACKET_SIZE);
		  readyToTransmit=false;
	  }
	  //---SIGNAL LOST PACKET SEND---
	  if(readyToTransmit==true&&signalLostDMX==1)
	  {
		  signalLostDMX=0;
		  awakePacketID+=1;
		  txBuff[0]=packetType;
		  txBuff[1]=awakePacketID;
		  for(uint8_t i=2; i<NUM_CHANNELS*numOfReceivers+NUM_COMMON_CHANNELS+2; i++)
		  {
			  txBuff[i]=0;
		  }
		  SI44_SendPacket(txBuff, dataPacketSize);
	  }

	 //---BUTTONS:----------------------------------------------------------
	  if(holdCount==0)
	  {
		  if(HAL_GPIO_ReadPin(BTN1_UP_GPIO_Port, BTN1_UP_Pin) == GPIO_PIN_RESET)
		  {
			  //dmx_activate();
			  //gui_buttonUp();
			  holdCount++;
			  clickedButtonGPIO=BTN1_UP_GPIO_Port;
			  clickedButtonPin=BTN1_UP_Pin;
			  reset_timer(&htim4);
			  HAL_TIM_Base_Start_IT(&htim4);
		  }
		  if(HAL_GPIO_ReadPin(BTN2_LEFT_GPIO_Port, BTN2_LEFT_Pin) == GPIO_PIN_RESET)
		  {
			  //broadcast_mode();
			  //gui_buttonLeft();
			  holdCount++;
			  clickedButtonGPIO=BTN2_LEFT_GPIO_Port;
			  clickedButtonPin=BTN2_LEFT_Pin;
			  reset_timer(&htim4);
			  HAL_TIM_Base_Start_IT(&htim4);
		  }
		  if(HAL_GPIO_ReadPin(BTN3_RIGHT_GPIO_Port, BTN3_RIGHT_Pin) == GPIO_PIN_RESET)
		  {
			  //individual_mode();
			  //gui_buttonRight();
			  holdCount++;
			  clickedButtonGPIO=BTN3_RIGHT_GPIO_Port;
			  clickedButtonPin=BTN3_RIGHT_Pin;
			  reset_timer(&htim4);
			  HAL_TIM_Base_Start_IT(&htim4);
		  }
		  if(HAL_GPIO_ReadPin(BTN4_DOWN_GPIO_Port, BTN4_DOWN_Pin) == GPIO_PIN_RESET)
		  {
			  //bluetooth_activate();
			  //gui_buttonDown();
			  holdCount++;
			  clickedButtonGPIO=BTN4_DOWN_GPIO_Port;
			  clickedButtonPin=BTN4_DOWN_Pin;
			  reset_timer(&htim4);
			  HAL_TIM_Base_Start_IT(&htim4);
		  }

	  }
	  //DEBOUNCE:
	  if(debounceTimerFinished==true)
	  {
		  HAL_TIM_Base_Stop_IT(&htim4);
		  debounceTimerFinished=false;
		  if(HAL_GPIO_ReadPin(clickedButtonGPIO, clickedButtonPin) == GPIO_PIN_SET)
		  {
			  //HAL_TIM_Base_Stop_IT(&htim4);
			  switch(clickedButtonPin)
			  {
			  case BTN1_UP_Pin:
				  gui_buttonUp();
				  break;
			  case BTN2_LEFT_Pin:
				  gui_buttonLeft();
			  	  break;
			  case BTN3_RIGHT_Pin:
				  gui_buttonRight();
				  break;
			  case BTN4_DOWN_Pin:
				  gui_buttonDown();
				  break;
			  }
			  holdCount=0;
		  }
		  else
		  {
			  if(holdCount>15)
			  {
				  switch(clickedButtonPin)
				  {
				  case BTN1_UP_Pin:
					  gui_buttonUp();
					  break;
				  case BTN4_DOWN_Pin:
					  gui_buttonDown();
					  break;
				  }
			  }
			  else
			  {
				  holdCount++;
			  }
			  HAL_TIM_Base_Start_IT(&htim4);
		  }
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
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 29999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BT_EN_Pin|SPI2_GPIO_ShutDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_GPIO_NSS_GPIO_Port, SPI2_GPIO_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_EN_Pin SPI2_GPIO_ShutDN_Pin */
  GPIO_InitStruct.Pin = BT_EN_Pin|SPI2_GPIO_ShutDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_STATE_Pin BTN1_UP_Pin BTN2_LEFT_Pin */
  GPIO_InitStruct.Pin = BT_STATE_Pin|BTN1_UP_Pin|BTN2_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_GPIO_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_GPIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SPI2_GPIO_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI2_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN3_RIGHT_Pin BTN4_DOWN_Pin */
  GPIO_InitStruct.Pin = BTN3_RIGHT_Pin|BTN4_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
