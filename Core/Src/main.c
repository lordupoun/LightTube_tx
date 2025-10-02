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
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
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

#define DATA_PACKET_LENGTH 12 //payload size; 32 = max for nrf; //ToDo: HEADER_SIZE+NUM_OF_CHANNELS
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


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//ToDo: Rozmístit do samostatných souborů DMX a RF

static uint8_t dmxRX[513]; 			 	 //DMX receive buffer
static uint8_t dmxRX_prev[513]; 	 //DMX data received in previous packet
static uint8_t dmxPacketRdy=0; 	     //Flag - dmxPacketReady
static uint16_t currentAddress=1; 	 //Current address of the DMX receiver


static uint8_t dataPacket[MAX_RX_NUM][DATA_PACKET_LENGTH]; //RF transmit buffer; ToDo: Rename to dataPacket
static uint8_t awakePacket[AWAKE_PACKET_LENGTH]; //RF transmit buffer - awake packet; only one -> changes it's address in a for loop
static uint8_t activeGroups;   //Number of receivers activated by user; also changes the number of channels receiver listens to
static uint8_t *receiver[MAX_RX_NUM]; 	 //pointer to dataPacket; receiver[0] -> dataPacket assigned to receiver number one


//static uint8_t ack[DATA_PACKET_LENGTH];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void assignBuffToGroups(void)
{
	/*//null = not used
	//ToDo: receiver by mělo být pole ukazatelů; ale adresy musí jít VŽDY postupně dle activeGroups, nešlo by to obejít ukazatelem?
	//ToDo: Replace with GUI multiplexerjí
	//Pokud bych assignoval takto, a měl stovku zařízení, pro každý by se vysílal samostatný paket, a to je nesmysl
	activeGroups =0;
	uint8_t rxCount[MAX_RX_NUM];
	receiver[0]=dataPacket[1]; //ToDo: Nahradit grafikou
	receiver[1]=dataPacket[1]; //ToDo: Nahradit grafikou
	receiver[2]=dataPacket[1]; //ToDo: Nahradit grafikou
	receiver[3]=dataPacket[2]; //ToDo: Nahradit grafikou
	receiver[4]=NULL; //ToDo: Nahradit grafikou
	receiver[5]=dataPacket[2]; //ToDo: Nahradit grafikou
	for(uint8_t i=0; i<MAX_RX_NUM; i++)
	{
		//rxCount[receiver[i]]=rxCount[receiver[i]]+1; //opravit

	}
	for(uint8_t i=0; i<MAX_RX_NUM; i++)
	{
		if(rxCount[i]!=0)
		{
			activeGroups=activeGroups+1; //ToDo: dát do samostatné funkce?
		}
	}
	//Nastavit novou adresu!
	for(uint8_t i=0; i<MAX_RX_NUM; i++)
	{
		//For i<hw_id[i]
		//zmen adresu receiveru 0 na 1
		//zmen adresu receiveru 1 na 1
		//zmen adresu receiveru 2 na 1
		//zmen adresu receiveru 3 na 3
		//zmen adresu receiveru 4 na NULL
		//zmen adresu receiveru 5 na 3

		//spočítat activeGroups=2

		//dataPacket[0]=1
		//dataPacket[1]=3
	}*/
	receiverGroup[1]=1 //skupina receivru 1 je jedna
	receiverGroup[2]=1 //skupina receivru 2 je jedna
	receiverGroup[3]=2
	receiverGroup[4]=1
	receiverGroup[5]=3
	receiverGroup[6]=1
	activeGroupCount=3

	//POZOR - zcela opomene mapu, kterou uzivatel vytvoril -> vytvori si vlastni cislovani pro rychlejsi kod
	for activegroupcount
		dataPacket[i][1]=address[i]; //address[i je promenna s pouzivanou adresaci

	for nastav adresy zatizenim

	//Zde se vybere activeGroups (kolik přijímačů je doopravdy aktivních), a spustí se funkce, která jim s ručně dělaným ACKEM přidělí nové adresy (11-66)
	//Ale jak se pak vysílače, které budou mít všechny stejnou adresu, vrátí zpátky??
}
void addAdressToPacket(void) //adds address into the header
{
	for(uint8_t i=0; i<MAX_RX_NUM; i++)
	{
		dataPacket[i][0]=i+1;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, dmxRX, DMXPACKET_SIZE);
    dmxPacketRdy=1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  assignBuffToGroups();
  addAdressToPacket();
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //NRF24L01+ ------------------------------------------------------------------
  //warning - nrf24 tends to keep its registry even with no power
  //->settings must be applied each run
  nrf24_init();
  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_1mbps);
  nrf24_set_channel(78); //ToDo: Set channel variable - send message with ACK? (or self implemented ack)
  nrf24_set_crc(en_crc, _1byte); //Cyclic redundancy - receiver flushes broken packets
  nrf24_pipe_pld_size(0, DATA_PACKET_LENGTH); //pipe 0;
  nrf24_dpl(enable); //ToDo: Dynamic payload - think about it
  uint8_t addr[5]={0x10, 0x21, 0x32, 0x43, 0x54};
  nrf24_open_tx_pipe(addr); //
  //nrf24_open_rx_pipe(0, addr);
  nrf24_stop_listen();
  //----------------------------------------------------------------------------
  awakePacket[0]=COMMON_ADDRESS;
  awakePacket[2]=AWAKE_PACKET_MARK;

  HAL_UART_Receive_IT(&huart1, dmxRX, 513);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //ToDo: ASSIGN
  while (1)
  {
	  static uint32_t lastTick=0;

	  //Send RF with data --- data are sent with the speed of DMX bus - max. period approx. 22ms?

//--------------ToDo: Do samostatné funkce void process_dmx(uint8_t *dmx_data) {
	  if(dmxPacketRdy==1) //if packet ready
	  {
		  if(dmxRX[0]==DMX_STARTBYTE) //don't listen to messages that are not meant for you...
		  {
			  for(uint16_t i=currentAddress; i<NUM_OF_CHANNELS*activeGroups; i++)//list packets that are used for LightTubes
			  {
				  if(dmxRX[i]!=dmxRX_prev[i])//if the packet changed
				  {
					  for(uint8_t i=0; i<activeGroups; i++)
					  {
						  memcpy(&dataPacket[i][DATA_START_POS], &dmxRX[currentAddress+(NUM_OF_CHANNELS*i)], NUM_OF_CHANNELS); //copy them to according RX buffers
						  nrf24_transmit(dataPacket[i], DATA_PACKET_LENGTH); //Sends only as many RF messages, as there are active receivers; but only MAX_RX_NUM
						  //ToDo: Remove HAL_Delay from nrf library?
						  //ToDo: Maybe add delay? If it doens't work - move it to timeToTransmit, make a flag, and add a delay
					  }
					  break;
				  }
			  }

		  }
		  dmxPacketRdy=0;
	  }
//--------------
	  //Send RF packet to keep receivers awake
	  //ToDo: set address function
	  if(HAL_GetTick()-lastTick>AWAKE_PACKET_PERIOD)
	  {
		  for(uint8_t i=0; i<activeGroups; i++) //awake packet doesnt isnt multi dimensional array
		  {
			  awakePacket[1]=i+1;
			  nrf24_transmit(awakePacket, AWAKE_PACKET_LENGTH);
		  }
		  lastTick=HAL_GetTick();
	  }
/*#ifdef tx

#else
	  nrf24_listen();
	  if(nrf24_data_available()){
		  nrf24_receive(data_R, sizeof(data_R));
		  char tmp[40];
		  sprintf(tmp, "| %s |\r\n", data_R);
		  HAL_UART_Transmit(&huart1, tmp, strlen(tmp), 100);
		  for(uint8_t i=0; i<sizeof(data_R); i++)
		  	  {
		  		  data_R[i]='\0';
		  	  }
	  }
#endif*/
      //printf("Tick\n");
      //HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
