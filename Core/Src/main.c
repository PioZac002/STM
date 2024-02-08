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
#include <stdarg.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_TXBUF_LEN 1512 //Długosc bufora kolowego do nadawania
#define USART_RXBUF_LEN 512  //Dlugosc bufora odbiorczego
UART_HandleTypeDef huart2;
uint8_t timer = 0;


uint8_t USART_TxBuf [USART_TXBUF_LEN];
uint8_t USART_RxBuf [USART_RXBUF_LEN];

//__IO przykrywa volatile, mówi kompilatorowi, aby nie optymalizował tej zmiennej. Za karzdym razem pobiera wartosc zmiennej z pamieci

//Wskazniki bufora nadawczego
__IO uint16_t USART_TX_Empty = 0;
__IO uint16_t USART_TX_Busy = 0;

//Wskazniki bufora odbiorczego
__IO uint16_t USART_RX_Empty = 0;
__IO uint16_t USART_RX_Busy = 0;
uint8_t debug = 0;

struct frame
{
	uint8_t frameDetected;
	uint16_t frameSize;
	char sender[4];
	char receiver[4];
	uint8_t dataSize;
	char *data;
	uint8_t checksumCalculated;
	uint8_t checksumFromFrame;
};

struct measurement
{
	uint8_t current;
	uint16_t interval;
	int16_t data[512];
};

struct frame frame;
struct measurement measurement;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void my_tout_1ms()
{
	static uint16_t ms = 0;
	ms++;

	if(ms >= measurement.interval)
	{
		ms = 0;
		timer = 1;
	}
}

//Zwraca 0, jesli bufor jest pusty. Natomiast 1, jezeli w buforze cos sie znajduje
uint8_t USART_khbit()
{
	if (USART_RX_Empty == USART_RX_Busy)
		return 0;
	else
		return 1;
}

//Pobieranie znaku z bufora
int16_t USART_getchar()
{
	uint8_t temp;
	if (USART_RX_Empty != USART_RX_Busy){
		temp = USART_RxBuf[USART_RX_Busy];
		USART_RX_Busy++;

		if (USART_RX_Busy >= USART_RXBUF_LEN)
			USART_RX_Busy = 0;

		return temp;
	}
	else return -1;
	// Zwraca -1, jezeli nic nie znajduje sie w buforze
}

//Pobieranie calej linii z bufora do podanego wskaznika
uint8_t USART_getline(char *buf)
{
	static uint8_t bf[300];
	static uint8_t idx = 0;
	int i;
	uint8_t ret;

	while(USART_khbit())
	{
		bf[idx] = USART_getchar();

		if(bf[idx] == 10 || bf[idx] == 13) //Sprawdzanie, czy znak nie jest znakiem konca linii, badz powrotu karetki
		{
			bf[idx] = 0; //Podmiana znaku na zero, poniewaz w C 0 to koniec stringa

			for(i = 0; i <= idx; i++)
				buf[i] = bf[i];

			ret = idx;
			idx = 0;
			return ret;
		}
		else
		{
			idx++;
			if (idx>= 300)
				idx = 0;
		}
	}
	return 0;
}

//Dopisywanie do bufora nadawczego
void USART_fsend(char* format,...)
{
	char tmp_rx[128];
	int i;
	__IO int idx;

	va_list arglist;
	va_start(arglist, format);
	vsprintf(tmp_rx, format, arglist);
	va_end(arglist);

	idx=USART_TX_Empty;

	for (i = 0 ; i < strlen(tmp_rx);i++)
	{
		USART_TxBuf[idx]=tmp_rx[i];
		idx++;
		if (idx>=USART_TXBUF_LEN){
			idx =0;
		}
	}

	__disable_irq(); //Blokuje przerwania w sposób softwerowy
	if ((USART_TX_Empty == USART_TX_Busy) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET))
	{
		USART_TX_Empty=idx;
		uint8_t tmp = USART_TxBuf[USART_TX_Busy];
		USART_TX_Busy++;
		if (USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}
	else
		USART_TX_Empty = idx;

	__enable_irq();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		if (USART_TX_Empty != USART_TX_Busy)
		{
			uint8_t temp = USART_TxBuf[USART_TX_Busy];
			USART_TX_Busy++;
			if (USART_TX_Busy >= USART_TXBUF_LEN) USART_TX_Busy =0;
			HAL_UART_Transmit_IT(&huart2, &temp, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2)
	{
		USART_RX_Empty++;
			if (USART_RX_Empty >= USART_RXBUF_LEN) USART_RX_Empty = 0;
			HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1);
	}
}

uint8_t calculateChecksum(char* command, uint8_t size)
{
	uint16_t checksum = 0;
	int i;

	for(i = 0; i < size; i++)
	{
		checksum += (int)command[i];
	}
	checksum %= 100;

	return checksum;
}

int16_t getArchiveValue(uint16_t i)
{
	int pointer = measurement.current - i;

	if(pointer < 0)
	{
		pointer = 511 + pointer;
	}

	return measurement.data[pointer];
}

void sendFrame(char* data)
{
	int dataLength = strlen(data);

	USART_fsend(":STMPC0%03d%s%02d;", dataLength, data, calculateChecksum(data, dataLength));
}

void executeCommand()
{
	// Odczytywanie komendy
	int i;
	int j;
	char* tmp = malloc(frame.dataSize * sizeof(char) + 1);
	uint8_t detectedParameter = 0;

	for(i = 0; i < frame.dataSize; i++)
	{
		if(frame.data[i] == ' ')
		{
			detectedParameter = 1;
			break;
		}
		tmp[i] = frame.data[i];
	}
	tmp[i] = 0;

	// Odczyt interwału pomiarowego
	if(strcmp(tmp, "GT") == 0)
	{
		sprintf(tmp, "STV %d", measurement.interval);
		sprintf(tmp, "Aktualny interwal wynosi: %d", measurement.interval);
		sendFrame(tmp);
		return;
	}
	// Odczyt aktualnej wartości
	else if(strcmp(tmp, "GM") == 0)
	{

		sprintf(tmp, " Odczytywanie aktualnej wartosci: SM %d", measurement.data[measurement.current]);
		sendFrame(tmp);
		return;
	}
	// Ustawienie interwału pomiarowego
	else if(strcmp(tmp, "ST") == 0)
	{
		if(detectedParameter == 0)
		{
			// Brak argumentu
			sprintf(tmp, "ERRARGV");
			sendFrame(tmp);
			return;
		}
		else
		{
			for(i = 3; i < frame.dataSize; i++)
			{
				tmp[i - 3] = frame.data[i];
			}
			tmp[i - 3] = 0;

			int interval = atoi(tmp);

			if(interval >= 100 && interval <= 10000)
			{
				// Informacja o aktualizacji intwrwału pomiarowego
				measurement.interval = interval;
				sprintf(tmp, "TU %d", measurement.interval);
				sendFrame(tmp);
				return;
			}
			else
			{
				// Nieprawidłowy zakres argumentu
				sprintf(tmp, "ERRARGVF");
				sendFrame(tmp);
				return;
			}
		}
	}
	else if(strcmp(tmp, "GAM") == 0)
		{
			if(detectedParameter == 0)
			{
				// Brak argumentu
				sprintf(tmp, "ERRARGV");
				sendFrame(tmp);
				return;
			}
			else
			{
				for(i = 3; i < frame.dataSize; i++)
				{
					tmp[i - 3] = frame.data[i];
				}

				int archiveValue = atoi(tmp);

				if(archiveValue >= 1 && archiveValue <= 511)
				{



					// Informacja o aktualizacji intwrwału pomiarowego
					sprintf(tmp, "SAM %d", getArchiveValue(archiveValue));
					sendFrame(tmp);
					return;
				}
				else
				{
					// Nieprawidłowy zakres argumentu
					sprintf(tmp, "ERRARGVF");
					sendFrame(tmp);
					return;
				}
			}
		}
	// Nie rozpoznana komenda
	else
	{
		sprintf(tmp, "ERRCMD");
		sendFrame(tmp);
		return;
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
  /* USER CODE BEGIN 2 */
USART_fsend("STM start\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // TO DO
//  MX_I2C1_Init(); // Make sure this is defined and initializes I2C1 properly
//  LPS25HB_INIT(&hi2c1);

  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[0], 1);
   int i;
   char dataSize[4];
   char checkSum[3];
   char recievedFrame[269];
   char recievedChar;
   uint8_t charCoddingDetected;
   srand(time(NULL));

   frame.frameDetected = 0;
   measurement.current = 0;
   measurement.interval = 1000;

  while (1)
  {
    /* USER CODE END WHILE */
  	if(timer)
  	{
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  measurement.current++;
  measurement.data[measurement.current] = rand() % 100;

  if(measurement.current > 511)
  	measurement.current = 0;

  timer = 0;
  	}

  	if(USART_khbit())
  	{
  recievedChar = USART_getchar();

  if(recievedChar == ':')
  {
  	frame.frameDetected = 1;
  	frame.frameSize = 0;
  	charCoddingDetected = 0;
  	continue;
  }

  if(frame.frameDetected && frame.frameSize < 269)
  {
  	// Dekodowanie znaków

  	if(recievedChar == '\\' && charCoddingDetected == 0)
  	{
  		charCoddingDetected = 1;
  	}
  	else if(charCoddingDetected)
  	{
  		if(recievedChar == 'A')
  		{
  			recievedFrame[frame.frameSize] = ':';
  		}
  		else if(recievedChar == 'B')
  		{
  			recievedFrame[frame.frameSize] = ';';
  		}
  		else if(recievedChar == 'C')
  		{
  			recievedFrame[frame.frameSize] = '\\';
  		}
  		else
  		{
  			// Nierozpoznane kodowanie
  			frame.frameDetected = 0;
  			USART_fsend(":STMPC0002ED%02d;", calculateChecksum("ED", 2));
  		}

  		frame.frameSize++;
  		charCoddingDetected = 0;
  	}
  	else if(recievedChar == ';')
  	{

  		if(frame.frameSize < 13)
  		{
  			frame.frameDetected = 0;
  			continue;
  		}

  		recievedFrame[frame.frameSize] = 0;

  		for(i = 0; i < 3; i++)
  		{
  			frame.sender[i] = recievedFrame[i];
  			frame.receiver[i] = recievedFrame[i + 3];
  			dataSize[i] = recievedFrame[i + 6];
  		}

  		frame.sender[3] = 0;
  		frame.receiver[3] = 0;
  		dataSize[3] = 0;
  		frame.dataSize = atoi(dataSize);
  		frame.data = malloc(frame.dataSize * sizeof(char) + 1);

  		// Przepisanie danych
  		for(i = 0; i < frame.dataSize; i++)
  		{
  			frame.data[i] = recievedFrame[i + 9];
  		}

  		frame.data[i] = 0;

  		// Odczy
  		for(i = 0; i < 2; i++)
  		{
  			checkSum[i] = recievedFrame[i + 9 + frame.dataSize];
  		}
  		checkSum[3] = 0;
  		frame.checksumFromFrame = atoi(checkSum);
  		frame.checksumCalculated = calculateChecksum(frame.data, frame.dataSize);

  		if(debug)
  		{
  			USART_fsend("\r\n\nDANE Z RAMKI:\r\n", recievedFrame);
  			USART_fsend("%s\r\n", recievedFrame);
  			USART_fsend("Sender: %s\r\n", frame.sender);
  			USART_fsend("Reciever: %s\r\n", frame.receiver);
  			USART_fsend("Data size: %03d\r\n", frame.dataSize);
  			USART_fsend("Data: %s\r\n", frame.data);
  			USART_fsend("Check sum from frame: %02d\r\n", frame.checksumFromFrame);
  			USART_fsend("Calculated checksum: %02d\r\n", frame.checksumCalculated);
  		}


  		// Ignorowanie ramki nie wysłanej przez PC0
  		if(strcmp(frame.sender, "PC0") != 0)
  			continue;

  		// Ignorowanie ramki nie przeznaczonej dla STM
  		if(strcmp(frame.receiver, "STM") != 0)
  			continue;

  		// Niewłaściwa suma kontrolna, wysłanie ramki z odpowiednim błędem
  		if(frame.checksumCalculated != frame.checksumFromFrame)
  		{
  			 USART_fsend("Otrzymana suma kontrolna: %02d, Obliczona suma kontrolna: %02d\r\n", frame.checksumFromFrame, frame.checksumCalculated);
  			USART_fsend("Dane do obliczenia sumy kontrolnej: '%s'\r\n", frame.data);
  			frame.checksumCalculated = calculateChecksum(frame.data, frame.dataSize);
  			 USART_fsend(":STMPC0005ERRCS%02d;", calculateChecksum("ERRCS", 5));
  			continue;
  		}

  		executeCommand();
  	}
  	else
  	{


  		recievedFrame[frame.frameSize] = recievedChar;
  		frame.frameSize++;

  		if(frame.frameSize >= 269)
  		{
  			frame.frameDetected = 0;
  		}

  	}
  }
  else if(frame.frameDetected && frame.frameSize >= 269)
  {
  	frame.frameDetected = 0;
  }
  	}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
