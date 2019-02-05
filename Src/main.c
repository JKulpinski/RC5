/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "przyciski.h"
//
#define buffer_size 255 //rozmiar bufora kolowego
#define adres_domowy 0x16
#define Start       0x80
#define end         0x40
#define czy_starszy           0B00010000
uint8_t i = 0;
uint8_t znak;
uint8_t bajt;

const int led_on_off = 0x11; // komenda led on/led off
const int miganie = 0x10; //miganie dioda
const int przycisk = 0x09; //ostatni wduszony przycisk

int flaga;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
uint32_t czas = 0;
uint8_t polbit = 0;
uint32_t zdekodowany_sygnal = 0;
uint8_t sukces = 0;
uint32_t cos = 0;
int miejsce = 0;
int8_t liczba = 0;

bool sygnal = false;
uint32_t timer = 0;
bool timer_go = false;
bool timer_ramka_go = false;
uint32_t timer_ramka = 0;

uint8_t Received;     //zmienna przechowujaca dane odebrane z usarta
uint8_t przerwanie = 0;  // flaga informujaca o uruchomieniu przerwania usarta
uint8_t buffer_rx[buffer_size]; //bufor konowy
uint8_t buffer_end = 0; //indeks konca zapisu
uint8_t buffer_start = 0; //indeks poczatku zapisu
uint8_t CMD_buffer_rx[buffer_size]; //bufor pomocniczy
uint8_t CMD_BUFFER_INDEX = 0; // indeks bufora
uint16_t zapalona; //jak dlugo dioda zapalona
uint16_t zgaszona; //jak dlugo dioda zgaszona

uint8_t buffer_tx[buffer_size]; //bufor kolowy
uint8_t buffer_xx[buffer_size];
uint8_t buffer_tx_end = 0; //indeks konca zapisu
uint8_t buffer_tx_start = 0; //indeks poczatku zapisu
uint8_t tx_transmit = 0;

uint8_t data[buffer_size];
uint8_t flag = 0;
uint8_t flaga_starszego = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		sygnal = true;
	}
}
int dekodowania = 0;

int minimum = 10000;
int maximum = 0;
void dekodowanie() {
	czas = timer;
	timer = 0;
//	if( czas > maximum)
//		maximum = czas;
//	if (czas < minimum)
//		minimum = czas;

	//odbioru pierwszego bitu
	if ((polbit == 0)
			&& (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET)) {
		//wyzerowanie oraz w³¹czenie licznika
		timer = 0;
		timer_go = true;
		//ustawienie wartoœci pocz¹tkowych
		polbit = 1;
		zdekodowany_sygnal = 0;
		sukces = 0;

	} else if (polbit > 0) {
		//aktualizacja stanu w którym znajduje sie dekodowanie rc5
		// ze sprawdzaniem czy czas trwania impulsu jest poprawny
		//news = licznik1;
		if ((czas >= 13) && (czas <= 22)) {
			polbit += 2;

		} else if ((czas >= 5) && (czas <= 11)) {
			polbit += 1;
		}

		//w przypadku b³êdnego czasu trwania stanu zerowanie
		else {
			polbit = 0;
		}
	}

	//b³¹d
	else {
		polbit = 0;
	}

	switch (polbit) {
	//rozpoznawanie stanów
	case 27:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x01;
		}
		//odebrano pomyœlnie
		sukces = 1;
		cos = zdekodowany_sygnal;
		info_przyciski(cos);
		polbit = 0;
		break;

	case 25:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0002;
		}
		break;
	case 23:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0004;
		}
		break;
	case 21:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0008;
		}
		break;
	case 19:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0010;
		}
		break;
	case 17:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0020;
		}
		break;
	case 15:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0040;
		}
		break;
	case 13:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0080;
		}
		break;
	case 11:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0100;
		}
		break;
	case 9:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0200;
		}
		break;
	case 7:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0400;
		}
		break;
	case 5:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x0800;
		}
		break;
	case 3:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x1000;
		}

		break;
	case 1:
		if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET) {
			zdekodowany_sygnal |= 0x2000;
		}

		break;
	default:
		break;
	}
}

//callback wywolywany po odborze
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		buffer_end++;
		if (buffer_end >= buffer_size) {
			buffer_end = 0;
		}
		HAL_UART_Receive_IT(&huart2, &buffer_rx[buffer_end], 1);
	}
}

#define POLYNOM 0x1021
uint16_t crc = 0xFFFF;
void error() {
	i = 0;
	crc = 0xFFFF;
	flag = 0;
	flaga_starszego = 1;
}

unsigned int crc16(unsigned int crcValue, unsigned char newByte) {
	unsigned char i;
	for (i = 0; i < 8; i++) {
		if (((crcValue & 0x8000) >> 8) ^ (newByte & 0x80)) {
			crcValue = (crcValue << 1) ^ POLYNOM;
		} else {
			crcValue = (crcValue << 1);
		}
		newByte <<= 1;
	}
	return crcValue;
}

void ramka() {
	if (buffer_start != buffer_end) {		//jezeli w buforze sa dane
		znak = buffer_rx[buffer_start];
		if (znak == Start) {//jezeli trafimy na poczatek ramki to usuwamy poprzednia ramke
			error();
			flag = 1; //flaga informuj¹ca ¿e odebrano znak rowny poczatkowi ramki
		} else if ((znak == end) && flag == 1) {
			wykonac();        //wykonywanie ramki
			error();

		} else {
			if ((znak & czy_starszy) && znak > 15 && znak <= 31 && flag == 1 // 15 < znak <= 31 poniewaz oczekujemy znaku od 0x10 do 0x1F
			&& flaga_starszego == 1) {
				bajt = (znak ^ czy_starszy) << 4;
				flaga_starszego = 0; // na zmiane musza przychodzic znaki zaczynajace sie od 0x1 i 0x0, kiedy flaga_starszego = 0
									 //to oczekujemy nastepnym razem na znak zaczynajacy sie od 0x0.
			} else if (!(znak & czy_starszy) && znak >= 0 && znak <= 15 // tutaj oczekujemy znaku od 0x00 do 0x0F
			&& flag == 1 && flaga_starszego == 0) {
				bajt += znak;
				CMD_buffer_rx[i] = bajt;
				crc = crc16(crc, bajt);
				i++;
				bajt = 0;
				flaga_starszego = 1; // nastepnym razem bedzie musial przyjsc znak od 0x10 do 0x1F
			} else {
				error();
			}
		}
		buffer_inc();
	}
}

void buffer_inc() {   // inkrementacja indeksu odczytu z bufora
	buffer_start++;
	if (buffer_start >= buffer_size) {
		buffer_start = 0;
	}
}

//char Temp[5];
//		sprintf(Temp, "%i", CMD_BUFFER_INDEX);
//		send(Temp,5);

void wykonac() {
	if (CMD_buffer_rx[0] == adres_domowy) {
		switch (CMD_buffer_rx[2]) {
		case 0x11:      //zapalenie diody
			if (CMD_buffer_rx[3] == 1) { // zapalenie
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				send("Dioda zapalona\n\r", 16);
			} else if (CMD_buffer_rx[3] == 0) { //zgaszenie
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				send("Dioda zgaszona\n\r", 16);
			}
			break;
		case 0x09:      //przycisk
			info_przyciski(cos);
			break;
		default:
			break;
		}
	}
}

void send(char *dane, uint16_t rozmiar) {
	//dodawanie danych do bufora
	for (int x = 0; x < rozmiar; x++) {
		buffer_tx[buffer_tx_start++] = dane[x];
		if (buffer_tx_start >= buffer_size) {
			buffer_tx_start = 0;
		}
	}
	int ile;
	__disable_irq(); // zablokowanie przerwania
	if (tx_transmit == 0) {
		tx_transmit = 1;
		ile = 0;
		while (buffer_tx_start != buffer_tx_end) {
			buffer_xx[ile++] = buffer_tx[buffer_tx_end++]; //zapis do bufora pomocniczego
			if (buffer_tx_end >= buffer_size) {
				buffer_tx_end = 0;
			}
		}
		HAL_UART_Transmit_IT(&huart2, buffer_xx, ile);
	}
	__enable_irq(); //odblokowanie przerwania
}
//przerwanie wywolywane po wyslaniu znaku
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (buffer_tx_end != buffer_tx_start) {
			int ile1 = 0;
			while (buffer_tx_end != buffer_tx_start) { /*dopoki w buforze jest cos do wyslania,
			 czego jeszcze nie wyslalismy, bedziemy dodawac do tablicy, a pozniej zostana wyslane*/
				buffer_xx[ile1++] = buffer_tx[buffer_tx_end++]; //zapisanie do bufora pomocniczego
				if (buffer_tx_end >= buffer_size) {
					buffer_tx_end = 0;
				}
			}
			HAL_UART_Transmit_IT(&huart2, buffer_xx, ile1);
		} else {
			tx_transmit = 0; //jesli nie ma juz nic do wyslania
		}
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart2, &buffer_rx[buffer_end], 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (timer_ramka >= 1000) {
			error();
			timer_ramka_go = false;
			timer_ramka = 0;
		}

		ramka();

		if (sygnal == true) {
			dekodowanie();
			sygnal = false;
		}

		//if (przycisk!= 0) {
		//	size = sprintf(data, "Wartosc jest rowna: %d.\n\r", news); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
		//	HAL_UART_Transmit_IT(&huart2, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
		//	info = false;
		//}
	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 10000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 4;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : IR_Pin */
	GPIO_InitStruct.Pin = IR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED1_Pin */
	GPIO_InitStruct.Pin = LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
