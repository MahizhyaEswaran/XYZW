/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "iwdg.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "OneWire.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TEMP130_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define VDD_CALIB ((uint16_t) (300))
#define VDD_APPLI ((uint16_t) (330))
#define BatLimit = 282, LowBatLimit = 261;
// #define FLASH_COUNT 0x0800FBD8
#define FLASH_STORAGE 0x08018000
#define page_size FLASH_PAGE_SIZE
#define LOG_PAGE1 0x0802FF80
#define LOG_PAGE 0x0801BE00
#define LOG_PAGE_X 0x0801BE80

void LogPageRead();
void LogPageWrite();
void LogPageErase();
void DataPageErase();
void LogPageRead_x();
void LogPageWrite_x();
void LogPageErase_x();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float ExTemp = 0;
int Tempera;
extern float Temp[MAXDEVICES_ON_THE_BUS];  //Ds18b20
int RT_Int = 0;
int flag;
int time_ready = 0;
int GSM = 0, imei=0;

uint8_t ongoing;
uint16_t size = 400;
uint8_t gsmreply[400];
uint8_t httpReply[200];

int on = 15;
int uart = 1;
int len = 0;
int try1 = 0;
int try2 = 0;
int try3 = 0;
int try4 = 0;
int try5 = 0;
int try6 = 0;
int i = 0;
int count = 0;
RTC_TimeTypeDef sTime = { 0 };
RTC_DateTypeDef sDate = { 0 };
GPIO_InitTypeDef GPIO_InitStruct = { 0 };
uint16_t ADC_VAL = 0;
uint16_t VAL = 0;
int NetSig;
char Coverage1[30];
char sig[2];
char strl1[100];
char strl2[100];
float ds_temp16;
uint8_t downloadedData[200];
int first_time = 1;
int Temp1 = 0;
int BAT_VF;
int BAT_VAL, BAT, tem, DAT;
float temperature;
char MonthSEND_CON;
char strlflashpub[100];
extern int notpub;
int pubok = 0;
int32_t RX_D[600];
int first_save = 1;
int first_erase = 1;
int32_t input[8];
int x;
//uint16_t x = 0;
//uint16_t count;
int a;
int Coverage_int = 0;
int count_flash, new_count, count_f;
volatile uint8_t write_cnt = 0, idx = 0;
volatile uint32_t read_cnt = 0;
volatile uint8_t next_page = 12;

int k = 0;
int hh = 0;
int vv = 0;

int con = 0;
char rt[50];
float b, c;
int first = 0;
float t, h;
double SenVWM1 = 0, SenVWM2 = 0;
int ARead_A1, ARead_A2, sw1int, sw2int, Moisture, EC;
int y, m, d1, h1, m1, s, sec, min, hour, month, date, year, NetSig, val1, val2,
		B, C;
int bint, cint, dint, eint, fint, gint, iint, jint, kint, Bint, Tint, mint,
		ecint;
uint16_t temp16, hum16, dsecs, emins, fhours, gyears, imon, jdate, knet, ltemp,
		Bat16, gbat, eit, moio, eco, SW1_16, SW2_16, sw1, sw2, EC16, Mois16,
		COUNT;
uint8_t hour8, min8, sec8, year8, month8, date8, sig8, blank, it8, rd_count,
		fl_count;
uint8_t it8;
uint32_t input_4_32;
int32_t tnh32;
uint32_t hnmns32;
uint32_t ynmonnda32;
uint32_t signit32;
float DS_int;
float lint;
uint8_t downloadedData[200];
uint8_t val;
char HTTP_SERVER_URL[100] = "http://service.senzmate.com/dia/devices/";
char topic[50] = "SenzMate/S2M/839170035255790";
char topic1[50] = "SenzMate/S2M/";
char da1[2];
char month1[2];
char year1[2];
char minutes[2];
char hours[2];
char seconds[2];
char sig[2];
char write_data[5];
char read_data[5];
char xx[3];
char Time[5];
char s1[] = "04:36";
int HoursSEND = 0;
int MinutesSEND = 0;
int SecondsSEND = 0;
int WeekdaySEND = 0;
int DateSEND = 0;
int MonthSEND = 0;
int YearSEND = 0;
char RTC_VAL[10];
char Imei_arr[50];
char Imei_no[15];

//ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

void IMEI_No_Auto_add();
void GetCoverage();
void GSM_Check();
void DS18b20();
void Power_Toggle();
void GSM_Init();
void HTTP_Init();
void HTTP_Init2();
void HTTP_POST();
void PowerUp();
void PowerDown();
void ADC_Battery();
void read_bat();
void temp();
void ADC_TEMP();
void arraycon();
void arrayallo();
void flash_func();
void variconst();
void flash_readpub();
void flash_writefinal();
uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *fdata,
		uint16_t length);
uint32_t read_flash(uint32_t StartPageAddress, uint8_t *data);
void FlagErase();
void FlagWrite();
void FlagRead();
void Set_Real_time();
void Get_Real_time();
void HTTP_Get(void);
void HTTP_Read(uint16_t Raddress, uint16_t Rsize);
void ConvertRTCTimeToString();
void Time_Date_Get();
void printTime();
void time();
void RePublish();
void rtc_sync();
void increment_rtc_alarm();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char Rx_Data[30];

//uint8_t rx_buff[50];

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

int intToAscii(int number) {
	return '0' + number;
}
int AsciiToint(int Ascii) {
	return Ascii - '0';
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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LogPageRead();
	LogPageRead_x();

	while (1) {

		if (on == 15) {
			HAL_IWDG_Refresh(&hiwdg);
			on = 0;
			PowerUp();
			HAL_Delay(1000);

			ADC_Battery();
			read_bat();
			HAL_ADC_DeInit(&hadc);

			ADC_TEMP();
			temp();
			HAL_ADC_DeInit(&hadc);

			DS18b20();
			HAL_IWDG_Refresh(&hiwdg);

			HAL_Delay(1000);
			if (BAT > BatLimit) {
				HAL_GPIO_WritePin(GSM_PWR_GPIO_Port, GSM_PWR_Pin, RESET); //GSM Power ON
				HAL_Delay(2000);
				Power_Toggle();
				if (uart != 1) {
					HAL_UART_Init(&huart1);
				}
				uart = 0;
				try1 = 0;
				try2 = 0;
				try3 = 0;
				try4 = 0;
				try5 = 0;
				try6 = 0;
				HAL_IWDG_Refresh(&hiwdg);
				HAL_Delay(1000);

				if (first == 0) {
					IMEI_No_Auto_add();
				}
				HAL_IWDG_Refresh(&hiwdg);
				GSM_Init();

				//Set_Real_time();
				if (GSM == 1) {
					Get_Real_time();
				}

				if (time_ready == 1 && imei==1) {
					rtc_sync();
				}
				HAL_IWDG_Refresh(&hiwdg);

				HTTP_Init();

				memset(strl1, 0, sizeof(strl1));
				Time_Date_Get();
				kint = Coverage_int;
				sprintf(strl1,
						"{\"data\":\"DT:%02d%02d%02d%02d|0-T:%0.2f;1-B:%d;2-SS:%02d;3-IT:%.2f;4-X:%d\"}",
						MonthSEND, DateSEND, HoursSEND, MinutesSEND, ExTemp,
						BAT, kint, temperature, count_flash);
				len = strlen(strl1); // length of message
				HAL_Delay(5000);
			    flash_readpub();
				HAL_IWDG_Refresh(&hiwdg);

				if (try2 != 3) {
					HTTP_POST(strl1);
				}

				HAL_Delay(1000);
				HTTP_Init2();
				if (try4 != 3) {
					HTTP_Get();
					HAL_Delay(2000);
				}
				HAL_IWDG_Refresh(&hiwdg);

				if (strstr((char*) downloadedData, "OTA=0")) {
					flag = 0;
				} else if (strstr((char*) downloadedData, "OTA=1")) {
					flag = 1;
				}
				HAL_IWDG_Refresh(&hiwdg);
                FlagWrite();

				if (first != 0) {
					flash_writefinal();
				}

				HAL_Delay(1000);

				if (flag == 0) {
					HAL_Delay(1000);
					HAL_UART_DeInit(&huart1);
					Power_Toggle();
					HAL_GPIO_WritePin(GSM_PWR_GPIO_Port, GSM_PWR_Pin, SET); //GSM Power OFF
					PowerDown();
					HAL_IWDG_Refresh(&hiwdg);
					HAL_NVIC_SystemReset();

				}
				con=0;
				HAL_UART_DeInit(&huart1);
				Power_Toggle();
				HAL_GPIO_WritePin(GSM_PWR_GPIO_Port, GSM_PWR_Pin, SET); //GSM Power OFF

			} else if (BAT > LowBatLimit) {
				HAL_Delay(5000);
				try1 = 3;
				Coverage_int = 0;
				printTime();
				flash_writefinal();
				HAL_IWDG_Refresh(&hiwdg);
			}

			if (on == 15 || on > 15) {
				on = 0;
			}
			PowerDown();
		}
		HAL_IWDG_Refresh(&hiwdg);
		HAL_SuspendTick();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); //Stop Mode
		HAL_ResumeTick();
		SystemClock_Config();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg);
	}
	HAL_IWDG_Refresh(&hiwdg);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* turn on/off GSM module
 */
void Power_Toggle(void) {
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GSM_TOG_GPIO_Port, GSM_TOG_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GSM_TOG_GPIO_Port, GSM_TOG_Pin, GPIO_PIN_RESET);
}

/* custom function to send AT commands at ease
 */

void AT_Command(uint8_t *p_string) {
	uint16_t length = 0;

	while (p_string[length] != '\0') {
		length++;
	}
	HAL_UART_Transmit(&huart1, p_string, length, 100);
}

/* Initialize the GSM module through a state machine. After running this http POST request can be send.
 */
void GSM_Init(void) {
	uint8_t downloadStateGSM = 0;
	GSM = 0;
	ongoing = 1;
	try1 = 0;
	memset(gsmreply, 0, size);

	while (ongoing && try1 < 3) {
		switch (downloadStateGSM) {
		case 0:
			AT_Command((uint8_t*) "AT\r\n");
			HAL_UART_Receive(&huart1, gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStateGSM = 1;
				try1 = 0;
			} else {
				downloadStateGSM = 0;
				try1++;
			}
			memset(gsmreply, 0, size);
			break;
		case 1:
			AT_Command((uint8_t*) "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStateGSM = 2;
				try1 = 0;
			} else {
				downloadStateGSM = 1;
				try1++;
			}
			memset(gsmreply, 0, size);
			break;
		case 2:
			AT_Command((uint8_t*) "AT+SAPBR=3,1,\"APN\",\"PPWAP\"\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStateGSM = 3;
				try1 = 0;
			} else {
				downloadStateGSM = 2;
				try1++;
			}
			memset(gsmreply, 0, size);
			break;
		case 3:
			AT_Command((uint8_t*) "AT+SAPBR=1,1\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT2);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStateGSM = 4;
				try1 = 0;
			} else {
				downloadStateGSM = 3;
				try1++;
			}
			memset(gsmreply, 0, size);
			break;
		case 4:
			AT_Command((uint8_t*) "AT+SAPBR=2,1\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT2);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				try1 = 0;
				GetCoverage();
				downloadStateGSM = 5;
				ongoing = 0;
				GSM = 1;
			} else {
				downloadStateGSM = 4;
				try1++;
			}
			memset(gsmreply, 0, size);
			break;
		}
	}
}
void HTTP_Init(void) {
	uint8_t downloadStateHTTP = 0;
	ongoing = 1;
	try2 = 0;
	char httpParaUrl[200];
	memset(httpParaUrl, 0, 200);
	memset(gsmreply, 0, size);

	while (ongoing && try2 < 3) {
		switch (downloadStateHTTP) {
		case 0:
			AT_Command((uint8_t*) "AT+HTTPINIT\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStateHTTP = 1;
				try2 = 0;
			} else {
				downloadStateHTTP = 0;
				try2++;
			}
			memset(gsmreply, 0, size);
			break;
		case 1:
			sprintf(httpParaUrl, "AT+HTTPPARA=\"URL\",\"%s\"\r\n",
					HTTP_SERVER_URL);
			AT_Command((uint8_t*) httpParaUrl);
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {

				downloadStateHTTP = 2;
				try2 = 0;
			} else {
				downloadStateHTTP = 1;
				try2++;
			}
			memset(gsmreply, 0, size);
			break;
		case 2:
			AT_Command((uint8_t*) "AT+HTTPPARA=\"CID\",1\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {

				downloadStateHTTP = 3;
				try2 = 0;
				ongoing = 0;
			} else {
				downloadStateHTTP = 2;
				try2++;
			}
			memset(gsmreply, 0, size);

			break;
		   	}
	}
	if (try2 != 3) {
				con = 1;
    }
}
void HTTP_Init2(void) {
	uint8_t downloadStateHTTP2 = 0;
	ongoing = 1;
	try4 = 0;
	char httpParaUrl[200];
	memset(httpParaUrl, 0, 200);
	memset(gsmreply, 0, size);

	while (ongoing && try4 < 3) {
		HAL_Delay(2000);
		switch (downloadStateHTTP2) {
		case 0:
			sprintf(httpParaUrl, "AT+HTTPPARA=\"URL\",\"%s\"\r\n",
			HTTP_SERVER_URL2);
			AT_Command((uint8_t*) httpParaUrl);
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStateHTTP2 = 1;
				try4 = 0;
			} else {
				downloadStateHTTP2 = 0;
				try4++;
			}
			memset(gsmreply, 0, size);
			break;
		case 1:
			AT_Command((uint8_t*) "AT+HTTPPARA=\"CID\",1\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT1);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {

				downloadStateHTTP2 = 2;
				try4 = 0;
				ongoing = 0;
			} else {
				downloadStateHTTP2 = 1;
				try4++;
			}
			memset(gsmreply, 0, size);
			break;
		}
	}
}

void HTTP_POST(uint8_t *message) {
	uint16_t length = 0;
	uint8_t downloadStatePost = 0;
	ongoing = 1;
	pubok = 0;
	try3 = 0;

	while (message[length] != '\0') {
		length++;
	}
	char httpData[200];
	memset(httpData, 0, 200);
	memset(gsmreply, 0, size);
	while (ongoing && try3 < 3) {
		switch (downloadStatePost) {
		case 0:
			AT_Command("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r");
			HAL_UART_Receive(&huart1, gsmreply, size, RX_TIMEOUT3);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(100);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStatePost = 1;
				try3 = 0;
			} else {
				downloadStatePost = 0;
				try3++;
			}
			memset(gsmreply, 0, size);
			break;
		case 1:
			sprintf(httpData, "AT+HTTPDATA=%d,%d\r\n", len, RX_TIMEOUT3);
			AT_Command((uint8_t*) httpData);
			HAL_UART_Receive(&huart1, gsmreply, size, RX_TIMEOUT2);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(50);
			if (strstr((char*) gsmreply, "DOWNLOAD")) {
				AT_Command((uint8_t*) message);
				HAL_UART_Receive(&huart1, gsmreply, size, RX_TIMEOUT3);
				HAL_Delay(50);
				downloadStatePost = 2;
				try3 = 0;
			} else {
				downloadStatePost = 1;
				try3++;
			}
			memset(gsmreply, 0, size);
			break;
		case 2:
			AT_Command((uint8_t*) "AT+HTTPACTION=1\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT3);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(100);

			if (strstr((char*) gsmreply, "200")) {
				downloadStatePost = 3;
				try3 = 0;
			} else {
				downloadStatePost = 2;
				try3++;
			}
			memset(gsmreply, 0, size);
			break;
		case 3:
			AT_Command((uint8_t*) "AT+HTTPTERM=?\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, 5000);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStatePost = 4;
				ongoing = 0;
				try3 = 0;
			} else {
				downloadStatePost = 3;
				try3++;
			}
			memset(gsmreply, 0, size);
			break;
		}
	}
	if (try3 != 3) {
		pubok = 1;

	}

}

void HTTP_Get(void) {
	try5 = 0;
	uint8_t downloadStateGet = 0;
	ongoing = 1;
	char httpRequest[35];
	memset(gsmreply, 0, size);
	while (ongoing && try5 < 3) {
		switch (downloadStateGet) {
		case 0:
			AT_Command((uint8_t*) "AT+HTTPACTION=0\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, RX_TIMEOUT2);
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(100);
			if (strstr((char*) gsmreply, "200")) {
				downloadStateGet = 1;
				try5 = 0;
			} else {
				downloadStateGet = 0;
				try5++;
			}
			memset(gsmreply, 0, sizeof(gsmreply));
			break;

		case 1:
			memset(downloadedData, 0, 200);
			sprintf(httpRequest, "AT+HTTPREAD=%d,%d\r\n", 0, 200);
			AT_Command((uint8_t*) httpRequest);
			HAL_UART_Receive(&huart1, (uint8_t*) downloadedData, 200, 10000);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) downloadedData, "OK")) {
				downloadStateGet = 2;
				try5 = 0;
			} else {
				downloadStateGet = 1;
				try5++;
			}
			break;
		case 2:
			AT_Command((uint8_t*) "AT+HTTPTERM=?\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, size, 5000);
			HAL_IWDG_Refresh(&hiwdg);
			if (strstr((char*) gsmreply, "OK")) {
				downloadStateGet = 3;
				ongoing = 0;
				try5 = 0;
			} else {
				downloadStateGet = 2;
				try5++;
			}
			memset(gsmreply, 0, size);
			break;
		}
	}
}

void GetCoverage() {
	Coverage_int = 0;
	int try_count = 0;
	memset(Coverage1, 0, sizeof(Coverage1));
	memset(gsmreply, 0, sizeof(gsmreply));
	while (try_count < 3) {
		AT_Command((uint8_t*) "AT+CSQ\r\n");
		HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, sizeof(gsmreply),
		RX_TIMEOUT2);
		HAL_IWDG_Refresh(&hiwdg);
		if (strstr((char*) gsmreply, "OK")) {
			try_count = 3;
			memset(Coverage1, 0, sizeof(Coverage1));
			memcpy(&Coverage1, gsmreply, sizeof(Coverage1));
			memset(gsmreply, 0, sizeof(gsmreply));
		} else {
			memset(Coverage1, 0, sizeof(Coverage1));
			memset(gsmreply, 0, sizeof(gsmreply));
		}
		try_count += 1;

	}

	if (Coverage1[0] != 0) {
		int i=0;
		int j=0;
		while ((Coverage1[i] != ':')) {
			i++;
			if (i > 30) {
				break;
			}
		}
		i++;

		char x[3] = { 0 };
		while ((Coverage1[i] != ',')) {
			if ((Coverage1[i] >= '0') && (Coverage1[i] <= '9')) {
				x[j++] = Coverage1[i];

			}
			i++;
		}

		Coverage_int = atoi(x);

	}

}

void PowerDown() {
	Power_Toggle();
	HAL_UART_DeInit(&huart1);
	HAL_ADC_DeInit(&hadc);
	//HAL_GPIO_WritePin(GSM_PWR_GPIO_Port, GSM_PWR_Pin, SET);
	HAL_GPIO_WritePin(BAT_GND_GPIO_Port, BAT_GND_Pin, SET);
	HAL_GPIO_DeInit(GPIOB, BAT_GND_Pin | GSM_PWR_Pin | GSM_TOG_Pin);
}

void PowerUp() {
	if (first_time != 1) {
		HAL_UART_Init(&huart1);
		HAL_ADC_Init(&hadc);

		GPIO_InitStruct.Pin = BAT_GND_Pin | GSM_PWR_Pin | GSM_TOG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	}
	first_time = 0;
	HAL_GPIO_WritePin(BAT_GND_GPIO_Port, BAT_GND_Pin, RESET);
}
void LogPageErase() {

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = LOG_PAGE;
	EraseInitStruct.NbPages = 1;

	if ((HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) != HAL_OK) {
		HAL_FLASH_Lock();
	}
	HAL_FLASH_Lock();

}

void LogPageErase_x() {

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = LOG_PAGE_X;
	EraseInitStruct.NbPages = 1;

	if ((HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) != HAL_OK) {
		HAL_FLASH_Lock();
	}
	HAL_FLASH_Lock();

}
void DataPageErase() {

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_STORAGE;
	EraseInitStruct.NbPages = 100;

	if ((HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) != HAL_OK) {
		HAL_FLASH_Lock();
	}
	HAL_FLASH_Lock();

}

void LogPageRead() {

	count_flash = *(uint8_t*) (LOG_PAGE);
}

void LogPageWrite() {
	LogPageErase();

	Flash_Write_Data(LOG_PAGE, (uint32_t*) &count_flash, 1);
}

void LogPageRead_x() {

	x = *(uint8_t*) (LOG_PAGE_X);
}

void LogPageWrite_x() {
	LogPageErase_x();

	Flash_Write_Data(LOG_PAGE_X, (uint32_t*) &x, 1);
}

void DS18b20() {
	HAL_UART_Init(&huart2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET); // Sensor Power
	HAL_Delay(2000);
	get_ROMid();
	HAL_Delay(5000);
	get_Temperature();

	HAL_Delay(2000);
	get_Temperature();  // get temperature value at Temp[0]
	ExTemp = Temp[0];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET);
	HAL_UART_DeInit(&huart2);

}

void ADC_Battery() {

	HAL_ADC_Init(&hadc);

	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;
	ADC1->SMPR |= ADC_SMPR_SMP;

}

void read_bat() {

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	BAT_VAL = HAL_ADC_GetValue(&hadc);
	BAT = (BAT_VAL / 4);
	HAL_ADC_Stop(&hadc);
	HAL_ADC_DeInit(&hadc);

}
void temp() {

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	ADC_VAL = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);

	temperature = ((ADC_VAL * VDD_APPLI / VDD_CALIB)
			- (int32_t) *TEMP30_CAL_ADDR);
	temperature = temperature * (int32_t) (130 - 30);
	temperature = temperature
			/ (int32_t) (*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR);
	temperature = temperature + 30;

	HAL_ADC_DeInit(&hadc);
}

void ADC_TEMP() {

	HAL_ADC_Init(&hadc);

	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	ADC1->CHSELR = ADC_CHSELR_CHSEL18;
	ADC1->SMPR |= ADC_SMPR_SMP;
	ADC->CCR |= ADC_CCR_TSEN;

}
void rtc_sync() {
	if (first == 0 || first == 50) {
		arraycon();
		MX_RTC_Init();
	}
	first += 1;
	if (first == 51) {
		first = 1;
	}
}
void flash_writefinal() {
	if (try1 == 3 || pubok == 0) {
		k = 22;
		arrayallo();
		flash_func();
	}
}
void flash_readpub() {
	if (count_flash != 0 && con == 1) {

		while (count_flash > 0) {
			HAL_IWDG_Refresh(&hiwdg);
			k = 24;
			variconst();
			HAL_IWDG_Refresh(&hiwdg);
			//  HTTP_Init();
			memset(strlflashpub, 0, sizeof(strlflashpub));
			sprintf(strlflashpub,
					"{\"data\":\"DT:%02d%02d%02d%02d|0-T:%0.2f;1-B:%d;2-SS:%02d;3-IT:%.2f\"}",
					iint, jint, fint, eint, DS_int, gint, kint, lint);
			HAL_IWDG_Refresh(&hiwdg);
			HTTP_POST(strlflashpub);
			HAL_IWDG_Refresh(&hiwdg);
			count_flash = count_flash - 1;
			LogPageWrite();		//anistus
			HAL_IWDG_Refresh(&hiwdg);
		}
		count_flash = new_count;
		if (count_flash > x) {
			count_flash = x;
		}
		HAL_IWDG_Refresh(&hiwdg);
		if (count_flash < 0) {
			count_flash = 0;
		}
		HAL_IWDG_Refresh(&hiwdg);
		LogPageWrite();		//anistus
		if (count_flash == 0) {
			DataPageErase();
			a = 0;
			x = 0;
			LogPageWrite_x();
			HAL_IWDG_Refresh(&hiwdg);
			memset(RX_D, 0, sizeof(RX_D));
		}
		new_count = 0;
		HAL_IWDG_Refresh(&hiwdg);
	}
}

void flash_func() {

	LogPageRead_x();
	Flash_Write_Data(FLASH_STORAGE + x * next_page, (uint32_t*) &input,
			fl_count);
	x += 1;
	count_flash += 1;
	LogPageWrite();		//anistus
	LogPageWrite_x();
}

void arrayallo() {

	hour8 = sTime.Hours;
	min8 = sTime.Minutes;
	sec8 = sTime.Seconds;
	year8 = sDate.Year;
	month8 = sDate.Month;
	date8 = sDate.Date;
	sig8 = Coverage_int;
	it8 = temperature;
	blank = 00;
	Bat16 = BAT;

	Tempera = (ExTemp + 100) * 100;

	temp16 = Tempera;

	fl_count = 0;
	input[fl_count] = (month8 << 24) | (date8 << 16) | (hour8 << 8) | min8;
	fl_count += 1;
	input[fl_count] = (Bat16 << 16) | (it8 << 8) | sig8;
	fl_count += 1;
	input[fl_count] = (temp16 << 16) | blank;
	HAL_IWDG_Refresh(&hiwdg);
	fl_count += 1;
	next_page = fl_count * 4;

}

void variconst() {

	read_flash(FLASH_STORAGE + (x - count_flash) * next_page, (uint32_t*) RX_D);

	rd_count = 0;
	while (a < sizeof(RX_D)) {
		a = 0;

		imon = (RX_D[rd_count + a] & 0xff000000) >> 24;
		jdate = (RX_D[rd_count + a] & 0x00ff0000) >> 16;
		fhours = (RX_D[rd_count + a] & 0x0000ff00) >> 8;
		emins = (RX_D[rd_count + a] & 0x000000ff);

		rd_count += 1;
		gbat = (RX_D[rd_count + a] & 0xffff0000) >> 16;
		ltemp = (RX_D[rd_count + a] & 0x0000ff00) >> 8;
		knet = (RX_D[rd_count + a] & 0x000000ff);

		rd_count += 1;
		ds_temp16 = (RX_D[rd_count + a] & 0xffff0000) >> 16;
		HAL_IWDG_Refresh(&hiwdg);
		eint = emins;
		fint = fhours;
		iint = imon;
		jint = jdate;
		lint = ltemp;
		gint = gbat;
		kint = knet;
		ds_temp16 = ds_temp16 / 100;
		ds_temp16 = ds_temp16 - 100;
		DS_int = ds_temp16;

		a += 4;

		break;
	}
}

void printTime() {

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

uint32_t read_flash(uint32_t StartPageAddress, uint8_t *data) {
	volatile uint32_t read_data;
	volatile uint32_t read_cnt = 0;

	while (read_cnt < 2400) {

		read_data = *(uint32_t*) (StartPageAddress + read_cnt);

		data[read_cnt] = (uint8_t) read_data;
		data[read_cnt + 1] = (uint8_t) (read_data >> 8);
		data[read_cnt + 2] = (uint8_t) (read_data >> 16);
		data[read_cnt + 3] = (uint8_t) (read_data >> 24);
		read_cnt += 4;
	}
	return 0;
}

uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *fdata,
		uint16_t length) {

	uint8_t idx = 0;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	while (idx < length) {
		if ((HAL_FLASH_Program(FLASH_TYPEPROGRAMDATA_WORD, StartPageAddress,
				fdata[idx])) == HAL_OK) {
			StartPageAddress += 4;
			idx++;

		} else {
			HAL_FLASH_Lock();
			return HAL_FLASH_GetError();
		}
	}
	return 0;
}

void FlagRead() {

	flag = *(uint8_t*) (LOG_PAGE1);
}

void FlagWrite() {
	FlagErase();

	Flash_Write_Data(LOG_PAGE1, (uint32_t*) &flag, 1);

}

void FlagErase() {

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = LOG_PAGE1;
	EraseInitStruct.NbPages = 1;

	if ((HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) != HAL_OK) {
		HAL_FLASH_Lock();
	}
	HAL_FLASH_Lock();

}

void Set_Real_time() {
	//Get Real Time from GSM Module
	AT_Command((uint8_t*) "AT+CLTS=1\r\n");
	HAL_UART_Receive(&huart2, (uint8_t*) gsmreply, sizeof(gsmreply), 2000);

	AT_Command((uint8_t*) "AT&W\r\n");
	HAL_UART_Receive(&huart2, (uint8_t*) gsmreply, sizeof(gsmreply), 2000);

}
void Get_Real_time() {
	time_ready = 0;
	int try_count = 0;
	while (try_count < 3) {
		AT_Command((uint8_t*) "AT+CCLK?\r\n");
		HAL_UART_Receive(&huart1, (uint8_t*) gsmreply, sizeof(gsmreply),
		RX_TIMEOUT2);
		HAL_IWDG_Refresh(&hiwdg);
		if (strstr((char*) gsmreply, "OK")) {
			try_count = 3;
			memset(rt, 0, sizeof(rt));
			memcpy(rt, gsmreply, sizeof(rt));
			memset(gsmreply, 0, sizeof(gsmreply));
			RT_Int = 1;
			time_ready = 1;
		} else {
			memset(rt, 0, sizeof(rt));
			memset(gsmreply, 0, sizeof(gsmreply));
		}
		try_count += 1;

	}

}

void arraycon() {

	y = 0;
	m = 0;
	d1 = 0;
	m1 = 0;
	h1 = 0;
	s = 0;
//	if(rt[0] != 0){
	if (strstr((char*) rt, "+CCLK:")) {
		char *token = strtok(rt, "\"");
		token = strtok(NULL, "\"");

		char *token1 = strtok(token, "/");
		y = atoi(token1);

		token1 = strtok(NULL, "/");
		m = atoi(token1);

		token1 = strtok(NULL, ",");
		d1 = atoi(token1);

		token1 = strtok(NULL, ":");
		h1 = atoi(token1);

		token1 = strtok(NULL, ":");
		m1 = atoi(token1);

		token1 = strtok(NULL, "+");
		s = atoi(token1);
	}

}

void IMEI_No_Auto_add() {
	imei=0;
	int atComp = 0;
	AT_Command((uint8_t*)"AT\r\n");
	HAL_UART_Receive(&huart1, gsmreply, size, RX_TIMEOUT1);
	HAL_IWDG_Refresh(&hiwdg);
	AT_Command((uint8_t*)"AT+CGSN\r\n");
	HAL_UART_Receive(&huart1, (uint8_t*)gsmreply, sizeof(gsmreply), RX_TIMEOUT2);
	HAL_IWDG_Refresh(&hiwdg);
	memset(Imei_arr, 0, sizeof(Imei_arr));
	memcpy(&Imei_arr, gsmreply, sizeof(Imei_arr));
	memset(gsmreply, 0, sizeof(gsmreply));
	int a;
	int num = 0;
	for (a = 0; a < 50; a = a + 1) {
		if (Imei_arr[a] == '\n' && atComp == 0) {
			atComp = 1;
			continue;
		}
		if (atComp == 1) {
			if (Imei_arr[a] == '\r') {
				break;
			} else {
				Imei_no[num] = Imei_arr[a];
				num = num + 1;
			}
		}

	}
	HAL_IWDG_Refresh(&hiwdg);
	strcat(topic1, Imei_no);
	memset(topic, 0, sizeof(topic));
	memcpy(&topic, topic1, sizeof(topic));
	strcat(HTTP_SERVER_URL, Imei_no);
	strcat(HTTP_SERVER_URL, "/data");
	imei = 1;
}

void Time_Date_Get() {
	// read rtc value;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HoursSEND = sTime.Hours;
	MinutesSEND = sTime.Minutes;
	SecondsSEND = sTime.Seconds;

	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	WeekdaySEND = 0;
	DateSEND = sDate.Date;
	MonthSEND = sDate.Month;
	YearSEND = sDate.Year;

	memset(Time, 0, sizeof(Time));
	Time[0] = rt[28];
	Time[1] = rt[29];
	Time[2] = rt[30];
	Time[3] = rt[31];
	Time[4] = rt[32];

	//convert data to string
	//  ConvertRTCTimeToString();

}

void increment_rtc_alarm() {
	on = on + 1;
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
