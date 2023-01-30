/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_Pin GPIO_PIN_0
#define BAT_GPIO_Port GPIOA
#define BAT_GND_Pin GPIO_PIN_10
#define BAT_GND_GPIO_Port GPIOB
#define GSM_PWR_Pin GPIO_PIN_11
#define GSM_PWR_GPIO_Port GPIOB
#define Sensor_PWR_Pin GPIO_PIN_15
#define Sensor_PWR_GPIO_Port GPIOA
#define GSM_TOG_Pin GPIO_PIN_5
#define GSM_TOG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define TX_TIMEOUT 1000
#define RX_TIMEOUT1 2000
#define RX_TIMEOUT2 4000
#define RX_TIMEOUT3 6000
#define TIME_FOR_MODULE_INIT 12000
//#define HTTP_SERVER_URL "http://service.senzmate.com/dia/devices/Server_auto_test_5/data"
#define HTTP_SERVER_URL2 "http://agro.senzmate.com:8082/test-url.txt"
#define HTTPS_SERVER_URL "https://www.example.com"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
